"""`algo: bc` - behaviour cloning from the room's own scripted solution (R131).

Why this algorithm exists, measured rather than assumed. PPO on a carry room (pick the
block up, put it in the box, go to the goal) returned success 0.0 under six reward designs,
two observation kinds and a reverse curriculum, about 32 M environment steps in all
(`docs/training-log.md`, specs/2026-11-17-arm-steady-state R127-R129). The same budget
learns a one-stage reach from the same body and the same `sensors_only` observation to 0.93
(R130), so the gap is not the observation and not the body: it is that the carry's reward
is reached only after a sequence a random policy never completes once.

The room already contains a controller that completes it - `ScriptedSolver`, which `mpg
check` runs on every room and which reads privileged state by design. This trainer treats
it as the demonstrator a real arm would get from teleoperation:

1. **collect**: run the solver in the trainer's own env, recording (observation, the
   solver's action) at every step, and keep only episodes it actually solved. On a share of
   the episodes the *executed* action is perturbed (`noise`) while the recorded label stays
   the solver's clean action, so the data covers states a slightly wrong policy reaches.
   This is DART (Laskey et al. 2017); without it a cloned policy leaves the demonstrated
   states and never comes back.
2. **fit**: an MLP on those pairs, mean squared error against the demonstrated action.
3. **roll out** the kept policy on the logged parallel envs for the rest of the budget,
   which is what writes `episodes.jsonl` (so curves, success rate and the per-term rewards
   in `summary.json` describe *this* policy, not the demonstrator).

What it is not: it is not reinforcement learning and it never improves on its demonstrator.
It needs a room whose `demo:` the solver solves, and it copies that solution's behaviour,
privileged reasoning included - what the policy ends up reading is only the task's `obs:`.

`algo_kwargs`: `demo_episodes` (default 200), `noise` (0.15), `noise_share` (0.5),
`epochs` (40), `batch` (256), `lr` (1e-3), `rollout_share` (0.25 of the time budget),
`tail_weight` (1, off) and `tail_share` (0.3): repeat the last share of each solved
episode that many times, so a multi-stage room's final stage is not a tenth of the data.
Measured on the contact carry room with weight 4: the fit improved (loss 0.0094 -> 0.0008)
and the policy stopped picking the block up at all (`stage` 2.03 -> 0.0). Whichever stage
the data emphasises is the stage it does, which is the reward ladder's result again (R127) -
so what that room is short of is not data or shaping.
`rounds` (1) and `round_episodes` (100): each round fits, then flies the fitted policy for
that many episodes with the solver labelling what it visits - which both scores the model
and feeds the next round's fit. The best-scoring model is what gets saved.
`net:` sets the hidden layers (default [256, 256]).
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, ClassVar

import numpy as np
import torch
from torch import nn

from ..registry import register_algo
from ..task import TaskError
from .base import Deadline, Trainer

DEFAULTS = {"demo_episodes": 200, "noise": 0.15, "noise_share": 0.5, "epochs": 40,
            "batch": 256, "lr": 1e-3, "rollout_share": 0.25, "rounds": 1, "round_episodes": 100,
            "tail_weight": 1, "tail_share": 0.3}


class Mlp(nn.Module):
    def __init__(self, obs_dim: int, act_dim: int, hidden: list[int]):
        super().__init__()
        layers: list[nn.Module] = []
        last = obs_dim
        for h in hidden:
            layers += [nn.Linear(last, h), nn.ReLU()]
            last = h
        layers += [nn.Linear(last, act_dim), nn.Tanh()]
        self.net = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


@register_algo("bc")
class BehaviourCloning(Trainer):
    name: ClassVar[str] = "bc"

    def __init__(self, env: Any, task: dict[str, Any], seed: int = 0):
        super().__init__(env, task, seed)
        self.kw = {**DEFAULTS, **(task.get("algo_kwargs") or {})}
        bad = sorted(set(self.kw) - set(DEFAULTS))
        if bad:
            raise TaskError(f"algo bc: unknown algo_kwargs {bad}; known: {sorted(DEFAULTS)}")
        self.hidden = [int(h) for h in (task.get("net") or [256, 256])]
        self.model: Mlp | None = None
        self.demo: dict[str, Any] = {}
        torch.manual_seed(seed)

    # ---- 1. demonstrations ---------------------------------------------------------
    def _collect(self, deadline: Deadline, drive: bool = False, tag: str = "",
                 episodes: int | None = None) -> tuple[list[np.ndarray], list[np.ndarray],
                                                       dict[str, Any]]:
        """Episodes of (observation, the solver's action).

        `drive=False` is the first round: the solver flies the arm, a share of the episodes
        with noise added to what is *sent* (DART), and only solved episodes are kept.
        `drive=True` is a DAgger round (Ross et al. 2011): the **current policy** flies, the
        solver labels every state it reaches, and every pair is kept - the failures are the
        point, because they are the states the policy actually visits and the label says what
        the demonstrator would have done there.
        """
        from ..env import PuzzleEnv
        from ..solver import ScriptedSolver

        env = PuzzleEnv(task=self.task)
        if not hasattr(env.observation_space, "shape") or env.observation_space.shape is None:
            env.close()
            raise TaskError("algo bc: needs a vector observation (a Dict obs, e.g. a camera, "
                            "is not supported)")
        rng = np.random.default_rng(self.seed + (1 if drive else 0))
        obs_rows: list[np.ndarray] = []
        act_rows: list[np.ndarray] = []
        solved = attempted = steps = 0
        try:
            n = int(self.kw["demo_episodes"] if episodes is None else episodes)
            for i in range(n):
                if deadline.done():
                    break
                obs, _ = env.reset(seed=self.seed + (10_000 if drive else 0) + i)
                solver = ScriptedSolver(env)
                solver.reset()
                noisy = rng.random() < float(self.kw["noise_share"])
                pairs: list[tuple[np.ndarray, np.ndarray]] = []
                attempted += 1
                ok = False
                while True:
                    want = np.asarray(solver.act(), dtype=np.float32)
                    pairs.append((np.asarray(obs, dtype=np.float32), want.copy()))
                    send = want
                    if drive:
                        send = np.asarray(self.predict(np.asarray(obs, dtype=np.float32)),
                                          dtype=np.float32)
                    elif noisy:
                        send = np.clip(want + rng.normal(0.0, float(self.kw["noise"]),
                                                         want.shape), -1.0, 1.0)
                    obs, _, term, trunc, info = env.step(send.astype(np.float32))
                    steps += 1
                    if term or trunc:
                        ok = bool(info.get("success"))
                        break
                solved += int(ok)
                if ok or drive:
                    obs_rows += [p[0] for p in pairs]
                    act_rows += [p[1] for p in pairs]
                    # The end of a demonstration is the smallest part of it, and on a
                    # multi-stage room it is the part a clone fails in: measured on the
                    # contact carry room, the policy put the block in the box in half its
                    # episodes and then never finished, while the last stage was about a
                    # tenth of the pairs. `tail_weight` repeats the last `tail_share` of each
                    # episode so the fit does not average it away.
                    reps = int(self.kw["tail_weight"]) - 1
                    if ok and reps > 0:
                        cut = int(len(pairs) * (1.0 - float(self.kw["tail_share"])))
                        tail = pairs[cut:]
                        obs_rows += [p[0] for p in tail for _ in range(reps)]
                        act_rows += [p[1] for p in tail for _ in range(reps)]
        finally:
            env.close()
        info = {f"{tag}episodes": attempted, f"{tag}episodes_solved": solved,
                f"{tag}steps": len(obs_rows), f"{tag}env_steps": steps}
        if not obs_rows and not drive:
            raise TaskError("algo bc: the scripted solution solved none of the demonstration "
                            "episodes, so there is nothing to clone (check `mpg check` first)")
        return obs_rows, act_rows, info

    # ---- 2. fit --------------------------------------------------------------------
    def _fit(self, x: np.ndarray, y: np.ndarray, deadline: Deadline) -> dict[str, Any]:
        self.model = Mlp(x.shape[1], y.shape[1], self.hidden)
        opt = torch.optim.Adam(self.model.parameters(), lr=float(self.kw["lr"]))
        xs = torch.as_tensor(x, dtype=torch.float32)
        ys = torch.as_tensor(y, dtype=torch.float32)
        batch = int(self.kw["batch"])
        g = torch.Generator().manual_seed(self.seed)
        last = float("nan")
        epochs = 0
        for _ in range(int(self.kw["epochs"])):
            if deadline.done():
                break
            order = torch.randperm(len(xs), generator=g)
            total = 0.0
            for k in range(0, len(order), batch):
                idx = order[k:k + batch]
                loss = ((self.model(xs[idx]) - ys[idx]) ** 2).mean()
                opt.zero_grad()
                loss.backward()
                opt.step()
                total += float(loss.detach()) * len(idx)
            last = total / max(len(xs), 1)
            epochs += 1
        return {"fit_epochs": epochs, "fit_loss": round(last, 5)}

    # ---- scoring: the policy alone, no solver in the loop ----------------------------
    def _score(self, episodes: int, deadline: Deadline) -> tuple[float, int]:
        """Success rate of the current model with **nothing else driving**.

        This has to be its own pass, and finding that out cost a wrong number: a DAgger pass
        calls `ScriptedSolver.act()` every step for the label, and that call is not a
        read - it runs the carrier, which commands the grasp pose and the jaw. Scored inside
        such a pass the first fit "solved" 100 of 100 while `mpg eval` on the same model
        scored 0.0 (R131). A score is only a score when the policy is alone in the loop.
        """
        from ..env import PuzzleEnv

        env = PuzzleEnv(task=self.task)
        ok = ran = steps = 0
        try:
            for i in range(episodes):
                if deadline.done():
                    break
                obs, _ = env.reset(seed=self.seed + 20_000 + i)
                ran += 1
                while True:
                    obs, _, term, trunc, info = env.step(
                        np.asarray(self.predict(np.asarray(obs, dtype=np.float32)),
                                   dtype=np.float32))
                    steps += 1
                    if term or trunc:
                        ok += int(bool(info.get("success")))
                        break
        finally:
            env.close()
        return (ok / max(ran, 1), steps)

    # ---- 3. logged rollout ----------------------------------------------------------
    def _rollout(self, deadline: Deadline) -> int:
        if self.env is None:
            return 0
        obs = self.env.reset()
        steps = 0
        while not deadline.done(steps):
            act = self.predict(np.asarray(obs, dtype=np.float32), deterministic=True)
            obs, _, _, _ = self.env.step(act)
            steps += int(self.env.num_envs)
        return steps

    def train(self, seconds: float, max_steps: int | None = None) -> int:
        """Collect, fit, score, and keep the model that actually solved the most.

        Each round fits, **scores the model alone** (`_score`, no solver in the loop), and
        then - if another round follows - flies it once more with the solver labelling what
        it visits, which is the next round's data. Selection by that score is not a
        refinement: measured on the contact carry room, one fit and a second fit over the
        aggregated set differed by the whole result, so a run without selection throws a
        working policy away.
        """
        share = float(self.kw["rollout_share"])
        rounds = max(int(self.kw["rounds"]), 1)
        learn = Deadline(seconds * (1.0 - share), max_steps)
        xs, ys, info = self._collect(learn, tag="demo_")
        collected = int(info["demo_env_steps"])
        best: tuple[float, dict[str, Any]] | None = None
        for r in range(1, rounds + 1):
            info.update({f"round{r}_{k}": v for k, v in
                         self._fit(np.asarray(xs), np.asarray(ys), learn).items()})
            score, scored = self._score(int(self.kw["round_episodes"]), learn)
            collected += scored
            info[f"round{r}_success"] = round(score, 3)
            if best is None or score > best[0]:
                best = (score, {k: v.clone() for k, v in self.model.state_dict().items()})
            if r < rounds and not learn.done():
                nx, ny, ninfo = self._collect(learn, drive=True, tag=f"round{r}_",
                                              episodes=int(self.kw["round_episodes"]))
                info.update(ninfo)
                collected += int(ninfo[f"round{r}_env_steps"])
                xs += nx
                ys += ny
        assert best is not None and self.model is not None
        self.model.load_state_dict(best[1])
        info["kept_success"] = best[0]
        rolled = self._rollout(Deadline(max(seconds * share, 1.0),
                                        None if max_steps is None else
                                        max(max_steps - collected, 1)))
        self.demo = {**info, "pairs": len(xs), "rollout_steps": rolled}
        return int(collected + rolled)

    # ---- save / load / predict -------------------------------------------------------
    def save(self, path: Path) -> Path:
        assert self.model is not None, "nothing trained yet"
        out = Path(path).with_suffix(".pt")
        torch.save({"state_dict": self.model.state_dict(), "hidden": self.hidden,
                    "obs_dim": self.model.net[0].in_features,
                    "act_dim": self.model.net[-2].out_features,
                    "demo": self.demo}, out)
        return out

    @classmethod
    def load(cls, path: Path, task: dict[str, Any], env: Any = None) -> "BehaviourCloning":
        blob = torch.load(Path(path), map_location="cpu", weights_only=True)
        self = cls(env, task)
        self.hidden = [int(h) for h in blob["hidden"]]
        self.model = Mlp(int(blob["obs_dim"]), int(blob["act_dim"]), self.hidden)
        self.model.load_state_dict(blob["state_dict"])
        self.model.eval()
        self.demo = blob.get("demo", {})
        return self

    def predict(self, obs: np.ndarray, deterministic: bool = True) -> np.ndarray:
        assert self.model is not None, "nothing trained yet"
        x = np.asarray(obs, dtype=np.float32)
        flat = x.ndim == 1
        with torch.no_grad():
            out = self.model(torch.as_tensor(x[None] if flat else x)).numpy()
        return out[0] if flat else out
