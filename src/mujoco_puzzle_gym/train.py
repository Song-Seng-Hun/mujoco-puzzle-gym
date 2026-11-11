"""`mpg train task.yaml --minutes N` (R8, R17).

Parallel PuzzleEnvs (SubprocVecEnv, spawn) go through `EpisodeLog`, which
writes one JSON line per finished episode (return, length, success, fall,
switches, NaN/contradiction flags, per-term reward sums), then to the chosen
`Trainer`. A run directory `runs/<id>/` gets:

    task.resolved.yaml   the task with every default filled in
    checkpoint.*         what the trainer saved (SB3: checkpoint.zip)
    episodes.jsonl       one line per training episode
    curves.png           return, success / fall / switch rates, per-term rewards
    summary.json         budget, steps, speed, first/last-window statistics

Training seeds: env i of a run with task `seed: s` starts from seed s + i and
then continues its own RNG stream; `mpg eval` uses seeds from EVAL_SEED_BASE
up, which training never uses (task seeds must stay below TRAIN_SEED_LIMIT).
"""

from __future__ import annotations

import functools
import json
import os
import re
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecEnvWrapper

from .registry import get_algo
from .task import TaskError, load_task, write_resolved

EPISODES_NAME = "episodes.jsonl"
SUMMARY_NAME = "summary.json"
CURVES_NAME = "curves.png"
CHECKPOINT_STEM = "checkpoint"
TRAIN_SEED_LIMIT = 900_000
EVAL_SEED_BASE = 1_000_000


def make_env(task: dict[str, Any]):
    """Env factory run inside each worker: `load_task` imports the task's plugins first."""
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    from .env import PuzzleEnv

    return PuzzleEnv(task=task)


def make_vec_env(task: dict[str, Any], n_envs: int):
    fns = [functools.partial(make_env, task) for _ in range(n_envs)]
    if n_envs == 1:
        return DummyVecEnv(fns)
    return SubprocVecEnv(fns, start_method="spawn")


class EpisodeLog(VecEnvWrapper):
    """Records every finished episode to a JSON-lines file; algorithm-agnostic."""

    def __init__(self, venv, path: Path):
        super().__init__(venv)
        self.path = Path(path)
        self._file = self.path.open("w")
        self.t0 = time.monotonic()
        self.steps = 0
        self.episodes: list[dict[str, Any]] = []
        self._acc = [self._new() for _ in range(self.num_envs)]

    @staticmethod
    def _new() -> dict[str, Any]:
        return {"return": 0.0, "length": 0, "switches_on": 0, "nan": False,
                "contradiction": False, "terms": {}}

    def reset(self):
        self._acc = [self._new() for _ in range(self.num_envs)]
        return self.venv.reset()

    def step_async(self, actions):
        self.venv.step_async(actions)

    def step_wait(self):
        obs, rewards, dones, infos = self.venv.step_wait()
        self.steps += self.num_envs
        for i, (r, done, info) in enumerate(zip(rewards, dones, infos)):
            a = self._acc[i]
            a["return"] += float(r)
            a["length"] += 1
            a["switches_on"] = max(a["switches_on"], int(info.get("switches_on", 0)))
            a["nan"] |= bool(info.get("nan", False))
            a["contradiction"] |= bool(info.get("contradiction", False))
            for k, v in (info.get("reward_terms") or {}).items():
                a["terms"][k] = a["terms"].get(k, 0.0) + float(v)
            if done:
                self._finish(a, info)
                self._acc[i] = self._new()
        return obs, rewards, dones, infos

    def _finish(self, a: dict[str, Any], info: dict[str, Any]) -> None:
        ep = {"i": len(self.episodes), "step": self.steps,
              "t": round(time.monotonic() - self.t0, 2),
              "return": round(a["return"], 4), "length": a["length"],
              "success": bool(info.get("success", False)), "fallen": bool(info.get("fallen", False)),
              "truncated": bool(info.get("TimeLimit.truncated", False)),
              "switches_on": a["switches_on"], "stage": info.get("stage"),
              "nan": a["nan"], "contradiction": a["contradiction"],
              "terms": {k: round(v, 4) for k, v in a["terms"].items()}}
        self.episodes.append(ep)
        self._file.write(json.dumps(ep) + "\n")
        self._file.flush()

    def close(self):
        self._file.close()
        self.venv.close()


# ---- statistics shared with diagnose -------------------------------------------
def window_stats(eps: list[dict[str, Any]], hz: float | None = None) -> dict[str, Any]:
    """Means and rates over a list of episode records (training log or rollouts)."""
    if not eps:
        return {"episodes": 0}
    terms: dict[str, float] = {}
    for e in eps:
        for k, v in e.get("terms", {}).items():
            terms[k] = terms.get(k, 0.0) + v
    n = len(eps)
    out = {
        "episodes": n,
        "mean_return": round(float(np.mean([e["return"] for e in eps])), 3),
        "std_return": round(float(np.std([e["return"] for e in eps])), 3),
        "success_rate": round(sum(e["success"] for e in eps) / n, 3),
        "fall_rate": round(sum(e["fallen"] for e in eps) / n, 3),
        "switch_rate": round(sum(e["switches_on"] > 0 for e in eps) / n, 3),
        "nan_episodes": sum(bool(e.get("nan")) for e in eps),
        "contradiction_episodes": sum(bool(e.get("contradiction")) for e in eps),
        "mean_length": round(float(np.mean([e["length"] for e in eps])), 1),
        "mean_terms": {k: round(v / n, 3) for k, v in terms.items()},
    }
    if hz:
        out["mean_seconds"] = round(out["mean_length"] / hz, 2)
    return out


def split_windows(eps: list[dict[str, Any]], frac: float = 0.2, cap: int = 500):
    k = min(cap, max(1, int(len(eps) * frac)))
    return eps[:k], eps[-k:]


def plot_curves(eps: list[dict[str, Any]], path: Path, title: str = "") -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
    if eps:
        steps = np.array([e["step"] for e in eps], dtype=float)
        win = max(1, min(200, len(eps) // 20))

        def smooth(x):
            x = np.asarray(x, dtype=float)
            c = np.cumsum(np.insert(x, 0, 0.0))
            out = np.empty_like(x)
            for i in range(len(x)):
                lo = max(0, i + 1 - win)
                out[i] = (c[i + 1] - c[lo]) / (i + 1 - lo)
            return out

        axes[0].plot(steps, smooth([e["return"] for e in eps]))
        axes[0].set_ylabel("episode return")
        for key, label in (("success", "success"), ("fallen", "fall"), ("switches_on", "switch on")):
            axes[1].plot(steps, smooth([float(bool(e[key])) for e in eps]), label=label)
        axes[1].set_ylabel("rate")
        axes[1].set_ylim(-0.02, 1.02)
        axes[1].legend(loc="upper left")
        names = sorted({k for e in eps for k in e.get("terms", {})})
        for k in names:
            axes[2].plot(steps, smooth([e["terms"].get(k, 0.0) for e in eps]), label=k)
        axes[2].set_ylabel("reward per episode, by term")
        axes[2].legend(loc="upper left")
    axes[2].set_xlabel(f"env steps (moving average over {win if eps else 0} episodes)")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)
    return path


# ---- the command ------------------------------------------------------------
def _run_id(task_path: str | Path | None, algo: str) -> str:
    stem = Path(task_path).stem if task_path else "task"
    return f"{datetime.now():%Y%m%d-%H%M%S}-{re.sub(r'[^A-Za-z0-9_.-]', '_', stem)}-{algo}"


def train(task_path: str | Path, minutes: float, algo: str | None = None,
          n_envs: int | None = None, runs_dir: str | Path = "runs", run_id: str | None = None,
          max_steps: int | None = None) -> dict[str, Any]:
    wall0 = time.monotonic()
    task = load_task(task_path)
    dropped = {}
    if algo and algo != task["algo"]:
        # algo_kwargs belong to the task's own algorithm (PPO's n_steps means nothing to SAC).
        dropped = task["algo_kwargs"]
        task["algo"], task["algo_kwargs"] = algo, {}
    if n_envs:
        task["n_envs"] = int(n_envs)
    if not 0 <= int(task["seed"]) < TRAIN_SEED_LIMIT:
        raise TaskError(f"seed must be in [0, {TRAIN_SEED_LIMIT}) (eval seeds start at {EVAL_SEED_BASE})")
    if minutes <= 0:
        raise TaskError("--minutes must be > 0")
    trainer_cls = get_algo(task["algo"])

    run_id = run_id or _run_id(task_path, task["algo"])
    run_dir = Path(runs_dir) / run_id
    if run_dir.exists():
        raise TaskError(f"run directory {run_dir} already exists")
    run_dir.mkdir(parents=True)
    write_resolved(task, run_dir)

    venv = EpisodeLog(make_vec_env(task, int(task["n_envs"])), run_dir / EPISODES_NAME)
    venv.seed(int(task["seed"]))
    try:
        trainer = trainer_cls(venv, task, seed=int(task["seed"]))
        t0 = time.monotonic()
        steps = trainer.train(seconds=minutes * 60.0, max_steps=max_steps)
        train_seconds = time.monotonic() - t0
        checkpoint = trainer.save(run_dir / CHECKPOINT_STEM)
    finally:
        venv.close()

    eps = venv.episodes
    first, last = split_windows(eps)
    hz = float(task["env"]["hz"])
    summary = {
        "run_id": run_id,
        "run_dir": str(run_dir.resolve()),
        "task": str(Path(task_path).resolve()),
        "algo": task["algo"],
        "checkpoint": checkpoint.name,
        "minutes_budget": minutes,
        "train_seconds": round(train_seconds, 1),
        "wall_seconds": round(time.monotonic() - wall0, 1),
        "env_steps": int(steps),
        "env_steps_logged": venv.steps,
        "steps_per_s": round(venv.steps / max(train_seconds, 1e-9), 1),
        "n_envs": int(task["n_envs"]),
        "train_seeds": [int(task["seed"]), int(task["seed"]) + int(task["n_envs"]) - 1],
        "episodes": len(eps),
        "first_window": window_stats(first, hz),
        "last_window": window_stats(last, hz),
        **({"algo_kwargs_dropped": dropped} if dropped else {}),
        "files": {"task": "task.resolved.yaml", "episodes": EPISODES_NAME,
                  "curves": CURVES_NAME, "checkpoint": checkpoint.name},
    }
    plot_curves(eps, run_dir / CURVES_NAME, title=f"{run_id}")
    (run_dir / SUMMARY_NAME).write_text(json.dumps(summary, indent=2))
    return summary


def load_run(run_dir: str | Path) -> dict[str, Any]:
    """Summary, resolved task and episode log of a finished run."""
    run_dir = Path(run_dir)
    summary_path = run_dir / SUMMARY_NAME
    if not summary_path.exists():
        raise FileNotFoundError(f"{summary_path} missing: not a finished `mpg train` run")
    summary = json.loads(summary_path.read_text())
    task = load_task(run_dir / "task.resolved.yaml")
    eps_path = run_dir / EPISODES_NAME
    eps = [json.loads(line) for line in eps_path.read_text().splitlines() if line.strip()] \
        if eps_path.exists() else []
    return {"dir": run_dir, "summary": summary, "task": task, "episodes": eps}


def load_trainer(run: dict[str, Any], env=None):
    cls = get_algo(run["summary"]["algo"])
    return cls.load(run["dir"] / run["summary"]["checkpoint"], run["task"], env=env)
