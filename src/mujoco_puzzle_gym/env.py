"""PuzzleEnv (R7): a Gymnasium env of one walking body in one puzzle room.

- Action: Box [-1, 1]^3 = (forward, lateral, turn), mapped to a body command by
  the body adapter (`BodyAdapter.command_from_action`, which knows the walker's
  dead zones). One env step = `1 / hz` seconds = several body steps.
- Observation: concatenation of the task's `obs` terms (registry.OBS).
- Reward: sum of weight * term over the task's `reward` terms (registry.REWARDS);
  per-term weighted values in `info["reward_terms"]`.
- Episode ends on success or fall (task `termination`), or is truncated after
  `episode_seconds`.
- `info` also carries `switches_on` (switches on now) and `contradiction`
  (a part state outside [0, 1], or success while a gate on the solution path
  never opened): evidence for `mpg diagnose`.
- `env.command_scale` != 1 multiplies the body command and turns off the
  body's command clipping; only for deliberately broken tasks.
- A part whose `failed()` is True ends the episode (`info["failed"]` names it).
- Parts that override `Part.substep` run after every physics step (R5).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import gymnasium as gym
import mujoco
import numpy as np

from .bodies import DEFAULT_BODY, BodyState, make_body
from .layout import Grid
from .logic import free_mask, solution_order
from .parts.base import Part, World
from .registry import OBS, REWARDS
from .room import Room
from .task import TaskError, load_task

# A gate counts as having let the body through once its state passed this
# (a half-open door already leaves a gap wider than the body).
GATE_PASSABLE = 0.3


class PuzzleEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 10}

    def __init__(self, task: str | Path | dict | None = None, room: str | Path | None = None,
                 render_mode: str | None = None):
        self.task = load_task(task, room=room)
        self.room = Room.load(self.task["room"])
        body_name = self.task.get("body") or "default"
        self.body = make_body(DEFAULT_BODY if body_name == "default" else body_name)
        self.parts = self.room.make_parts()
        self.signals = self.parts.bus
        self.mj_spec = self.room.build_spec(self.body, self.parts)
        self.model = self.mj_spec.compile()
        self.data = mujoco.MjData(self.model)
        for p in self.parts.values():
            p.bind(self.model, self.data)
        self.world = self._make_world()
        self.signals.dt = self.body.control_dt
        subs = [p for p in self.parts.values() if type(p).substep is not Part.substep]
        self._on_substep = None
        if subs:
            def on_substep(model, data, subs=subs):
                for p in subs:
                    p.substep(model, data)
            self._on_substep = on_substep

        hz = float(self.task["env"]["hz"])
        ratio = 1.0 / (hz * self.body.control_dt)
        self.body_steps = int(round(ratio))
        if abs(ratio - self.body_steps) > 1e-6:
            raise TaskError(f"env.hz={hz}: 1/hz must be a whole number of body steps "
                            f"({self.body.control_dt * 1000:.0f} ms); e.g. 5, 10 or 25")
        self.hz = hz
        self.max_steps = int(round(float(self.task["env"]["episode_seconds"]) * hz))
        self.metadata = {**self.metadata, "render_fps": hz}
        self.render_mode = render_mode

        self.grid: Grid = self.room.grid()
        self.walls = self.room.walls()
        self.obs_terms = [(n, OBS[n]) for n in self.task["obs"]]
        self.reward_terms = [(n, REWARDS[n], float(w)) for n, w in self.task["reward"].items()]
        self.term_fall = bool(self.task["termination"].get("fall", True))
        self.term_success = bool(self.task["termination"].get("success", True))
        self.command_scale = float(self.task["env"].get("command_scale", 1.0))
        if self.command_scale != 1.0:
            self.body.clip_commands = False

        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)
        self._renderer = None
        self._reset_sim(np.random.default_rng(0))
        dim = self._obs().shape[0]
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(dim,), dtype=np.float32)

    def _make_world(self) -> World:
        world = World(dt=self.body.control_dt)
        world.kinds[int(self.model.body_rootid[self.body.root_body_id(self.model)])] = "body"
        for p in self.parts.values():
            if p.kind:
                for rid in p.root_body_ids(self.model):
                    world.kinds[int(self.model.body_rootid[rid])] = p.kind
                    world.objects[int(self.model.body_rootid[rid])] = p
        for p in self.parts.values():
            p.world = world
        return world

    # ---- layout and state ---------------------------------------------------
    def sample_layout(self, rng: np.random.Generator | None) -> dict[str, Any]:
        """Spawn pose and part positions; `rng=None` gives the nominal layout."""
        return {
            "spawn": self.room.sample_spawn(rng),
            "parts": {n: (p.sample_xy(rng) if rng is not None else p.base_xy.copy())
                      for n, p in self.parts.items()},
        }

    def _reset_sim(self, rng: np.random.Generator | None, layout: dict | None = None) -> None:
        layout = layout or self.sample_layout(rng)
        self.layout = layout
        mujoco.mj_resetData(self.model, self.data)
        self.world.held.clear()
        for name, xy in layout["parts"].items():
            self.parts[name].place(self.model, xy)
        self.body.reset(self.data, layout["spawn"])
        self.signals.reset()
        for p in self.parts.values():
            p.reset(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.body_state: BodyState = self.body.state(self.data)
        self.plan = solution_order(self.grid, self.walls, self.parts, layout["spawn"][:2])
        self.targets = [self.parts[n] for n in self.plan["targets"]] if self.plan["ok"] else \
            [p for p in self.parts.values() if p.role in ("switch", "goal")]
        self._fields: dict[int, np.ndarray] = {}
        self.stage = self._stage()
        self.prev_dist = self.target_distance()
        self.steps = 0
        self.events = {"progress": 0.0, "new_on": 0, "success": False, "fallen": False}
        self._gates_opened: set[str] = set()

    def _stage(self) -> int:
        for i, t in enumerate(self.targets):
            if not t.satisfied():
                return i
        return len(self.targets)

    def current_target(self):
        return self.targets[self.stage] if self.stage < len(self.targets) else None

    def stage_free(self, stage: int, inflate: float = 0.0) -> np.ndarray:
        """Free grid cells at `stage`: gates open if the targets before it opened them."""
        on = {t.name for t in self.targets[:stage]}
        return free_mask(self.grid, self.walls, self.parts, on, inflate)

    def target_field(self, stage: int | None = None) -> np.ndarray | None:
        stage = self.stage if stage is None else stage
        if stage >= len(self.targets):
            return None
        if stage not in self._fields:
            self._fields[stage] = self.grid.distance(self.stage_free(stage),
                                                     self.targets[stage].target_mask(self.grid))
        return self._fields[stage]

    def target_distance(self, stage: int | None = None) -> float:
        field = self.target_field(stage)
        return 0.0 if field is None else self.grid.lookup(field, self.body_state.pos[:2])

    def _obs(self) -> np.ndarray:
        return np.concatenate([np.asarray(fn(self), dtype=np.float64).ravel()
                               for _, fn in self.obs_terms]).astype(np.float32)

    # ---- stepping -----------------------------------------------------------
    def advance(self, cmd) -> bool:
        """Run one env step of body steps with command `cmd`; False if the sim blew up."""
        for _ in range(self.body_steps):
            self.body.step(self.data, cmd, on_substep=self._on_substep)
            self.signals.step(self.model, self.data, self.body.state(self.data))
        self.body_state = self.body.state(self.data)
        return bool(np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all()
                    and self.data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number == 0)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        layout = (options or {}).get("layout")
        self._reset_sim(self.np_random, layout)
        return self._obs(), self._info()

    def step(self, action):
        on_before = {n for n, p in self.parts.items() if p.role == "switch" and p.satisfied()}
        cmd = self.body.command_from_action(action) * self.command_scale
        sane = self.advance(cmd)
        self.steps += 1
        on_after = {n for n, p in self.parts.items() if p.role == "switch" and p.satisfied()}

        stage_before = self.stage
        dist_old_target = self.target_distance(stage_before)
        progress = self.prev_dist - dist_old_target
        self.stage = self._stage()
        if self.stage != stage_before:
            self.prev_dist = self.target_distance()
        else:
            self.prev_dist = dist_old_target
        if not np.isfinite(progress):
            progress = 0.0
        success = any(p.role == "goal" and p.satisfied() for p in self.parts.values())
        fallen = bool(self.body_state.fallen) or not sane
        failed = [n for n, p in self.parts.items() if p.failed()]
        self.events = {"progress": float(progress), "new_on": len(on_after - on_before),
                       "success": success, "fallen": fallen}
        terms = {name: weight * float(fn(self)) for name, fn, weight in self.reward_terms}
        reward = float(sum(terms.values()))
        terminated = ((success and self.term_success) or (fallen and self.term_fall) or not sane
                      or bool(failed))
        truncated = (not terminated) and self.steps >= self.max_steps
        info = self._info()
        info.update(reward_terms=terms, command=cmd.tolist(), nan=not sane,
                    switches_on=len(on_after), contradiction=self._contradiction(info, success),
                    failed=failed)
        obs = self._obs()
        if not sane:
            obs = np.nan_to_num(obs, nan=0.0, posinf=0.0, neginf=0.0)
        return obs, reward, terminated, truncated, info

    def _contradiction(self, info: dict[str, Any], success: bool) -> bool:
        states = info["parts"].values()
        if not all(np.isfinite(v) and -1e-6 <= v <= 1.0 + 1e-6 for v in states):
            return True
        for name in self.plan["order"]:
            part = self.parts[name]
            if part.role == "gate" and part.state() >= GATE_PASSABLE:
                self._gates_opened.add(name)
        if success and self.plan["ok"]:
            gates = [n for n in self.plan["order"] if self.parts[n].role == "gate"]
            return any(g not in self._gates_opened for g in gates)
        return False

    def _info(self) -> dict[str, Any]:
        t = self.current_target()
        return {
            "stage": self.stage,
            "target": t.name if t is not None else None,
            "success": any(p.role == "goal" and p.satisfied() for p in self.parts.values()),
            "fallen": bool(self.body_state.fallen),
            "parts": {n: p.state() for n, p in self.parts.items()},
            "plan_ok": self.plan["ok"],
        }

    # ---- rendering ----------------------------------------------------------
    def render(self):
        if self.render_mode != "rgb_array":
            return None
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=480, width=640)
            self._cam = mujoco.MjvCamera()
            self._cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self._cam.elevation = -50.0
            self._cam.distance = max(self.room.size) * 1.3
        self._cam.lookat[:] = [0.0, 0.0, 0.0]
        self._renderer.update_scene(self.data, camera=self._cam)
        return self._renderer.render()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
