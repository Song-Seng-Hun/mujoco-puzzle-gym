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
- Checkpoints (R6): when a flag turns on, `advance` saves a snapshot at that
  body step. With task `env.respawn: checkpoint`, a fall or a failure restores
  the last one instead of ending the episode (`info["respawned"]`, reward term
  `respawn_penalty`, default weight -1); without a flag reached yet it ends as usual.
- Parts that override `Part.substep` run after every physics step (R5).
- Tile floors (R10): `fallen` is measured from the top of the floor tile under
  the body (`Room.floor_map`); over a hole (open trapdoor, no floor) the body
  has fallen once its trunk is below the rim.
- Letter maps (R37): the planner grid blocks walls, cliffs, steps above the
  body's `max_step` and ramps above its `max_slope`; with room `on_fall: reset`
  a fall into a hole restores the last checkpoint (or the start) instead of
  ending the episode (`info["respawned"]`).
- Warps (R14): parts append requests to `World.requests`; `advance` carries
  them out after the body step (teleport, `reset` to the last checkpoint or the
  episode start, `next_stage`). `info["warped"]` names the pads used this env
  step and `info["stage_changed"]` is True after a `next_stage` warp.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import gymnasium as gym
import mujoco
import numpy as np

from . import snapshot
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
RESPAWN = ("start", "checkpoint")
RESPAWN_PENALTY = -1.0
_NOW = object()    # target_field: the target's current key


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
        self.world.data = self.data
        self.floor = self.room.floor_map(self.parts)
        self.zone_spawns = self.room.zone_spawns()
        self._needs_start = (any(p.role == "warp" for p in self.parts.values())
                             or self.room.on_fall == "reset")
        self.start_snap: snapshot.Snapshot | None = None
        self.zone = 0
        self._warp_events: dict[str, Any] = {"warped": [], "stage_changed": False}
        self.signals.dt = self.body.control_dt
        subs = [p for p in self.parts.values() if type(p).substep is not Part.substep]
        self._on_substep = None
        if subs:
            def on_substep(model, data, subs=subs):
                for p in subs:
                    p.substep(model, data)
            self._on_substep = on_substep
        self._model0 = snapshot.model_values(self.model)
        self.checkpoint_snap: snapshot.Snapshot | None = None

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

        self.grid: Grid = self.room.grid(body_values=dict(self.body.capabilities.values))
        self.walls = self.room.walls()
        self.obs_terms = [(n, OBS[n]) for n in self.task["obs"]]
        self.reward_terms = [(n, REWARDS[n], float(w)) for n, w in self.task["reward"].items()]
        self.term_fall = bool(self.task["termination"].get("fall", True))
        self.term_success = bool(self.task["termination"].get("success", True))
        self.command_scale = float(self.task["env"].get("command_scale", 1.0))
        self.respawn = self.task["env"].get("respawn", "start")
        if self.respawn not in RESPAWN:
            raise TaskError(f"env.respawn must be one of {RESPAWN}, got {self.respawn!r}")
        if self.respawn == "checkpoint" and "respawn_penalty" not in self.task["reward"]:
            self.reward_terms.append(("respawn_penalty", REWARDS["respawn_penalty"], RESPAWN_PENALTY))
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
        world.checkpoints = [p for p in self.parts.values() if p.role == "checkpoint"]
        for p in self.parts.values():
            p.world = world
        return world

    # ---- snapshots (R6) -------------------------------------------------------
    def save_snapshot(self, **meta) -> snapshot.Snapshot:
        return snapshot.save(self.model, self.data, self.parts, self.signals, self.body,
                             held=sorted(self.world.held), zone=self.zone, **meta)

    # ---- floor (R10) ----------------------------------------------------------
    def floor_height(self, xy) -> tuple[float | None, float]:
        """(top of the floor under `xy` or None over a hole, rim height); 0 on the plane."""
        return (0.0, 0.0) if self.floor is None else self.floor.height_at(xy)

    def read_body(self):
        """Body state with `fallen` measured from the floor under the body (R10)."""
        if self.floor is None:
            return self.body.state(self.data)
        top, rim = self.floor.height_at(self.body.state(self.data).pos[:2])
        z = top if top is not None else rim - self.body.fall_height
        return self.body.state(self.data, floor_z=z)

    def _place_body(self, pose) -> None:
        """Teleport the body onto the floor at `pose` (raised floors, R10/R37)."""
        top, rim = self.floor_height(pose[:2])
        self.body.teleport(self.data, tuple(pose), z=top if top is not None else rim)

    # ---- warps (R14) ----------------------------------------------------------
    def _run_requests(self) -> None:
        reqs, self.world.requests = list(self.world.requests), []
        for req in reqs:
            if req[0] != "warp":
                continue
            _, part, key, dest = req
            self._warp_events["warped"].append(part.name)
            if dest == "reset":
                snap = self.checkpoint_snap or self.start_snap
                if snap is not None:
                    self.restore_snapshot(snap)
                return
            if dest == "next_stage":
                self.zone = part.zone + 1
                pose = self.zone_spawns[self.zone]
                self._warp_events["stage_changed"] = True
            else:
                pose = (*dest["pos"], dest["yaw"])
            self.warp(key, pose)
            self.world.warp_until[key] = self.data.time + part.cooldown
        self.body_state = self.read_body()

    def warp(self, key, pose) -> None:
        """Move the walking body (key "body") or a free body (root body id) to `pose`."""
        if key == "body":
            self._place_body(pose)
            return
        m, d = self.model, self.data
        rid = int(key)
        jnt = int(m.body_jntadr[rid])
        qa, va = int(m.jnt_qposadr[jnt]), int(m.jnt_dofadr[jnt])
        old = d.qpos[qa:qa + 3].copy()
        top_old, rim_old = self.floor_height(old[:2])
        top_new, rim_new = self.floor_height(pose[:2])
        z = (top_new if top_new is not None else rim_new) + old[2] - (top_old if top_old is not None else rim_old)
        d.qpos[qa:qa + 3] = [pose[0], pose[1], z]
        d.qvel[va:va + 6] = 0.0
        mujoco.mj_forward(m, d)

    def restore_snapshot(self, snap: snapshot.Snapshot) -> None:
        snapshot.restore(snap, self.model, self.data, self.parts, self.signals, self.body)
        self.world.held = set(snap.meta.get("held", []))
        self.zone = int(snap.meta.get("zone", 0))
        self.world.requests.clear()
        self.body_state = self.read_body()
        self.stage = self._stage()
        self._mark_target()

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
        snapshot.load_model_values(self.model, self._model0)
        mujoco.mj_resetData(self.model, self.data)
        self.world.held.clear()
        self.world.requests.clear()
        self.world.warp_until.clear()
        self.checkpoint_snap = None
        self.zone = 0
        for name, xy in layout["parts"].items():
            self.parts[name].place(self.model, xy)
        self.body.reset(self.data, layout["spawn"])
        if self.floor is not None:
            self._place_body(layout["spawn"])
        self.signals.reset()
        for p in self.parts.values():
            p.reset(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.body_state: BodyState = self.read_body()
        self.plan = solution_order(self.grid, self.walls, self.parts, layout["spawn"][:2])
        self.targets = [self.parts[n] for n in self.plan["targets"]] if self.plan["ok"] else \
            [p for p in self.parts.values() if p.role in ("switch", "goal")]
        self._fields: dict[tuple, np.ndarray] = {}
        self.stage = self._stage()
        self._mark_target()
        self.steps = 0
        self.events = {"progress": 0.0, "new_on": 0, "success": False, "fallen": False,
                       "respawned": False}
        self._gates_opened: set[str] = set()
        self.start_snap = self.save_snapshot(start=True) if self._needs_start else None

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

    def target_field(self, stage: int | None = None, key: Any = _NOW) -> np.ndarray | None:
        """Distance field to the target of `stage`; `key` picks a cached field of an
        earlier `target_key` (a coin group's field before a coin was picked)."""
        stage = self.stage if stage is None else stage
        if stage >= len(self.targets):
            return None
        t = self.targets[stage]
        k = (stage, t.target_key() if key is _NOW else key)
        if k not in self._fields:
            self._fields[k] = self.grid.distance(self.stage_free(stage), t.target_mask(self.grid))
        return self._fields[k]

    def target_distance(self, stage: int | None = None, key: Any = _NOW) -> float:
        field = self.target_field(stage, key)
        return 0.0 if field is None else self.grid.lookup(field, self.body_state.pos[:2])

    def _target_key(self) -> Any:
        t = self.current_target()
        return t.target_key() if t is not None else None

    def _mark_target(self) -> None:
        self.prev_dist = self.target_distance()
        self.prev_key = self._target_key()

    def _obs(self) -> np.ndarray:
        return np.concatenate([np.asarray(fn(self), dtype=np.float64).ravel()
                               for _, fn in self.obs_terms]).astype(np.float32)

    # ---- stepping -----------------------------------------------------------
    def advance(self, cmd) -> bool:
        """Run one env step of body steps with command `cmd`; False if the sim blew up."""
        for _ in range(self.body_steps):
            self.body.step(self.data, cmd, on_substep=self._on_substep)
            self.signals.step(self.model, self.data, self.read_body())
            if self.world.requests:
                self._run_requests()
            for cp in self.world.checkpoints:
                if cp.pending_snapshot:
                    cp.pending_snapshot = False
                    self.checkpoint_snap = self.save_snapshot(checkpoint=cp.name)
        self.body_state = self.read_body()
        return bool(np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all()
                    and self.data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number == 0)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        layout = (options or {}).get("layout")
        self._reset_sim(self.np_random, layout)
        return self._obs(), self._info()

    def step(self, action):
        on_before = {n for n, p in self.parts.items() if p.role == "switch" and p.satisfied()}
        self._warp_events = {"warped": [], "stage_changed": False}
        cmd = self.body.command_from_action(action) * self.command_scale
        sane = self.advance(cmd)
        self.steps += 1
        on_after = {n for n, p in self.parts.items() if p.role == "switch" and p.satisfied()}

        stage_before = self.stage
        dist_old_target = self.target_distance(stage_before, self.prev_key)
        progress = self.prev_dist - dist_old_target
        self.stage = self._stage()
        if self.stage != stage_before or self._target_key() != self.prev_key:
            self._mark_target()
        else:
            self.prev_dist = dist_old_target
        if not np.isfinite(progress):
            progress = 0.0
        success = any(p.role == "goal" and p.satisfied() for p in self.parts.values())
        fallen = bool(self.body_state.fallen) or not sane
        failed = [n for n, p in self.parts.items() if p.failed()]
        respawned = False
        if (sane and (fallen or failed) and self.respawn == "checkpoint"
                and self.checkpoint_snap is not None):
            self.restore_snapshot(self.checkpoint_snap)
            respawned, fallen, failed = True, False, []
        elif (sane and fallen and not failed and self.room.on_fall == "reset"
              and self.floor_height(self.body_state.pos[:2])[0] is None):
            # R37 / R16 `on_fall: reset`: a fall into a hole goes back, it does not end the episode
            self.restore_snapshot(self.checkpoint_snap or self.start_snap)
            respawned, fallen = True, False
        self.events = {"progress": float(progress), "new_on": len(on_after - on_before),
                       "success": success, "fallen": fallen, "respawned": respawned}
        terms = {name: weight * float(fn(self)) for name, fn, weight in self.reward_terms}
        reward = float(sum(terms.values()))
        terminated = ((success and self.term_success) or (fallen and self.term_fall) or not sane
                      or bool(failed))
        truncated = (not terminated) and self.steps >= self.max_steps
        info = self._info()
        info.update(reward_terms=terms, command=cmd.tolist(), nan=not sane,
                    switches_on=len(on_after), contradiction=self._contradiction(info, success),
                    failed=failed, respawned=respawned, warped=self._warp_events["warped"],
                    stage_changed=self._warp_events["stage_changed"], zone=self.zone)
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
