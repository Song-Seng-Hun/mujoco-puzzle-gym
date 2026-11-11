"""Scripted privileged solver for `mpg check` (c).

It reads the true layout (part positions, solution order) and plans on the room
grid with walls inflated by the body's `footprint_radius`, then steers along
the geodesic distance field to the current target. Actions use only the
body-agnostic action convention (see BodyAdapter.command_from_action): it
walks at full forward speed when roughly facing its aim point, and otherwise
walks slowly (action[0] = 0) while turning, which keeps every body command out
of the walker's dead zones.
"""

from __future__ import annotations

import math

import numpy as np

CLEARANCE = 0.02       # extra margin beyond the footprint [m]
LOOKAHEAD = 0.3        # aim point distance along the planned path [m]
TURN_GAIN = 2.5        # action[2] per rad of heading error
FACING = 0.4           # |heading error| [rad] below which it walks at full speed
PEEK = 0.12            # look this far ahead for walls before creeping while turning [m]


class ScriptedSolver:
    def __init__(self, env):
        from .push import Pusher

        self.env = env
        self._fields: dict[tuple, np.ndarray] = {}
        self.pusher = Pusher(env)          # targets with `push_target` (R38)

    def reset(self) -> None:
        self._fields = {}
        self.pusher.reset()

    def field(self, stage: int) -> np.ndarray:
        env = self.env
        key = (stage, env.targets[stage].target_key())
        if key not in self._fields:
            free = env.stage_free(stage, inflate=env.body.footprint_radius + CLEARANCE)
            self._fields[key] = env.grid.distance(free, env.targets[stage].target_mask(env.grid))
        return self._fields[key]

    def act(self) -> np.ndarray:
        env = self.env
        if env.current_target() is None:
            return np.array([-1.0, 0.0, 0.0], dtype=np.float32)
        target = env.current_target()
        if hasattr(target, "push_target"):
            return self.pusher.act(target)
        s = env.body_state
        aim = env.grid.descend(self.field(env.stage), s.pos[:2], LOOKAHEAD)
        if np.hypot(*(target.xy - s.pos[:2])) < LOOKAHEAD and np.isfinite(self.field(env.stage)).any():
            # Close: aim at the target itself unless it is behind a wall.
            iy, ix = env.grid.cell(target.xy)
            if np.isfinite(self.field(env.stage)[iy, ix]):
                aim = target.xy
        d = aim - s.pos[:2]
        err = math.atan2(d[1], d[0]) - s.yaw
        err = (err + math.pi) % (2 * math.pi) - math.pi
        turn = float(np.clip(TURN_GAIN * err, -1.0, 1.0))
        forward = 1.0 if abs(err) < FACING else 0.0
        if forward == 0.0:
            ahead = s.pos[:2] + PEEK * np.array([math.cos(s.yaw), math.sin(s.yaw)])
            if not np.isfinite(self.field(env.stage)[env.grid.cell(ahead)]):
                forward = -1.0        # wall ahead: turn in place instead of creeping into it
        return np.array([forward, 0.0, turn], dtype=np.float32)


def run_episode(env, seed: int, solver: ScriptedSolver | None = None) -> dict:
    solver = solver or ScriptedSolver(env)
    env.reset(seed=seed)
    solver.reset()
    info: dict = {}
    steps = 0
    while True:
        _, _, terminated, truncated, info = env.step(solver.act())
        steps += 1
        if terminated or truncated:
            break
    return {"seed": seed, "success": bool(info.get("success")), "fallen": bool(info.get("fallen")),
            "nan": bool(info.get("nan")), "steps": steps, "seconds": steps / env.hz,
            "stage": info.get("stage"), "target": info.get("target")}
