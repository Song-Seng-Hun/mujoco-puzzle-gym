"""Scripted herding: the `demo:` sub-goal `herd <npc> <part or x y>` (R29).

Done once the robot's centre is in the target: a part with a plate (`half`,
e.g. a button) counts its rectangle, anything else a disk of `AT_TARGET` m.

- flee robots are driven from behind. The robot runs straight away from the
  body, so the body stands on the robot -> target line, on the far side:
  1. approach: walk round the robot's `sense` disk (planned on the room grid,
     the disk blocked so it is not scared off the wrong way) to the point
     `OUT` m outside the disk on that line;
  2. drive: walk to the point `DRIVE` x `sense` behind the robot on the
     (recomputed) line, slowly, and stand while there; the robot keeps
     running towards the target. If the body ends up more than `LOST_ANGLE`
     off the line, it approaches again.
- follow robots are led: the body walks to the point `keep + LEAD` m
  past the target on the robot -> target line, so the robot stopping `keep`
  short of the body stops on the target. It stands while the robot falls
  behind more than `WAIT` x `sense` (the robot follows only within `sense`)
  and walks back towards it when it is out of `sense`.
- lure robots are led the same way by the bait the body holds (`grab <bait>`
  before `herd`): the bait, not the body, goes to that point, and the body
  stands instead of turning to the robot (turning swings the bait).
"""

from __future__ import annotations

import math

import numpy as np

from .parts.input._sense import in_rect

AT_TARGET = 0.05     # [m] robot centre this close to a point target: done
OUT = 0.1            # [m] approach point outside the sense disk
DRIVE = 0.75         # drive point distance behind the robot, x sense
AT_POINT = 0.06      # [m] approach / drive point reached
LOST_ANGLE = 1.0     # [rad] body off the target -> robot line by this: approach again
LEAD = 0.05          # [m] lead point beyond keep
WAIT = 0.85          # follow: stand while the robot is this far x sense behind
REPLAN = 0.04        # [m] re-plan when the approach point moved this far
SLOW = -0.5          # action[0] while driving / leading (slow walk)
TURN_GAIN = 2.5
FACING = 0.4
STAND = np.array([-1.0, 0.0, 0.0], dtype=np.float32)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Herder:
    def __init__(self, env):
        self.env = env
        self.reset()

    def reset(self) -> None:
        self.phase = "approach"
        self._goal = None
        self._field = None
        self._free_now = None

    # ---- targets ---------------------------------------------------------------------
    def target_xy(self, target) -> np.ndarray:
        if isinstance(target, tuple):
            return np.asarray(target, dtype=float)
        return np.asarray(self.env.parts[target].xy, dtype=float)

    def done(self, npc, target) -> bool:
        me = npc.pos(self.env.data)
        if not isinstance(target, tuple):
            part = self.env.parts[target]
            half = getattr(part, "half", None)
            if half is not None and np.ndim(half) == 1 and len(half) >= 2:
                return in_rect(me, part.xy, np.asarray(half[:2]))
        return float(np.linalg.norm(me - self.target_xy(target))) <= AT_TARGET

    # ---- walking ---------------------------------------------------------------------
    def _free(self) -> np.ndarray:
        """Free cells (body clearance) when this sub-goal started (copy)."""
        from .env import GATE_PASSABLE
        from .logic import live_free
        from .solver import CLEARANCE

        env = self.env
        if self._free_now is None:
            self._free_now = live_free(env.grid, env.walls, env.parts, GATE_PASSABLE,
                                       inflate=env.body.footprint_radius + CLEARANCE)
        return self._free_now.copy()

    def _reachable(self, free, xy) -> np.ndarray:
        """`xy`, or the centre of the nearest free cell when `xy` is not free."""
        grid = self.env.grid
        if free[grid.cell(xy)]:
            return np.asarray(xy, dtype=float)
        cell = grid.nearest_free(free, xy)
        return np.asarray(xy, dtype=float) if cell is None else grid.center(*cell)

    def _path_aim(self, goal, avoid_xy=None, avoid_r: float = 0.0) -> np.ndarray:
        env = self.env
        grid = env.grid
        if self._goal is None or np.linalg.norm(goal - self._goal) > REPLAN:
            free = self._free()
            if avoid_xy is not None:
                free &= ~grid.disk(avoid_xy, avoid_r)
            goal = self._reachable(free, goal)
            self._field = grid.distance(free, grid.disk(goal, grid.res))
            self._goal = np.asarray(goal, dtype=float).copy()
        here = env.body_state.pos[:2]
        if np.linalg.norm(self._goal - here) < 0.25 or not np.isfinite(grid.lookup(self._field, here)):
            return self._goal
        return grid.descend(self._field, here, 0.25)

    def _steer(self, aim, forward: float) -> np.ndarray:
        s = self.env.body_state
        d = np.asarray(aim) - s.pos[:2]
        err = _wrap(math.atan2(d[1], d[0]) - s.yaw)
        turn = float(np.clip(TURN_GAIN * err, -1.0, 1.0))
        if abs(err) < FACING:
            return np.array([forward, 0.0, turn], dtype=np.float32)
        return np.array([-1.0, 0.0, float(np.sign(err))], dtype=np.float32)

    def act(self, npc, target) -> np.ndarray | None:
        """Action herding `npc` to `target`, or None when it is there."""
        if self.done(npc, target):
            return None
        if npc.behavior == "flee":
            return self._drive(npc, self.target_xy(target))
        return self._lead(npc, self.target_xy(target))

    def _drive(self, npc, goal) -> np.ndarray:
        env = self.env
        me = npc.pos(env.data)
        here = env.body_state.pos[:2]
        sense = float(npc.live["sense"])
        line = me - goal
        n = float(np.linalg.norm(line))
        u = line / n if n > 1e-6 else np.array([1.0, 0.0])
        rel = here - me
        off = abs(_wrap(math.atan2(rel[1], rel[0]) - math.atan2(u[1], u[0])))
        if self.phase == "drive" and off > LOST_ANGLE:
            self.phase, self._goal = "approach", None
        if self.phase == "approach":
            point = me + u * (sense + OUT)
            if np.linalg.norm(point - here) < AT_POINT or (off < LOST_ANGLE / 2
                                                          and np.linalg.norm(rel) < sense + OUT):
                self.phase = "drive"
            else:
                return self._steer(self._path_aim(point, me, sense), forward=1.0)
        point = self._reachable(self._free(), me + u * DRIVE * sense)
        if np.linalg.norm(point - here) < AT_POINT:
            return self._face(me)
        return self._steer(point, forward=SLOW)

    def _lead(self, npc, goal) -> np.ndarray:
        env = self.env
        me = npc.pos(env.data)
        here = env.body_state.pos[:2]
        bait = npc.bait_pos() if npc.behavior == "lure" else None
        lead = here if bait is None else bait
        gap, sense = float(np.linalg.norm(here - me)), float(npc.live["sense"])
        if gap > sense:                         # out of its reach: walk back towards it
            return self._steer(self._path_aim(me), forward=SLOW)
        if gap > WAIT * sense:
            return self._wait(me, bait)
        line = goal - me
        n = float(np.linalg.norm(line))
        u = line / n if n > 1e-6 else np.array([1.0, 0.0])
        point = goal + u * (float(npc.live["keep"]) + LEAD) + (here - lead)
        if np.linalg.norm(point - here) < AT_POINT:
            return self._wait(me, bait)
        return self._steer(self._path_aim(point), forward=SLOW)

    def _wait(self, me, bait) -> np.ndarray:
        """Lead: face the robot, or stand still holding a bait (turning swings it)."""
        return STAND.copy() if bait is not None else self._face(me)

    def _face(self, xy) -> np.ndarray:
        s = self.env.body_state
        d = np.asarray(xy) - s.pos[:2]
        err = _wrap(math.atan2(d[1], d[0]) - s.yaw)
        return np.array([-1.0, 0.0, float(np.sign(err)) if abs(err) > FACING else 0.0], dtype=np.float32)
