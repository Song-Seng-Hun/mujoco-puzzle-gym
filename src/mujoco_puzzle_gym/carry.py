"""Scripted beak work for the `demo:` sub-goals `grab <cube>`, `place <cube> <part>`
and pressing a wall button (R4, R18, R19); used by solver.py.

- grab: pick the side handle (+-x, +-y; the body cannot reach a top handle)
  whose approach point, APPROACH m out from it, is nearest on the free grid;
  walk there, then walk slowly at the handle with the beak closed, lowering
  the head so the beak tip stays at the handle's height. Done once held.
- place: walk at the part's centre carrying the cube (near it, only while it
  is farther than the cube, then turn in place); once the cube's centre
  is within PLACE_TOL of it, stand until the cube hangs still (the head sways
  while walking, so letting go at once throws it), open the beak and stand
  until it rests (or SETTLE s each). The head goes back to level and the body
  turns round and walks off until AWAY m from the cube (later walking would
  kick it off).
- wall button: walk to the point APPROACH m in front of the plate, then
  slowly square at it, the beak tip held at the plate's height, until it is
  on; then (`off_wall`, before anything else) walk off until APPROACH m away.
The head and grab values are kept by the solver between env steps (`head`,
`grab`, in action units: head -1 level .. 1 lowest, grab > 0.5 closed).
"""

from __future__ import annotations

import math

import numpy as np

APPROACH = 0.3       # [m] approach point in front of a handle / wall plate
AT_POINT = 0.06      # [m] approach point reached
PLACE_TOL = 0.03     # [m] cube centre this close to the target centre: let go
SETTLE = 1.5         # [s] longest stand after letting go
REST_SPEED = 0.02    # [m/s] cube this slow: placed
AWAY = 0.35          # [m] walk off this far from a placed cube
HEAD_GAIN = 6.0      # head level per metre of tip height error, per env step
SLOW = -0.5          # action[0] while closing in (slow walk)
TURN_GAIN = 2.5
FACING = 0.4
LOOK = 0.25
STAND = np.array([-1.0, 0.0, 0.0], dtype=np.float32)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Carrier:
    def __init__(self, env):
        self.env = env
        self.head = -1.0
        self.grab = -1.0
        self.wall = None          # wall button last pressed: walk off it first (off_wall)
        self.reset()

    def reset(self) -> None:
        self.phase = "approach"
        self._point = None
        self._field = None
        self._settle = 0

    # ---- walking ---------------------------------------------------------------------
    def _free(self) -> np.ndarray:
        from .env import GATE_PASSABLE
        from .logic import live_free
        from .solver import CLEARANCE

        env = self.env
        return live_free(env.grid, env.walls, env.parts, GATE_PASSABLE,
                         inflate=env.body.footprint_radius + CLEARANCE)

    def _walk(self, point) -> np.ndarray | None:
        """Action walking to `point` on the free grid; None once there."""
        env = self.env
        grid = env.grid
        here = env.body_state.pos[:2]
        if self._point is None or np.linalg.norm(self._point - point) > 0.02:
            free = self._free()
            cell = grid.cell(point)
            if not free[cell]:
                near = grid.nearest_free(free, point)
                point = point if near is None else grid.center(*near)
            self._field = grid.distance(free, grid.disk(point, grid.res))
            self._point = np.asarray(point, dtype=float).copy()
        if np.linalg.norm(self._point - here) < AT_POINT:
            return None
        aim = self._point
        if np.linalg.norm(self._point - here) > LOOK and np.isfinite(grid.lookup(self._field, here)):
            aim = grid.descend(self._field, here, LOOK)
        return self.steer(aim, 1.0)

    def steer(self, aim, forward: float) -> np.ndarray:
        s = self.env.body_state
        d = np.asarray(aim) - s.pos[:2]
        err = _wrap(math.atan2(d[1], d[0]) - s.yaw)
        if abs(err) < FACING:
            return np.array([forward, 0.0, float(np.clip(TURN_GAIN * err, -1.0, 1.0))], dtype=np.float32)
        return np.array([-1.0, 0.0, float(np.sign(err))], dtype=np.float32)

    def _tip(self) -> np.ndarray:
        env = self.env
        return env.data.site_xpos[env.world.beak_site]

    def _hold_tip_at(self, z: float) -> None:
        """Lower or raise the head so the beak tip goes to height `z`."""
        self.head = float(np.clip(self.head + 2.0 * HEAD_GAIN * (self._tip()[2] - z), -1.0, 1.0))

    def _close_in(self, approach, target_xyz) -> np.ndarray:
        """Approach point first, then slowly at `target_xyz` with the tip at its height."""
        if self.phase == "approach":
            act = self._walk(approach)
            if act is not None:
                return act
            self.phase = "in"
        self._hold_tip_at(float(target_xyz[2]))
        return self.steer(target_xyz[:2], SLOW)

    # ---- sub-goals ---------------------------------------------------------------------
    def grab_act(self, cube) -> np.ndarray | None:
        env = self.env
        beak = env.parts.get("_beak")
        if beak is not None and beak.held is not None and beak.objects[beak.held] is cube:
            return None
        self.grab = 1.0
        if self.phase == "approach" and self._point is None:
            self._handle = self._pick_handle(cube)
        pts = cube.handle_points(env.data)
        h = pts[self._handle]
        n = h[:2] - cube.pos(env.data)[:2]
        n = n / max(float(np.linalg.norm(n)), 1e-9)
        return self._close_in(h[:2] + n * APPROACH, h)

    def _pick_handle(self, cube) -> int:
        env = self.env
        pts = cube.handle_points(env.data)
        centre = cube.pos(env.data)[:2]
        free = self._free()
        here = env.body_state.pos[:2]
        field = env.grid.distance(free, env.grid.disk(here, env.grid.res * 2))
        best, best_d = 0, np.inf
        for i, p in enumerate(pts):
            n = p[:2] - centre
            if np.linalg.norm(n) < 1e-6:            # top handle: out of the body's reach
                continue
            a = p[:2] + n / np.linalg.norm(n) * APPROACH
            d = env.grid.lookup(field, a) if free[env.grid.cell(a)] else np.inf
            if d < best_d:
                best, best_d = i, d
        return best

    def place_act(self, cube, part) -> np.ndarray | None:
        env = self.env
        beak = env.parts.get("_beak")
        holding = beak is not None and beak.held is not None and beak.objects[beak.held] is cube
        target = np.asarray(part.xy, dtype=float)
        if holding and self.phase not in ("still", "drop"):
            if np.linalg.norm(cube.pos(env.data)[:2] - target) <= PLACE_TOL:
                self.phase = "still"
            here = env.body_state.pos[:2]
            d = np.linalg.norm(target - here)
            if d < 2 * LOOK:                      # the cube leads the body: stop when it is as far
                lead = np.linalg.norm(cube.pos(env.data)[:2] - here)
                return self.steer(target, SLOW if d > lead else -1.0)
            else:
                act = self._walk(target)
                return act if act is not None else self.steer(target, SLOW)
        self._settle += 1
        rest = self._settle > SETTLE * env.hz or np.linalg.norm(cube.vel(env.data)) < REST_SPEED and self._settle > 2
        if self.phase == "still":
            if rest:
                self.phase, self._settle = "drop", 0
            return STAND.copy()
        self.grab = -1.0
        if self.phase == "drop":
            if not rest:
                return STAND.copy()
            self.phase, self.head = "away", -1.0
        return self._walk_off(cube.pos(env.data)[:2], AWAY)

    def _walk_off(self, xy, dist) -> np.ndarray | None:
        """Turn away from `xy` and walk until `dist` m from it; None once there."""
        here = self.env.body_state.pos[:2]
        off = here - np.asarray(xy)
        if np.linalg.norm(off) > dist:
            return None
        c, s = math.cos(0.5), math.sin(0.5)              # a little to the left: no dithering at err = pi
        return self.steer(here + np.array([c * off[0] - s * off[1], s * off[0] + c * off[1]]), 1.0)

    def off_wall(self) -> np.ndarray | None:
        """After a wall press: walk off the wall (the planner cannot start against it)."""
        if self.wall is None or not self.wall.satisfied():
            return None
        act = self._walk_off(self.wall.xy, APPROACH)
        if act is None:
            self.wall = None
        return act

    def press_wall_act(self, button) -> np.ndarray | None:
        if button.satisfied():
            self.head = -1.0
            return None
        self.wall = button
        env = self.env
        z = float(env.data.xpos[button._bid][2]) + float(button.cfg["height"])
        deep = button.xy - button.normal * APPROACH       # steer square on, not at the plate itself
        return self._close_in(button.xy + button.normal * APPROACH, np.array([*deep, z]))
