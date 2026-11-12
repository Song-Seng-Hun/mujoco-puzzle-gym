"""Scripted beak work for the `demo:` sub-goals `grab <object>`, `place <object>
<part>`, `drop <object> <part>` and pressing a wall button (R4, R18, R19, R28);
used by solver.py. "Object": any grabbable part (cube, laundry, key, ...).

- grab: pick the side handle (+-x, +-y; the body cannot reach a top handle)
  whose approach point, APPROACH m out from it, is nearest on the free grid;
  walk there, then walk slowly at the handle with the beak closed, lowering
  the head so the beak tip stays at the handle's height. Done once held.
- place: walk at the part's centre carrying the object (near it, only while
  it is farther than the object, then turn in place); once the object's
  centre is within the part's `place_tolerance` (default PLACE_TOL) of it,
  stand until the object hangs still (the head sways while walking, so
  letting go at once throws it), open the beak and stand until it rests (or
  SETTLE s each). The head goes back to level and the body turns round and
  walks off until AWAY m from the object (later walking would kick it off).
  Standing STUCK s with the object as far as the target but beside it (too
  close to turn onto it), it walks off AWAY m from the target and comes again.
  On a hook (a receptacle of shape `hook`) it walks up to the peg instead,
  bringing the object's leading handle (the one farthest ahead) to the peg tip
  at its height, and lets go there.
- drop: as place, but the head goes level first (the highest drop; an egg
  breaks, R28).
- wall button: walk to the point APPROACH m in front of the plate, then
  slowly square at it, the beak tip held at the plate's height, until it is
  on; then (`off_wall`, before anything else) walk off until APPROACH m away.
  Holding a stick, it does the same with the stick's far end instead of the
  beak tip (standing farther off by the stick's reach).
The head and grab values are kept by the solver between env steps (`head`,
`grab`, in action units: head -1 level .. 1 lowest, grab > 0.5 closed).
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

APPROACH = 0.3       # [m] approach point in front of a handle / wall plate
AT_POINT = 0.06      # [m] approach point reached
PLACE_TOL = 0.03     # [m] cube centre this close to the target centre: let go
HANG_TOL = 0.75      # hook: let go once the leading handle is this part of `tolerance` from the
                     # peg tip (or past it)
SETTLE = 1.5         # [s] longest stand after letting go
REST_SPEED = 0.02    # [m/s] cube this slow: placed
AWAY = 0.35          # [m] walk off this far from a placed cube
STUCK = 1.5          # [s] place: standing this long with the object beside the target: walk off, retry
HEAD_GAIN = 6.0      # head level per metre of tip height error, per env step
TIP_PER_HEAD = 0.0375   # [m] beak tip drop per head unit (0.21 -> 0.135 m over -1 .. 1, OQ2)
PITCH_PER_HEAD = 0.8    # [rad] a point `reach` m ahead of the tip moves TIP_PER_HEAD +
                        # PITCH_PER_HEAD * reach per head unit. Measured 0.43 (held stick,
                        # 2026-11-12); doubled: at 0.43 the stick end overshot (head lag)
SLOW = -0.5          # action[0] while closing in (slow walk)
TURN_GAIN = 2.5
FACING = 0.4
LOOK = 0.25
STAND = np.array([-1.0, 0.0, 0.0], dtype=np.float32)
BLOCK_Z = 0.05       # [m] obstacle ray from the trunk centre at this height ...
BLOCK_DIST = 0.16    # [m] ... hitting within this (footprint front 0.11 m) blocks walking off
NOT_GROUP_3 = np.array([1, 1, 1, 0, 1, 1], dtype=np.uint8)   # the body's collision geoms are group 3


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
        self._stuck = 0

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

    def _hold_tip_at(self, z: float, point=None) -> None:
        """Lower or raise the head so the beak tip (or world point `point()`, a point
        the head carries) goes to height `z`. The gain shrinks for points ahead of
        the tip (they move more per head unit; at the tip's gain a stick end overshoots)."""
        tip = self._tip()
        now = tip if point is None else point()
        reach = float(np.linalg.norm((now - tip)[:2]))
        gain = 2.0 * HEAD_GAIN * TIP_PER_HEAD / (TIP_PER_HEAD + PITCH_PER_HEAD * reach)
        self.head = float(np.clip(self.head + gain * (now[2] - z), -1.0, 1.0))

    def _close_in(self, approach, target_xyz, point=None) -> np.ndarray:
        """Approach point first, then slowly at `target_xyz` with the tip (or `point()`)
        at its height."""
        if self.phase == "approach":
            act = self._walk(approach)
            if act is not None:
                return act
            self.phase = "in"
        self._hold_tip_at(float(target_xyz[2]), point)
        return self.steer(target_xyz[:2], SLOW)

    def held(self):
        """The object the beak holds, or None."""
        beak = self.env.parts.get("_beak")
        return beak.objects[beak.held] if beak is not None and beak.held is not None else None

    def _ahead(self, obj) -> np.ndarray:
        """World position of `obj`'s handle farthest along the body's heading."""
        yaw = self.env.body_state.yaw
        pts = obj.handle_points(self.env.data)
        return pts[int(np.argmax(pts[:, :2] @ np.array([math.cos(yaw), math.sin(yaw)])))]

    # ---- sub-goals ---------------------------------------------------------------------
    def grab_act(self, cube) -> np.ndarray | None:
        env = self.env
        if self.held() is cube:
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

    def place_act(self, cube, part, drop: bool = False) -> np.ndarray | None:
        env = self.env
        holding = self.held() is cube
        if getattr(part, "shape", None) == "hook":
            return self._hang_act(cube, part, holding)
        target = np.asarray(part.xy, dtype=float)
        tol = part.place_tolerance(cube) if hasattr(part, "place_tolerance") else PLACE_TOL
        if holding and self.phase == "back":           # overshot: walk off and come again
            act = self._walk_off(target, AWAY)
            if act is not None:
                return act
            self.phase, self._stuck = "approach", 0
        if holding and self.phase not in ("still", "drop"):
            if np.linalg.norm(cube.pos(env.data)[:2] - target) <= tol:
                self.phase = "still"
                if drop:
                    self.head = -1.0
            here = env.body_state.pos[:2]
            d = np.linalg.norm(target - here)
            if d < 2 * LOOK:                      # the cube leads the body: stop when it is as far
                lead = np.linalg.norm(cube.pos(env.data)[:2] - here)
                if d <= lead and self.phase != "still":
                    self._stuck += 1              # as far, but beside it: no in-place turn that small
                    if self._stuck > STUCK * env.hz:
                        self.phase = "back"
                return self.steer(target, SLOW if d > lead else -1.0)
            else:
                act = self._walk(target)
                return act if act is not None else self.steer(target, SLOW)
        return self._let_go(cube)

    def _hang_act(self, obj, hook, holding: bool) -> np.ndarray | None:
        """Bring `obj`'s leading handle to the peg tip, let go there, walk off."""
        env = self.env
        tip = hook.peg_tip()
        if holding and self.phase not in ("still", "drop"):
            lead = self._ahead(obj)
            passed = float((lead[:2] - tip[:2]) @ hook.normal) < 0.0       # past the peg tip
            if np.linalg.norm(lead - tip) <= HANG_TOL * float(hook.cfg["tolerance"]) or passed:
                self.phase = "still"
                return STAND.copy()
            reach = float(np.linalg.norm((lead - self._tip())[:2]))
            left = np.array([hook.normal[1], -hook.normal[0]])        # the body's left facing the post
            side = left * float((lead[:2] - env.body_state.pos[:2]) @ left)   # lead off the body's line
            approach = tip[:2] + hook.normal * (APPROACH + reach) - side
            deep = tip[:2] - hook.normal * APPROACH - side
            return self._close_in(approach, np.array([*deep, tip[2]]), lambda: self._ahead(obj))
        return self._let_go(obj)

    def _let_go(self, obj) -> np.ndarray | None:
        """Stand still, let go, stand until it rests, walk off (place after "still")."""
        env = self.env
        self._settle += 1
        rest = self._settle > SETTLE * env.hz or np.linalg.norm(obj.vel(env.data)) < REST_SPEED and self._settle > 2
        if self.phase == "still":
            if rest:
                self.phase, self._settle = "drop", 0
            return STAND.copy()
        self.grab = -1.0
        if self.phase == "drop":
            if not rest:
                return STAND.copy()
            self.phase, self.head = "away", -1.0
        return self._walk_off(obj.pos(env.data)[:2], AWAY)

    def _walk_off(self, xy, dist) -> np.ndarray | None:
        """Turn away from `xy` and walk until `dist` m from it; None once there."""
        here = self.env.body_state.pos[:2]
        off = here - np.asarray(xy)
        if np.linalg.norm(off) > dist:
            return None
        c, s = math.cos(0.5), math.sin(0.5)              # a little to the left: no dithering at err = pi
        act = self.steer(here + np.array([c * off[0] - s * off[1], s * off[0] + c * off[1]]), 1.0)
        yaw = self.env.body_state.yaw
        ahead = float(-off @ np.array([math.cos(yaw), math.sin(yaw)]))
        in_front = 0.0 < ahead < BLOCK_DIST + 0.05 and ahead > 0.8 * np.linalg.norm(off)
        if in_front or self._blocked_ahead():
            act[0] = -1.0               # the object let go right ahead (walking while turning sweeps
            act[1] = float(np.sign(act[2]))   # it off the scale), or a wall: turn in place, stepping sideways
        else:                           # turn while walking slowly: an in-place turn may not start
            act[0] = max(act[0], SLOW)  # from the stance after letting go (cube_plate seed 11)
        return act

    def _blocked_ahead(self) -> bool:
        """Something fixed (a wall, a basket wall) right in front of the feet: turn in
        place there (walking into it while turning trips the body, basket rooms)."""
        env = self.env
        s = env.body_state
        f = np.array([math.cos(s.yaw), math.sin(s.yaw), 0.0])
        start = np.array([*s.pos[:2], BLOCK_Z])
        gid = np.zeros(1, dtype=np.int32)
        d = mujoco.mj_ray(env.model, env.data, start, f, NOT_GROUP_3, 1, -1, gid)
        fixed = gid[0] >= 0 and env.model.body_weldid[env.model.geom_bodyid[gid[0]]] == 0
        return bool(fixed) and 0.0 <= d <= BLOCK_DIST      # loose objects (the one let go) do not count

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
        tool = self.held()
        if tool is None or not hasattr(tool, "tip_point"):
            return self._close_in(button.xy + button.normal * APPROACH, np.array([*deep, z]))
        reach = float(np.linalg.norm((tool.tip_point(env.data) - self._tip())[:2]))
        return self._close_in(button.xy + button.normal * (APPROACH + reach), np.array([*deep, z]),
                              lambda: tool.tip_point(env.data))
