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
  or, at a box (a basket), once a wall is right ahead and the object is over
  the inside (walking on leans the body over the wall and it falls after
  letting go), stand until the object hangs still (the head sways while
  walking, so letting go at once throws it), open the beak and stand until it
  rests (or SETTLE s each). The head goes back to level and the body turns
  round and walks off until AWAY m from the object (later walking would kick
  it off); from a box, until half its width + the footprint radius from its
  centre (clear of the walls along an axis; the solver plans round box walls
  from the nearest free cell, solver.py `plan_free`).
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
- turn (a push lever, R28): walk to the point PUSH_START m out from the lever
  on the side that pushes it toward the goal value (the facing side raises
  it), then walk slowly at the lever until its cell is the goal; then walk off
  PUSH_START m.
- turn (a dial, R28): walk to the start of its pass line (PASS_OFF beside the
  axis, PASS_FROM before it), face along it (clockwise in place), walk slowly
  past (one cell a pass); until the cell is the goal, loop round clockwise
  with walking turns (back along the line's far side, LOOP_PAST beyond its
  start, then onto it again). Needs about 1 m clear on the facing side.
- pull (a pull cord or a drawer, R28): close in on the knob as for grab, then turn round
  in place clockwise (the body barely turns in place counter-clockwise from a
  stand) until it faces away from the cord, and walk slowly on until the cord
  is on; then let go. A drawer latches open during the turn. Holding PULL_STALL s
  without progress (the body pivots round the knob), it lets go and comes again.
- look (R27): walk to a point LOOK_DIST m in front of a face of the part's
  marker (a top face: on the body's side of it), then turn to face the marker
  (walking slowly while turning) and stand until `<part>.seen` is 1 (done once
  it was 1 in the episode: a door that opens when seen slides its marker away).
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
PUSH_START = 0.35    # [m] push lever: start the push this far out (and walk off as far)
BACK_STEP = -1.0     # action[1] while turning off a pushed lever (step right as it turns right)
PULL_STALL = 6.0     # [s] pull: holding this long without drawing 1 cm more: let go, come again
DIAL_AIM = 0.3       # [m] dial: steer at the pass line this far ahead
LOOP_PAST = 0.1      # [m] dial: walk back this far past the start of the line before turning in
LOOK_DIST = 0.6      # [m] look: stand this far from the marker (gaze max_dist 2 m, max_tilt 70 deg)
LOOK_AIM = 0.1       # [rad] look: facing the marker this well, stand
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
        self._look = None
        self._point = None
        self._field = None
        self._settle = 0
        self._stuck = 0
        self._at = None              # where `_let_go` let the object go

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
            over = np.linalg.norm(cube.pos(env.data)[:2] - target) <= tol
            if not over and getattr(part, "shape", None) == "box" and self._blocked_ahead():
                inner = part.half - part.wall_t        # against a basket wall: walking on leans the
                over = bool((np.abs(cube.pos(env.data)[:2] - target) <= inner).all())   # body over it
            if over:
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
        return self._let_go(cube, part)

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

    def _let_go(self, obj, part=None) -> np.ndarray | None:
        """Stand still, let go, stand until it rests, walk off (place after "still"): from a
        box receptacle `part`, until half its width + footprint radius from its centre."""
        from .solver import CLEARANCE

        env = self.env
        if not obj.parked:
            self._at = obj.pos(env.data)[:2].copy()
        self._settle += 1
        rest = self._settle > SETTLE * env.hz or (not obj.parked and self._settle > 2      # consumed: let the
                                                  and np.linalg.norm(obj.vel(env.data)) < REST_SPEED)  # body settle
        if self.phase == "still":
            if rest:
                self.phase, self._settle = "drop", 0
            return STAND.copy()
        self.grab = -1.0
        if self.phase == "drop":
            if not rest:
                return STAND.copy()
            self.phase, self.head = "away", -1.0
        if getattr(part, "shape", None) == "box":
            return self._walk_off(part.xy, float(max(part.half)) + env.body.footprint_radius + CLEARANCE)
        if self._at is None:                # gone before it was ever seen here (consumed)
            return None
        return self._walk_off(self._at, AWAY)

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

    # ---- devices (R28) and looking (R27) ------------------------------------------------
    def turn_act(self, lever, goal: float) -> np.ndarray | None:
        """Push lever `lever` until its value is `goal`, then walk off (a dial: `dial_act`)."""
        if getattr(lever, "object_kind", None) == "dial":
            return self.dial_act(lever, int(goal))
        if self.phase == "away":
            return self._back_off(lever)
        done = bool(np.isclose(lever.value, goal))
        if lever.cells is None and self.phase == "in":         # continuous: once pushed past it
            done |= (lever.value - goal) * self._push_dir >= 0.0
        if done:
            if self.phase == "approach":
                return None
            self.phase, self._settle = "away", 0
            return self._back_off(lever)
        if self.phase == "approach":
            self._push_dir = 1.0 if goal > lever.value else -1.0
            act = self._walk(lever.xy + lever.f * self._push_dir * PUSH_START)
            if act is not None:
                return act
            self.phase = "in"
        self.head = -1.0
        return self.steer(lever.xy, SLOW)

    def _back_off(self, lever) -> np.ndarray | None:
        """After a push: stand SETTLE / 3 s, turn round in place clockwise (walking on
        while turning drags the paddle on to the next cell), walk PUSH_START m off."""
        env = self.env
        self._settle += 1
        if self._settle < SETTLE / 3 * env.hz:
            return STAND.copy()
        back = lever.f * self._push_dir
        if np.linalg.norm(env.body_state.pos[:2] - lever.xy) >= PUSH_START:
            return None
        err = _wrap(math.atan2(back[1], back[0]) - env.body_state.yaw)
        if abs(err) > FACING:
            return np.array([-1.0, BACK_STEP, -1.0], dtype=np.float32)
        return np.array([1.0, 0.0, float(np.clip(TURN_GAIN * err, -1.0, 1.0))], dtype=np.float32)

    def dial_act(self, dial, goal: int) -> np.ndarray | None:
        """Walk past the dial on its pass line (one cell a pass) until its cell is `goal`;
        between passes loop round clockwise (walking turns) to the start of the line."""
        from .parts.beak.devices import PASS_FROM, PASS_OFF, PASS_TO

        if dial.cell == goal:
            return None
        env = self.env
        s = env.body_state
        w = dial.walk
        along = float((s.pos[:2] - dial.xy) @ w)
        heading = math.atan2(w[1], w[0])
        self.head = -1.0
        if self.phase == "approach":
            act = self._walk(dial.pass_start())
            if act is not None:
                return act
            self.phase = "align"
        if self.phase == "align":                       # face along the line (clockwise in place)
            if abs(_wrap(heading - s.yaw)) > FACING:
                return np.array([-1.0, 0.0, -1.0], dtype=np.float32)
            self.phase = "line"
        if self.phase == "line":
            if along > PASS_TO:
                self.phase = "turn_back"
            else:                                       # pure pursuit onto the line, 0.3 m ahead
                aim = dial.xy + dial.f * PASS_OFF + w * (along + DIAL_AIM)
                return self.steer(aim, SLOW)
        if self.phase == "turn_back":
            if abs(_wrap(heading + math.pi - s.yaw)) > FACING:
                return np.array([1.0, 0.0, -1.0], dtype=np.float32)
            self.phase = "back"
        if self.phase == "back":
            if along > -PASS_FROM - LOOP_PAST:
                err = _wrap(heading + math.pi - s.yaw)
                return np.array([1.0, 0.0, float(np.clip(TURN_GAIN * err, -1.0, 1.0))], dtype=np.float32)
            self.phase = "turn_in"
        if abs(_wrap(heading - s.yaw)) > FACING:         # turn_in: clockwise onto the line
            return np.array([1.0, 0.0, -1.0], dtype=np.float32)
        self.phase = "line"
        return self.steer(dial.xy + dial.f * PASS_OFF + w * (along + DIAL_AIM), SLOW)

    def pull_act(self, cord) -> np.ndarray | None:
        """Hold the knob (a cord's or a drawer's strap), turn round clockwise, walk on until
        the cord is on / the drawer latched open. No progress for PULL_STALL s while holding
        (the body pivots round the knob instead of turning): let go and come again."""
        env = self.env
        if cord.satisfied():
            self.grab = -1.0
            return None
        self.grab = -1.0 if self.phase == "approach" else 1.0     # no grab while walking up
        if self.held() is not cord:
            if self.phase in ("turn", "walk"):         # let go on the way: come again
                self.reset()
                self.grab = -1.0
            knob = cord.handle_points(env.data)[0]
            self._pulled, self._stall = cord.progress(env.data), 0
            return self._close_in(knob[:2] + cord.f * APPROACH, knob)
        p = cord.progress(env.data)
        if p > self._pulled + 0.01:
            self._pulled, self._stall = p, 0
        self._stall += 1
        if self._stall > PULL_STALL * env.hz:
            self.reset()
            self.grab = -1.0
            return STAND.copy()
        err = _wrap(math.atan2(cord.f[1], cord.f[0]) - env.body_state.yaw)
        if self.phase != "walk" and abs(err) > FACING:
            self.phase = "turn"
            return np.array([-1.0, 0.0, -1.0], dtype=np.float32)
        self.phase = "walk"
        return np.array([SLOW, 0.0, float(np.clip(TURN_GAIN * err, -1.0, 1.0))], dtype=np.float32)

    def look_act(self, part) -> np.ndarray | None:
        """Stand in front of `part`'s marker and face it until `<part>.seen` is 1."""
        from . import markers

        from .gaze import LOOK_PREFIX

        env = self.env
        if env.parts[LOOK_PREFIX + part.name].satisfied():          # `.seen` was 1 this episode
            return None
        if self._look is None:
            self._look = self._look_spot(part, markers.face_poses(env.data, markers.geom_ids(env.model, part)))
        point, centre = self._look
        if self.phase == "approach":
            act = self._walk(point)
            if act is not None:
                return act
            self.phase = "in"
        self.head = -1.0
        s = env.body_state
        err = _wrap(math.atan2(centre[1] - s.pos[1], centre[0] - s.pos[0]) - s.yaw)
        if abs(err) > LOOK_AIM:
            return np.array([SLOW, 0.0, float(np.sign(err))], dtype=np.float32)
        return STAND.copy()

    def _look_spot(self, part, faces) -> tuple[np.ndarray, np.ndarray]:
        """(standing point, marker centre) of the face whose point is nearest on the free grid."""
        env = self.env
        free = self._free()
        here = env.body_state.pos[:2]
        field = env.grid.distance(free, env.grid.disk(here, env.grid.res * 2))
        best, best_d = None, np.inf
        for _, c, n in faces:
            out = n[:2] if abs(n[2]) < 0.7 else here - c[:2]
            p = c[:2] + out / max(float(np.linalg.norm(out)), 1e-9) * LOOK_DIST
            cell = env.grid.cell(p)
            d = env.grid.lookup(field, p) if free[cell] else np.inf
            if best is None or d < best_d:
                best, best_d = (p, c[:2].copy()), d
        return best
