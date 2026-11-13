"""Scripted privileged solver for `mpg check` (c), the dynamic stage (R20).

It reads the true layout (part positions, solution order) and plans on the room
grid with walls inflated by the body's `footprint_radius`, then steers along
the geodesic distance field to the current target. Actions use only the
body-agnostic action convention (see BodyAdapter.command_from_action): it
walks at full forward speed when roughly facing its aim point, and otherwise
walks slowly (action[0] = 0) while turning, which keeps every body command out
of the walker's dead zones.

`demo:` (R20, room.py): when the room lists sub-goals, the solver runs them in
order first, planning on the parts' state now (a door counts as open once it
is passable, an opening trapdoor as a hole):
- `goto <part>`: walk until the body centre is in the part's target area (a
  goal: until reached);
- `press <part>`: walk onto it until it is on (`satisfied()`);
- `wait <s>`: stand for s seconds;
- `push <ball> to <target>`: push that ball (push.py) until the target counts
  it (a goal net) or is on (a pin);
- `herd <npc> <part or x y>`: drive a flee robot from behind, or lead a follow
  or lure robot, until its centre is on the target (herd.py, R29);
- `grab <object>`: walk up to a handle and close the beak on it (carry.py, R18);
- `place <object> <part>`: carry it until it is over the part, let go and wait
  for it to rest; on a hook, hang it on the peg (R28);
- `drop <object> <part>`: as place, letting go from the level head (R28 egg);
- `turn <lever> <value>`: push a push lever to that cell; `turn <dial> <cell>`: walk
  past the dial, looping round clockwise, until it is at that cell (R28);
- `pull <cord or drawer>`: hold the knob, turn round, walk away until the cord is on
  or the drawer latched open, then let go (R28);
- `look <part>`: stand facing the part's marker until `<part>.seen` is 1 (R27);
- `take <dispenser>`: step on its floor plate, press its wall plate with the beak
  tip, or (no plate) stand, until it hands out a copy, then stand until the copy
  rests (at most SETTLE s); later `grab` / `place` / `drop` with the dispenser's
  name work on that copy (R28);
- `addon <station>`: walk onto the station's pad and stand until it switches the
  body's addon (R30; a station with `in` channels waits for them).
- `glide <low bar>`: on wheels, go back to at least GLIDE_START before the bar's
  centre line if closer (to GLIDE_RUNUP), face across the bar, push at full
  strength along the line through the bar's centre and send `crouch` once the
  trunk is GLIDE_CROUCH before the line; done GLIDE_PAST beyond it (R30; the
  task needs the `crouch` action).
- `tow <rope> <hitch>`: done once the rope ties the body and the hitch; else walk
  onto the rope's attach (or toggle) hitch station nearest the hitch and stand until
  it ties (no such station: stand, a channel ties it) (R31);
- `untow <rope>`: done once the rope is off the body; else the same with a detach,
  toggle or handoff station (a handoff moves the end to the station's `to:` hitch).
- `drag <rope> <part>`: the towed hitch is the rope's end away from the body (its
  other end, or for a rope between two towed things the end farther along the
  chain); walk forward to a point beyond the part's centre, on the line from the
  hitch (where it is when the sub-goal starts) through that centre, so the hitch
  follows into the part: DRAG_PAST farther than the rope length or than the body's
  distance from the hitch then (a chain of ropes), short of walls; done once the hitch is
  DRAG_MARGIN inside the part's floor area, or once the hitch comes loose on the
  way (the part's own detach channel) (R31). Nothing towed through the rope at
  the start: stand (a channel may tie it).
`press` on a wall button (and a wall button in the grid answer) walks up to
the plate with the beak tip at its height, or with a held stick's far end
(R28). Head and grab values are kept between steps (a held object stays held
through later sub-goals until `place`)
and go to the env through `env.scripted_action`.
After the last sub-goal, and in rooms without `demo:`, it follows the grid
answer: the env's solution-order targets, pushing a ball for a target with
`push_target` (goal nets, pins), pushing a lever to 1, pulling a cord,
looking at a part for a `_look_` switch (gaze.py) and, for a receptacle,
carrying an object it accepts there: the nearest free one not in it yet, or
else a copy taken from a dispenser with stock whose item it accepts (R5, R28).
"""

from __future__ import annotations

import math

import numpy as np

CLEARANCE = 0.02       # extra margin beyond the footprint [m]
LOOKAHEAD = 0.3        # aim point distance along the planned path [m]
TURN_GAIN = 2.5        # action[2] per rad of heading error
FACING = 0.4           # |heading error| [rad] below which it walks at full speed
PEEK = 0.12            # look this far ahead for walls before creeping while turning [m]
STAND = np.array([-1.0, 0.0, 0.0], dtype=np.float32)
TAKE_SETTLE = 1.5      # [s] take: wait at most this long for a new copy to rest
WALL_WAIT = 0.35        # [m] off a dispenser's wall plate while it cannot hand out (take)
REST_SPEED = 0.02      # [m/s] a copy this slow rests
ADDON_MARGIN = 0.06    # [m] addon: stand this far inside the station's pad edge
PECK_CLOSE = 0.5       # [m] break via peck: steer straight at the part this close (R34)
PECK_STAND = 0.3       # [m] break via peck: walk to this point in front of a cover first
LEVER_NEAR = 0.7       # [m] break via shell: plan the way to the aim lever until this close
BACK_OFF = 0.35        # [m] break / repair: walk this far off the part before the next sub-goal
BACK_SECONDS = 3.0     # [s] longest try at that (against a wall there may be no room)
PECK_SPEED = -0.5      # action[0] while leaning on the part to peck it
# glide (R30, measured 2026-11-13 at a 0.228 m bar, 20 seeds, crouch at distance d before
# the bar's centre line): entering at 0.32-0.48 m/s, d 0.2 / 0.25 / 0.3 / 0.35 / 0.4 / 0.5
# glides under 6 / 14 / 20 / 20 / 20 / 7 of 20; at 0.35-0.41 m/s 19 / 20 / 20 / 20 / 0 / 0.
# Full push reaches 0.38-0.45 m/s after 3-4 s. At 10 Hz the crouch starts 0-4 cm inside
# GLIDE_CROUCH.
GLIDE_RUNUP = 1.2      # [m] glide: run-up start before the bar's centre line
GLIDE_START = 1.0      # [m] closer than this: go back to GLIDE_RUNUP first
GLIDE_CROUCH = 0.30    # [m] send `crouch` once the trunk is this close to the line
GLIDE_PAST = 0.3       # [m] done this far beyond the line
GLIDE_FACE = 0.15      # [rad] heading error at which the run-up starts
GLIDE_LINE = 1.5       # [rad/m] heading correction per metre off the line
DRAG_PAST = 0.25       # [m] drag: aim this far beyond the rope length past the part's centre
DRAG_MARGIN = 0.05     # [m] drag: done with the hitch this far inside the part's edge
DRAG_STOP = 0.1        # [m] drag: at the aim point with the hitch still outside: stand


def plan_free(env, free: np.ndarray, inflate: float) -> np.ndarray:
    """`free` less the cells within `inflate` of a box receptacle's walls (a basket): walking
    along one, the body sticks to it and cannot turn."""
    segs = [s for p in env.parts.values() if hasattr(p, "wall_segments") for s in p.wall_segments()]
    return free & ~env.grid.blocked(segs, inflate) if segs else free


class ScriptedSolver:
    def __init__(self, env):
        from .carry import Carrier
        from .herd import Herder
        from .push import Pusher

        self.env = env
        self.demo: list[tuple] = list(getattr(env.room, "demo", []) or [])
        self._fields: dict[tuple, np.ndarray] = {}
        self._live: dict[tuple, tuple[np.ndarray, np.ndarray]] = {}
        self.pusher = Pusher(env)          # `push` sub-goals and push targets (R38)
        self.herder = Herder(env)          # `herd` sub-goals (R29)
        self.carrier = Carrier(env)        # `grab` / `place` / `drop` sub-goals, wall buttons (R18)
        self.reset()

    def reset(self) -> None:
        self._fields = {}
        self._live = {}
        self.pusher.reset()
        self.herder.reset()
        self.carrier.reset()
        self.carrier.head = self.carrier.grab = -1.0
        self.carrier.wall = None
        self.step_i = 0                    # current `demo:` entry
        self._wait = 0                     # env steps left in a `wait`
        self._worked = None                # grid-answer device / look target the carrier works
        self.taken: dict[str, object] = {}     # dispenser -> copy its last `take` handed out
        self._take_from: int | None = None     # take: the dispenser's count when it began
        self._step: tuple | None = None        # grid-answer delivery: ("take" | "grab" | "place", part)
        self._addon_from: str | None = None    # addon: the body's mode when the sub-goal began
        self._glide: dict | None = None        # glide: travel direction and phase
        self._crouch = False                   # glide: send `crouch` this step
        self._tow_off = False                  # tow / untow: stepping off a used station pad
        self._drag: dict | None = None         # drag: the aim point
        self._aim: tuple | None = None         # break via shell: (part, plan) worked out once
        self._peck_on = False                  # break via peck: the beak closed last step
        self._back = 0                         # break / repair: env steps spent walking off

    @property
    def in_demo(self) -> bool:
        return self.step_i < len(self.demo)

    # ---- grid answer -----------------------------------------------------------------
    def field(self, stage: int) -> np.ndarray:
        env = self.env
        key = (stage, env.targets[stage].target_key())
        if key not in self._fields:
            inflate = env.body.footprint_radius + CLEARANCE
            free = plan_free(env, env.stage_free(stage, inflate=inflate), inflate)
            self._fields[key] = env.grid.distance(free, env.targets[stage].target_mask(env.grid))
        return self._fields[key]

    def act(self) -> np.ndarray:
        c = self.carrier
        self._crouch = False
        move = self._move()
        return self.env.scripted_action(move, c.head, c.grab, 1.0 if self._crouch else -1.0)

    def _move(self) -> np.ndarray:
        act = self.carrier.off_wall()
        if act is not None:
            return act
        while self.in_demo:
            act = self._demo_act(self.demo[self.step_i])
            if act is not None:
                return act
            self.step_i += 1
            self.pusher.reset()
            self.herder.reset()
            self.carrier.reset()
        env = self.env
        if env.current_target() is None:
            return STAND.copy()
        target = env.current_target()
        if self._step is not None and self._step[0] == "place" and self._worked is not target:
            act = self._deliver(self._worked)          # the receptacle turned on: let go, walk off
            if act is not None:
                return act
        if self._worked is not None and self._worked is not target:   # done with it (a cord turns
            self._worked = None                                        # on mid-walk): let go
            self.carrier.reset()
            self.carrier.grab = -1.0
        if hasattr(target, "push_target"):
            return self.pusher.act(target)
        if getattr(target, "mount", None) == "wall":
            return self.carrier.press_wall_act(target) if not target.satisfied() else STAND.copy()
        act = self._deliver(target)
        if act is not None:
            return act
        act = self._work(target)
        if act is not None:
            return act
        self.carrier.head = -1.0
        return self._steer(self.field(env.stage), target.target_xy())

    def _work(self, target) -> np.ndarray | None:
        """Grid answer: work a lever, dial, cord or drawer, or look for a `_look_` switch (None: not one)."""
        from .gaze import Look

        kind = getattr(target, "object_kind", None)
        if not isinstance(target, Look) and kind not in ("lever", "dial", "pull_cord", "drawer"):
            return None
        c = self.carrier
        if self._worked is not target:
            self._worked = target
            c.reset()
        if isinstance(target, Look):
            act = c.look_act(target.watched)
        elif kind == "lever":
            act = c.turn_act(target, 1.0)
        elif kind == "dial":
            act = c.dial_act(target, target.lo)
        else:
            act = c.pull_act(target)
        return STAND.copy() if act is None else act

    def _deliver(self, target) -> np.ndarray | None:
        """Grid answer: carry an object receptacle `target` accepts to it, taking one from a
        dispenser when no free one is left (None: not a receptacle, or nothing to carry)."""
        from .parts.object.grabbable import Grabbable

        if not hasattr(target, "accepts"):
            return None
        env, c = self.env, self.carrier
        if self._worked is not target:
            self._worked, self._step = target, None
            c.reset()
        if self._step is None:
            held = c.held()
            if held is not None and target.accepts(held):
                self._step = ("place", held)
            else:
                here = env.body_state.pos[:2]
                free = [o for o in env.parts.values() if isinstance(o, Grabbable) and not o.parked
                        and target.accepts(o) and o.name not in target.inside and not self._in_use(o)]
                free.sort(key=lambda o: float(np.linalg.norm(o.pos(env.data)[:2] - here)))
                ds = [d for d in env.parts.values() if hasattr(d, "copies") and d.stock_left() > 0
                      and isinstance(d.copies[0], Grabbable) and target.accepts(d.copies[0])]
                if free:
                    self._step = ("grab", free[0])
                elif ds:
                    self._step = ("take", ds[0])
                else:
                    return None
            self._take_from = None
            c.reset()
        kind, part = self._step
        act = (c.place_act(part, target) if kind == "place" else c.grab_act(part) if kind == "grab"
               else self.take_act(part))
        if act is None:
            self._step = None
            return STAND.copy()
        return act

    def _in_use(self, obj) -> bool:
        """`obj` lies in or on another input that is on (a filled receptacle, a plate it
        holds down): taking it away would turn that input off."""
        from .parts.input._sense import in_rect

        xy = obj.pos(self.env.data)[:2]
        for p in self.env.parts.values():
            if p.stage != "input" or not p.satisfied():
                continue
            if obj.name in getattr(p, "inside", ()):
                return True
            half = getattr(p, "half", None)
            if half is not None and getattr(p, "mount", None) != "wall" and in_rect(xy, p.xy, half):
                return True
        return False

    def take_act(self, disp) -> np.ndarray | None:
        """Trigger dispenser `disp` until it hands out a copy, then stand until the copy
        rests; None once it does (the copy is `taken[disp.name]`)."""
        env = self.env
        if self._take_from is None:
            self._take_from, self._take_wait = disp.dispensed, 0
        copy = disp.last_copy()
        if disp.dispensed > self._take_from and copy is not None:
            self.carrier.head = -1.0
            self._take_wait += 1
            if (self._take_wait > 2 and float(np.linalg.norm(copy.vel(env.data))) < REST_SPEED) \
                    or self._take_wait > TAKE_SETTLE * env.hz:
                self.taken[disp.name] = copy
                self._take_from = None
                return None
            return STAND.copy()
        self._take_from = disp.dispensed
        b = disp.press_plate
        if b is None:
            return STAND.copy()               # the `in` channels hand it out
        if b.on:                              # pressed, nothing came (cooldown, or pressed before):
            self.carrier.head = -1.0          # let go of it (a wall plate: head up, the tip clears
            if b.mount == "wall":             # its top), press again when ready
                return STAND.copy()
            act = self.carrier._walk_off(b.xy, float(max(b.half)) + env.body.footprint_radius + CLEARANCE)
            return STAND.copy() if act is None else act
        if not disp.ready(float(env.data.time)):
            if b.mount == "wall":             # wait off the plate: coming in again square on
                act = self.carrier._walk_off(b.xy, WALL_WAIT)     # (turning in place to it is slow)
                return STAND.copy() if act is None else act
            return STAND.copy()
        if b.mount == "wall":
            act = self.carrier.press_wall_act(b)
            return STAND.copy() if act is None else act
        return self._walk_to(b)

    def addon_act(self, station) -> np.ndarray | None:
        """Walk onto addon station `station` and stand until the body's addon mode
        changes (R30); None once it has."""
        from .parts.input._sense import in_rect

        env = self.env
        if self._addon_from is None:
            self._addon_from = env.body.addon
        if env.body.addon != self._addon_from:
            self._addon_from = None
            return None
        s = env.body_state
        if in_rect(s.pos[:2], station.xy, np.maximum(station.half - ADDON_MARGIN, 0.02)):
            return STAND.copy()
        return self._walk_to(station)

    def tow_act(self, rope, hitch: str | None) -> np.ndarray | None:
        """`tow` (hitch given) / `untow` (None) with rope `rope` (R31); None once done."""
        from .parts.input._sense import in_rect

        env = self.env
        h = rope._index.get(hitch) if hitch is not None else None
        if hitch is None:
            if not rope.on_body:
                return None
            modes = ("detach", "toggle", "handoff")
        else:
            if rope.on_body and h in rope.tied:
                return None
            modes = ("attach", "toggle")
        sts = [p for p in env.parts.values() if getattr(p, "rope", None) is rope and getattr(p, "mode", None) in modes]
        if not sts:
            return STAND.copy()
        if h is not None:
            hp = env.data.site_xpos[rope.hitches[h].sid][:2]
            sts.sort(key=lambda p: float(np.linalg.norm(p.xy - hp)))
        st = sts[0]
        inside = in_rect(env.body_state.pos[:2], st.xy, st.half)
        if inside and not st.inputs and not st.armed:      # used: entering arms it again
            self._tow_off = True
        if self._tow_off:
            act = self.carrier._walk_off(st.xy, float(max(st.half)) + env.body.footprint_radius + CLEARANCE)
            if act is not None:
                return act
            self._tow_off = False
        if in_rect(env.body_state.pos[:2], st.xy, np.maximum(st.half - ADDON_MARGIN, 0.02)):
            return STAND.copy()
        return self._walk_to(st)

    def towed_hitch(self, rope):
        """The hitch `rope` tows now: its end away from the body (the other end when one
        end is on the body; between two towed things, the end not tied to something
        the body tows through other ropes), or None."""
        env = self.env
        m = env.model
        ends = [rope.tied_hitch(e) for e in range(2)]
        if rope.on_body:
            return ends[1 - rope.tied.index(rope._body_h)]
        from .parts.device.rope import far_end

        others = []
        for r in env.world.ropes:                 # the other ropes tied at both ends (a chain)
            hs = [r.tied_hitch(e) for e in range(2)]
            if r is not rope and None not in hs:
                others.append((hs[0].rid, hs[1].rid))
        e = far_end([h.rid if h is not None else None for h in ends], others,
                    int(m.body_rootid[env.body.root_body_id(m)]))
        return None if e is None else ends[e]

    def drag_act(self, rope, target) -> np.ndarray | None:
        """Walk on so that `rope`'s towed hitch follows into `target`'s floor area (R31);
        None once it is DRAG_MARGIN inside."""
        from .parts.input._sense import in_rect

        env = self.env
        h = self.towed_hitch(rope)
        if h is None:
            if self._drag is not None:     # let go on the way (a zone's detach channel): done
                self._drag = None
                return None
            return STAND.copy()            # nothing towed through it (yet)
        hp = env.data.site_xpos[h.sid][:2].copy()
        if in_rect(hp, target.xy, np.maximum(target.half - DRAG_MARGIN, 0.02)):
            self._drag = None
            return None
        if self._drag is None:
            d = target.xy - hp
            n = float(np.linalg.norm(d))
            if n < 1e-6:
                yaw = float(env.body_state.yaw)
                d, n = np.array([math.cos(yaw), math.sin(yaw)]), 1.0
            gap = max(float(rope.cur), float(np.linalg.norm(env.body_state.pos[:2] - hp)))   # a chain: longer
            self._drag = {"aim": self._free_along(target.xy, d / n, gap + DRAG_PAST)}
        aim = self._drag["aim"]
        if float(np.linalg.norm(aim - env.body_state.pos[:2])) < DRAG_STOP:
            return STAND.copy()            # there, and the hitch did not follow in
        return self._steer(self._point_field(aim), aim)

    def _free_along(self, xy, d, dist: float) -> np.ndarray:
        """The point farthest along `d` from `xy`, up to `dist`, on a free cell now (walls
        inflated by the footprint)."""
        self._point_field(xy)
        free = self._live[("point", round(float(xy[0]), 3), round(float(xy[1]), 3))][0]
        env = self.env
        for t in np.arange(dist, 0.0, -env.grid.res):
            p = np.asarray(xy, float) + d * t
            if free[env.grid.cell(p)]:
                return p
        return np.asarray(xy, float)

    def glide_act(self, bar) -> np.ndarray | None:
        """Run up to low bar `bar` on wheels and crouch under it (R30); None once the
        trunk is GLIDE_PAST beyond the bar's centre line."""
        env = self.env
        s = env.body_state
        pos = s.pos[:2]
        if self._glide is None:
            n = bar.across()
            d = -n if float((pos - bar.xy) @ n) > 0 else n            # towards the bar and on
            ahead = float((bar.xy - pos) @ d)
            self._glide = {"d": d, "phase": "face" if ahead >= GLIDE_START else "back"}
        g = self._glide
        d = g["d"]
        ahead = float((bar.xy - pos) @ d)            # trunk to the bar's centre line
        if ahead <= -GLIDE_PAST:
            self._glide = None
            return None
        heading = math.atan2(d[1], d[0])
        if g["phase"] == "back":
            start = bar.xy - d * GLIDE_RUNUP
            if ahead >= GLIDE_RUNUP - 0.1:
                g["phase"] = "face"
            else:
                return self._steer(self._point_field(start), start)
        if g["phase"] == "face":
            err = (heading - s.yaw + math.pi) % (2 * math.pi) - math.pi
            if abs(err) > GLIDE_FACE:
                return np.array([-1.0, 0.0, math.copysign(1.0, err)], dtype=np.float32)
            g["phase"] = "run"
        side = float((pos - bar.xy) @ np.array([-d[1], d[0]]))       # left of the line: > 0
        want = heading - float(np.clip(GLIDE_LINE * side, -0.5, 0.5))
        err = (want - s.yaw + math.pi) % (2 * math.pi) - math.pi
        if ahead <= GLIDE_CROUCH:
            self._crouch = True
        return np.array([1.0, 0.0, float(np.clip(TURN_GAIN * err, -1.0, 1.0))], dtype=np.float32)

    def _point_field(self, xy) -> np.ndarray:
        """Distance field (free cells now) to a 0.1 m disk around `xy`."""
        from .env import GATE_PASSABLE
        from .logic import live_free

        env = self.env
        key = ("point", round(float(xy[0]), 3), round(float(xy[1]), 3))
        if key not in self._live:
            inflate = env.body.footprint_radius + CLEARANCE
            free = plan_free(env, live_free(env.grid, env.walls, env.parts, GATE_PASSABLE, inflate=inflate),
                             inflate)
            self._live[key] = (free, env.grid.distance(free, env.grid.disk(np.asarray(xy, float), 0.1)))
        return self._live[key][1]

    def _steer(self, field: np.ndarray, target_xy) -> np.ndarray:
        env = self.env
        s = env.body_state
        aim = env.grid.descend(field, s.pos[:2], LOOKAHEAD)
        if np.hypot(*(target_xy - s.pos[:2])) < LOOKAHEAD and np.isfinite(field).any():
            # Close: aim at the target itself unless it is behind a wall.
            iy, ix = env.grid.cell(target_xy)
            if np.isfinite(field[iy, ix]):
                aim = target_xy
        d = aim - s.pos[:2]
        err = math.atan2(d[1], d[0]) - s.yaw
        err = (err + math.pi) % (2 * math.pi) - math.pi
        turn = float(np.clip(TURN_GAIN * err, -1.0, 1.0))
        forward = 1.0 if abs(err) < FACING else 0.0
        if forward == 0.0:
            ahead = s.pos[:2] + PEEK * np.array([math.cos(s.yaw), math.sin(s.yaw)])
            if not np.isfinite(field[env.grid.cell(ahead)]):
                forward = -1.0        # wall ahead: turn in place instead of creeping into it
        return np.array([forward, 0.0, turn], dtype=np.float32)

    # ---- demo sub-goals (R20) --------------------------------------------------------
    def _live_field(self, part) -> tuple[np.ndarray, np.ndarray]:
        """(free cells now, distance field to `part`'s target area), cached by the
        gates open and the holes there now."""
        from .env import GATE_PASSABLE
        from .logic import gate_passable, live_free

        env = self.env
        parts = env.parts.values()
        key = (part.name, part.target_key(),
               tuple(p.name for p in parts if gate_passable(p, GATE_PASSABLE)),
               tuple(p.name for p in parts if p.hole_now(env.grid) is not None))
        if key not in self._live:
            inflate = env.body.footprint_radius + CLEARANCE
            free = plan_free(env, live_free(env.grid, env.walls, env.parts, GATE_PASSABLE, inflate=inflate),
                             inflate)
            self._live[key] = (free, env.grid.distance(free, part.target_mask(env.grid)))
        return self._live[key]

    def _walk_to(self, part) -> np.ndarray:
        env = self.env
        free, field = self._live_field(part)
        start = env.grid.nearest_free(free, env.body_state.pos[:2])
        if start is None or not np.isfinite(field[start]):
            return STAND.copy()        # behind a closed door or a hole: wait for it to change
        return self._steer(field, part.xy)

    def _walk_point(self, xy) -> np.ndarray:
        """Walk to a point (not a part) on the live grid: the stand point in front of a
        wire's cover."""
        from .env import GATE_PASSABLE
        from .logic import live_free

        env = self.env
        xy = np.asarray(xy, dtype=float)
        key = ("point", round(float(xy[0]), 3), round(float(xy[1]), 3))
        if key not in self._fields:
            inflate = env.body.footprint_radius + CLEARANCE
            free = plan_free(env, live_free(env.grid, env.walls, env.parts, GATE_PASSABLE,
                                            inflate=inflate), inflate)
            cell = env.grid.nearest_free(free, xy)
            mask = np.zeros(free.shape, dtype=bool)
            if cell is not None:
                mask[cell] = True
            self._fields[key] = env.grid.distance(free, mask)
        return self._steer(self._fields[key], xy)

    def _object(self, name: str):
        """Part `name`, or for a dispenser the copy its last `take` handed out."""
        part = self.env.parts[name]
        return self.taken[name] if hasattr(part, "copies") else part

    # ---- break / repair sub-goals (R34) ----------------------------------------------
    def break_act(self, part, cause: str) -> np.ndarray | None:
        """Break `part` through `cause`, written out with the moves already there:
        `peck` stands at it and works the beak, `crush` drops an object on it, `shell`
        aims the cannon's lever and presses its button, `destroyer` presses the switch
        that drives the press. None only once the part is broken; while a cause has run
        its course and the part is still whole it starts again (the episode's step
        budget ends it)."""
        if part.broken:
            act = self._clear_of(part)
            if act is None:
                self._aim = None
            return act
        act = {"peck": self.peck_act, "crush": self.crush_act,
               "destroyer": self.destroyer_act}.get(cause, self.shell_act)(part)
        if act is None:
            self.carrier.reset()
            self.carrier.grab = self.carrier.head = -1.0
            return STAND.copy()
        return act

    def peck_act(self, part) -> np.ndarray | None:
        """Head down at the part, then the grab on and off once per env step: each
        falling edge within `peck_dist` costs the part one hit."""
        env = self.env
        c = self.carrier
        c.head = 1.0
        if part.beak_distance(env.model, env.data) > float(part.cfg["peck_dist"]):
            c.grab = -1.0
            self._peck_on = False
            at = part.break_xy(env.data)
            here = env.body_state.pos[:2]
            n = part.peck_normal()
            if n is not None and (float((here - at) @ n) <= 0.0
                                  or np.hypot(*(at - here)) > PECK_CLOSE):
                return self._walk_point(at + n * PECK_STAND)   # round to the cover's front
            if np.hypot(*(at - here)) < PECK_CLOSE:
                return c.steer(at, PECK_SPEED)           # lean on it: the body needs the stop
            return self._walk_to(part)
        self._peck_on = not self._peck_on
        c.grab = 1.0 if self._peck_on else -1.0
        return STAND.copy()

    def crush_act(self, part) -> np.ndarray | None:
        """Drop a carried object on the part: from beak height a 30 g cube peaks at
        0.87 N, while setting it down gently is only its 0.30 N weight (16f step 0)."""
        obj = self._loose_object()
        if obj is None:
            return None
        if self.carrier.held() is not obj:
            return self.carrier.grab_act(obj)
        return self.carrier.place_act(obj, part, drop=True)

    def destroyer_act(self, part) -> np.ndarray | None:
        """Press the switch that drives the press whose reach holds `part`."""
        press = next((p for p in self.env.parts.values()
                      if hasattr(p, "target_names") and hasattr(p, "reach")
                      and (part.name in p.target_names or part in p.reach())), None)
        return self._press_act(self._switch_for(press))

    def shell_act(self, part) -> np.ndarray | None:
        """Turn the cannon's aim levers to a cell whose shot hits the part, then press
        its fire button (R16)."""
        plan = self._aim_plan(part)
        if plan is None:
            return None
        _, cells, button = plan
        for lever, goal in cells:
            if (not np.isclose(lever.value, goal)
                    and np.hypot(*(np.asarray(lever.xy) - self.env.body_state.pos[:2])) > LEVER_NEAR):
                return self._walk_to(lever)      # the lever may be behind a door: plan the way
            act = self.carrier.turn_act(lever, goal)
            if act is not None:
                return act
            self.carrier.reset()
        return self._press_act(button)

    def repair_act(self, part) -> np.ndarray | None:
        """Put the `repair_with` fuse in the part's socket, or press the switch that
        turns its `repair` channel on (R34)."""
        if not getattr(part, "broken", False):
            return self._clear_of(part)
        if part.repair_with is not None:
            fuse = self._fuse(part)
            if fuse is None:
                return None
            if self.carrier.held() is not fuse:
                return self.carrier.grab_act(fuse)
            act = self.carrier.place_act(fuse, part)
        else:
            act = self._press_act(self._writer(part.repair_channel))
        if act is None:
            self.carrier.reset()
            self.carrier.grab = self.carrier.head = -1.0
            return STAND.copy()
        return act

    # ---- helpers for the break sub-goals ---------------------------------------------
    def _clear_of(self, part) -> np.ndarray | None:
        """Walk off the part once it is broken or whole again: a body left standing
        against a breakable box sits in a cell the planner calls blocked, and the next
        sub-goal would wait there for ever."""
        self._back += 1
        act = self.carrier._walk_off(np.asarray(part.xy, dtype=float), BACK_OFF)
        if act is not None and self._back <= BACK_SECONDS * self.env.hz:
            return act
        self._back = 0
        self.carrier.reset()
        self.carrier.grab = self.carrier.head = -1.0
        return None

    def _writer(self, channel: str | None):
        """The part writing `channel`, or None."""
        bus = self.env.signals
        if not channel or bus is None:
            return None
        return self.env.parts.get(bus.writers.get(channel))

    def _switch_for(self, device):
        """A part the duck can press that writes one of `device`'s input channels."""
        if device is None:
            return None
        for ch in device.inputs:
            src = self._writer(ch)
            if src is not None and src.role in ("switch", "checkpoint", "goal"):
                return src
        return None

    def _press_act(self, part) -> np.ndarray | None:
        if part is None or part.satisfied():
            return None
        if getattr(part, "mount", None) == "wall":
            return self.carrier.press_wall_act(part)
        return self._walk_to(part)

    def _loose_object(self):
        """The grabbable object nearest the body that is free to pick up."""
        env = self.env
        here = env.body_state.pos[:2]
        objs = [o for rid, o in env.world.objects.items()
                if env.world.kinds.get(rid) == "object" and not o.parked
                and hasattr(o, "handle_points")]
        held = self.carrier.held()
        objs = [o for o in objs if o is held or int(o._bid) not in env.world.held]
        return min(objs, key=lambda o: float(np.hypot(*(o.pos(env.data)[:2] - here))), default=None)

    def _fuse(self, part):
        """A free object whose marker id is the part's `repair_with` (a dispenser copy
        counts once it has been taken)."""
        env = self.env
        for rid, o in env.world.objects.items():
            mark = getattr(o, "marker", None)
            if env.world.kinds.get(rid) != "object" or o.parked or mark is None:
                continue
            if int(mark.id) != part.repair_with:
                continue
            if o is not self.carrier.held() and rid in env.world.held:
                continue
            return o
        return None

    def _aim_plan(self, part):
        """(cannon, [(lever, cell value), ...], fire button) that hits `part`, or None."""
        if self._aim is not None and self._aim[0] is part:
            return self._aim[1]
        env = self.env
        plan = None
        for cannon in [p for p in env.parts.values() if hasattr(p, "can_hit")]:
            board = part.break_board(env.model, env.data, cannon.xy)
            cells = self._aim_cells(cannon, board) if board is not None else None
            button = self._switch_for(cannon)
            if cells is not None and button is not None:
                plan = (cannon, cells, button)
                break
        self._aim = (part, plan)
        return plan

    def _aim_cells(self, cannon, board):
        """Lever cells that make `cannon` hit `board`, or None (a continuous aim channel
        or no cell that hits)."""
        from .parts.trap.cannon import direction

        opts: dict[str, tuple] = {}
        for key in ("aim_yaw", "aim_pitch"):
            if key not in cannon.binds:
                opts[key] = (None, [(None, float(cannon.live[key]))])
                continue
            ch, (i0, i1, o0, o1) = cannon.binds[key]
            src = self._writer(ch)
            cells = getattr(src, "channel_cells", None)
            cells = cells(ch) if callable(cells) else None
            if src is None or cells is None or i1 == i0:
                return None
            opts[key] = (src, [(float(v), o0 + (float(v) - i0) * (o1 - o0) / (i1 - i0))
                               for v in cells])
        for vy, ay in opts["aim_yaw"][1]:
            for vp, ap in opts["aim_pitch"][1]:
                d = direction(np.array(math.radians(cannon.yaw + ay)), np.array(math.radians(ap)))
                if bool(np.all(cannon.hits_board(d, board))):
                    return [(opts[k][0], v) for k, v in (("aim_yaw", vy), ("aim_pitch", vp))
                            if opts[k][0] is not None and v is not None]
        return None

    def _demo_act(self, entry: tuple) -> np.ndarray | None:
        """Action for the `demo:` entry, or None when it is done."""
        env = self.env
        cmd = entry[0]
        if cmd == "wait":
            if self._wait == 0:
                self._wait = max(1, int(round(entry[1] * env.hz)))
            self._wait -= 1
            return STAND.copy() if self._wait > 0 else None
        if cmd == "push":
            ball, target = env.parts[entry[1]], env.parts[entry[2]]
            done = (ball.name in target.inside) if hasattr(target, "inside") else target.satisfied()
            return None if done else self.pusher.act(target, ball=ball)
        if cmd == "herd":
            return self.herder.act(env.parts[entry[1]], entry[2])
        if cmd == "take":
            return self.take_act(env.parts[entry[1]])
        if cmd == "grab":
            return self.carrier.grab_act(self._object(entry[1]))
        if cmd in ("place", "drop"):
            return self.carrier.place_act(self._object(entry[1]), env.parts[entry[2]], drop=cmd == "drop")
        if cmd == "turn":
            return self.carrier.turn_act(env.parts[entry[1]], entry[2])
        if cmd == "pull":
            return self.carrier.pull_act(env.parts[entry[1]])
        if cmd == "look":
            return self.carrier.look_act(env.parts[entry[1]])
        if cmd == "addon":
            return self.addon_act(env.parts[entry[1]])
        if cmd == "glide":
            return self.glide_act(env.parts[entry[1]])
        if cmd in ("tow", "untow"):
            return self.tow_act(env.parts[entry[1]], entry[2] if cmd == "tow" else None)
        if cmd == "drag":
            return self.drag_act(env.parts[entry[1]], env.parts[entry[2]])
        if cmd == "break":
            return self.break_act(env.parts[entry[1]], entry[2])
        if cmd == "repair":
            return self.repair_act(env.parts[entry[1]])
        part = env.parts[entry[1]]
        if part.satisfied():
            return None
        if getattr(part, "mount", None) == "wall":
            return self.carrier.press_wall_act(part)
        if cmd == "goto" and part.role != "goal":
            iy, ix = env.grid.cell(env.body_state.pos[:2])
            if part.target_mask(env.grid)[iy, ix]:
                return None
        return self._walk_to(part)


def run_episode(env, seed: int, solver: ScriptedSolver | None = None) -> dict:
    """One scripted episode. `snaps` maps each checkpoint flag reached to the
    snapshot the env saved there (R6, written by `mpg check`)."""
    solver = solver or ScriptedSolver(env)
    env.reset(seed=seed)
    solver.reset()
    info: dict = {}
    steps = 0
    snaps: dict = {}
    while True:
        _, _, terminated, truncated, info = env.step(solver.act())
        steps += 1
        snap = env.checkpoint_snap
        if snap is not None and snap.meta.get("checkpoint") not in snaps:
            snaps[snap.meta["checkpoint"]] = snap
        if terminated or truncated:
            break
    return {"seed": seed, "success": bool(info.get("success")), "fallen": bool(info.get("fallen")),
            "nan": bool(info.get("nan")), "steps": steps, "seconds": steps / env.hz,
            "stage": info.get("stage"), "target": info.get("target"),
            "demo_step": solver.step_i if solver.demo else None, "snaps": snaps}
