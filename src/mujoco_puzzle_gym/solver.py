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
  name work on that copy (R28).
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

    @property
    def in_demo(self) -> bool:
        return self.step_i < len(self.demo)

    # ---- grid answer -----------------------------------------------------------------
    def field(self, stage: int) -> np.ndarray:
        env = self.env
        key = (stage, env.targets[stage].target_key())
        if key not in self._fields:
            free = env.stage_free(stage, inflate=env.body.footprint_radius + CLEARANCE)
            self._fields[key] = env.grid.distance(free, env.targets[stage].target_mask(env.grid))
        return self._fields[key]

    def act(self) -> np.ndarray:
        c = self.carrier
        return self.env.scripted_action(self._move(), c.head, c.grab)

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
        from .logic import live_free

        env = self.env
        parts = env.parts.values()
        key = (part.name, part.target_key(),
               tuple(p.name for p in parts if p.role == "gate" and p.state() >= GATE_PASSABLE),
               tuple(p.name for p in parts if p.hole_now(env.grid) is not None))
        if key not in self._live:
            free = live_free(env.grid, env.walls, env.parts, GATE_PASSABLE,
                             inflate=env.body.footprint_radius + CLEARANCE)
            self._live[key] = (free, env.grid.distance(free, part.target_mask(env.grid)))
        return self._live[key]

    def _walk_to(self, part) -> np.ndarray:
        env = self.env
        free, field = self._live_field(part)
        start = env.grid.nearest_free(free, env.body_state.pos[:2])
        if start is None or not np.isfinite(field[start]):
            return STAND.copy()        # behind a closed door or a hole: wait for it to change
        return self._steer(field, part.xy)

    def _object(self, name: str):
        """Part `name`, or for a dispenser the copy its last `take` handed out."""
        part = self.env.parts[name]
        return self.taken[name] if hasattr(part, "copies") else part

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
