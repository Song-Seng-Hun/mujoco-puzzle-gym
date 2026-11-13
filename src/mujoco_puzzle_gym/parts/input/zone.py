"""Zone sensor (R5): 1 while a counted target is inside a box region.

Targets: what `trigger` counts (body, object, npc, any), or only the object
parts named in `objects`. A target is inside when its centre (the walking
body's trunk) is in the box `size` x `height` above the floor at `pos`.

- `direction: [dx, dy]`: a target turns the zone on only if it entered moving
  that way (its step-to-step motion has a positive component along it); a
  target that entered another way, or started inside, does not count until it
  leaves and enters again.
- `rearm: leave`: once on for a target, the zone stays on until that target is
  outside the box grown by `leave_margin` on every side. Standing in the zone
  or wobbling on its edge then makes one pass one turn-on (lap counters).
- `min_speed` [m/s] (R30 speed gate): a target counts only while it moves at
  least this fast horizontally. The speed is the step-to-step motion averaged
  over about `SPEED_TAU` s (a low-pass of the velocity vector), so the walking
  gait's sway does not count: at full speed it stays at 0.20-0.25 m/s, while
  rolling on the roller addon gives 0.32-0.53 m/s. With `rearm: leave` or a
  `direction` the speed counts at the turn-on only. None or 0: no speed rule.
With `enable` off the output holds (targets are still tracked).
"""

from __future__ import annotations

import mujoco
import numpy as np

from ...schema import Field
from ..base import TRIGGERS, Part
from ._sense import candidates, in_rect

RGBA_OFF = (0.3, 0.6, 0.95, 0.15)
SPEED_TAU = 0.5      # [s] min_speed: time constant of the averaged velocity
JUMP_SPEED = 5.0     # [m/s] min_speed: a faster step is a jump (warp, reappearing object), not motion
RGBA_ON = (0.3, 0.95, 0.4, 0.3)


class Zone(Part):
    role = "switch"
    stage = "input"
    emits = True
    state_keys = ("on", "accepted", "was_inside", "prev_pos", "vel", "seen", "prev_t")
    fields = (
        Field("size", "vec", [0.4, 0.4], length=2, unit="m", min=0.05, max=10.0,
              doc="box size [x, y]"),
        Field("height", "float", 0.5, unit="m", min=0.02, max=3.0, doc="box height above the floor"),
        Field("trigger", "enum", "body", choices=tuple(TRIGGERS)),
        Field("objects", "list", None, doc="only these object parts count (overrides trigger)"),
        Field("direction", "vec", None, length=2, doc="count only entries moving this way"),
        Field("rearm", "enum", "always", choices=("always", "leave")),
        Field("leave_margin", "float", 0.05, unit="m", min=0.0, max=1.0,
              doc="rearm leave: how far outside the box a target must go"),
        Field("min_speed", "float", None, unit="m/s", min=0.0, max=2.0,
              doc="count a target only while its horizontal speed (averaged ~0.5 s) is at least this"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        self.half = np.asarray(c["size"], dtype=float) / 2
        self.height = float(c["height"])
        self.trigger = "any" if c["objects"] else c["trigger"]
        self.names = c["objects"]
        d = c["direction"]
        self.direction = None if d is None else np.asarray(d, dtype=float)
        if self.direction is not None and not np.linalg.norm(self.direction) > 0:
            raise ValueError(f"part {name!r}: direction must not be [0, 0]")
        self.rearm = c["rearm"]
        self.leave_margin = float(c["leave_margin"])
        self.min_speed = float(c["min_speed"] or 0.0)
        if self.names or "body" not in TRIGGERS[self.trigger]:
            self.role = "sensor"
        self.needs_body("walk" if self.role == "switch" else "beak_grab")
        self.stateful = self.rearm == "leave" or self.direction is not None or self.min_speed > 0
        self._clear()

    def _clear(self):
        self.on = False
        self.accepted: dict = {}
        self.was_inside: dict = {}
        self.prev_pos: dict = {}
        self.vel: dict = {}            # min_speed: averaged horizontal velocity per target
        self.seen: dict = {}           # min_speed: time each target was last tracked
        self.prev_t: float | None = None

    def build(self, spec):
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/box", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*self.half, self.height / 2], pos=[0, 0, self.height / 2],
                      rgba=RGBA_OFF, contype=0, conaffinity=0, group=1)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._gid = model.geom(self.body_name + "/box").id

    def reset(self, model, data):
        self._clear()
        self._set(model, False)

    def _set(self, model, on: bool) -> None:
        self.on = on
        if model is not None:
            model.geom_rgba[self._gid] = RGBA_ON if on else RGBA_OFF
        for ch in self.outputs:
            self.write(float(on), ch)

    def _inside(self, pos, margin: float = 0.0) -> bool:
        return in_rect(pos, self.xy, self.half, margin) and -margin <= pos[2] <= self.height + margin

    def _fast(self, key, pos, now: float) -> bool:
        """Update the averaged velocity of `key`; whether it is at least `min_speed`."""
        prev, t0 = self.prev_pos.get(key), self.seen.get(key)
        v = self.vel.get(key, np.zeros(2))
        # only steps from the previous update count: a target that was out of the
        # candidates (held, parked) has a stale position, and a warp is a jump
        if prev is not None and t0 is not None and t0 == self.prev_t and now > t0:
            dt = now - t0
            step = (pos[:2] - prev[:2]) / dt
            if float(np.hypot(*step)) <= JUMP_SPEED:
                v = v + min(1.0, dt / SPEED_TAU) * (step - v)
        self.vel[key] = v
        self.seen[key] = now
        return float(np.hypot(*v)) >= self.min_speed

    def update(self, model, data, body):
        now = float(data.time)
        for key, pos in candidates(self, data, body, self.trigger, self.names):
            inside = self._inside(pos)
            fast = self._fast(key, pos, now) if self.min_speed > 0 else True
            if self.accepted.get(key):
                if self.rearm == "leave":
                    self.accepted[key] = self._inside(pos, self.leave_margin)
                else:
                    self.accepted[key] = inside and (fast or self.direction is not None)
            elif inside and self.direction is None:
                # without a direction an accepted target stays accepted while inside,
                # so "inside, not accepted" is an entry (or, with min_speed, a speed-up)
                self.accepted[key] = fast
            elif inside and not self.was_inside.get(key, False):
                prev = self.prev_pos.get(key)
                if prev is not None:
                    self.accepted[key] = fast and float((pos[:2] - prev[:2]) @ self.direction) > 0
            self.was_inside[key] = inside
            self.prev_pos[key] = pos
        self.prev_t = now
        if not self.enabled():
            return
        on = any(self.accepted.values())
        if on != self.on:
            self._set(model, on)

    def state(self):
        return float(self.on)

    def satisfied(self):
        return self.on

    def target_mask(self, grid):
        return grid.rect(self.xy, tuple(self.half))
