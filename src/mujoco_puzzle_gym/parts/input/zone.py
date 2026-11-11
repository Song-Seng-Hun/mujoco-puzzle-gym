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
With `enable` off the output holds (targets are still tracked).
"""

from __future__ import annotations

import mujoco
import numpy as np

from ...schema import Field
from ..base import TRIGGERS, Part
from ._sense import candidates, in_rect

RGBA_OFF = (0.3, 0.6, 0.95, 0.15)
RGBA_ON = (0.3, 0.95, 0.4, 0.3)


class Zone(Part):
    role = "switch"
    stage = "input"
    emits = True
    state_keys = ("on", "accepted", "was_inside", "prev_pos")
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
        if self.names or "body" not in TRIGGERS[self.trigger]:
            self.role = "sensor"
        self.needs_body("walk" if self.role == "switch" else "beak_grab")
        self.stateful = self.rearm == "leave" or self.direction is not None
        self._clear()

    def _clear(self):
        self.on = False
        self.accepted: dict = {}
        self.was_inside: dict = {}
        self.prev_pos: dict = {}

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

    def update(self, model, data, body):
        for key, pos in candidates(self, data, body, self.trigger, self.names):
            inside = self._inside(pos)
            if self.accepted.get(key):
                keep = self._inside(pos, self.leave_margin) if self.rearm == "leave" else inside
                self.accepted[key] = keep
            elif inside and not self.was_inside.get(key, False):
                prev = self.prev_pos.get(key)
                if self.direction is None:
                    self.accepted[key] = True
                elif prev is not None:
                    self.accepted[key] = float((pos[:2] - prev[:2]) @ self.direction) > 0
            self.was_inside[key] = inside
            self.prev_pos[key] = pos
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
