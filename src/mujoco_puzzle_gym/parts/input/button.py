"""Floor button (R4): a flat plate (no collision) pressed while something stands on it.

Pressed when the centre of a body that `trigger` counts is over the plate: the
walking body's trunk (as M1 switches), or the centre of an object / NPC lower
than `max_height` (resting on the plate, not carried over it). Objects the
body holds never press it.

kind:
- `momentary`: 1 while pressed;
- `oneshot`:   1 from the first press on;
- `toggle`:    each new press flips 0 <-> 1.
With `enable` off the output does not change and presses are not counted (a
press held while power returns is not a new press).

`mount: floor` only for now; wall buttons pressed with the beak tip come with
the grab milestone (E3).
"""

from __future__ import annotations

import mujoco
import numpy as np

from ...schema import Field
from ..base import TRIGGERS, Part
from ._sense import candidates, in_rect

OFF_RGBA = (0.9, 0.35, 0.2, 1.0)
ON_RGBA = (0.2, 0.8, 0.3, 1.0)
KINDS = ("momentary", "oneshot", "toggle")


class Button(Part):
    role = "switch"
    stage = "input"
    emits = True
    state_keys = ("on", "prev")
    fields = (
        Field("kind", "enum", "momentary", choices=KINDS),
        Field("mount", "enum", "floor", choices=("floor",), doc="floor (wall buttons: E3)"),
        Field("trigger", "enum", "any", choices=tuple(TRIGGERS),
              doc="what presses it: body, object, npc or any (not shells or rope knots)"),
        Field("size", "vec", [0.25, 0.25], length=2, unit="m", min=0.05, max=2.0,
              doc="plate size [x, y]"),
        Field("max_height", "float", 0.2, unit="m", min=0.01, max=1.0,
              doc="objects press it when their centre is lower than this"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.kind_ = self.cfg["kind"]
        self.trigger = self.cfg["trigger"]
        self.half = np.asarray(self.cfg["size"], dtype=float) / 2
        self.max_height = float(self.cfg["max_height"])
        if "body" not in TRIGGERS[self.trigger]:
            self.role = "sensor"            # the planner cannot press it by walking
        self.needs_body("walk" if "body" in TRIGGERS[self.trigger] else "beak_grab")
        self.stateful = self.kind_ != "momentary"
        self.on = False
        self.prev = False

    def build(self, spec):
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/plate", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*self.half, 0.002], pos=[0, 0, 0.002], rgba=OFF_RGBA,
                      contype=0, conaffinity=0)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._gid = model.geom(self.body_name + "/plate").id

    def reset(self, model, data):
        self.prev = False
        self._set(model, False)

    def _set(self, model, on: bool) -> None:
        self.on = on
        if model is not None:
            model.geom_rgba[self._gid] = ON_RGBA if on else OFF_RGBA
        for ch in self.outputs:
            self.write(float(on), ch)

    def pressed(self, data, body) -> bool:
        for key, pos in candidates(self, data, body, self.trigger):
            if in_rect(pos, self.xy, self.half) and (key == "body" or pos[2] < self.max_height):
                return True
        return False

    def update(self, model, data, body):
        p = self.pressed(data, body)
        rising, self.prev = p and not self.prev, p
        if not self.enabled():
            return
        on = (p if self.kind_ == "momentary" else self.on or p if self.kind_ == "oneshot"
              else (not self.on) if rising else self.on)
        if on != self.on:
            self._set(model, on)

    def state(self):
        return float(self.on)

    def satisfied(self):
        return self.on

    def target_mask(self, grid):
        return grid.rect(self.xy, tuple(self.half))
