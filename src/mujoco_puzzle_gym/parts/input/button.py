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

`mount: wall` (R4): a plate `size` [width, height] on a wall, its centre
`height` above the floor at `pos` (a point on the wall face), facing `facing`
degrees (0 = +x, the side the body stands on). It is pressed while the beak
tip (`BodyAdapter.grab_site`) is over the plate and at most PRESS_DEPTH in
front of it (or up to PRESS_DEPTH behind its face: the tip may sink into it);
`trigger` does not apply. The plate does not collide. Planning: the body
stands STAND in front of it; the scripted solver walks up to it facing the
wall and lowers the head to the plate height (solver.py). Measured (OQ2,
default body): walking into a wall, the tip comes within 0.1-1.3 cm of it
with the head level (tip 0.21 m high) and 3.7-5.4 cm with it lowest (0.135 m)
(the chest meets the wall first), so PRESS_DEPTH is 6 cm.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from ... import markers
from ...schema import Field
from ..base import TRIGGERS, Part
from ._sense import candidates, in_rect

OFF_RGBA = (0.9, 0.35, 0.2, 1.0)
ON_RGBA = (0.2, 0.8, 0.3, 1.0)
KINDS = ("momentary", "oneshot", "toggle")
THICK = 0.006        # [m] wall plate: how far it stands off the wall
PRESS_DEPTH = 0.06   # [m] wall plate: the beak tip this far in front of (or behind) its face presses it
STAND = 0.12         # [m] wall plate: body centre this far in front of it (planning)


class Button(Part):
    role = "switch"
    marked = True
    marker_anchor = "/plate"
    marker_faces = markers.FACES_FLAT
    stage = "input"
    emits = True
    state_keys = ("on", "prev")
    fields = (
        Field("kind", "enum", "momentary", choices=KINDS),
        Field("mount", "enum", "floor", choices=("floor", "wall"),
              doc="floor: stepped on; wall: pressed with the beak tip"),
        Field("trigger", "enum", "any", choices=tuple(TRIGGERS),
              doc="what presses it: body, object, npc or any (not shells or rope knots)"),
        Field("size", "vec", [0.25, 0.25], length=2, unit="m", min=0.02, max=2.0,
              doc="plate size [x, y]; wall: [width, height]"),
        Field("height", "float", 0.2, unit="m", min=0.02, max=1.0,
              doc="wall: plate centre above the floor"),
        Field("facing", "float", 0.0, unit="deg", min=-180.0, max=180.0,
              doc="wall: the direction the plate faces (0 = +x)"),
        Field("max_height", "float", 0.2, unit="m", min=0.01, max=1.0,
              doc="objects press it when their centre is lower than this"),
    )

    def marker_face_size(self, face):
        return float(min(self.cfg["size"]))

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.kind_ = self.cfg["kind"]
        self.trigger = self.cfg["trigger"]
        self.half = np.asarray(self.cfg["size"], dtype=float) / 2
        self.max_height = float(self.cfg["max_height"])
        self.mount = self.cfg["mount"]
        a = math.radians(self.cfg["facing"])
        self.normal = np.array([math.cos(a), math.sin(a)])
        if self.mount == "wall":
            self.marker_faces = ("front",)
            self.needs_body("walk", "beak_grab")
        elif "body" not in TRIGGERS[self.trigger]:
            self.role = "sensor"            # the planner cannot press it by walking
        if self.mount == "floor":
            self.needs_body("walk" if "body" in TRIGGERS[self.trigger] else "beak_grab")
        self.stateful = self.kind_ != "momentary"
        self.on = False
        self.prev = False

    def build(self, spec):
        body = super().build(spec)
        if self.mount == "wall":                  # local +x = the way the plate faces
            a = math.radians(self.cfg["facing"])
            body.quat = [math.cos(a / 2), 0.0, 0.0, math.sin(a / 2)]
            body.add_geom(name=self.body_name + "/plate", type=mujoco.mjtGeom.mjGEOM_BOX,
                          size=[THICK / 2, *self.half], pos=[THICK / 2, 0, self.cfg["height"]],
                          rgba=OFF_RGBA, contype=0, conaffinity=0)
            return body
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
        if self.mount == "wall":
            return self.beak_on(data)
        for key, pos in candidates(self, data, body, self.trigger):
            if in_rect(pos, self.xy, self.half) and (key == "body" or pos[2] < self.max_height):
                return True
        return False

    def beak_on(self, data) -> bool:
        """Wall plate: whether the beak tip presses it now."""
        w = self.world
        if w is None or w.beak_site < 0:
            return False
        tip = data.site_xpos[w.beak_site]
        d = tip[:2] - self.xy
        out = float(d @ self.normal)
        side = float(d @ np.array([-self.normal[1], self.normal[0]]))
        top = float(data.xpos[self._bid][2]) + float(self.cfg["height"])
        return (-PRESS_DEPTH <= out - THICK <= PRESS_DEPTH and abs(side) <= self.half[0]
                and abs(tip[2] - top) <= self.half[1])

    def stand_xy(self) -> np.ndarray:
        """Wall plate: where the body centre stands to press it (planning)."""
        return self.xy + self.normal * STAND

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
        if self.mount == "wall":
            return grid.disk(self.stand_xy(), 0.06)
        return grid.rect(self.xy, tuple(self.half))
