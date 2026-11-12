"""Spring pad (R15): launches what stands on it with a set velocity.

When a body or object (`trigger`, default `any`) is on the pad, the spring is
enabled and its `in` channels are on (no `in`: always), it gets `velocity`
[vx, vy, vz] (world frame, m/s): the walking body through
`BodyAdapter.kick` (its linear velocity; posture and walking controller
memory stay), a free object through its free joint (spin cleared). The env
does it after the body step (`World.requests` "kick"). A launched body or
object ignores this spring for `cooldown` seconds. The pad is drawn on the
floor (no collision).

A cube kicked to (vh, vz) from height z0 lands vh (vz + sqrt(vz^2 + 2 g dz)) / g
away; tests/test_spring.py checks the launch speed (+-5 %) and that distance (+-15 %).
"""

from __future__ import annotations

import mujoco
import numpy as np

from ... import markers
from ...schema import Field
from ..base import TRIGGERS, Part
from ..input._sense import candidates, in_rect

RGBA = (0.85, 0.45, 0.15, 1.0)


class Spring(Part):
    marked = True
    marker_anchor = "/pad"
    marker_faces = markers.FACES_FLAT
    stage = "device"
    state_keys = ("uses", "until")
    fields = (
        Field("size", "vec", [0.2, 0.2], length=2, unit="m", min=0.05, max=2.0, doc="pad size [x, y]"),
        Field("velocity", "vec", [0.0, 0.0, 1.5], length=3, unit="m/s", min=-5.0, max=5.0,
              runtime=True, doc="launch velocity [vx, vy, vz] (world frame)"),
        Field("trigger", "enum", "any", choices=tuple(TRIGGERS), doc="what it launches"),
        Field("cooldown", "float", 1.0, unit="s", min=0.1, max=60.0,
              doc="time a launched body or object ignores this spring"),
        Field("max_height", "float", 0.2, unit="m", min=0.01, max=1.0,
              doc="objects count when their centre is lower than this"),
    )

    def marker_face_size(self, face):
        return float(min(self.cfg["size"])) * 0.8

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.half = np.asarray(self.cfg["size"], float) / 2
        self.trigger = self.cfg["trigger"]
        self.cooldown = float(self.cfg["cooldown"])
        self.max_height = float(self.cfg["max_height"])
        self.uses = 0
        self.until: dict = {}

    def build(self, spec):
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/pad", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*self.half, 0.003], pos=[0, 0, 0.003], rgba=RGBA,
                      contype=0, conaffinity=0)
        for i, r in enumerate((0.7, 0.45)):          # coil rings
            body.add_geom(name=f"{self.body_name}/coil{i}", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                          size=[min(self.half) * r, 0.0035], pos=[0, 0, 0.0035],
                          rgba=(0.95, 0.9, 0.8, 1.0) if i == 0 else RGBA, contype=0, conaffinity=0)
        return body

    def reset(self, model, data):
        self.uses = 0
        self.until = {}

    def update(self, model, data, body):
        if not self.enabled() or (self.inputs and not self.inputs_on()) or self.world is None:
            return
        vel = np.asarray(self.live["velocity"], float)
        for key, pos in candidates(self, data, body, self.trigger):
            if not in_rect(pos, self.xy, self.half) or (key != "body" and pos[2] >= self.max_height):
                continue
            if data.time < self.until.get(key, -np.inf):
                continue
            self.until[key] = data.time + self.cooldown
            self.uses += 1
            self.world.requests.append(("kick", self, key, vel.copy()))

    def state(self):
        return float(self.uses > 0)

    def target_mask(self, grid):
        return grid.rect(self.xy, tuple(self.half))
