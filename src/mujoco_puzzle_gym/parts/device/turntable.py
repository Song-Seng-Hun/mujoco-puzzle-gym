"""Turntable (R12): a floor disk that spins while its signal is on.

While its `in` channels are on (always, without `in`) the disk turns at
`speed` [deg/s, + = counter-clockwise]; off, it stops. A hinge joint about z
with a velocity actuator and gravity compensation; the disk does not collide
with the static world. What stands on it turns with it through friction.

Planning: the disk's cells are blocked (standing on a spinning disk is outside
the walking policy's training).
"""

from __future__ import annotations

import math

import mujoco

from ... import markers
from ...schema import Field
from ..base import Part

KV = 5.0             # N m s/rad; holds a 2 kg disk loaded with a cube near its set speed


class Turntable(Part):
    marked = True
    marker_anchor = "/disk"
    marker_faces = markers.FACES_FLAT
    stage = "device"
    state_keys = ("spinning",)
    fields = (
        Field("radius", "float", 0.25, unit="m", min=0.05, max=1.5),
        Field("height", "float", 0.008, unit="m", min=0.004, max=0.1, doc="top face above the floor"),
        Field("speed", "float", 45.0, unit="deg/s", min=-360.0, max=360.0, runtime=True,
              doc="spin rate while on (+: counter-clockwise)"),
    )

    def marker_face_size(self, face):
        return markers.disk_square(float(self.cfg["radius"]) * 0.6)

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.radius = float(self.cfg["radius"])
        self.height = float(self.cfg["height"])
        self.spinning = False

    def build(self, spec):
        body = super().build(spec)
        body.gravcomp = 1.0
        j = body.add_joint(name=self.body_name + "/spin", type=mujoco.mjtJoint.mjJNT_HINGE,
                           axis=[0, 0, 1], damping=0.1)
        body.add_geom(name=self.body_name + "/disk", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[self.radius, self.height / 2], pos=[0, 0, self.height / 2],
                      mass=2.0, rgba=(0.45, 0.6, 0.4, 1.0))
        body.add_geom(name=self.body_name + "/stripe", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[self.radius * 0.9, 0.01, 0.001], pos=[0, 0, self.height + 0.001],
                      rgba=(0.9, 0.9, 0.9, 1.0), contype=0, conaffinity=0, mass=0.0)
        act = spec.add_actuator(name=self.body_name + "/motor", target=j.name,
                                trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.set_to_velocity(kv=KV)
        spec.add_exclude(bodyname1="world", bodyname2=self.body_name)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        j = model.joint(self.body_name + "/spin")
        self._qadr, self._vadr = int(j.qposadr[0]), int(j.dofadr[0])
        self._act = model.actuator(self.body_name + "/motor").id

    def reset(self, model, data):
        data.qpos[self._qadr] = data.qvel[self._vadr] = data.ctrl[self._act] = 0.0
        self.spinning = False

    def update(self, model, data, body):
        if not self.enabled():
            return
        self.spinning = self.inputs_on() if self.inputs else True
        data.ctrl[self._act] = math.radians(float(self.live["speed"])) if self.spinning else 0.0

    def angle(self, data) -> float:
        """Disk angle now [rad]."""
        return float(data.qpos[self._qadr])

    def rate(self, data) -> float:
        """Spin rate now [deg/s]."""
        return math.degrees(float(data.qvel[self._vadr]))

    def state(self):
        return float(self.spinning)

    def hole_mask(self, grid, get):
        return grid.disk(self.xy, self.radius)

    def hole_now(self, grid):
        return grid.disk(self.xy, self.radius)

    def target_mask(self, grid):
        return grid.disk(self.xy, self.radius)

