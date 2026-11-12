"""Spinning bar (R16): a bar at floor height that turns about a hub.

While active the bar (`length` from the hub to each tip, centre `height` above
the floor) turns at `power` [deg/s, + = counter-clockwise]; inactive, it stops.
A hinge joint about z with a velocity actuator and gravity compensation; the
bar does not collide with the static world. It sweeps legs and objects.

Planning: the disk it sweeps is blocked.
"""

from __future__ import annotations

import math

import mujoco

from ...schema import Field
from .base import Trap

KV = 10.0            # N m s/rad: keeps its rate when it hits the body
ARMATURE = 0.05      # kg m^2: keeps the explicit velocity servo stable (kv dt / I < 1)


class Spinner(Trap):
    state_keys = ("spinning",)
    fields = (
        Field("length", "float", 0.4, unit="m", min=0.05, max=1.5, doc="hub to each tip"),
        Field("height", "float", 0.03, unit="m", min=0.01, max=0.3, doc="bar centre above the floor"),
        Field("thickness", "float", 0.03, unit="m", min=0.01, max=0.1),
        Field("power", "float", 90.0, unit="deg/s", min=-720.0, max=720.0, runtime=True,
              doc="turn rate while active (+: counter-clockwise)"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.length = float(self.cfg["length"])
        self.height = float(self.cfg["height"])
        self.thick = float(self.cfg["thickness"])
        self.spinning = False

    def build(self, spec):
        body = super().build(spec)
        body.gravcomp = 1.0
        j = body.add_joint(name=self.body_name + "/spin", type=mujoco.mjtJoint.mjJNT_HINGE,
                           axis=[0, 0, 1], damping=0.1, armature=ARMATURE)
        body.add_geom(name=self.body_name + "/bar", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[self.length, self.thick / 2, self.thick / 2], pos=[0, 0, self.height],
                      mass=0.5, rgba=(0.85, 0.25, 0.2, 1.0))
        body.add_geom(name=self.body_name + "/hub", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[self.thick, self.height / 2 + self.thick / 2],
                      pos=[0, 0, self.height / 2 + self.thick / 2], mass=0.1,
                      rgba=(0.3, 0.3, 0.3, 1.0))
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
        self.spinning = self.active()
        data.ctrl[self._act] = math.radians(float(self.live["power"])) if self.spinning else 0.0

    def angle(self, data) -> float:
        return float(data.qpos[self._qadr])

    def rate(self, data) -> float:
        """Turn rate now [deg/s]."""
        return math.degrees(float(data.qvel[self._vadr]))

    def state(self):
        return float(self.spinning)

    def hole_mask(self, grid, get):
        return grid.disk(self.xy, self.length + self.thick)

    def hole_now(self, grid):
        return grid.disk(self.xy, self.length + self.thick)

    def target_mask(self, grid):
        return grid.disk(self.xy, self.length)
