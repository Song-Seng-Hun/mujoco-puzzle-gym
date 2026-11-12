"""Pendulum (R16): a ball on a rod that swings under a frame.

The pivot is `height` above `pos`; the ball (`radius`, `mass`) hangs
`length` below it and swings along `yaw` (deg, 0 = along x). While active the
swing amplitude grows to `power` [deg] (at `AMP_RATE`) and the pendulum
follows amplitude * sin(2 pi t / period); inactive, the amplitude shrinks to
0 and it hangs still. A hinge with a position actuator; the frame is drawn
only (no collision), the ball collides.

Planning: the strip the ball sweeps is blocked.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from ...schema import Field, SchemaError
from .base import Trap, swept_half, yaw_quat

KP = 60.0            # N m/rad
AMP_RATE = 60.0      # deg/s: how fast the amplitude follows `power`


class Pendulum(Trap):
    state_keys = ("amp", "phase", "on")
    fields = (
        Field("height", "float", 0.6, unit="m", min=0.1, max=2.0, doc="pivot above the floor"),
        Field("length", "float", 0.45, unit="m", min=0.05, max=1.9, doc="pivot to ball centre"),
        Field("radius", "float", 0.06, unit="m", min=0.01, max=0.3, doc="ball radius"),
        Field("mass", "float", 0.5, unit="kg", min=0.01, max=5.0, doc="ball mass"),
        Field("yaw", "float", 0.0, unit="deg", min=-180.0, max=180.0, doc="swing direction"),
        Field("period", "float", 1.6, unit="s", min=0.3, max=10.0, runtime=True, doc="one full swing"),
        Field("power", "float", 40.0, unit="deg", min=0.0, max=80.0, runtime=True,
              doc="swing amplitude while active"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        self.height, self.length = float(c["height"]), float(c["length"])
        self.radius, self.mass, self.yaw = float(c["radius"]), float(c["mass"]), float(c["yaw"])
        if self.length + self.radius > self.height:
            raise SchemaError(f"part {name!r}: length + radius = {self.length + self.radius:g} m "
                              f"must not exceed height = {self.height:g} m (the ball would hit the floor)")
        self.amp = self.phase = 0.0
        self.on = False

    def build(self, spec):
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, 0.0],
                                       quat=yaw_quat(self.yaw))
        frame = dict(type=mujoco.mjtGeom.mjGEOM_CAPSULE, rgba=(0.4, 0.4, 0.45, 1.0),
                     contype=0, conaffinity=0, mass=0.0)
        w = self.radius + 0.08
        for i, y in enumerate((-w, w)):
            body.add_geom(name=f"{self.body_name}/post{i}", size=[0.012, 0, 0],
                          fromto=[0, y, 0, 0, y, self.height], **frame)
        body.add_geom(name=self.body_name + "/beam", size=[0.012, 0, 0],
                      fromto=[0, -w, self.height, 0, w, self.height], **frame)
        arm = body.add_body(name=self.body_name + "/arm", pos=[0, 0, self.height])
        j = arm.add_joint(name=self.body_name + "/swing", type=mujoco.mjtJoint.mjJNT_HINGE,
                          axis=[0, 1, 0], damping=0.05)
        arm.add_geom(name=self.body_name + "/rod", type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.006, 0, 0],
                     fromto=[0, 0, 0, 0, 0, -self.length], rgba=(0.6, 0.6, 0.6, 1.0),
                     contype=0, conaffinity=0, mass=0.01)
        arm.add_geom(name=self.body_name + "/ball", type=mujoco.mjtGeom.mjGEOM_SPHERE,
                     size=[self.radius, 0, 0], pos=[0, 0, -self.length], mass=self.mass,
                     rgba=(0.55, 0.2, 0.6, 1.0))
        act = spec.add_actuator(name=self.body_name + "/motor", target=j.name,
                                trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.set_to_position(kp=KP, dampratio=1.0)
        spec.add_exclude(bodyname1="world", bodyname2=self.body_name + "/arm")
        return body

    def bind(self, model, data):
        super().bind(model, data)
        j = model.joint(self.body_name + "/swing")
        self._qadr, self._vadr = int(j.qposadr[0]), int(j.dofadr[0])
        self._act = model.actuator(self.body_name + "/motor").id

    def reset(self, model, data):
        data.qpos[self._qadr] = data.qvel[self._vadr] = data.ctrl[self._act] = 0.0
        self.amp = self.phase = 0.0
        self.on = False

    def update(self, model, data, body):
        self.on = self.active()
        dt = self.bus.dt
        goal = float(self.live["power"]) if self.on else 0.0
        step = AMP_RATE * dt
        self.amp = goal if abs(goal - self.amp) <= step else self.amp + math.copysign(step, goal - self.amp)
        self.phase = (self.phase + dt / float(self.live["period"])) % 1.0
        data.ctrl[self._act] = math.radians(self.amp) * math.sin(2 * math.pi * self.phase)

    def angle(self, data) -> float:
        """Swing angle now [deg]."""
        return math.degrees(float(data.qpos[self._qadr]))

    def state(self):
        return float(self.on)

    def _sweep(self, grid):
        reach = self.length * math.sin(math.radians(float(self.live["power"]))) + self.radius
        c, half = swept_half(self.yaw, (-reach, reach), 2 * self.radius)
        return grid.rect(self.xy + c, tuple(half))

    def hole_mask(self, grid, get):
        return self._sweep(grid)

    def hole_now(self, grid):
        return self._sweep(grid)

    def target_mask(self, grid):
        return self._sweep(grid)
