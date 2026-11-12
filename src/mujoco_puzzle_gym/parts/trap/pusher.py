"""Pusher (R16): a paddle that shoves sideways out of a housing.

The housing stands at `pos` and the paddle faces `yaw` (deg, 0 = +x). While
active the paddle runs out `stroke` [m] and back at `power` [m/s], again and
again; inactive, it goes back in and stays. A slide joint with a position
actuator (kp 2e4, critically damped, gravity compensated) whose target moves
every physics step (`substep`), like the platform (R11). The housing is solid.

Planning: the housing and the strip the paddle sweeps are blocked.
"""

from __future__ import annotations

import mujoco
import numpy as np

from ...layout import Segment
from ...schema import Field
from .base import Trap, heading, swept_half, yaw_quat

KP = 2.0e4
HOUSING = 0.04       # [m] housing depth behind the paddle
PADDLE = 0.03        # [m] paddle thickness


class Pusher(Trap):
    state_keys = ("s", "dir", "on")
    fields = (
        Field("size", "vec", [0.3, 0.1], length=2, unit="m", min=0.03, max=1.5,
              doc="paddle [width, height]"),
        Field("stroke", "float", 0.3, unit="m", min=0.05, max=1.5, doc="how far the paddle runs out"),
        Field("yaw", "float", 0.0, unit="deg", min=-180.0, max=180.0, doc="direction it pushes (0: +x)"),
        Field("power", "float", 0.3, unit="m/s", min=0.02, max=2.0, runtime=True, doc="paddle speed"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.width, self.h = (float(v) for v in self.cfg["size"])
        self.stroke = float(self.cfg["stroke"])
        self.yaw = float(self.cfg["yaw"])
        self.s, self.dir, self.on = 0.0, 1, False

    def build(self, spec):
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, 0.0],
                                       quat=yaw_quat(self.yaw))
        body.add_geom(name=self.body_name + "/housing", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[HOUSING / 2, self.width / 2 + 0.01, self.h / 2 + 0.01],
                      pos=[-HOUSING / 2, 0, self.h / 2 + 0.01], rgba=(0.35, 0.35, 0.4, 1.0))
        paddle = body.add_body(name=self.body_name + "/paddle", gravcomp=1.0)
        j = paddle.add_joint(name=self.body_name + "/slide", type=mujoco.mjtJoint.mjJNT_SLIDE,
                             axis=[1, 0, 0], range=[0.0, self.stroke],
                             limited=mujoco.mjtLimited.mjLIMITED_TRUE, damping=5.0)
        paddle.add_geom(name=self.body_name + "/plate", type=mujoco.mjtGeom.mjGEOM_BOX,
                        size=[PADDLE / 2, self.width / 2, self.h / 2],
                        pos=[PADDLE / 2, 0, self.h / 2 + 0.005], mass=1.0, rgba=(0.9, 0.55, 0.1, 1.0))
        act = spec.add_actuator(name=self.body_name + "/motor", target=j.name,
                                trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.set_to_position(kp=KP, dampratio=1.0)
        spec.add_exclude(bodyname1="world", bodyname2=self.body_name + "/paddle")
        spec.add_exclude(bodyname1=self.body_name, bodyname2=self.body_name + "/paddle")
        return body

    def bind(self, model, data):
        super().bind(model, data)
        j = model.joint(self.body_name + "/slide")
        self._qadr, self._vadr = int(j.qposadr[0]), int(j.dofadr[0])
        self._act = model.actuator(self.body_name + "/motor").id

    def reset(self, model, data):
        data.qpos[self._qadr] = data.qvel[self._vadr] = data.ctrl[self._act] = 0.0
        self.s, self.dir, self.on = 0.0, 1, False

    def update(self, model, data, body):
        self.on = self.active()

    def substep(self, model, data):
        step = float(self.live["power"]) * float(model.opt.timestep)
        if self.on:
            self.s += self.dir * step
            if self.s >= self.stroke:
                self.s, self.dir = self.stroke, -1
            elif self.s <= 0.0:
                self.s, self.dir = 0.0, 1
        else:
            self.s, self.dir = max(0.0, self.s - step), 1
        data.ctrl[self._act] = self.s

    def extension(self, data) -> float:
        """Paddle position now [m] out of the housing."""
        return float(data.qpos[self._qadr])

    def state(self):
        return float(self.on)

    def blockers(self, is_open):
        d = heading(self.yaw)
        n = np.array([-d[1], d[0]]) * (self.width / 2 + 0.01)
        c = self.xy - d * HOUSING / 2
        return [Segment(tuple(c - n), tuple(c + n), HOUSING)]

    def _sweep(self, grid):
        c, half = swept_half(self.yaw, (0.0, self.stroke + PADDLE), self.width)
        return grid.rect(self.xy + c, tuple(half))

    def hole_mask(self, grid, get):
        return self._sweep(grid)

    def hole_now(self, grid):
        return self._sweep(grid)

    def target_mask(self, grid):
        return self._sweep(grid)
