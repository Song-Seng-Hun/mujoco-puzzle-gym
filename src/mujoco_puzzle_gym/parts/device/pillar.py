"""Lift pillar (R12): a block that rises out of the floor and becomes a wall.

Down, its top is flush with the floor; while its `in` channels are on it
rises to `height` at `speed` [m/s] (`mode: follow`), or stays up once raised
(`mode: oneshot`), the same rules as doors (`Opening`). A slide joint with a
position actuator (kp 2e4, critically damped) and gravity compensation; it
does not collide with the static world (it sinks into the floor).

Planning: once the inputs that raise it are on, its cells are blocked (like a
closed gate), so solution orders pass it before it rises.
"""

from __future__ import annotations

import mujoco
import numpy as np

from ... import markers
from ...schema import Field
from .door import Opening

KP = 2.0e4


class Pillar(Opening):
    marked = True
    marker_anchor = "/block"
    marker_faces = markers.FACES_PANEL
    state_keys = ("q", "goal", "latched")
    fields = (
        Field("size", "vec", [0.1, 0.4], length=2, unit="m", min=0.03, max=2.0, doc="footprint [x, y]"),
        Field("height", "float", 0.25, unit="m", min=0.05, max=1.0, doc="top height when up"),
        Field("speed", "float", 0.2, unit="m/s", min=0.01, max=2.0, runtime=True, doc="rising speed"),
    )

    def marker_face_size(self, face):
        return float(min(min(self.cfg["size"]), self.cfg["height"]))

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.half = np.asarray(self.cfg["size"], float) / 2
        self.height = float(self.cfg["height"])
        self.q = self.goal = 0.0
        self._init_opening()

    def build(self, spec):
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, -self.height],
                                       gravcomp=1.0)
        j = body.add_joint(name=self.body_name + "/lift", type=mujoco.mjtJoint.mjJNT_SLIDE,
                           axis=[0, 0, 1], range=[0.0, self.height],
                           limited=mujoco.mjtLimited.mjLIMITED_TRUE, damping=5.0)
        body.add_geom(name=self.body_name + "/block", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*self.half, self.height / 2], pos=[0, 0, self.height / 2],
                      mass=2.0, rgba=(0.5, 0.5, 0.55, 1.0))
        act = spec.add_actuator(name=self.body_name + "/motor", target=j.name,
                                trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.set_to_position(kp=KP, dampratio=1.0)
        spec.add_exclude(bodyname1="world", bodyname2=self.body_name)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        j = model.joint(self.body_name + "/lift")
        self._qadr, self._vadr = int(j.qposadr[0]), int(j.dofadr[0])
        self._act = model.actuator(self.body_name + "/motor").id

    def reset(self, model, data):
        data.qpos[self._qadr] = data.qvel[self._vadr] = data.ctrl[self._act] = 0.0
        self.q = self.goal = 0.0
        self.latched = False

    def update(self, model, data, body):
        if self.enabled():
            target = self.height if self.want_open() else 0.0
            self.goal = self.ramp(self.goal, target, float(self.live["speed"]), self.bus.dt)
            data.ctrl[self._act] = self.goal
        self.q = float(data.qpos[self._qadr])

    def top(self) -> float:
        """Top height above the floor now [m]."""
        return self.q

    def state(self):
        return float(np.clip(self.q / self.height, 0.0, 1.0))

    # ---- planning ---------------------------------------------------------------
    def hole_mask(self, grid, get):
        return grid.rect(self.xy, tuple(self.half)) if self.plan_inputs_on(get) else None

    def hole_now(self, grid):
        return grid.rect(self.xy, tuple(self.half)) if self.q > 0.01 or self.goal > 0.01 else None

    def target_mask(self, grid):
        return grid.rect(self.xy, tuple(self.half))
