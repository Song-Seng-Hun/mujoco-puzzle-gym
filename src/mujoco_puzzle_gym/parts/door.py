"""Sliding door: a panel on a slide joint driven by a position actuator.

Closed (joint at 0) it spans a gap of `width` in a wall; open it slides by
`width` along its axis into the wall. It opens while its linked switches are on
(`require: all` or `any`) and closes when they are off. The panel collides with
the walking body but not with the static world (walls), so it can slide into a wall.
"""

from __future__ import annotations

import mujoco
import numpy as np

from ..layout import Segment
from .base import Part

OPEN_FRACTION = 0.9   # state above this counts as open


class Door(Part):
    role = "gate"

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.width = float(cfg.get("width", 0.5))
        self.axis = cfg.get("axis", "y")
        if self.axis not in ("x", "y"):
            raise ValueError(f"door {name!r}: `axis` must be 'x' or 'y'")
        self.thickness = float(cfg.get("thickness", 0.04))
        self.height = float(cfg.get("height", 0.25))
        self.direction = 1.0 if float(cfg.get("open_dir", 1)) >= 0 else -1.0
        self.travel = self.width + 0.02
        self.q = 0.0

    def _axis_vec(self) -> np.ndarray:
        return np.array([0.0, 1.0]) if self.axis == "y" else np.array([1.0, 0.0])

    def build(self, spec):
        body = super().build(spec)
        a = self._axis_vec()
        joint = body.add_joint(name=self.body_name + "/slide", type=mujoco.mjtJoint.mjJNT_SLIDE,
                               axis=[*(a * self.direction), 0.0], range=[0.0, self.travel],
                               limited=mujoco.mjtLimited.mjLIMITED_TRUE, damping=5.0)
        half_len = self.width / 2 + 0.01
        size = [half_len, self.thickness / 2] if self.axis == "x" else [self.thickness / 2, half_len]
        body.add_geom(name=self.body_name + "/panel", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*size, self.height / 2], pos=[0, 0, self.height / 2 + 0.003],
                      rgba=(0.55, 0.35, 0.2, 1.0), density=300.0)
        act = spec.add_actuator(name=self.body_name + "/motor", target=joint.name,
                                trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.set_to_position(kp=200.0, dampratio=1.0)
        spec.add_exclude(bodyname1="world", bodyname2=self.body_name)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        j = model.joint(self.body_name + "/slide")
        self._qadr = int(j.qposadr[0])
        self._vadr = int(j.dofadr[0])
        self._act = model.actuator(self.body_name + "/motor").id

    def reset(self, model, data):
        data.qpos[self._qadr] = 0.0
        data.qvel[self._vadr] = 0.0
        data.ctrl[self._act] = 0.0
        self.q = 0.0

    def update(self, model, data, body):
        data.ctrl[self._act] = self.travel if self.sources_on() else 0.0
        self.q = float(data.qpos[self._qadr])

    def state(self):
        return float(np.clip(self.q / self.travel, 0.0, 1.0))

    def is_open(self) -> bool:
        return self.state() >= OPEN_FRACTION

    def blockers(self, is_open):
        if is_open:
            return []
        a = self._axis_vec() * (self.width / 2 + 0.01)
        return [Segment(tuple(self.xy - a), tuple(self.xy + a), self.thickness)]
