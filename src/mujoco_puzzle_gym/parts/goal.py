"""Goal: reached when the body centre is within `radius` of it (no collision)."""

from __future__ import annotations

import mujoco
import numpy as np

from ..schema import Field
from .base import Part


class Goal(Part):
    role = "goal"
    stage = "input"
    state_keys = ("reached",)
    fields = (
        Field("radius", "float", 0.15, unit="m", min=0.03, max=1.0, doc="reach distance"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.radius = float(self.cfg["radius"])
        self.reached = False

    def build(self, spec):
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/disk", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[self.radius, 0.002], pos=[0, 0, 0.002],
                      rgba=(0.2, 0.45, 0.95, 0.8), contype=0, conaffinity=0)
        return body

    def reset(self, model, data):
        self.reached = False

    def update(self, model, data, body):
        if self.enabled():
            self.reached = self.reached or bool(np.hypot(*(body.pos[:2] - self.xy)) < self.radius)

    def state(self):
        return float(self.reached)

    def satisfied(self):
        return self.reached

    def target_mask(self, grid):
        return grid.disk(self.xy, self.radius * 0.8)
