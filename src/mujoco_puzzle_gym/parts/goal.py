"""Goal: reached when the body centre is within `radius` of it (no collision)."""

from __future__ import annotations

import mujoco
import numpy as np

from .. import markers
from ..schema import Field
from .base import Part


class Goal(Part):
    role = "goal"
    stage = "input"
    state_keys = ("reached",)
    #: A goal may carry an ArUco marker **if the room asks for it** (R116). `marked` stays
    #: False, so no existing room grows one.
    #:
    #: Why it was added: with no marker on the goal, a `sensors_only` observation cannot see
    #: where the episode wants the body to go, so no room whose success is "reach the goal"
    #: can be solved from sensors alone - measured, `switch_door` with marker gaze plus a
    #: rangefinder turns its switch on in 0.66 of episodes and reaches the goal in 0.08,
    #: against 0.99 for the same room and budget with privileged observation. The refusal
    #: was not a decision about goals; `marker_faces` is simply empty on `Part`, and nothing
    #: had asked. A flat top marker on a floor-level disk is the pattern floor tiles and
    #: conveyors already use.
    marker_faces = markers.FACES_FLAT
    marker_anchor = "/disk"
    fields = (
        Field("radius", "float", 0.15, unit="m", min=0.03, max=1.0, doc="reach distance"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.radius = float(self.cfg["radius"])
        self.reached = False

    def marker_face_size(self, face: str) -> float:
        """The disk is round, so the largest square inside it: `radius * sqrt(2)`, less a
        little so the print does not overhang the rim."""
        return float(self.radius) * 1.41 * 0.9

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
