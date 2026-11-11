"""Checkpoint flag (R6): turns on when the body reaches it in its turn.

Flags carry `order: 1, 2, ...`. The next flag is the lowest order not reached
yet. When the walking body's trunk comes within `radius` of the next flag, the
flag turns on for good (writes 1), and the env saves a snapshot at that body
step (snapshot.py): MuJoCo state, model values, parts, signals, body history.

Reaching a later flag out of turn does nothing with `skip: ignore` (default)
and fails the episode with `skip: fail`. With the task's `env.respawn:
checkpoint`, a fall or failure goes back to the last snapshot instead of
ending the episode (env.py); the default `start` ends it.
"""

from __future__ import annotations

import mujoco
import numpy as np

from ...schema import Field
from ..base import Part

OFF_RGBA = (0.85, 0.85, 0.85, 1.0)
ON_RGBA = (0.2, 0.8, 0.3, 1.0)
POLE_H = 0.3


class Checkpoint(Part):
    role = "checkpoint"
    stage = "input"
    emits = True
    stateful = True
    state_keys = ("reached", "fail", "was_inside")
    fields = (
        Field("order", "int", 1, min=1, max=100, doc="flags turn on in this order"),
        Field("radius", "float", 0.15, unit="m", min=0.03, max=1.0, doc="reach distance"),
        Field("skip", "enum", "ignore", choices=("ignore", "fail"),
              doc="reaching this flag before its turn: ignore it, or fail the episode"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.order = self.cfg["order"]
        self.radius = float(self.cfg["radius"])
        self.skip = self.cfg["skip"]
        self.reached = self.fail = self.was_inside = False

    def build(self, spec):
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/disk", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[self.radius, 0.0015], pos=[0, 0, 0.0015], rgba=(*OFF_RGBA[:3], 0.4),
                      contype=0, conaffinity=0)
        body.add_geom(name=self.body_name + "/pole", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[0.006, POLE_H / 2], pos=[0, 0, POLE_H / 2], rgba=(0.3, 0.3, 0.3, 1.0),
                      contype=0, conaffinity=0)
        body.add_geom(name=self.body_name + "/flag", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[0.04, 0.002, 0.025], pos=[0.04, 0, POLE_H - 0.025], rgba=OFF_RGBA,
                      contype=0, conaffinity=0)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._flag = model.geom(self.body_name + "/flag").id

    def reset(self, model, data):
        self.reached = self.fail = self.was_inside = False
        self.pending_snapshot = False
        model.geom_rgba[self._flag] = OFF_RGBA
        for ch in self.outputs:
            self.write(0.0, ch)

    def update(self, model, data, body):
        inside = bool(np.hypot(*(body.pos[:2] - self.xy)) < self.radius)
        entered, self.was_inside = inside and not self.was_inside, inside
        if self.reached or not inside or not self.enabled():
            return
        nxt = self.world.next_checkpoint() if self.world is not None else self.order
        if self.order == nxt:
            self.reached = True
            self.pending_snapshot = True
            model.geom_rgba[self._flag] = ON_RGBA
            for ch in self.outputs:
                self.write(1.0, ch)
        elif entered and nxt is not None and self.order > nxt and self.skip == "fail":
            self.fail = True

    def state(self):
        return float(self.reached)

    def satisfied(self):
        return self.reached

    def failed(self):
        return self.fail

    def target_mask(self, grid):
        return grid.disk(self.xy, self.radius * 0.8)
