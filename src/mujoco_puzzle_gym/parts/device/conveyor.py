"""Conveyor (R12, Open question 1 (c)): a belt that carries what stands on it.

Every body step while its `in` channels are on (always, without `in`), the
walking body and free objects (`trigger`, default `any`) standing on the belt
move by `speed` x dt along `axis` (+: toward +axis): the env shifts their root
position (`World.requests` "move"). The belt is a drawn strip on the floor
(no collision), so the surface has the room floor's normal friction.

Why not a moving surface (2026-11-12 measurements): a belt of mocap pieces
did not carry a cube at all (94 cm behind over 3 s); a tangential force on
touching bodies moved a cube at 98 % of the belt speed only with surface
friction 0.02 (0.014 m/s at normal friction), too slippery to walk on.
"""

from __future__ import annotations

import mujoco
import numpy as np

from ... import markers
from ...schema import Field
from ..base import TRIGGERS, Part
from ..input._sense import candidates, in_rect

STRIPES = 6


class Conveyor(Part):
    marked = True
    marker_anchor = "/belt"
    marker_faces = markers.FACES_FLAT
    stage = "device"
    state_keys = ("running",)
    fields = (
        Field("size", "vec", [0.8, 0.3], length=2, unit="m", min=0.1, max=5.0, doc="belt size [x, y]"),
        Field("axis", "enum", "x", choices=("x", "y"), doc="direction the belt runs along"),
        Field("speed", "float", 0.1, unit="m/s", min=-0.5, max=0.5, runtime=True,
              doc="belt speed (+: toward +axis)"),
        Field("trigger", "enum", "any", choices=tuple(TRIGGERS), doc="what it carries"),
        Field("max_height", "float", 0.2, unit="m", min=0.01, max=1.0,
              doc="objects count when their centre is lower than this"),
    )

    def marker_face_size(self, face):
        return float(min(self.cfg["size"])) * 0.8

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.half = np.asarray(self.cfg["size"], float) / 2
        self.dir = np.array([1.0, 0.0, 0.0]) if self.cfg["axis"] == "x" else np.array([0.0, 1.0, 0.0])
        self.trigger = self.cfg["trigger"]
        self.max_height = float(self.cfg["max_height"])
        self.running = False

    def build(self, spec):
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/belt", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*self.half, 0.002], pos=[0, 0, 0.002], rgba=(0.2, 0.2, 0.22, 1.0),
                      contype=0, conaffinity=0)
        a = 0 if self.cfg["axis"] == "x" else 1
        long = self.half[a]
        for i in range(STRIPES):                     # chevrons along the belt
            off = np.zeros(3)
            off[a] = -long + (i + 0.5) * 2 * long / STRIPES
            size = [0.008, self.half[1] * 0.8, 0.001] if a == 0 else [self.half[0] * 0.8, 0.008, 0.001]
            body.add_geom(name=f"{self.body_name}/stripe{i}", type=mujoco.mjtGeom.mjGEOM_BOX,
                          size=size, pos=[off[0], off[1], 0.0045], rgba=(0.9, 0.75, 0.1, 1.0),
                          contype=0, conaffinity=0)
        return body

    def reset(self, model, data):
        self.running = False

    def update(self, model, data, body):
        if not self.enabled():
            return
        self.running = self.inputs_on() if self.inputs else True
        if not self.running or self.world is None:
            return
        delta = self.dir * float(self.live["speed"]) * self.bus.dt
        for key, pos in candidates(self, data, body, self.trigger):
            if in_rect(pos, self.xy, self.half) and (key == "body" or pos[2] < self.max_height):
                self.world.requests.append(("move", self, key, delta))

    def state(self):
        return float(self.running)

    def target_mask(self, grid):
        return grid.rect(self.xy, tuple(self.half))

