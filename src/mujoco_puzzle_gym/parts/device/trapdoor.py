"""Trapdoor (R9): a floor tile that swings down and leaves a hole.

The tile turns about a hinge along its `hinge_side` edge; open, it hangs
straight down (90 deg). It opens while its `in` channels are on (`mode:
follow`) or for good once they were (`mode: oneshot`), at `speed` deg/s.
Closed, the tile rests on its joint limit, so a body standing on it does not
press it down.

Once the tile has turned more than `HOLE_AT` the cell counts as a hole for the
fall rule (floor.py): a body whose trunk drops below the rim (the tile's
closed top, z = 0) has fallen. What falls in lands on the catch plane.

Planning: once the inputs that open it are on, the planner grid has no floor
on the tile (`hole_mask`), so solution orders and solver paths go round it.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from ...floor import TILE_THICKNESS
from ...schema import Field
from ..floor.tile import FloorPart
from .door import Opening

SIDES = {"-x": (1.0, 0.0), "+x": (-1.0, 0.0), "-y": (0.0, 1.0), "+y": (0.0, -1.0)}
HOLE_AT = math.radians(10.0)
OPEN_ANGLE = math.pi / 2


class Trapdoor(FloorPart, Opening):
    role = "prop"
    state_keys = ("q", "goal", "latched")
    fields = (
        Field("hinge_side", "enum", "-x", choices=tuple(SIDES), doc="edge the tile hangs from"),
        Field("speed", "float", 180.0, unit="deg/s", min=10.0, max=2000.0, doc="opening speed"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.side = self.cfg["hinge_side"]
        self.speed = math.radians(float(self.cfg["speed"]))
        self.dir = np.asarray(SIDES[self.side])             # hinge -> tile centre
        self.q = self.goal = 0.0
        self._init_opening()

    def _span(self) -> float:
        return float(self.half[0] if self.dir[0] else self.half[1])

    def build(self, spec):
        hinge = self.base_xy - self.dir * self._span()
        body = spec.worldbody.add_body(name=self.body_name, pos=[*hinge, 0.0], gravcomp=1.0)
        axis = np.cross([*self.dir, 0.0], [0.0, 0.0, -1.0])      # positive turns the tile down
        j = body.add_joint(name=self.body_name + "/hinge", type=mujoco.mjtJoint.mjJNT_HINGE,
                           axis=axis.tolist(), range=[0.0, 90.0],   # degrees (MjSpec default)
                           limited=mujoco.mjtLimited.mjLIMITED_TRUE, damping=2.0, armature=0.05)
        c = self.dir * self._span()
        body.add_geom(name=self.body_name + "/tile", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*self.half, TILE_THICKNESS / 2], pos=[c[0], c[1], -TILE_THICKNESS / 2],
                      priority=1, mass=1.0, rgba=(0.6, 0.45, 0.3, 1.0))
        act = spec.add_actuator(name=self.body_name + "/motor", target=j.name,
                                trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.set_to_position(kp=100.0, dampratio=1.0)
        spec.add_exclude(bodyname1="world", bodyname2=self.body_name)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        j = model.joint(self.body_name + "/hinge")
        self._qadr, self._vadr = int(j.qposadr[0]), int(j.dofadr[0])
        self._act = model.actuator(self.body_name + "/motor").id

    def reset(self, model, data):
        data.qpos[self._qadr] = data.qvel[self._vadr] = data.ctrl[self._act] = 0.0
        self.q = self.goal = 0.0
        self.latched = False

    def update(self, model, data, body):
        if self.enabled():
            # Below 0 the joint limit holds the closed tile; the motor only pulls it open.
            target = OPEN_ANGLE if self.want_open() else -0.05
            self.goal = self.ramp(self.goal, target, self.speed, self.bus.dt)
            data.ctrl[self._act] = self.goal
        self.q = float(data.qpos[self._qadr])

    def state(self):
        return float(np.clip(self.q / OPEN_ANGLE, 0.0, 1.0))

    def is_open(self) -> bool:
        return self.q > HOLE_AT

    def height_at(self, xy):
        return (None, 0.0) if self.is_open() else (0.0, 0.0)

    # ---- planning (R20: the planner keeps the body off trapdoors that open) -------------
    def hole_mask(self, grid, get):
        return grid.rect(self.xy, tuple(self.half)) if self.plan_inputs_on(get) else None

    def hole_now(self, grid):
        return grid.rect(self.xy, tuple(self.half)) if self.q > 0.01 or self.goal > 0.01 else None
