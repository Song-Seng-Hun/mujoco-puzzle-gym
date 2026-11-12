"""Pit (R16): a hole in the floor.

A floor part (tile floor, R10) with no tile: the cell is "no floor", so a body
whose trunk drops below the rim has fallen (floor.py). `on_fall` says what
then happens: `fail` ends the episode (`failed()`, also without the fall
termination), `reset` puts the body back at the last checkpoint or the start
(env.py), like room `on_fall: reset` (R37). What falls in lands on the catch
plane.

Without `in` the pit is always open. With `in` a lid covers it and slides
away while the inputs are on (and `enable` and `enabled`, trap base); the lid
is a collision toggle (contype / conaffinity, kept by snapshots), drawn only
while closed.

Planning: an open pit is a hole (`hole_mask`, `hole_now`), so solution
orders and solver paths go round it.
"""

from __future__ import annotations

import mujoco
import numpy as np

from ...floor import TILE_THICKNESS
from ...schema import Field
from ..floor.tile import FloorPart
from ..input._sense import in_rect
from .base import Trap

LID_RGBA = (0.45, 0.4, 0.35, 1.0)
DARK = (0.08, 0.08, 0.08, 1.0)


class Pit(FloorPart, Trap):
    state_keys = ("open", "fell")
    fields = (
        Field("on_fall", "enum", "fail", choices=("fail", "reset"),
              doc="fail: the episode ends; reset: back to the last checkpoint or the start"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.on_fall = self.cfg["on_fall"]
        self.open = not self.inputs
        self.fell = False

    def build(self, spec):
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, 0.0])
        body.add_geom(name=self.body_name + "/dark", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*self.half, 0.005], pos=[0, 0, -0.25], rgba=DARK,
                      contype=0, conaffinity=0)
        if self.inputs:
            body.add_geom(name=self.body_name + "/lid", type=mujoco.mjtGeom.mjGEOM_BOX,
                          size=[*self.half, TILE_THICKNESS / 2], pos=[0, 0, -TILE_THICKNESS / 2],
                          priority=1, rgba=LID_RGBA)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._lid = model.geom(self.body_name + "/lid").id if self.inputs else -1

    def _set_lid(self, model, closed: bool) -> None:
        if self._lid >= 0:
            model.geom_contype[self._lid] = model.geom_conaffinity[self._lid] = int(closed)
            model.geom_rgba[self._lid] = LID_RGBA if closed else (*LID_RGBA[:3], 0.0)

    def reset(self, model, data):
        self.open = not self.inputs
        self.fell = False
        self._set_lid(model, not self.open)

    def update(self, model, data, body):
        if self.inputs:
            now = self.active()
            if now != self.open:
                self.open = now
                self._set_lid(model, not now)
        if body is not None and body.fallen and self.open and in_rect(body.pos, self.xy, self.half):
            self.fell = True

    def height_at(self, xy):
        return (None, 0.0) if self.open else (0.0, 0.0)

    def failed(self):
        return self.fell and self.on_fall == "fail"

    def state(self):
        return float(self.open)

    def hole_mask(self, grid, get):
        return grid.rect(self.xy, tuple(self.half)) if (not self.inputs or self.plan_inputs_on(get)) else None

    def hole_now(self, grid):
        return grid.rect(self.xy, tuple(self.half)) if self.open else None
