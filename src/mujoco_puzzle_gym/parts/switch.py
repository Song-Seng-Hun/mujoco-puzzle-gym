"""Floor switch: a thin plate with no collision. On when the body centre is over it."""

from __future__ import annotations

import mujoco
import numpy as np

from ..bodies import BodyState
from ..layout import Grid
from ..schema import Field
from .base import Part

OFF_RGBA = (0.95, 0.75, 0.1, 1.0)
ON_RGBA = (0.2, 0.8, 0.3, 1.0)


class Switch(Part):
    """Writes its channel (its name unless `out` is set): 1 while on. With
    `enable` off it keeps its state (R1)."""

    role = "switch"
    stage = "input"
    emits = True
    state_keys = ("on",)
    fields = (
        Field("size", "vec", [0.25, 0.25], length=2, unit="m", min=0.05, max=2.0,
              doc="plate size [x, y]"),
        Field("latch", "bool", True, doc="stay on after the body leaves (false: momentary)"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.half = np.asarray(self.cfg["size"], dtype=float) / 2
        # Latching by default: one body cannot hold a switch and walk through
        # the door it opens. `latch: false` makes it momentary.
        self.latch = self.cfg["latch"]
        self.stateful = self.latch
        self.on = False

    def build(self, spec):
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/plate", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*self.half, 0.002], pos=[0, 0, 0.002], rgba=OFF_RGBA,
                      contype=0, conaffinity=0)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._gid = model.geom(self.body_name + "/plate").id

    def reset(self, model, data):
        self._set(model, False)

    def _set(self, model, on: bool) -> None:
        self.on = on
        model.geom_rgba[self._gid] = ON_RGBA if on else OFF_RGBA
        for ch in self.outputs:
            self.write(float(on), ch)

    def over(self, xy) -> bool:
        d = np.abs(np.asarray(xy[:2]) - self.xy)
        return bool((d <= self.half).all())

    def update(self, model, data, body: BodyState):
        if not self.enabled():
            return
        inside = self.over(body.pos)
        on = (self.on or inside) if self.latch else inside
        if on != self.on:
            self._set(model, on)

    def state(self):
        return float(self.on)

    def satisfied(self):
        return self.on

    def target_mask(self, grid: Grid):
        return grid.rect(self.xy, tuple(self.half))
