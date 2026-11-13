"""Logic parts that can stand in the room (R33).

A gate (R7), counter (R8b) or signal source (R33) is a channel, not a place: by
default it has no MuJoCo body and `pos` only says where a guide draws it.
`body: true` puts it in the room as a small box with a lamp on top that shows
its current value, so a picture shows where a signal comes from, and so R34 can
break it (only a part with a body can be broken).

The box is static and collides, so it is also an obstacle: the walking body
cannot climb it (`max_step` is 0.01 m), and `blockers` reports its footprint so
the planner and the `demo:` executor walk round it instead of into it. Being
blocked by it is what makes a logic body a usable `peck` target (R34): the
walking body drifts forward at command 0 and cannot walk backwards, so it can
only hold a place where something stops it (Open question 8).
"""

from __future__ import annotations

import mujoco
import numpy as np

from ...layout import Segment
from ...schema import Field
from ..base import Part

OFF_RGBA = (0.25, 0.25, 0.30, 1.0)
ON_RGBA = (1.0, 0.85, 0.25, 1.0)
BOX_RGBA = (0.45, 0.45, 0.5, 1.0)
LAMP = 0.018          # lamp radius [m]


class BodiedLogic(Part):
    """Base for logic parts whose body is optional (`body: true`)."""

    has_body = False
    fields = (
        Field("body", "bool", False,
              doc="put the part in the room as a small box (R33); needed to be breakable (R34)"),
        Field("size", "vec", [0.08, 0.08], length=2, unit="m", min=0.04, max=0.4,
              doc="`body: true`: footprint [x, y]"),
        Field("height", "float", 0.12, unit="m", min=0.04, max=0.6,
              doc="`body: true`: box height"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.has_body = bool(self.cfg["body"])
        self.half = np.asarray(self.cfg["size"], dtype=float) / 2
        self.height = float(self.cfg["height"])
        self._lamp = -1
        self._lamp_on = False

    # ---- scene -------------------------------------------------------------------
    def build(self, spec):
        body = super().build(spec)
        if body is None:
            return None
        body.add_geom(name=self.body_name + "/box", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[self.half[0], self.half[1], self.height / 2],
                      pos=[0, 0, self.height / 2], rgba=BOX_RGBA)
        body.add_geom(name=self.body_name + "/lamp", type=mujoco.mjtGeom.mjGEOM_SPHERE,
                      size=[LAMP, 0, 0], pos=[0, 0, self.height + LAMP * 0.5],
                      rgba=OFF_RGBA, contype=0, conaffinity=0)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._lamp = model.geom(self.body_name + "/lamp").id if self.has_body else -1
        self._lamp_on = False

    def paint(self, model) -> None:
        """Lamp colour from `state()`. Subclasses call it at the end of `reset`,
        `update` and `load_state`; without a body it does nothing."""
        if self._lamp < 0:
            return
        on = self.state() > 0.5
        if on != self._lamp_on:
            model.geom_rgba[self._lamp] = ON_RGBA if on else OFF_RGBA
            self._lamp_on = on

    # ---- planning -----------------------------------------------------------------
    def blockers(self, is_open):
        if not self.has_body:
            return []
        return [Segment((self.xy[0] - self.half[0], self.xy[1]),
                        (self.xy[0] + self.half[0], self.xy[1]), 2 * self.half[1])]
