"""Low bar (R30 `glide_under`): a static bar across the way, low enough that only
a crouching roller glides under it.

A box `depth` 4 cm by `THICK` 3 cm in section and `length` long along `axis`,
its underside `bottom` above the floor. It has no joint and no state (nothing
for snapshots) and collides with the walking body and objects.

`bottom` range [0.21, 0.246] m is measured (2026-11-13, 20 seeds): the roller
crouch (`roller_crouch.onnx`) lowers the trunk, but the head (`jaw_soft`) top
only drops from 0.256-0.268 m to 0.197-0.200 m (t 0.7-2.3 s into the crouch),
so the underside must be at least 1 cm above the crouched head and 1 cm below
the rolling one. At 0.228 m across a 1.5 m corridor: rolling without a crouch
is blocked 20/20; entering at 0.38-0.40 m/s and starting the crouch with the
trunk 0.25-0.30 m before the bar's centre line glides under 20/20 without a
touch; entering at 0.29-0.35 m/s, at most 9/20. Full push reaches 0.38-0.45 m/s
after a 3-4 s run-up, so give the run-up 1 m or more (the solver step
`glide <bar>`).

Planning: the bar blocks walking like a wall, unless the room has a station
that puts the roller on (`addon_station` with `addon: roller`, `mode: attach`
or `toggle`): then the planner treats it as passable (a gliding body goes
under). Without such a station `mpg check` warns (stage `parts`).
"""

from __future__ import annotations

import mujoco
import numpy as np

from ...layout import Segment
from ...schema import Field
from ..base import Part

THICK = 0.03         # [m] bar height (section)
DEPTH = 0.04         # [m] bar depth across the way (section)
RGBA = (0.85, 0.25, 0.2, 1.0)


class LowBar(Part):
    stage = "device"
    fields = (
        Field("length", "float", 1.5, unit="m", min=0.1, max=5.0, doc="bar length along `axis`"),
        Field("axis", "enum", "y", choices=("x", "y"), doc="direction the bar lies along"),
        Field("bottom", "float", 0.228, unit="m", min=0.21, max=0.246,
              doc="underside height; measured window: crouched head 0.20 m, rolling head 0.256 m"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        self.length = float(c["length"])
        self.axis = c["axis"]
        self.bottom = float(c["bottom"])
        self.half = np.array([DEPTH / 2, self.length / 2]) if self.axis == "y" else \
            np.array([self.length / 2, DEPTH / 2])
        self.glide_ok = False          # a station in the room puts the roller on

    @classmethod
    def add_room_parts(cls, parts):
        ok = any(getattr(p, "addon", None) == "roller" and getattr(p, "mode", None) in ("attach", "toggle")
                 and hasattr(p, "max_speed") for p in parts.values())
        for p in parts.values():
            if isinstance(p, LowBar):
                p.glide_ok = ok

    def along(self) -> np.ndarray:
        """Unit vector the bar lies along."""
        return np.array([0.0, 1.0]) if self.axis == "y" else np.array([1.0, 0.0])

    def across(self) -> np.ndarray:
        """Unit vector across the bar (the way a body glides under it)."""
        return np.array([1.0, 0.0]) if self.axis == "y" else np.array([0.0, 1.0])

    def build(self, spec):
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, self.bottom + THICK / 2])
        body.add_geom(name=self.body_name + "/bar", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*self.half, THICK / 2], rgba=RGBA)
        return body

    def blockers(self, is_open):
        if self.glide_ok:
            return []
        a = self.along() * self.length / 2
        return [Segment(tuple(self.xy - a), tuple(self.xy + a), DEPTH)]

    def target_mask(self, grid):
        return grid.rect(self.xy, tuple(self.half))

    def check_warnings(self, env) -> list[str]:
        if not self.glide_ok:
            return [f"part {self.name!r}: low bar in a room without a station that puts the roller on "
                    f"(it blocks like a wall)"]
        return []
