"""Wind (R16): a constant push on everything in a box zone.

While active (trap base), each body or object (`trigger`, default `any`)
whose centre is inside the zone (`size` [x, y] around `pos`, up to `height`
above the floor) gets the force `power` [N] in direction `yaw` (deg, 0 = +x)
through `xfrc_applied` on its root body (the walking body's trunk). Every
wind adds its share and takes it back when the body leaves or the wind stops,
so overlapping winds add up. The zone is drawn as a faint box (no collision).
"""

from __future__ import annotations

import mujoco
import numpy as np

from ...schema import Field
from ..base import TRIGGERS
from ..input._sense import BODY_KEY, candidates, in_rect
from .base import Trap, heading

RGBA = (0.6, 0.85, 1.0, 0.12)


class Wind(Trap):
    state_keys = ("applied",)
    fields = (
        Field("size", "vec", [0.6, 0.6], length=2, unit="m", min=0.05, max=5.0, doc="zone size [x, y]"),
        Field("height", "float", 0.4, unit="m", min=0.05, max=2.0, doc="zone height above the floor"),
        Field("yaw", "float", 0.0, unit="deg", min=-180.0, max=180.0, runtime=True,
              doc="direction the wind blows (0: +x, 90: +y)"),
        Field("power", "float", 1.0, unit="N", min=0.0, max=20.0, runtime=True,
              doc="force on each body or object in the zone"),
        Field("trigger", "enum", "any", choices=tuple(TRIGGERS), doc="what it pushes"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.half = np.asarray(self.cfg["size"], float) / 2
        self.height = float(self.cfg["height"])
        self.trigger = self.cfg["trigger"]
        self.applied: dict[int, np.ndarray] = {}

    def build(self, spec):
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/zone", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*self.half, self.height / 2], pos=[0, 0, self.height / 2],
                      rgba=RGBA, contype=0, conaffinity=0, mass=0.0)
        return body

    def reset(self, model, data):
        self.applied = {}

    def force(self) -> np.ndarray:
        return np.array([*heading(float(self.live["yaw"])) * float(self.live["power"]), 0.0])

    def update(self, model, data, body):
        if self.world is None:
            return
        want: dict[int, np.ndarray] = {}
        if self.active():
            f = self.force()
            for key, pos in candidates(self, data, body, self.trigger):
                if not in_rect(pos, self.xy, self.half) or pos[2] > self.height:
                    continue
                rid = (next(r for r, k in self.world.kinds.items() if k == "body")
                       if key == BODY_KEY else int(key))
                want[rid] = f
        zero = np.zeros(3)
        for rid in set(self.applied) | set(want):
            data.xfrc_applied[rid, :3] += want.get(rid, zero) - self.applied.get(rid, zero)
        self.applied = want

    def state(self):
        return float(bool(self.applied))
