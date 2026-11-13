"""Brake strip (R30): a floor strip that brakes the roller addon's wheels.

While the walking body's trunk is over the strip (`size` [x, y] around `pos`)
and the strip acts (trap rule: `in` on or no `in`, `enable`, `enabled`), the
wheel joints of the body (`BodyAdapter.wheel_dofs`) get `frictionloss` [N m]
and `damping` [N m s/rad] instead of their built values; when the body leaves
(or the strip stops) they go back.

Measured (2026-11-13, roller at 0.29-0.48 m/s, 5 seeds): `frictionloss` 0.01
stops the duck 0.08-0.12 m after the strip edge in 0.4-0.6 s with a zero push,
and 0.12-0.13 m while it keeps pushing, without a fall; 0.004 lets a push roll
on, 0.03 and any `damping` >= 0.002 tip it over from 0.44 m/s (a stop too
sudden). A duck stopped on a 0.01 strip cannot push off it on wheels: put a
detach station on the strip, or make the strip the end of the run. A rolling duck then stops on the strip, so
a strip just before a button or an addon station lets it stop there. Several
strips: the room's first strip sets the wheels from all of them (the largest
values of the strips the body is on). The values are model values
(`dof_frictionloss`, `dof_damping`) and are saved in snapshots
(snapshot.MODEL_FIELDS); `reset` restores the built ones. On feet the wheels
do not touch the floor, so the strip does nothing. The strip is drawn on the
floor (no collision).
"""

from __future__ import annotations

import mujoco
import numpy as np

from ...schema import Field
from ..input._sense import in_rect
from .base import Trap

RGBA_OFF = (0.55, 0.2, 0.15, 1.0)
RGBA_ON = (0.9, 0.3, 0.2, 1.0)


class BrakeStrip(Trap):
    fields = (
        Field("size", "vec", [0.3, 0.6], length=2, unit="m", min=0.05, max=5.0, doc="strip size [x, y]"),
        Field("frictionloss", "float", 0.01, unit="N m", min=0.0, max=1.0, runtime=True,
              doc="wheel joint friction while the body is on the strip"),
        Field("damping", "float", 0.0, unit="N m s/rad", min=0.0, max=1.0, runtime=True,
              doc="wheel joint damping while the body is on the strip"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.half = np.asarray(self.cfg["size"], float) / 2
        self.needs_body("walk", "roller")
        self.strips: list[BrakeStrip] = []     # the room's first strip: all of them
        self.braking = False
        self._dofs: np.ndarray | None = None
        self._base: tuple[np.ndarray, np.ndarray] | None = None
        self._gid = -1

    @classmethod
    def add_room_parts(cls, parts):
        strips = [p for p in parts.values() if isinstance(p, BrakeStrip)]
        if strips and not strips[0].strips:
            strips[0].strips = strips

    def build(self, spec):
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/strip", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*self.half, 0.002], pos=[0, 0, 0.002], rgba=RGBA_OFF,
                      contype=0, conaffinity=0)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._gid = model.geom(self.body_name + "/strip").id
        self._base = (model.dof_frictionloss.copy(), model.dof_damping.copy())
        self._dofs = None

    def reset(self, model, data):
        self.braking = False

    def on_body(self, body) -> bool:
        return body is not None and self.active() and in_rect(body.pos[:2], self.xy, self.half)

    def update(self, model, data, body):
        self.braking = self.on_body(body)
        model.geom_rgba[self._gid] = RGBA_ON if self.braking else RGBA_OFF
        if not self.strips or self.world is None or self.world.body is None:
            return
        if self._dofs is None:
            self._dofs = np.asarray(self.world.body.wheel_dofs(model), dtype=int)
        if not self._dofs.size:
            return
        on = [s for s in self.strips if (s.braking if s is self else s.on_body(body))]
        fl, dp = self._base
        if on:
            model.dof_frictionloss[self._dofs] = max(float(s.live["frictionloss"]) for s in on)
            model.dof_damping[self._dofs] = max(float(s.live["damping"]) for s in on)
        else:
            model.dof_frictionloss[self._dofs] = fl[self._dofs]
            model.dof_damping[self._dofs] = dp[self._dofs]

    def state(self):
        return float(self.braking)

    def target_mask(self, grid):
        return grid.rect(self.xy, tuple(self.half))

    def check_warnings(self, env) -> list[str]:
        if "roller" not in getattr(env.body, "addons", ()):
            return [f"part {self.name!r}: brake strip in a room without a roller addon station "
                    f"(no wheels to brake)"]
        return []
