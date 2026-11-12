"""Pressure plate (R5): 1 while the mass resting on it is at least `min_mass`.

A thin raised plate that collides. Every body step it sums the vertical
component of the contact forces between the plate and the bodies `trigger`
counts (default objects only, so standing on it does not solve a weight
puzzle), divided by g. Objects the walking body holds do not count.

NPC robots (R29) have no vertical joint and press nothing on the floor, so a
robot whose centre is over the plate adds its `mass` when `trigger` counts npc.

It reads the forces at `update` time, not the peak over the physics steps in
between: a peak reads a 30 g cube dropped from 5 cm as 0.35 kg (spec R5), so
it does not use the `substep` hook.
"""

from __future__ import annotations

import mujoco
import numpy as np

from ... import markers
from ...schema import Field
from ..base import TRIGGERS, Part
from ._sense import in_rect

THICK = 0.004        # plate top above the floor [m]
OFF_RGBA = (0.55, 0.55, 0.6, 1.0)
ON_RGBA = (0.2, 0.8, 0.3, 1.0)


class PressurePlate(Part):
    role = "sensor"
    marked = True
    marker_anchor = "/plate"
    marker_faces = markers.FACES_FLAT
    stage = "input"
    emits = True
    state_keys = ("on", "mass")
    fields = (
        Field("size", "vec", [0.25, 0.25], length=2, unit="m", min=0.05, max=2.0),
        Field("min_mass", "float", 0.1, unit="kg", min=0.001, max=100.0, runtime=True,
              doc="mass needed to turn on"),
        Field("trigger", "enum", "object", choices=tuple(TRIGGERS)),
    )

    def marker_face_size(self, face):
        return float(min(self.cfg["size"]))

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.half = np.asarray(self.cfg["size"], dtype=float) / 2
        self.trigger = self.cfg["trigger"]
        if "body" in TRIGGERS[self.trigger]:
            self.role = "switch"
        self.needs_body("walk" if self.role == "switch" else "beak_grab")
        self.on = False
        self.mass = 0.0
        self._f = np.zeros(6)

    def build(self, spec):
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/plate", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*self.half, THICK / 2], pos=[0, 0, THICK / 2], rgba=OFF_RGBA)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._gid = model.geom(self.body_name + "/plate").id

    def reset(self, model, data):
        self.on, self.mass = False, 0.0
        model.geom_rgba[self._gid] = OFF_RGBA
        for ch in self.outputs:
            self.write(0.0, ch)

    def load_mass(self, model, data) -> float:
        """Mass [kg] the counted bodies rest on the plate with now."""
        world = self.world
        if world is None:
            return 0.0
        kinds = TRIGGERS[self.trigger]
        total = 0.0
        for i in range(data.ncon):
            c = data.contact[i]
            if c.efc_address < 0 or self._gid not in (c.geom1, c.geom2):
                continue
            other = c.geom2 if c.geom1 == self._gid else c.geom1
            rid = int(model.body_rootid[model.geom_bodyid[other]])
            if world.kinds.get(rid) not in kinds or rid in world.held:
                continue
            mujoco.mj_contactForce(model, data, i, self._f)
            force = c.frame.reshape(3, 3).T @ self._f[:3]
            total += abs(force[2])
        total /= abs(model.opt.gravity[2])
        if "npc" in kinds:
            for rid, owner in world.objects.items():
                if world.kinds.get(rid) == "npc" and in_rect(data.xpos[rid], self.xy, self.half):
                    total += owner.mass
        return total

    def update(self, model, data, body):
        self.mass = self.load_mass(model, data)
        if not self.enabled():
            return
        on = self.mass >= self.live["min_mass"]
        if on != self.on:
            self.on = on
            model.geom_rgba[self._gid] = ON_RGBA if on else OFF_RGBA
            for ch in self.outputs:
                self.write(float(on), ch)

    def state(self):
        return float(self.on)

    def satisfied(self):
        return self.on

    def target_mask(self, grid):
        return grid.rect(self.xy, tuple(self.half))
