"""Laser sensor (R5): 1 while something `trigger` counts breaks the beam.

The beam runs from the emitter post at `pos` to the receiver post at `to`, at
`height` above the floor, and is drawn as a thin red line (no collision).
Every body step one ray (`mj_ray`) is cast along it; the nearest hit counts
when it belongs to the walking body, an object or an NPC as `trigger` says
(held objects excluded). Anything else in the way (a wall, a door) hides the
beam without turning the sensor on. `to` moves with `pos` in random layouts.
"""

from __future__ import annotations

import mujoco
import numpy as np

from ...schema import Field
from ..base import TRIGGERS, Part

POST = 0.02          # half size of the emitter and receiver posts [m]
BEAM_RGBA = (1.0, 0.1, 0.1, 0.6)


class Laser(Part):
    role = "switch"
    stage = "input"
    emits = True
    state_keys = ("on",)
    fields = (
        Field("to", "vec", length=2, unit="m", doc="receiver position [x, y]"),
        Field("height", "float", 0.05, unit="m", min=0.01, max=0.5, doc="beam height"),
        Field("trigger", "enum", "any", choices=tuple(TRIGGERS)),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.rel = np.asarray(self.cfg["to"], dtype=float) - self.base_xy
        self.length = float(np.linalg.norm(self.rel))
        if self.length < 4 * POST:
            raise ValueError(f"part {name!r}: `to` must be at least {4 * POST:g} m from `pos`")
        self.height = float(self.cfg["height"])
        self.trigger = self.cfg["trigger"]
        if "body" not in TRIGGERS[self.trigger]:
            self.role = "sensor"
        self.needs_body("walk" if self.role == "switch" else "beak_grab")
        self.on = False
        self.hit: str | None = None

    def build(self, spec):
        body = super().build(spec)
        h = self.height
        for tag, at in (("emitter", [0.0, 0.0]), ("receiver", list(self.rel))):
            body.add_geom(name=f"{self.body_name}/{tag}", type=mujoco.mjtGeom.mjGEOM_BOX,
                          size=[POST, POST, (h + POST) / 2], pos=[*at, (h + POST) / 2],
                          rgba=(0.3, 0.3, 0.3, 1.0), contype=0, conaffinity=0)
        body.add_geom(name=self.body_name + "/beam", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                      fromto=[0.0, 0.0, h, *self.rel, h], size=[0.003, 0, 0], rgba=BEAM_RGBA,
                      contype=0, conaffinity=0)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._gid = model.geom(self.body_name + "/beam").id
        self._geomid = np.zeros((1, 1), dtype=np.int32)

    def reset(self, model, data):
        self.on, self.hit = False, None
        for ch in self.outputs:
            self.write(0.0, ch)

    def blocked(self, model, data) -> bool:
        d = self.rel / self.length
        start = np.array([*(self.xy + d * POST), self.height])
        vec = np.array([*d, 0.0])
        dist = mujoco.mj_ray(model, data, start, vec, None, 1, self._bid, self._geomid)
        self.hit = None
        if dist < 0 or dist > self.length - 2 * POST:
            return False
        bid = int(model.geom_bodyid[self._geomid[0, 0]])
        rid = int(model.body_rootid[bid])
        world = self.world
        if world is None or rid in world.held:
            return False
        kind = world.kinds.get(rid)
        self.hit = kind
        return kind in TRIGGERS[self.trigger]

    def update(self, model, data, body):
        b = self.blocked(model, data)
        if not self.enabled():
            return
        if b != self.on:
            self.on = b
            model.geom_rgba[self._gid] = (0.2, 1.0, 0.3, 0.8) if b else BEAM_RGBA
            for ch in self.outputs:
                self.write(float(b), ch)

    def state(self):
        return float(self.on)

    def satisfied(self):
        return self.on

    def target_mask(self, grid):
        """Cells along the beam (where the body must stand to break it)."""
        p0, d = self.xy, self.rel
        px, py = grid.X - p0[0], grid.Y - p0[1]
        t = np.clip((px * d[0] + py * d[1]) / (self.length ** 2), 0.15, 0.85)
        return np.hypot(px - t * d[0], py - t * d[1]) <= 0.05
