"""Hitch station and anchor post (R31): where tow ropes (rope.py) tie and come loose.

`hitch_station`: a floor pad. With the walking body's trunk inside it, slower than
`max_speed` (default 0.1 m/s, as the addon station, R30), the signal ties the body to
the nearest other hitch within the rope's `hitch_radius` of the body's hitch (`attach`;
the rope end already on that hitch stays, the other end moves to the body; a rope
already on the body with its other end loose ties that end) or unties the body's end
(`detach`); `toggle` does whichever applies (attach unless the rope ties the body to
something). A hitch farther than the
rope's length pulses `<rope>.attach_failed` and ties nothing. Signal: with `in`
channels, their turning on (later while they stay on, once the body is on the pad and
slow); without, entering the pad. `enable` off holds it; after a change it waits
`cooldown` seconds. `rope` names the rope it works (default: the room's only rope).
Output `<name>.towing`: 1 while its rope is tied to the body.

`anchor`: a fixed post a rope ties to (a hitch, kind "anchor") at `hitch_height` above
the floor. Drawn only: the body walks through it. Warps untie a rope from its anchor
before taking the rest along (rope.py).
"""

from __future__ import annotations

import mujoco
import numpy as np

from ... import markers
from ...schema import Field, SchemaError
from ..base import Part
from ..input._sense import in_rect

MODES = ("attach", "detach", "toggle")
RGBA_OFF = (0.4, 0.3, 0.2, 1.0)
RGBA_ON = (0.95, 0.75, 0.2, 1.0)
POST_RGBA = (0.45, 0.35, 0.25, 1.0)


class HitchStation(Part):
    marked = True
    marker_anchor = "/pad"
    marker_faces = markers.FACES_FLAT
    stage = "device"
    suffixes = (".towing",)
    state_keys = ("armed", "inside", "was_on", "until", "uses")
    fields = (
        Field("size", "vec", [0.3, 0.3], length=2, unit="m", min=0.2, max=2.0, doc="pad size [x, y]"),
        Field("rope", "str", None, doc="the rope it ties (default: the room's only rope)"),
        Field("mode", "enum", "attach", choices=MODES, doc="attach, detach or toggle"),
        Field("cooldown", "float", 1.0, unit="s", min=0.0, max=60.0,
              doc="time after a change before the station works again"),
        Field("max_speed", "float", 0.1, unit="m/s", min=0.01, max=1.0,
              doc="the body must be slower than this (horizontal)"),
    )

    def marker_face_size(self, face):
        return float(min(self.cfg["size"])) * 0.6

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        self.half = np.asarray(c["size"], float) / 2
        self.mode = c["mode"]
        self.cooldown = float(c["cooldown"])
        self.max_speed = float(c["max_speed"])
        self.rope_name: str | None = c["rope"]
        self.rope = None
        self._geoms: list[int] = []
        self.reset(None, None)

    @classmethod
    def add_room_parts(cls, parts):
        from .rope import Rope

        ropes = [p for p in parts.values() if isinstance(p, Rope)]
        for st in [p for p in parts.values() if isinstance(p, HitchStation)]:
            if st.rope_name is None:
                if len(ropes) != 1:
                    raise SchemaError(f"part {st.name!r}: name its `rope` (the room has {len(ropes)} ropes)")
                st.rope_name = ropes[0].name
            elif not isinstance(parts.get(st.rope_name), Rope):
                raise SchemaError(f"part {st.name!r}: rope {st.rope_name!r} is not a rope "
                                  f"(ropes: {[r.name for r in ropes]})")
            st.rope = parts[st.rope_name]

    def build(self, spec):
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/pad", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*self.half, 0.003], pos=[0, 0, 0.003], rgba=RGBA_OFF, contype=0, conaffinity=0)
        r = min(self.half) * 0.3
        body.add_geom(name=self.body_name + "/ring", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[r, 0.0035], pos=[0, 0, 0.0035], rgba=(0.15, 0.12, 0.1, 1.0),
                      contype=0, conaffinity=0)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._geoms = [model.geom(self.body_name + "/pad").id]

    def reset(self, model, data):
        self.armed = False
        self.inside = False
        self.was_on = False
        self.until = -np.inf
        self.uses = 0

    def update(self, model, data, body):
        rope = self.rope
        towing = bool(rope is not None and rope.on_body)
        self.write(float(towing), self.name + ".towing")
        for g in self._geoms:
            model.geom_rgba[g] = RGBA_ON if towing else RGBA_OFF
        inside = body is not None and in_rect(body.pos[:2], self.xy, self.half)
        if self.inputs:
            on = self.inputs_on()
            if on and not self.was_on:
                self.armed = True
            elif not on:
                self.armed = False
            self.was_on = on
        elif inside and not self.inside:
            self.armed = True
        elif not inside:
            self.armed = False
        self.inside = inside
        if not (self.armed and inside and self.enabled()) or rope is None:
            return
        if data.time < self.until or float(np.hypot(*body.lin_vel[:2])) >= self.max_speed:
            return
        self.armed = False
        full = towing and None not in rope.tied      # the body tows something (a loose far end does not)
        attach = self.mode == "attach" or (self.mode == "toggle" and not full)
        if (attach and full) or (not attach and not towing):     # nothing to change
            return
        done = rope.station_attach(model, data) if attach else rope.station_detach(model, data)
        if done:
            self.until = data.time + self.cooldown
            self.uses += 1
            self.write(float(rope.on_body), self.name + ".towing")

    def state(self):
        return float(self.rope is not None and self.rope.on_body)

    def target_mask(self, grid):
        return grid.rect(self.xy, tuple(np.maximum(self.half - 0.05, 0.03)))


class Anchor(Part):
    """A fixed rope hitch (R31)."""
    is_anchor = True
    hitch_kind = "anchor"         # a fixed rope hitch (rope.py `hitch_parts`)
    stage = "device"
    fields = (
        Field("hitch_height", "float", 0.04, unit="m", min=0.01, max=0.5,
              doc="where the rope ties, above the floor (the body's hitch is at 0.04 m)"),
        Field("height", "float", 0.08, unit="m", min=0.02, max=0.5, doc="drawn post height"),
    )

    def hitch_offset(self) -> list[float]:
        return [0.0, 0.0, float(self.cfg["hitch_height"])]

    def build(self, spec):
        body = super().build(spec)
        h = float(self.cfg["height"])
        body.add_geom(name=self.body_name + "/post", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[0.015, h / 2, 0], pos=[0, 0, h / 2], rgba=POST_RGBA, contype=0, conaffinity=0)
        body.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.04, 0.003, 0], pos=[0, 0, 0.003],
                      rgba=POST_RGBA, contype=0, conaffinity=0)
        return body
