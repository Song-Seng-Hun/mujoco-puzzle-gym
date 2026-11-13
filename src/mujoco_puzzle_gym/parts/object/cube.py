"""Cube weight (R17): a free box with beak handles, carried by the weld grab (R18).

`cube` (kind "object", object kind "cube"): a box of edge `size` and `mass`
with a knob (`handle_size` long) on each face in `handles` (+x, -x, +y, -y,
+z; world axes at reset). The beak grabs the nearest knob within
`grab_radius` of its tip (grab.py); above the body's `max_lift_mass` it cannot
(`beak.too_heavy`), and a hold force above `max_hold_force` drops it
(`<name>.dropped`). Common grab values and planning: grabbable.py.

Stacking (R31 `overload`): `rest_on: <cube>` starts it resting on that cube's top face
at every reset, its `pos` kept relative to that cube's (so it moves with that
cube's random range; its own `random` must be 0). The cube under it must not be
stacked itself, and the centre must be over its top face. Its box gets contact
priority REST_PRIORITY (as a hitch object's, rope.py): on a hitch cube (priority 2) the
larger of the two frictions holds, not the lower one alone.

Reach (OQ2 measurement, default walking body): the beak tip is 0.135-0.21 m
above the floor and at most 0.085 m ahead of the trunk, so a cube on the floor
is out of reach. A cube on a stand 0.17-0.19 m high, flush with its edge, with
a side handle facing the body comes within 4 cm of the tip as the body walks
into the stand (8/8 seeds). In letter-map rooms a raised floor cell is such a
stand (the env rests free objects on the floor under them).
"""

from __future__ import annotations

import mujoco
import numpy as np

from ... import markers
from ...schema import Field, SchemaError
from .grabbable import FACES, Grabbable, face_list, knobs, takes

__all__ = ["Cube", "FACES", "takes"]

REST_PRIORITY = 2       # contact priority of a `rest_on` cube's box (docstring)

RGBA = (0.85, 0.55, 0.2, 1.0)


class Cube(Grabbable):
    object_kind = "cube"
    marked = True
    marker_anchor = "/box"
    marker_faces = markers.FACES_FLAT
    fields = (
        Field("size", "float", 0.04, unit="m", min=0.02, max=0.3, doc="edge length"),
        Field("handles", "list", ["+x", "-x", "+y", "-y"],
              doc="faces with a beak handle: +x, -x, +y, -y, +z"),
        Field("handle_size", "float", 0.015, unit="m", min=0.005, max=0.1,
              doc="how far a handle sticks out of its face"),
        Field("rest_on", "str", None, doc="start resting on this cube's top (pos relative to it; random must be 0)"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        self.size = float(c["size"])
        self.handles = face_list(name, c["handles"])
        out = self.size / 2 + float(c["handle_size"]) / 2
        self._local = np.array([np.asarray(FACES[f]) * out for f in self.handles])
        self.rest_on = c.get("rest_on")
        if self.rest_on is not None and np.any(self.random != 0):
            raise SchemaError(f"part {name!r}: a cube with `rest_on` starts on another: `random` must be [0, 0] "
                              f"(it moves with {self.rest_on!r})")

    @classmethod
    def add_room_parts(cls, parts):
        super().add_room_parts(parts)
        for p in parts.values():
            if not isinstance(p, Cube) or p.rest_on is None:
                continue
            base = parts.get(p.rest_on)
            if not isinstance(base, Cube) or base.rest_on is not None or base is p:
                raise SchemaError(f"part {p.name!r}: rest_on: {p.rest_on!r} must name another cube that is not "
                                  f"stacked itself")
            d = np.abs(p.base_xy - base.base_xy)
            if (d > base.size / 2 + 1e-9).any():
                raise SchemaError(f"part {p.name!r}: its centre is {d.max():.3f} m off the middle of "
                                  f"{base.name!r}, outside its top face (half size {base.size / 2:g} m)")

    def top(self) -> float:
        """Height of the top face above the cube's floor [m] (cubes with `rest_on` it rest there)."""
        return self.size

    def extent(self) -> tuple[float, float, float]:
        """(x, y, z) size [m] with the handles, at the reset orientation (pit fit, R16)."""
        hs = float(self.cfg["handle_size"])
        return tuple(self.size + hs * sum(f in self.handles for f in (f"+{a}", f"-{a}")) for a in "xyz")

    def marker_face_size(self, face):
        return self.size

    def _rest_z(self) -> float:
        return self.size / 2

    def build(self, spec):
        h = self.size / 2
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, h])
        body.add_freejoint(name=self.body_name + "/free")
        body.add_geom(name=self.body_name + "/box", type=mujoco.mjtGeom.mjGEOM_BOX, size=[h, h, h],
                      mass=self.mass, friction=[self.cfg["friction"], 0.005, 0.0001], rgba=RGBA,
                      priority=REST_PRIORITY if self.rest_on else 0)
        knobs(body, self.body_name, self.handles, self._local, float(self.cfg["handle_size"]))
        return body
