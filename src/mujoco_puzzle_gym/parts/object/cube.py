"""Cube weight (R17): a free box with beak handles, carried by the weld grab (R18).

`cube` (kind "object", object kind "cube"): a box of edge `size` and `mass`
with a knob (`handle_size` long) on each face in `handles` (+x, -x, +y, -y,
+z; world axes at reset). The beak grabs the nearest knob within
`grab_radius` of its tip (grab.py); above the body's `max_lift_mass` it cannot
(`beak.too_heavy`), and a hold force above `max_hold_force` drops it
(`<name>.dropped`).

Reach (OQ2 measurement, default walking body): the beak tip is 0.135-0.21 m
above the floor and at most 0.085 m ahead of the trunk, so a cube on the floor
is out of reach. A cube on a stand 0.17-0.19 m high, flush with its edge, with
a side handle facing the body comes within 4 cm of the tip as the body walks
into the stand (8/8 seeds). In letter-map rooms a raised floor cell is such a
stand (the env rests free objects on the floor under them).

Planning (R20): input parts that only objects set off (pressure plates,
receptacles, object buttons and zones) count as switches when the room has a
cube they can take, so the solution order passes them; the scripted solver
turns them on through the `demo:` sub-goals `grab <cube>` and `place <cube>
<part>`.
"""

from __future__ import annotations

import mujoco
import numpy as np

from ... import markers
from ...schema import Field, SchemaError
from ..base import TRIGGERS
from .ball import FreeObject

FACES = {"+x": (1.0, 0.0, 0.0), "-x": (-1.0, 0.0, 0.0), "+y": (0.0, 1.0, 0.0),
         "-y": (0.0, -1.0, 0.0), "+z": (0.0, 0.0, 1.0)}
RGBA = (0.85, 0.55, 0.2, 1.0)
HANDLE_RGBA = (0.3, 0.3, 0.35, 1.0)
HANDLE_HALF = 0.006      # [m] handle cross-section half side


class Cube(FreeObject):
    object_kind = "cube"
    marked = True
    marker_anchor = "/box"
    marker_faces = markers.FACES_FLAT
    fields = (
        Field("size", "float", 0.04, unit="m", min=0.02, max=0.3, doc="edge length"),
        Field("mass", "float", 0.03, unit="kg", min=0.001, max=10.0,
              doc="above the body's max_lift_mass the beak cannot lift it"),
        Field("friction", "float", 1.0, min=0.0, max=2.0, doc="sliding friction"),
        Field("handles", "list", ["+x", "-x", "+y", "-y"],
              doc="faces with a beak handle: +x, -x, +y, -y, +z"),
        Field("handle_size", "float", 0.015, unit="m", min=0.005, max=0.1,
              doc="how far a handle sticks out of its face"),
        Field("grab_radius", "float", 0.04, unit="m", min=0.005, max=0.2,
              doc="the beak grabs a handle this close to its tip (R18)"),
        Field("max_hold_force", "float", 3.0, unit="N", min=0.01, max=100.0,
              doc="the grab lets go when holding it takes more force"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        bad = [f for f in c["handles"] if f not in FACES]
        if bad or not c["handles"]:
            raise SchemaError(f"part {name!r}: handles must be faces from {list(FACES)}, got {c['handles']!r}")
        self.size = float(c["size"])
        self.mass = float(c["mass"])
        self.handles = list(dict.fromkeys(c["handles"]))
        self.grab_radius = float(c["grab_radius"])
        self.max_hold_force = float(c["max_hold_force"])
        self.needs_body("beak_grab")
        out = self.size / 2 + float(c["handle_size"]) / 2
        self._local = np.array([np.asarray(FACES[f]) * out for f in self.handles])

    def marker_face_size(self, face):
        return self.size

    @classmethod
    def add_room_parts(cls, parts):
        """Add the beak part (grab.py) in front, then let input parts a cube can set
        off count as switches for planning (see the module docstring)."""
        from ...grab import add_beak

        add_beak(parts)
        cubes = [p for p in parts.values() if isinstance(p, Cube)]
        for p in parts.values():
            if p.stage == "input" and p.role == "sensor" and p.plan_driver is None \
                    and any(takes(p, c) for c in cubes):
                p.role = "switch"

    def _rest_z(self) -> float:
        return self.size / 2

    def build(self, spec):
        h = self.size / 2
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, h])
        body.add_freejoint(name=self.body_name + "/free")
        body.add_geom(name=self.body_name + "/box", type=mujoco.mjtGeom.mjGEOM_BOX, size=[h, h, h],
                      mass=self.mass, friction=[self.cfg["friction"], 0.005, 0.0001], rgba=RGBA)
        ln = float(self.cfg["handle_size"]) / 2
        for f, pos in zip(self.handles, self._local):
            n = np.abs(FACES[f])
            size = np.where(n > 0, ln, HANDLE_HALF)
            body.add_geom(name=f"{self.body_name}/handle{f}", type=mujoco.mjtGeom.mjGEOM_BOX,
                          size=size.tolist(), pos=pos.tolist(), mass=0.0, rgba=HANDLE_RGBA)
        return body

    def handle_points(self, data) -> np.ndarray:
        """World centres of the handles (one row each)."""
        rot = data.xmat[self._bid].reshape(3, 3)
        return data.xpos[self._bid] + self._local @ rot.T


def takes(part, cube: Cube) -> bool:
    """Whether input part `part` can be set off by `cube` (planning)."""
    if hasattr(part, "accepts"):                       # receptacle
        return part.accepts(cube)
    if "object" not in TRIGGERS.get(getattr(part, "trigger", None), ()):
        return False
    names = getattr(part, "names", None)
    if names and cube.name not in names:
        return False
    return cube.mass >= float(getattr(part, "live", {}).get("min_mass", 0.0))
