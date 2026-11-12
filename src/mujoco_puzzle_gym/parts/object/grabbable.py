"""Grabbable objects (R17, R18, R28): free bodies the beak carries by a handle.

`Grabbable` is the common class of everything the beak grab (grab.py) can
hold: the cube (R17) and the R28 objects (laundry, key, handle ball, egg,
stick). A subclass builds its own geoms and sets `_local`, the handle centres
in its body frame (one row each); the beak grabs the handle nearest its tip
within `grab_radius`, unless `mass` is above the body's `max_lift_mass`, and
lets go when holding takes more than `max_hold_force`.

Planning (R20): input parts that only objects set off (pressure plates,
receptacles, object buttons and zones) count as switches when the room has a
grabbable object they can take, so the solution order passes them; the
scripted solver turns them on through the `demo:` sub-goals `grab <object>`
and `place <object> <part>` (carry.py).
"""

from __future__ import annotations

import mujoco
import numpy as np

from ...schema import Field, SchemaError
from ..base import TRIGGERS
from .ball import FreeObject

FACES = {"+x": (1.0, 0.0, 0.0), "-x": (-1.0, 0.0, 0.0), "+y": (0.0, 1.0, 0.0),
         "-y": (0.0, -1.0, 0.0), "+z": (0.0, 0.0, 1.0)}
HANDLE_RGBA = (0.3, 0.3, 0.35, 1.0)
HANDLE_HALF = 0.006      # [m] knob cross-section half side


def knobs(body, prefix: str, faces: list[str], local: np.ndarray, length: float) -> None:
    """Add a handle knob geom (no mass) per face at `local` (one row each)."""
    for f, pos in zip(faces, local):
        n = np.abs(FACES[f])
        size = np.where(n > 0, length / 2, HANDLE_HALF)
        body.add_geom(name=f"{prefix}/handle{f}", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=size.tolist(), pos=pos.tolist(), mass=0.0, rgba=HANDLE_RGBA)


def face_list(name: str, faces, allowed=tuple(FACES)) -> list[str]:
    bad = [f for f in faces if f not in allowed]
    if bad or not faces:
        raise SchemaError(f"part {name!r}: handles must be faces from {list(allowed)}, got {faces!r}")
    return list(dict.fromkeys(faces))


class Grabbable(FreeObject):
    #: Handle centres in the body frame (one row each), set by the subclass.
    _local: np.ndarray
    fields = (
        Field("mass", "float", 0.03, unit="kg", min=0.001, max=10.0,
              doc="above the body's max_lift_mass the beak cannot lift it"),
        Field("friction", "float", 1.0, min=0.0, max=2.0, doc="sliding friction"),
        Field("grab_radius", "float", 0.04, unit="m", min=0.005, max=0.2,
              doc="the beak grabs a handle this close to its tip (R18)"),
        Field("max_hold_force", "float", 3.0, unit="N", min=0.01, max=100.0,
              doc="the grab lets go when holding it takes more force"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.mass = float(self.cfg["mass"])
        self.grab_radius = float(self.cfg["grab_radius"])
        self.max_hold_force = float(self.cfg["max_hold_force"])
        self.needs_body("beak_grab")

    @property
    def grabbable_now(self) -> bool:
        """A parked object (consumed, or a dispenser's stored copy) is out of the grab."""
        return not self.parked

    @classmethod
    def add_room_parts(cls, parts):
        """Add the beak part (grab.py) in front, then let input parts an object can set
        off count as switches for planning (see the module docstring)."""
        from ...grab import add_beak

        add_beak(parts)
        objs = [p for p in parts.values() if isinstance(p, Grabbable)]
        for p in parts.values():
            if p.stage == "input" and p.role == "sensor" and p.plan_driver is None \
                    and any(takes(p, o) for o in objs):
                p.role = "switch"

    @property
    def handle_body(self) -> str:
        """Body the beak weld holds (grab.py); devices (R28) hold a handle body instead."""
        return self.body_name

    @property
    def handle_bid(self) -> int:
        return self._bid

    def handle_points(self, data) -> np.ndarray:
        """World centres of the handles (one row each)."""
        rot = data.xmat[self._bid].reshape(3, 3)
        return data.xpos[self._bid] + self._local @ rot.T


def takes(part, obj: Grabbable) -> bool:
    """Whether input part `part` can be set off by object `obj` (planning)."""
    if hasattr(part, "accepts"):                       # receptacle
        return part.accepts(obj)
    if "object" not in TRIGGERS.get(getattr(part, "trigger", None), ()):
        return False
    names = getattr(part, "names", None)
    if names and obj.name not in names:
        return False
    return obj.mass >= float(getattr(part, "live", {}).get("min_mass", 0.0))
