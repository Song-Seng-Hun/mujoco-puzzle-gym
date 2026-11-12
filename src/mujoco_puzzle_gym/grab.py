"""Beak grab (R18): a weld constraint between the beak and an object's handle.

Not a physical grip: the body model has no jaw joint, so a grab switches on a
weld (`eq_active`) between the head body carrying the beak-tip site
(`BodyAdapter.grab_site()`) and the object, keeping their relative pose at that
moment; the head and the objects do not collide. "Object" here is any
grabbable part (a part with handles: the cube, R17, and the R28 objects,
parts/object/grabbable.py). A room with them gets the hidden input part
`_beak` (first in the room order, so sensors see this body step's
`World.held`); it builds one inactive weld per object once the body is in the
scene (`attach_body`).

Every body step it reads the grab command `World.grab` (the env's `grab`
action, R19):
- above 0.5 and nothing held: the object whose handle is nearest the beak tip,
  within that object's `grab_radius`, is grabbed, unless its mass is above the
  body's `max_lift_mass` (then `beak.too_heavy` is 1 while it tries);
- 0.5 or below: the held object is let go;
- holding takes more than the object's `max_hold_force` (weld force): it is
  let go and `<object>.dropped` pulses; the beak grabs again only after the
  command went to 0.5 or below.
The held object's root body id is in `World.held`, so floor buttons, pressure
plates, zones and receptacles do not count it (R5). A held object with a
`tip_point` (the stick, R28) puts that point in `World.tool_tip`, where wall
buttons look for a press as for the beak tip.

Channels: `beak.holding`, `beak.held_id` (marker id of the held object, -1:
none or no marker), `beak.too_heavy`, and per object `<object>.held`,
`<object>.dropped`. Snapshots keep the weld through `eq_active` (MuJoCo state)
and its pose through `eq_data` (snapshot.MODEL_FIELDS).
"""

from __future__ import annotations

import mujoco
import numpy as np

from .parts.base import Part
from .schema import Field

BEAK_PART = "_beak"
GRAB_ON = 0.5
WELD_SOLREF = [0.02, 1.0]
WELD_SOLIMP = [0.95, 0.99, 0.001, 0.5, 2.0]


def weld_now(model, data, eq: int, parent: int, child: int) -> None:
    """Switch on weld `eq` (body1 `parent`, body2 `child`) at their present relative pose."""
    rp = data.xmat[parent].reshape(3, 3)
    q = np.zeros(4)
    mujoco.mju_mulQuat(q, np.array([data.xquat[parent][0], *-data.xquat[parent][1:]]), data.xquat[child])
    model.eq_data[eq, 3:6] = rp.T @ (data.xpos[child] - data.xpos[parent])
    model.eq_data[eq, 6:10] = q
    model.eq_data[eq, :3] = 0.0
    data.eq_active[eq] = 1


def add_beak(parts: dict[str, Part]) -> None:
    """Put the `_beak` part first in `parts` (once), holding every grabbable part."""
    if BEAK_PART in parts:
        if isinstance(parts[BEAK_PART], Beak):
            return
        raise ValueError(f"part name {BEAK_PART!r} is reserved for the beak grab")
    objs = [p for p in parts.values() if hasattr(p, "handle_points")]
    rest = list(parts.items())
    parts.clear()
    parts[BEAK_PART] = Beak(BEAK_PART, objs)
    parts.update(rest)


class Beak(Part):
    type_name = "beak"
    has_body = False
    stage = "input"
    stateful = True
    state_keys = ("held", "armed", "too_heavy")
    fields = (Field("pos", "vec", [0.0, 0.0], length=2, unit="m", doc="unused (no body)"),)

    def __init__(self, name: str, objects: list[Part]):
        super().__init__(name, {"out": ["beak.holding", "beak.held_id", "beak.too_heavy"]})
        self.objects = objects
        for o in objects:
            self.outputs += [o.name + ".held", o.name + ".dropped"]
        self.held: int | None = None          # index into objects
        self.armed = True
        self.too_heavy = False
        self._site_name: str | None = None

    # ---- scene ------------------------------------------------------------------
    def attach_body(self, spec: mujoco.MjSpec, body) -> None:
        """One inactive weld per object between the head (the beak site's body) and the
        object, and no contacts between the two (a held handle sits at the beak tip)."""
        self._site_name = body.grab_site()
        if self._site_name is None:
            return
        head = spec.site(self._site_name).parent
        for o in self.objects:
            spec.add_equality(name=f"{self.name}/{o.name}", type=mujoco.mjtEq.mjEQ_WELD,
                              objtype=mujoco.mjtObj.mjOBJ_BODY, name1=head.name, name2=o.body_name,
                              active=False, solref=WELD_SOLREF, solimp=WELD_SOLIMP)
            spec.add_exclude(bodyname1=head.name, bodyname2=o.body_name)

    def bind(self, model, data):
        if self._site_name is None:
            self._eq = []
            return
        self._site = model.site(self._site_name).id
        self._head = int(model.site_bodyid[self._site])
        self._eq = [model.equality(f"{self.name}/{o.name}").id for o in self.objects]

    def reset(self, model, data):
        self.held, self.armed, self.too_heavy = None, True, False
        if self.world is not None:
            self.world.tool_tip = None
        self._write([False] * len(self.objects))

    # ---- grabbing ---------------------------------------------------------------
    def _write(self, dropped: list[bool]) -> None:
        o = self.held
        held_id = -1
        if o is not None and self.objects[o].marker is not None:
            held_id = self.objects[o].marker.id
        self.write(float(o is not None), "beak.holding")
        self.write(float(held_id), "beak.held_id")
        self.write(float(self.too_heavy), "beak.too_heavy")
        for i, obj in enumerate(self.objects):
            self.write(float(i == o), obj.name + ".held")
            self.write(float(dropped[i]), obj.name + ".dropped")

    def weld_force(self, data, i: int) -> float:
        """Force [N] the weld of object `i` carries now (its three translational rows)."""
        rows = np.nonzero((data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY)
                          & (data.efc_id == self._eq[i]))[0][:3]
        return float(np.linalg.norm(data.efc_force[rows])) if len(rows) else 0.0

    def _grab(self, model, data, i: int) -> None:
        """Switch on object i's weld at the present relative pose (head -> object)."""
        obj = self.objects[i]
        weld_now(model, data, self._eq[i], self._head, obj._bid)
        self.held = i
        self.world.held = {int(model.body_rootid[obj._bid])}

    def _release(self, data) -> None:
        data.eq_active[self._eq[self.held]] = 0
        self.held = None
        self.world.held = set()

    def nearest(self, data) -> tuple[int, float] | None:
        """(object index, distance [m]) of the handle nearest the beak tip, or None."""
        if not self._eq:
            return None
        tip = data.site_xpos[self._site]
        best = None
        for i, o in enumerate(self.objects):
            d = float(np.min(np.linalg.norm(o.handle_points(data) - tip, axis=1)))
            if best is None or d < best[1]:
                best = (i, d)
        return best

    def update(self, model, data, body):
        w = self.world
        if w is None or not self._eq:
            return
        cmd = float(w.grab)
        dropped = [False] * len(self.objects)
        if self.held is not None:
            i = self.held
            if self.weld_force(data, i) > self.objects[i].max_hold_force:
                dropped[i] = True
                self.armed = False
                self._release(data)
            elif cmd <= GRAB_ON:
                self._release(data)
        self.too_heavy = False
        if cmd <= GRAB_ON:
            self.armed = True
        elif self.held is None and self.armed:
            near = self.nearest(data)
            if near is not None and near[1] <= self.objects[near[0]].grab_radius:
                limit = float(w.body.capabilities.values.get("max_lift_mass", np.inf)) if w.body else np.inf
                if self.objects[near[0]].mass > limit:
                    self.too_heavy = True
                else:
                    self._grab(model, data, near[0])
        held = self.objects[self.held] if self.held is not None else None
        w.tool_tip = held.tip_point(data) if held is not None and hasattr(held, "tip_point") else None
        self._write(dropped)
