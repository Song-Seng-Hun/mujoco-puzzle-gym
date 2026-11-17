"""Beak grab (R18): a weld constraint between the beak and an object's handle.

Not a physical grip: the body model has no jaw joint, so a grab switches on a
weld (`eq_active`) between the head body carrying the beak-tip site
(`BodyAdapter.grab_site()`) and the object, keeping their relative pose at that
moment; the head and the objects do not collide. "Object" here is any
grabbable part (a part with handles: the cube, R17, and the R28 objects,
parts/object/grabbable.py) and the R28 devices worked by a handle (grab
lever, pull cord, drawer strap, parts/beak/devices.py), whose weld holds their handle
body (`handle_body`, a knob on a ball joint) instead of a free body. A room with them gets the hidden input part
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


#: Control steps a contact hold may lose its pinch before it counts as dropped (R121).
CONTACT_GRACE = 3


def add_beak(parts: dict[str, Part]) -> None:
    """Put the `_beak` part first in `parts` (once), holding every grabbable part; a later
    call adds grabbable parts added since (rope knots, R31)."""
    objs = [p for p in parts.values() if hasattr(p, "handle_points") and getattr(p, "grab_handle", True)]
    if BEAK_PART in parts:
        beak = parts[BEAK_PART]
        if not isinstance(beak, Beak):
            raise ValueError(f"part name {BEAK_PART!r} is reserved for the beak grab")
        for o in objs:
            if all(o is not q for q in beak.objects):
                beak.objects.append(o)
                beak.outputs += [o.name + ".held", o.name + ".dropped"]
        return
    rest = list(parts.items())
    parts.clear()
    beak = Beak(BEAK_PART, objs)
    if any(str(o.cfg.get("hold", "weld")) == "contact" for o in objs):
        # One object asking for a contact hold puts the whole beak in that mode: the mode
        # decides the scene (no weld, and no contact exclusion), so it cannot be per-object
        # at run time. A room that mixes them is refused below.
        beak.hold = "contact"
        mixed = [o.name for o in objs if str(o.cfg.get("hold", "weld")) != "contact"]
        if mixed:
            raise ValueError(f"`hold: contact` is all-or-nothing for a room's grabbables; "
                             f"these still ask for a weld: {mixed}")
    parts[BEAK_PART] = beak
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
        #: "weld" (the default, R18) or "contact" (R121): what holds a grabbed object.
        #:
        #: `weld` attaches the object to the head with an `eq_active` constraint once a
        #: handle is inside `grab_radius`. It is not a grip, and it is the single biggest
        #: thing between this package and hardware. `contact` holds nothing: the object
        #: stays put because two fingers are pressing it, and `held` is set from the
        #: contacts themselves. Measured on the arm in a bare scene (no weld in the scene
        #: at all): 10, 30 and 60 g lift 91-95 mm with friction 0.3 to 2.0.
        #:
        #: It needs a body with real fingers (`BodyAdapter.jaw_geoms`), so a walking body
        #: with a beak keeps the weld whatever a room asks for.
        self.hold = "weld"
        self._contact_lost = 0
        self.armed = True
        self.too_heavy = False
        self._site_name: str | None = None

    # ---- contact hold (R121) -----------------------------------------------------
    def _contact_mode(self, world) -> bool:
        return self.hold == "contact" and getattr(world.body, "jaw_geoms", None) is not None

    def _pinched(self, model, data, world, obj) -> bool:
        """True while both jaw sides touch `obj`: a pinch, not a brush."""
        sides = world.body.jaw_geoms(model)
        if not sides or len(sides) < 2:
            return False
        roots = set(obj.root_body_ids(model))
        if not roots:
            return False
        touched = [False] * len(sides)
        for i in range(data.ncon):
            c = data.contact[i]
            for g_a, g_b in ((int(c.geom1), int(c.geom2)), (int(c.geom2), int(c.geom1))):
                # The object's whole subtree, not just its root: a cube's handles are child
                # bodies, so that is what the fingers actually touch. Measured, matching the
                # root alone reported no pinch while both jaw sides had 8 and 7 contacts on
                # it (R124).
                body = int(model.geom_bodyid[g_b])
                if body not in roots and int(model.body_rootid[body]) not in roots:
                    continue
                for k, side in enumerate(sides):
                    if g_a in side:
                        touched[k] = True
        return all(touched)

    def _update_contact(self, model, data, world, cmd, dropped) -> None:
        """`held` from the fingers instead of from a weld. Nothing is attached: the object
        is held because it is being squeezed, so it can also be dropped by physics."""
        if self.held is not None:
            if cmd <= GRAB_ON:
                self.held, self._contact_lost = None, 0
            elif self._pinched(model, data, world, self.objects[self.held]):
                self._contact_lost = 0
            else:
                self._contact_lost += 1
                if self._contact_lost > CONTACT_GRACE:
                    dropped[self.held] = True
                    self.held, self._contact_lost = None, 0
            return
        if cmd <= GRAB_ON:
            self.armed = True
            return
        if not self.armed:
            return
        # Asking for a grab again with nothing held: back into the grasp pose. **Not** on
        # release - that moment is the middle of a `place`, and measured, turning the
        # orientation task back on there ended the place with the block outside the box in
        # 10 of 10 episodes (R125).
        world.body.set_grasp_pose(True)
        for i, obj in enumerate(self.objects):
            if getattr(obj, "grabbable_now", True) and self._pinched(model, data, world, obj):
                limit = world.body.capabilities.limit("max_lift_mass") if world.body else np.inf
                if obj.mass > limit:
                    self.too_heavy = True
                else:
                    self.held, self._contact_lost = i, 0
                    # The grasp pose has done its job; holding it through the carry keeps a
                    # row of the solve busy and the place suffers for it (R125): measured,
                    # `place` ended with the block outside the box in 4 of 4 episodes.
                    world.body.set_grasp_pose(False)
                return

    # ---- scene ------------------------------------------------------------------
    def attach_body(self, spec: mujoco.MjSpec, body) -> None:
        """One inactive weld per object between the head (the beak site's body) and the
        object, and no contacts between the two (a held handle sits at the beak tip)."""
        self._site_name = body.grab_site()
        if self._site_name is None:
            return
        head = spec.site(self._site_name).parent
        if self.hold == "contact":
            # Nothing to build: no weld, and crucially **no contact exclusion**. The weld
            # path excludes gripper-to-object contacts so a welded handle can sit inside
            # the jaw; leave that in and a pinch can never be detected, which is measured -
            # the rooms stalled on their first `grab` with the mode on at run time only.
            self._eq = []
            return
        for o in self.objects:
            spec.add_equality(name=f"{self.name}/{o.name}", type=mujoco.mjtEq.mjEQ_WELD,
                              objtype=mujoco.mjtObj.mjOBJ_BODY, name1=head.name, name2=o.handle_body,
                              active=False, solref=WELD_SOLREF, solimp=WELD_SOLIMP)
            spec.add_exclude(bodyname1=head.name, bodyname2=o.handle_body)

    def bind(self, model, data):
        if self._site_name is None:
            self._eq = []
            return
        self._site = model.site(self._site_name).id
        self._head = int(model.site_bodyid[self._site])
        self._eq = ([] if self.hold == "contact"
                    else [model.equality(f"{self.name}/{o.name}").id for o in self.objects])

    def reset(self, model, data):
        self.held, self.armed, self.too_heavy = None, True, False
        self._contact_lost = 0
        if self.world is not None and self.world.body is not None:
            # A contact hold needs the gripper held in a grasp pose, and a room has to stand
            # on its own: `mpg check` configures nothing (R125).
            self.world.body.set_grasp_pose(self.hold == "contact")
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
        weld_now(model, data, self._eq[i], self._head, obj.handle_bid)
        self.held = i
        self.world.held = {int(model.body_rootid[obj.handle_bid])}

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
            if not getattr(o, "grabbable_now", True):      # a latched drawer's strap (R28)
                continue
            d = float(np.min(np.linalg.norm(o.handle_points(data) - tip, axis=1)))
            if best is None or d < best[1]:
                best = (i, d)
        return best

    def update(self, model, data, body):
        w = self.world
        if w is None:
            return
        if not self._eq and self.hold != "contact":
            # No welds built (a body with no grab site) and no contact hold to run: there is
            # nothing for this part to do. A contact hold has an empty `_eq` by design, so
            # the old `not self._eq` test alone skipped its whole update - measured, both jaw
            # sides had 8 contacts each and `_pinched` returned True while `held` stayed None,
            # because this method never ran (R124).
            return
        cmd = float(w.grab)
        dropped = [False] * len(self.objects)
        if self._contact_mode(w):
            # Before the weld bookkeeping, not after: a contact hold has no welds, so
            # `weld_force` on an empty `_eq` raised straight past it (R124).
            self.too_heavy = False
            self._update_contact(model, data, w, cmd, dropped)
            got = self.objects[self.held] if self.held is not None else None
            w.tool_tip = (got.tip_point(data) if got is not None
                          and hasattr(got, "tip_point") else None)
            self._write(dropped)
            return
        if self.held is not None:
            i = self.held
            if self.weld_force(data, i) > self.objects[i].max_hold_force:
                dropped[i] = True
                self.armed = False
                self._release(data)
            elif cmd <= GRAB_ON or not getattr(self.objects[i], "grabbable_now", True):
                self._release(data)
        self.too_heavy = False
        if cmd <= GRAB_ON:
            self.armed = True
        elif self.held is None and self.armed:
            near = self.nearest(data)
            if near is not None and near[1] <= self.objects[near[0]].grab_radius:
                # A body that declares no lift limit gets 0, not unlimited: the grabber
                # is the body's own beak, so an undeclared limit means it cannot carry
                # (`bodies.declared_limit`). With no body at all there is no body limit.
                limit = w.body.capabilities.limit("max_lift_mass") if w.body else np.inf
                if self.objects[near[0]].mass > limit:
                    self.too_heavy = True
                else:
                    self._grab(model, data, near[0])
        held = self.objects[self.held] if self.held is not None else None
        w.tool_tip = held.tip_point(data) if held is not None and hasattr(held, "tip_point") else None
        self._write(dropped)
