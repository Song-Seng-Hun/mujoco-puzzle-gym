"""Devices the beak works (R28, category `beak`): lever and pull cord.

Each stands at `pos` and is worked from its `facing` side (deg; the direction
from the device to where the body stands). Those worked by a handle are held
by the beak weld (grab.py) at a small knob body on a ball joint
(`handle_body`), so the weld's turn rows cost nothing and only the knob's
position ties the head to the device. The body never walks backwards (vx
0.25-0.4 or standing for the default body).

- `lever`: an arm on a horizontal hinge. `<name>.angle` in -1..1. `detents`
  cells (default 3: -1, 0, 1; 0: continuous) pull the arm to the nearest cell
  (a joint spring whose rest angle is that cell, `detent` N m at the cell
  edge) and the channel gives that cell's value (`channel_cells`: the cannon's
  ballistic check aims per cell, R16).
  - `mode: push` (preset default): a floor lever whose paddle the body's chest
    pushes over by walking forward from the facing side, from -1 (leaning at
    the body) toward 1. The paddle starts 7 cm up, so the feet pass under it.
  - `mode: grab`: the arm stands up in the plane facing the body, the knob on
    top at `height`; hold it and step sideways: to the body's left raises the
    value. `max_angle` defaults to GRAB_MAX, what the weld held while stepping
    (plan.md E3b-2: holding a knob fixed in space stalls the sidestep).
- `pull_cord`: a cord (1D flexcomp, weightless: gravity compensated) hanging
  from a gallows arm, the knob at its end at `height`. The top of the cord is
  on a spring slider in the arm: pulling draws cord out. `<name>` turns on once
  `distance` m is drawn out (and stays on with `latch`). Hold the knob, turn
  round in place (clockwise: the body barely turns in place the other way),
  walk away.

The R28 `dial` and `drawer` are not built: holding a knob that the device keeps
on a fixed path stalls the body's sidestep (plan.md E3b-2 measurement).

Planning (R20): both are switches. Their target area is the work point (where
the body stands to work them); the scripted solver works them with the `demo:`
sub-goals `turn <lever> <value>` (push levers) and `pull <cord>` (carry.py),
and so does the grid answer when one is next in the solution order (lever to
1, cord pulled).
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from ...layout import Segment
from ...schema import Field, SchemaError
from ..base import Part
from ..object.grabbable import HANDLE_RGBA
from ..trap.base import yaw_quat

#: Default `max_angle` [deg] of a `grab` lever. Measured (2026-11-12, plan.md E3b-2): holding
#: the knob and stepping to the body's right turned the arm 9.5-15.7 deg before the weld
#: passed 3 N and let go; stepping left the body did not move (0.0-0.2 deg in 12 s).
GRAB_MAX = 10.0
POST_BACK = 0.06         # [m] grab lever: its post stands this far behind the arm
PUSH_MAX = 35.0          # [deg] default `max_angle` of a `push` lever
KNOB = 0.012             # [m] knob radius
WORK_REACH = 0.085       # [m] trunk centre -> beak tip, standing, level head (work point)
BODY_RGBA = (0.45, 0.45, 0.5, 1.0)
ARM_RGBA = (0.85, 0.55, 0.15, 1.0)


def _visual(**kw) -> dict:
    return {"contype": 0, "conaffinity": 0, **kw}


class HandleDevice(Part):
    """A device the body works (switch role, input stage); `grab_handle` ones carry a
    knob body the beak weld holds."""

    role = "switch"
    stage = "input"
    mass = 0.0                       # never lifted (grab.py `max_lift_mass` test)
    grab_handle = True
    fields = (
        Field("facing", "float", 0.0, unit="deg", min=-180.0, max=180.0,
              doc="direction from the device to where the body works it"),
        Field("grab_radius", "float", 0.04, unit="m", min=0.005, max=0.2,
              doc="the beak grabs the knob this close to its tip (R18)"),
        Field("max_hold_force", "float", 3.0, unit="N", min=0.01, max=100.0,
              doc="the grab lets go when holding the knob takes more force"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        a = math.radians(float(self.cfg["facing"]))
        self.f = np.array([math.cos(a), math.sin(a)])           # out of the work side
        self.left = np.array([self.f[1], -self.f[0]])           # the working body's left
        self.grab_radius = float(self.cfg["grab_radius"])
        self.max_hold_force = float(self.cfg["max_hold_force"])
        if self.grab_handle:
            self.needs_body("beak_grab")

    @classmethod
    def add_room_parts(cls, parts):
        from ...grab import add_beak

        if any(getattr(p, "grab_handle", False) for p in parts.values() if isinstance(p, HandleDevice)):
            add_beak(parts)

    def attach_body(self, spec, body):
        """The head does not collide with the device's moving parts (it works them)."""
        site = body.grab_site()
        if site is None:
            return
        head = spec.site(site).parent.name
        for b in self._moving:
            spec.add_exclude(bodyname1=head, bodyname2=b)

    # ---- the knob -------------------------------------------------------------------
    @property
    def handle_body(self) -> str:
        return self.body_name + "/knob"

    def _add_knob(self, parent, pos) -> None:
        knob = parent.add_body(name=self.handle_body, pos=list(pos))
        knob.add_joint(name=self.handle_body + "/ball", type=mujoco.mjtJoint.mjJNT_BALL, damping=0.0005)
        knob.add_geom(name=self.handle_body + "/g", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[KNOB, 0, 0],
                      mass=0.005, rgba=HANDLE_RGBA, **_visual())

    def bind(self, model, data):
        super().bind(model, data)
        if self.grab_handle:
            self.handle_bid = model.body(self.handle_body).id

    def handle_points(self, data) -> np.ndarray:
        return data.xpos[self.handle_bid][None, :].copy()

    def _joint(self, model, name):
        j = model.joint(name)
        return int(j.qposadr[0]), int(j.dofadr[0])

    def place(self, model, xy):
        self.xy = np.asarray(xy, dtype=float)
        model.body_pos[self._bid, :2] = self.xy

    # ---- planning -------------------------------------------------------------------
    def work_point(self) -> np.ndarray:
        """Where the body centre stands to work it (facing the knob)."""
        return self.handle_rest()[:2] + self.f * WORK_REACH

    def handle_rest(self) -> np.ndarray:
        raise NotImplementedError

    def target_mask(self, grid):
        return grid.disk(self.work_point(), 0.08)

    def satisfied(self):
        return self.bus.on(self.outputs[0])

    def state(self):
        return float(self.satisfied())


# ---- lever --------------------------------------------------------------------------------
class Lever(HandleDevice):
    object_kind = "lever"
    suffixes = (".angle",)
    state_keys = ("value",)
    fields = (
        Field("mode", "enum", "grab", choices=("grab", "push"),
              doc="grab: hold the knob, step sideways; push: walk into the paddle"),
        Field("detents", "int", 3, min=0, max=9, doc="cells the arm clicks into (0: continuous, else >= 2)"),
        Field("max_angle", "float", None, unit="deg", min=5.0, max=60.0,
              doc=f"arm turn either way (default: grab {GRAB_MAX:g}, push {PUSH_MAX:g})"),
        Field("start", "float", None, min=-1.0, max=1.0,
              doc="value on reset (default: grab 0, push -1)"),
        Field("length", "float", 0.12, unit="m", min=0.06, max=0.25, doc="pivot to knob / paddle top"),
        Field("height", "float", 0.19, unit="m", min=0.12, max=0.3, doc="grab: knob height"),
        Field("detent", "float", 0.03, unit="N m", min=0.0, max=1.0,
              doc="torque pulling to the nearest cell at its edge (continuous: friction)"),
    )

    def __init__(self, name, cfg):
        self.grab_handle = cfg.get("mode", "grab") == "grab"
        super().__init__(name, cfg)
        c = self.cfg
        self.mode = c["mode"]
        self.detents = int(c["detents"])
        if self.detents == 1:
            raise SchemaError(f"part {name!r}: detents must be 0 (continuous) or 2..9, got 1")
        deg = c["max_angle"] if c["max_angle"] is not None else (GRAB_MAX if self.grab_handle else PUSH_MAX)
        self.max = math.radians(float(deg))
        self.start = float(c["start"]) if c["start"] is not None else (0.0 if self.grab_handle else -1.0)
        self.length = float(c["length"])
        # value = sign * q / max: grab +q swings the knob to the working body's left;
        # push +q leans the paddle at the body
        self.sign = 1.0 if self.grab_handle else -1.0
        self.cells = np.linspace(-1.0, 1.0, self.detents) if self.detents else None
        self.value = self.start
        if self.cells is not None:
            self.value = float(self.cells[np.argmin(np.abs(self.cells - self.start))])

    @classmethod
    def add_room_parts(cls, parts):
        """A lever whose `.angle` only `bind`s read (a turret's aim) is a setting, not a
        switch: no `new_on` for pushing it to 1, no grid-answer target (R16, R28)."""
        super().add_room_parts(parts)
        for lv in parts.values():
            if not isinstance(lv, Lever):
                continue
            ch = lv.name + ".angle"
            binds = any(ch in p.watch for p in parts.values())
            reads = any(ch in [*p.inputs, *([p.enable] if p.enable else [])] for p in parts.values())
            if binds and not reads:
                lv.role = "prop"

    def channel_cells(self, channel: str):
        """Values `<name>.angle` can rest at (the cannon ballistic check aims per cell);
        None when continuous."""
        return self.cells if channel == self.name + ".angle" else None

    def build(self, spec):
        c = self.cfg
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, 0.0],
                                       quat=yaw_quat(float(c["facing"])))
        stiff = 0.0
        if self.detents:
            half = self.max / (self.detents - 1)          # half the cell spacing
            stiff = float(c["detent"]) / half
        jkw = dict(type=mujoco.mjtJoint.mjJNT_HINGE, range=[-math.degrees(self.max), math.degrees(self.max)],
                   limited=mujoco.mjtLimited.mjLIMITED_TRUE, damping=0.02, armature=0.002,
                   stiffness=[stiff, 0.0, 0.0], frictionloss=0.0 if self.detents else float(c["detent"]))
        if self.grab_handle:
            pivot = float(c["height"]) - self.length
            body.add_geom(name=self.body_name + "/post", type=mujoco.mjtGeom.mjGEOM_BOX,
                          size=[0.015, 0.015, pivot / 2], pos=[-POST_BACK, 0, pivot / 2], rgba=BODY_RGBA)
            arm = body.add_body(name=self.body_name + "/arm", pos=[0, 0, pivot])
            arm.add_joint(name=self.body_name + "/hinge", axis=[1, 0, 0], **jkw)
            arm.add_geom(name=self.body_name + "/axle", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                         size=[0.01, POST_BACK / 2], pos=[-POST_BACK / 2, 0, 0], quat=[math.sqrt(0.5), 0, math.sqrt(0.5), 0],
                         mass=0.005, rgba=BODY_RGBA, **_visual())
            arm.add_geom(name=self.body_name + "/rod", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                         fromto=[0, 0, 0, 0, 0, self.length - KNOB], size=[0.006, 0, 0], mass=0.02,
                         rgba=ARM_RGBA, **_visual())
            self._add_knob(arm, [0, 0, self.length])
            self._moving = [arm.name, self.handle_body]
        else:
            body.add_geom(name=self.body_name + "/base", type=mujoco.mjtGeom.mjGEOM_BOX,
                          size=[0.02, 0.08, 0.008], pos=[0, 0, 0.008], rgba=BODY_RGBA, **_visual())
            arm = body.add_body(name=self.body_name + "/arm", pos=[0, 0, 0.012])
            arm.add_joint(name=self.body_name + "/hinge", axis=[0, 1, 0], **jkw)
            arm.add_geom(name=self.body_name + "/rod", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                         fromto=[0, 0, 0, 0, 0, self.length], size=[0.006, 0, 0], mass=0.01,
                         rgba=ARM_RGBA, **_visual())
            lo = 0.07          # paddle from the trunk's height up (the feet pass under it)
            arm.add_geom(name=self.body_name + "/paddle", type=mujoco.mjtGeom.mjGEOM_BOX,
                         size=[0.008, 0.06, (self.length - lo) / 2], pos=[0, 0, (self.length + lo) / 2],
                         mass=0.02, rgba=ARM_RGBA)
            self._moving = [arm.name]
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._qa, self._va = self._joint(model, self.body_name + "/hinge")

    def _q_of(self, value: float) -> float:
        return self.sign * value * self.max

    def reset(self, model, data):
        data.qpos[self._qa] = self._q_of(self.start)
        data.qvel[self._va] = 0.0
        self._settle(model, data)

    def _settle(self, model, data) -> None:
        """Spring rest at the nearest cell; write the value."""
        v = self.sign * float(data.qpos[self._qa]) / self.max
        if self.cells is not None:
            v = float(self.cells[np.argmin(np.abs(self.cells - v))])
            model.qpos_spring[self._qa] = self._q_of(v)
        self.value = float(np.clip(v, -1.0, 1.0))
        self.write(self.value, self.name + ".angle")

    def update(self, model, data, body):
        self._settle(model, data)

    def load_state(self, model, data, st):
        super().load_state(model, data, st)
        self._settle(model, data)

    def satisfied(self):
        return self.value >= 0.5

    def handle_rest(self):
        c = self.cfg
        return np.array([*self.xy, float(c["height"]) if self.grab_handle else 0.12])

    def work_point(self):
        if self.grab_handle:
            return super().work_point()
        return self.xy + self.f * 0.35            # push: start of the walk into the paddle

    def blockers(self, is_open):
        if not self.grab_handle:          # the paddle's sweep (walking there pushes it)
            d = self.f * self.length * math.sin(self.max)
            return [Segment(tuple(self.xy - d), tuple(self.xy + d), 0.12)]
        p = self.xy - self.f * POST_BACK
        return [Segment(tuple(p - self.left * 0.015), tuple(p + self.left * 0.015), 0.03)]


# ---- pull cord ----------------------------------------------------------------------------
CORD_NODES = 9
GALLOWS_BACK = 0.12      # [m] post behind the cord
SLIDER_PRELOAD = 0.04    # [m] the spring's rest is this far above the slider's top stop
#: Default spring [N/m]. Measured (2026-11-12, plan.md E3b-2): with 2 N/m the turn and the
#: first steps drew 7.1-7.3 cm; at 4 N/m (0.26 N on the beak) walking stalled at 3.5-3.7 cm.
CORD_STIFFNESS = 2.0


class PullCord(HandleDevice):
    object_kind = "pull_cord"
    emits = True
    stateful = True
    state_keys = ("on",)
    fields = (
        Field("anchor", "float", 0.45, unit="m", min=0.3, max=1.0, doc="height the cord hangs from"),
        Field("height", "float", 0.19, unit="m", min=0.12, max=0.3, doc="knob height at rest"),
        Field("distance", "float", 0.06, unit="m", min=0.01, max=0.25,
              doc="cord drawn out of the arm that turns it on (default: 6 cm, a turn and a step)"),
        Field("stiffness", "float", CORD_STIFFNESS, unit="N/m", min=0.5, max=100.0,
              doc="spring pulling the cord back (stiffer: the body stalls pulling)"),
        Field("latch", "bool", True, doc="stays on once pulled (until reset)"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        self.anchor, self.h = float(c["anchor"]), float(c["height"])
        if self.anchor - self.h < 0.1:
            raise SchemaError(f"part {name!r}: anchor must be at least 0.1 m above height")
        self.length = self.anchor - self.h
        self.travel = float(c["distance"]) + 0.15
        self.on = False

    def build(self, spec):
        c = self.cfg
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, 0.0],
                                       quat=yaw_quat(float(c["facing"])))
        box = mujoco.mjtGeom.mjGEOM_BOX
        top = self.anchor + 0.03
        body.add_geom(name=self.body_name + "/post", type=box, size=[0.015, 0.015, top / 2],
                      pos=[-GALLOWS_BACK, 0, top / 2], rgba=BODY_RGBA)
        body.add_geom(name=self.body_name + "/arm", type=box, size=[GALLOWS_BACK / 2 + 0.02, 0.015, 0.015],
                      pos=[-GALLOWS_BACK / 2 + 0.02, 0, top], rgba=BODY_RGBA, **_visual())
        slider = body.add_body(name=self.body_name + "/slider", pos=[0, 0, self.anchor])
        slider.add_joint(name=self.body_name + "/slide", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 0, -1],
                         range=[0.0, self.travel], limited=mujoco.mjtLimited.mjLIMITED_TRUE,
                         stiffness=[float(c["stiffness"]), 0.0, 0.0], springref=-SLIDER_PRELOAD,
                         damping=0.5, armature=0.01)
        slider.add_geom(name=self.body_name + "/bob", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.008, 0, 0],
                        mass=0.005, rgba=BODY_RGBA, **_visual())
        n, L = CORD_NODES, self.length
        h = math.sqrt(0.5)
        cord = slider.make_flex(self.body_name + "/cord", type="grid", dim=1, count=[n, 1, 1],
                                spacing=[L / (n - 1)] * 3, radius=0.003, mass=0.01,
                                dof="full", equality=1, quat=[h, 0, h, 0], pos=[0, 0, -L / 2])
        cord.contype = cord.conaffinity = 0
        for eq in spec.equalities:                # the cord's edge constraint: little stretch
            if eq.type == mujoco.mjtEq.mjEQ_FLEX and eq.name1 == cord.name:
                eq.solref = [0.005, 1.0]
        cord.rgba = [0.9, 0.85, 0.7, 1.0]
        first = spec.body(f"{self.body_name}/cord_0")
        for j in list(first.joints):              # the top node rides the slider
            spec.delete(j)
        for i in range(n):                         # a light cord: the spring alone pulls it in
            spec.body(f"{self.body_name}/cord_{i}").gravcomp = 1.0
        end = spec.body(f"{self.body_name}/cord_{n - 1}")
        self._add_knob(end, [0, 0, 0])
        spec.body(self.handle_body).gravcomp = 1.0
        self._moving = [slider.name, *(f"{self.body_name}/cord_{i}" for i in range(n)), self.handle_body]
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._qa, self._va = self._joint(model, self.body_name + "/slide")

    def reset(self, model, data):
        data.qpos[self._qa] = 0.0
        data.qvel[self._va] = 0.0
        self.on = False
        self.write(0.0)

    def drawn(self, data) -> float:
        """Cord drawn out of the arm [m]."""
        return float(data.qpos[self._qa])

    def update(self, model, data, body):
        now = self.drawn(data) >= float(self.cfg["distance"]) - 1e-9
        self.on = (self.on and bool(self.cfg["latch"])) or now
        self.write(float(self.on))

    def handle_rest(self):
        return np.array([*self.xy, self.h])

    def blockers(self, is_open):
        p = self.xy - self.f * GALLOWS_BACK
        return [Segment(tuple(p - self.left * 0.015), tuple(p + self.left * 0.015), 0.03)]
