"""Receptacle (R5): a box or socket that signals when the right things are put in.

Shapes:
- `box`:   an open-top box (`size`, walls `wall_height` high). An object is in
  when its centre is inside the walls and lower than `wall_height` + 0.1 m.
- `plate`: a socket base plate. An object is in when its centre is within
  `tolerance` of the plate centre and its heading is within `angle_tolerance`
  of `yaw` (modulo `angle_period`: 90 degrees for a cube).
- `hook`: a post at `pos` with a peg pointing `facing` (degrees, 0 = +x),
  its tip `peg` m out from the post at `height`. A grabbable object (R28)
  that is not held and has a handle within `tolerance` of the peg tip is hung
  there: a weld to the post at that moment's pose, so it stays put. The beak
  takes it off again by grabbing it (the hook lets go of held objects). Hung
  objects are in. Peg and post do not collide with objects (the hook is a
  catch, like the beak grab, not a physical peg in a loop).
Socket, hook and basket presets build on this one part.

`accept`: object part names, object kinds (`key`, `egg`, ...) or marker ids
(numbers: an object with that marker, R26; a key in "the socket with the same
number"); empty accepts every object. Objects the body holds are never in
(spec R5). A receptacle may show the marker id it accepts on itself.

`require` (or the mapping form `require: {count: 2}`):
- `all`:   every `accept` entry is matched by an object in (empty: one object);
- `count`: at least `count` accepted objects are in;
- `mass`:  accepted objects in weigh at least `mass` kg;
- `order`: accepted objects went in in the order `order` lists and are all in.

`wrong` (an object that is not accepted goes in; with `order`, an accepted
object out of turn counts as wrong too):
- `ignore`: nothing happens;
- `eject`:  the object is lifted over the wall and thrown out (`eject_speed`
  m/s); a stand-in for the tilting floor of the spec;
- `lock`:   the lid closes, nothing more goes in;
- `fail`:   the episode fails.
`close_on_ok: true` closes the lid when the condition is first met. The lid is
a mocap body, so its state is part of `mjSTATE_INTEGRATION` snapshots.

`consume: true` (R28): an accepted object that comes in vanishes (a
dispenser's copy goes back to that dispenser's storage, in stock again with
its `recycle`; any other object is parked PARK_Z below where it was) and
stays counted: `count`, `mass`, `all` and `order` add up everything consumed
so far (a copy taken twice counts twice). Wrong objects are not consumed.

Channels: `<name>.ok`, `<name>.count` (accepted objects in, and consumed), `<name>.wrong`
(1 while a wrong object is in, and from a `lock` on), `<name>.mass` (mass of
the accepted objects in [kg]; the `basket` preset, R28).
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from ... import markers
from ...schema import Field, SchemaError
from ..base import Part

REQUIRES = ("all", "count", "mass", "order")
WRONGS = ("ignore", "eject", "lock", "fail")
LID_HIDDEN_Z = -1.0
RGBA = (0.6, 0.45, 0.25, 1.0)
RGBA_OK = (0.25, 0.75, 0.35, 1.0)
POST = 0.015         # [m] hook post half side
PEG_R = 0.005        # [m] hook peg radius
PLACE_TOL = 0.03     # [m] scripted place: centre error to let go at (carry.py)
PLACE_MARGIN = 0.05  # [m] box: ... at most this far inside the walls


class Receptacle(Part):
    role = "sensor"
    marked = True
    marker_anchor = "/base"
    marker_faces = markers.FACES_FLAT
    stage = "input"
    suffixes = (".ok", ".count", ".wrong", ".mass")
    state_keys = ("ok", "inside", "progress", "wrong_in", "locked", "closed", "fail", "hung",
                  "consumed", "consumed_mass")
    fields = (
        Field("shape", "enum", "box", choices=("box", "plate", "hook")),
        Field("size", "vec", [0.3, 0.3], length=2, unit="m", min=0.05, max=2.0),
        Field("wall_height", "float", 0.06, unit="m", min=0.01, max=0.5),
        Field("wall_thickness", "float", 0.01, unit="m", min=0.005, max=0.1),
        Field("accept", "any", [], doc="object names, kinds or marker ids that count (empty: any)"),
        Field("require", "enum", "all", choices=REQUIRES),
        Field("count", "int", 1, min=1, max=100, doc="require count: objects needed"),
        Field("mass", "float", 0.1, unit="kg", min=0.001, max=100.0, doc="require mass"),
        Field("order", "list", None, doc="require order: names in the order they go in"),
        Field("tolerance", "float", 0.05, unit="m", min=0.005, max=1.0,
              doc="plate: position error; hook: handle to peg tip"),
        Field("angle_tolerance", "float", 15.0, unit="deg", min=0.0, max=180.0,
              doc="plate: heading error"),
        Field("yaw", "float", 0.0, unit="deg", min=-180.0, max=180.0, doc="plate: wanted heading"),
        Field("angle_period", "float", 90.0, unit="deg", min=1.0, max=360.0,
              doc="plate: headings this far apart look the same"),
        Field("wrong", "enum", "ignore", choices=WRONGS),
        Field("eject_speed", "float", 1.0, unit="m/s", min=0.1, max=5.0),
        Field("close_on_ok", "bool", False),
        Field("height", "float", 0.19, unit="m", min=0.05, max=1.0, doc="hook: peg height"),
        Field("facing", "float", 0.0, unit="deg", min=-180.0, max=180.0,
              doc="hook: the way the peg points (0 = +x)"),
        Field("peg", "float", 0.06, unit="m", min=0.02, max=0.3, doc="hook: peg tip out from the post"),
        Field("consume", "bool", False,
              doc="an accepted object vanishes and stays counted (a dispenser copy goes back, R28)"),
    )

    def marker_face_size(self, face):
        wall = 2 * float(self.cfg["wall_thickness"]) if self.cfg["shape"] == "box" else 0.0
        return float(min(self.cfg["size"])) - wall

    def __init__(self, name, cfg):
        cfg = dict(cfg)
        req = cfg.get("require")
        if isinstance(req, dict):           # require: {count: 2} / {mass: 0.2} / {order: [a, b]}
            if len(req) != 1:
                raise SchemaError(f"part {name!r}: require needs one of {REQUIRES}, got {req}")
            (k, v), = req.items()
            cfg["require"] = k
            cfg[k] = v
        super().__init__(name, cfg)
        c = self.cfg
        self.shape = c["shape"]
        self.half = np.asarray(c["size"], dtype=float) / 2
        self.wall_h, self.wall_t = float(c["wall_height"]), float(c["wall_thickness"])
        acc = c["accept"]
        acc = [acc] if isinstance(acc, (str, int)) else acc
        if not isinstance(acc, list) or not all((isinstance(e, str) and e) or (type(e) is int and 0 <= e < markers.N_IDS)
                                                for e in acc):
            raise SchemaError(f"part {name!r}: accept must be a list of object names, kinds or marker ids "
                              f"(0..{markers.N_IDS - 1}), got {c['accept']!r}")
        self.accept = list(acc)
        self.require = c["require"]
        self.order = list(c["order"] or [])
        if self.require == "order" and not self.order:
            raise SchemaError(f"part {name!r}: require order needs `order: [names]`")
        self.wrong_mode = c["wrong"]
        self.close_on_ok = c["close_on_ok"]
        self.consume = bool(c["consume"])
        a = math.radians(c["facing"])
        self.normal = np.array([math.cos(a), math.sin(a)])
        self.hook_objs: list[Part] = []      # hook: grabbable objects it can hang (add_room_parts)
        self.marker_match = {e for e in self.accept if type(e) is int}
        self.stateful = True
        self.needs_body("beak_grab")          # objects get in by being carried (R18)
        self._clear()

    def _clear(self):
        self.ok = False
        self.inside: list[str] = []          # names in the order they went in
        self.progress = 0                    # require order: how many came in turn
        self.wrong_in: list[str] = []
        self.locked = self.closed = self.fail = False
        self.hung: list[str] = []            # hook: objects hung, by name
        self.consumed: list[str] = []        # consume: names of the objects consumed, in order
        self.consumed_mass = 0.0
        self._mass_in = 0.0

    @classmethod
    def add_room_parts(cls, parts):
        """Hooks learn the grabbable objects they can hang (one weld each)."""
        from ..object.grabbable import Grabbable

        objs = [p for p in parts.values() if isinstance(p, Grabbable)]
        for p in parts.values():
            if isinstance(p, Receptacle) and p.shape == "hook":
                p.hook_objs = objs

    # ---- scene ----------------------------------------------------------------------
    def peg_tip(self) -> np.ndarray:
        """Hook: world position of the peg tip (at reset; the post does not move)."""
        return np.array([*(self.xy + self.normal * self.cfg["peg"]), self._floor + self.cfg["height"]])

    def build(self, spec):
        body = super().build(spec)
        if self.consume:      # a fixed body: no force, but the model gets ngravcomp > 0 (ball.py `park`)
            body.gravcomp = 1.0
        hx, hy = self.half
        t, h = self.wall_t, self.wall_h
        body.add_geom(name=self.body_name + "/base", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[hx, hy, 0.002], pos=[0, 0, 0.002], rgba=RGBA, contype=0, conaffinity=0)
        if self.shape == "hook":
            ph = float(self.cfg["height"]) + 0.02
            body.add_geom(name=self.body_name + "/post", type=mujoco.mjtGeom.mjGEOM_BOX,
                          size=[POST, POST, ph / 2], pos=[0, 0, ph / 2], rgba=RGBA)
            tip = [*(self.normal * self.cfg["peg"]), self.cfg["height"]]
            body.add_geom(name=self.body_name + "/peg", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                          fromto=[0, 0, self.cfg["height"] - 0.01, *tip], size=[PEG_R, 0, 0],
                          rgba=(0.3, 0.3, 0.35, 1.0), contype=0, conaffinity=0)
            for o in self.hook_objs:
                spec.add_equality(name=f"{self.body_name}/hang/{o.name}", type=mujoco.mjtEq.mjEQ_WELD,
                                  objtype=mujoco.mjtObj.mjOBJ_BODY, name1=self.body_name,
                                  name2=o.body_name, active=False)
                spec.add_exclude(bodyname1=self.body_name, bodyname2=o.body_name)
        if self.shape == "box":
            for i, (sx, sy, x, y) in enumerate(((hx, t / 2, 0, hy - t / 2), (hx, t / 2, 0, -hy + t / 2),
                                                (t / 2, hy, hx - t / 2, 0), (t / 2, hy, -hx + t / 2, 0))):
                body.add_geom(name=f"{self.body_name}/wall{i}", type=mujoco.mjtGeom.mjGEOM_BOX,
                              size=[sx, sy, h / 2], pos=[x, y, h / 2], rgba=RGBA)
        lid = spec.worldbody.add_body(name=self.body_name + "/lid", mocap=True,
                                      pos=[*self.base_xy, LID_HIDDEN_Z])
        lid.add_geom(name=self.body_name + "/lid", type=mujoco.mjtGeom.mjGEOM_BOX,
                     size=[hx, hy, 0.005], rgba=(0.4, 0.3, 0.2, 1.0))
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._mocap = int(model.body_mocapid[model.body(self.body_name + "/lid").id])
        self._base = model.geom(self.body_name + "/base").id
        self._hang = {o.name: model.equality(f"{self.body_name}/hang/{o.name}").id for o in self.hook_objs}
        self._floor = float(model.body_pos[self._bid][2])

    def _lid(self, data, closed: bool) -> None:
        z = self.wall_h + 0.005 if closed else LID_HIDDEN_Z
        data.mocap_pos[self._mocap] = [*self.xy, z]

    def reset(self, model, data):
        for eq in self._hang.values():
            data.eq_active[eq] = 0
        self._clear()
        self._lid(data, False)
        model.geom_rgba[self._base] = RGBA
        self._write()

    def _write(self) -> None:
        o = self.outputs
        self.write(float(self.ok), o[0])
        self.write(float(len([n for n in self.inside if n not in self.wrong_in]) + len(self.consumed)), o[1])
        self.write(float(bool(self.wrong_in) or self.locked), o[2])
        self.write(round(self._mass_in, 3), o[3])

    # ---- judging -----------------------------------------------------------------------
    def accepts(self, obj: Part) -> bool:
        if not self.accept:
            return True
        return self._matches(obj, self.accept)

    @staticmethod
    def _matches(obj: Part, entries) -> bool:
        """Whether `obj` is named, of a kind or has a marker id in `entries`."""
        mid = obj.marker.id if getattr(obj, "marker", None) is not None else None
        return any(e == obj.name or e == getattr(obj, "object_kind", None)
                   or (type(e) is int and e == mid) for e in entries)

    def place_tolerance(self, obj) -> float:
        """Scripted `place` (carry.py): let go once the object's centre is this close
        to the centre; a box takes it anywhere well inside its walls."""
        if self.shape == "box":
            return max(PLACE_TOL, float(min(self.half)) - self.wall_t - PLACE_MARGIN)
        return PLACE_TOL

    def _is_in(self, data, rid: int) -> bool:
        if self.shape == "hook":
            return self.world.objects[rid].name in self.hung
        p = data.xpos[rid]
        d = p[:2] - self.xy
        if self.shape == "box":
            inner = self.half - self.wall_t
            return bool((np.abs(d) <= inner).all() and p[2] < self.wall_h + 0.1)
        if np.hypot(*d) > self.cfg["tolerance"] or p[2] > 0.3:
            return False
        q = data.xquat[rid]
        yaw = math.degrees(math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2)))
        per = self.cfg["angle_period"]
        err = (yaw - self.cfg["yaw"] + per / 2) % per - per / 2
        return abs(err) <= self.cfg["angle_tolerance"]

    def _hook(self, data) -> None:
        """Hook: let go of held objects; hang objects with a handle at the peg tip."""
        from ...grab import weld_now

        world = self.world
        tip = self.peg_tip()
        for o in self.hook_objs:
            rid = int(o._bid)
            if o.name in self.hung and rid in world.held:
                self._unhang(data, o.name)
            elif o.name not in self.hung and rid not in world.held and not (self.locked or self.closed):
                if float(np.min(np.linalg.norm(o.handle_points(data) - tip, axis=1))) <= self.cfg["tolerance"]:
                    weld_now(self._model, data, self._hang[o.name], self._bid, o._bid)
                    self.hung.append(o.name)

    def _unhang(self, data, name: str) -> None:
        data.eq_active[self._hang[name]] = 0
        self.hung.remove(name)

    def _eject(self, model, data, rid: int) -> None:
        """Lift the object above the walls and throw it outward."""
        if self.shape == "hook":
            self._unhang(data, self.world.objects[rid].name)
        d = data.xpos[rid][:2] - self.xy
        n = float(np.linalg.norm(d))
        d = d / n if n > 1e-6 else np.array([1.0, 0.0])
        j = model.body_jntadr[rid]
        qadr, vadr = int(model.jnt_qposadr[j]), int(model.jnt_dofadr[j])
        if self.shape == "box":
            data.qpos[qadr + 2] += self.wall_h + 0.02
        speed = self.cfg["eject_speed"]
        data.qvel[vadr:vadr + 6] = [*(d * speed), 1.0, 0.0, 0.0, 0.0]

    def _met(self, model, data, objs: dict[str, int]) -> bool:
        good = [n for n in self.inside if n not in self.wrong_in]
        if self.require == "all":
            if not self.accept:
                return bool(good or self.consumed)
            names = {o.name: o for o in self.world.objects.values()}
            owners = [names[n] for n in good + self.consumed]
            return all(any(self._matches(o, [e]) for o in owners) for e in self.accept)
        if self.require == "count":
            return len(good) + len(self.consumed) >= self.cfg["count"]
        if self.require == "mass":
            return self._mass_in >= self.cfg["mass"]
        return self.progress == len(self.order) and all(n in good or n in self.consumed for n in self.order)

    def update(self, model, data, body):
        world = self.world
        if world is None:
            return
        self._model = model
        if self.shape == "hook":
            self._hook(data)
        objs = {o.name: rid for rid, o in world.objects.items()
                if world.kinds.get(rid) == "object" and rid not in world.held and not o.parked}
        now = {n for n, rid in objs.items() if self._is_in(data, rid)}
        if not self.enabled():
            return
        for n in [n for n in self.inside if n not in now]:          # left
            self.inside.remove(n)
            if n in self.wrong_in:
                self.wrong_in.remove(n)
        for n in sorted(now - set(self.inside)):                      # came in
            self.inside.append(n)
            obj = world.objects[objs[n]]
            wrong = not self.accepts(obj)
            if not wrong and self.require == "order":
                if self.progress < len(self.order) and n == self.order[self.progress]:
                    self.progress += 1
                else:
                    wrong = True
            if wrong:
                self.wrong_in.append(n)
                if self.wrong_mode == "eject":
                    self._eject(model, data, objs[n])
                elif self.wrong_mode == "lock":
                    self.locked = True
                elif self.wrong_mode == "fail":
                    self.fail = True
            elif self.consume:
                self._consume(model, data, n, objs[n])
        self._mass_in = self.consumed_mass + sum(float(model.body_subtreemass[objs[n]])
                                                 for n in self.inside if n not in self.wrong_in and n in objs)
        ok = self._met(model, data, objs)
        if ok and not self.ok and self.close_on_ok:
            self.closed = True
        self.ok = ok
        self._lid(data, self.locked or self.closed)
        model.geom_rgba[self._base] = RGBA_OK if ok else RGBA
        self._write()

    def _consume(self, model, data, name: str, rid: int) -> None:
        """`consume`: the accepted object vanishes and stays counted."""
        if name in self.hung:
            self._unhang(data, name)
        self.inside.remove(name)
        self.consumed.append(name)
        self.consumed_mass = round(self.consumed_mass + float(model.body_subtreemass[rid]), 6)
        self.world.objects[rid].vanish(model, data)

    def state(self):
        return float(self.ok)

    def satisfied(self):
        return self.ok

    def failed(self):
        return self.fail

    def load_state(self, model, data, st):
        super().load_state(model, data, st)
        self._lid(data, self.locked or self.closed)
