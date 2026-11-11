"""Receptacle (R5): a box or socket that signals when the right things are put in.

Shapes:
- `box`:   an open-top box (`size`, walls `wall_height` high). An object is in
  when its centre is inside the walls and lower than `wall_height` + 0.1 m.
- `plate`: a socket base plate. An object is in when its centre is within
  `tolerance` of the plate centre and its heading is within `angle_tolerance`
  of `yaw` (modulo `angle_period`: 90 degrees for a cube).
Socket, hook and basket presets build on this one part (hook: E3).

`accept`: object part names or object kinds (`key`, `egg`, ...); empty
accepts every object. Objects the body holds are never in (spec R5).

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

Channels: `<name>.ok`, `<name>.count` (accepted objects in), `<name>.wrong`
(1 while a wrong object is in, and from a `lock` on).
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from ...schema import Field, SchemaError
from ..base import Part

REQUIRES = ("all", "count", "mass", "order")
WRONGS = ("ignore", "eject", "lock", "fail")
LID_HIDDEN_Z = -1.0
RGBA = (0.6, 0.45, 0.25, 1.0)
RGBA_OK = (0.25, 0.75, 0.35, 1.0)


class Receptacle(Part):
    role = "sensor"
    stage = "input"
    suffixes = (".ok", ".count", ".wrong")
    state_keys = ("ok", "inside", "progress", "wrong_in", "locked", "closed", "fail")
    fields = (
        Field("shape", "enum", "box", choices=("box", "plate")),
        Field("size", "vec", [0.3, 0.3], length=2, unit="m", min=0.05, max=2.0),
        Field("wall_height", "float", 0.06, unit="m", min=0.01, max=0.5),
        Field("wall_thickness", "float", 0.01, unit="m", min=0.005, max=0.1),
        Field("accept", "list", [], doc="object names or kinds that count (empty: any)"),
        Field("require", "enum", "all", choices=REQUIRES),
        Field("count", "int", 1, min=1, max=100, doc="require count: objects needed"),
        Field("mass", "float", 0.1, unit="kg", min=0.001, max=100.0, doc="require mass"),
        Field("order", "list", None, doc="require order: names in the order they go in"),
        Field("tolerance", "float", 0.05, unit="m", min=0.005, max=1.0, doc="plate: position error"),
        Field("angle_tolerance", "float", 15.0, unit="deg", min=0.0, max=180.0,
              doc="plate: heading error"),
        Field("yaw", "float", 0.0, unit="deg", min=-180.0, max=180.0, doc="plate: wanted heading"),
        Field("angle_period", "float", 90.0, unit="deg", min=1.0, max=360.0,
              doc="plate: headings this far apart look the same"),
        Field("wrong", "enum", "ignore", choices=WRONGS),
        Field("eject_speed", "float", 1.0, unit="m/s", min=0.1, max=5.0),
        Field("close_on_ok", "bool", False),
    )

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
        self.accept = list(c["accept"])
        self.require = c["require"]
        self.order = list(c["order"] or [])
        if self.require == "order" and not self.order:
            raise SchemaError(f"part {name!r}: require order needs `order: [names]`")
        self.wrong_mode = c["wrong"]
        self.close_on_ok = c["close_on_ok"]
        self.stateful = True
        self.needs_body("beak_grab")          # objects get in by being carried (R18)
        self._clear()

    def _clear(self):
        self.ok = False
        self.inside: list[str] = []          # names in the order they went in
        self.progress = 0                    # require order: how many came in turn
        self.wrong_in: list[str] = []
        self.locked = self.closed = self.fail = False

    # ---- scene ----------------------------------------------------------------------
    def build(self, spec):
        body = super().build(spec)
        hx, hy = self.half
        t, h = self.wall_t, self.wall_h
        body.add_geom(name=self.body_name + "/base", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[hx, hy, 0.002], pos=[0, 0, 0.002], rgba=RGBA, contype=0, conaffinity=0)
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

    def _lid(self, data, closed: bool) -> None:
        z = self.wall_h + 0.005 if closed else LID_HIDDEN_Z
        data.mocap_pos[self._mocap] = [*self.xy, z]

    def reset(self, model, data):
        self._clear()
        self._lid(data, False)
        model.geom_rgba[self._base] = RGBA
        self._write()

    def _write(self) -> None:
        o = self.outputs
        self.write(float(self.ok), o[0])
        self.write(float(len([n for n in self.inside if n not in self.wrong_in])), o[1])
        self.write(float(bool(self.wrong_in) or self.locked), o[2])

    # ---- judging -----------------------------------------------------------------------
    def accepts(self, obj: Part) -> bool:
        if not self.accept:
            return True
        return obj.name in self.accept or getattr(obj, "object_kind", None) in self.accept

    def _is_in(self, data, rid: int) -> bool:
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

    def _eject(self, model, data, rid: int) -> None:
        """Lift the object above the walls and throw it outward."""
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
                return bool(good)
            owners = {n: self.world.objects[objs[n]] for n in good}
            return all(any(e == o.name or getattr(o, "object_kind", None) == e for o in owners.values())
                       for e in self.accept)
        if self.require == "count":
            return len(good) >= self.cfg["count"]
        if self.require == "mass":
            return sum(float(model.body_subtreemass[objs[n]]) for n in good) >= self.cfg["mass"]
        return self.progress == len(self.order) and all(n in good for n in self.order)

    def update(self, model, data, body):
        world = self.world
        if world is None:
            return
        objs = {o.name: rid for rid, o in world.objects.items()
                if world.kinds.get(rid) == "object" and rid not in world.held}
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
        ok = self._met(model, data, objs)
        if ok and not self.ok and self.close_on_ok:
            self.closed = True
        self.ok = ok
        self._lid(data, self.locked or self.closed)
        model.geom_rgba[self._base] = RGBA_OK if ok else RGBA
        self._write()

    def state(self):
        return float(self.ok)

    def satisfied(self):
        return self.ok

    def failed(self):
        return self.fail

    def load_state(self, model, data, st):
        super().load_state(model, data, st)
        self._lid(data, self.locked or self.closed)
