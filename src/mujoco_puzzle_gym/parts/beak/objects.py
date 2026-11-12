"""Objects the beak carries (R28, category `beak`): laundry, key, handle ball, egg, stick.

All are `Grabbable` (parts/object/grabbable.py): a free body with handles, a mass and the
grab values. They rest upright at `pos` on reset (world axes as below). The
cube weight (`cube_weight`) is the cube (R17, parts/object/cube.py).

- `laundry`: a flat light cloth, simulated as a rigid plate `size` [x, y]
  `thickness` thick; only the visual mesh is wrinkled. Its handles are hems on
  the top edges in `handles` (+x, -x, +y, -y).
- `key`: a flat key: a square head (the marker sits on it, R26) and a shaft
  along +x. One handle, a ring on the head's -x end. A receptacle whose
  `accept` lists the key's marker id takes it (R5, "the socket with the same
  number"); that socket may show the same marker id.
- `handle_ball`: a ball with a ring handle on each face in `handles` (at the
  equator). The rings do not collide, so the ball rolls once let go. Not the
  R38 `ball` (no pushing, nets do not take it).
- `egg`: an egg (ellipsoid look, box contact shape so it does not roll off a
  stand) with side knobs. A contact that starts at a normal speed of
  sqrt(2 g `break_height`) or more (a drop from `break_height`, or a knock as
  hard) breaks it: `<name>.broken` goes to 1 and stays (until reset), and with
  `on_break: fail` the episode fails. The speed is the relative normal speed
  in the physics step before the contact started (substep hook, as the target
  of R5). Default break_height: see BREAK_HEIGHT.
- `stick`: a square rod `length` long along +x with one handle knob at its -x
  end. While the beak holds it, its +x end presses wall buttons as the beak tip
  does (same window, R4; `tip_point`, grab.py writes `World.tool_tip`).
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from ... import markers
from ...schema import Field
from ..object.grabbable import FACES, HANDLE_HALF, HANDLE_RGBA, Grabbable, face_list, knobs

#: Default `break_height` [m] (0.99 m/s). Measured (2026-11-12, default body,
#: egg 25 g, contact-start normal speeds): let go from the beak onto the floor
#: 1.42 m/s (head lowest) to 1.86 m/s (level), as free fall predicts; bumps
#: while grabbing from a stand and carrying <= 0.49 m/s; knocks after it landed
#: <= 0.35 m/s. So any let-go onto the floor breaks it and carrying does not.
BREAK_HEIGHT = 0.05


SIDES = ("+x", "-x", "+y", "-y")


def _box(body, name, half, pos=(0.0, 0.0, 0.0), rgba=(0.8, 0.8, 0.8, 1.0), mass=None, **kw):
    g = body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, size=list(half), pos=list(pos),
                      rgba=rgba, **kw)
    if mass is not None:
        g.mass = mass
    return g


def _visual(**kw) -> dict:
    return {"contype": 0, "conaffinity": 0, "mass": 0.0, **kw}


# ---- laundry ------------------------------------------------------------------------------
LAUNDRY_RGBA = (0.55, 0.75, 0.95, 1.0)
WRINKLES = 3            # folds across the plate (visual mesh)
WRINKLE_DEPTH = 0.004   # [m]


def wrinkled_slab(hx: float, hy: float, h: float, n: int = 12) -> tuple[list[float], list[int]]:
    """Vertices and faces of a closed slab, top and bottom waved by folds along y."""
    xs, ys = np.linspace(-hx, hx, n), np.linspace(-hy, hy, n // 2)
    verts = []
    for z0 in (h, -h):
        for y in ys:
            for x in xs:
                wave = WRINKLE_DEPTH * math.sin(WRINKLES * math.pi * (x / hx + 1) / 2) * math.cos(y / hy)
                verts.append([x, y, z0 + wave])
    nx, ny = len(xs), len(ys)
    idx = lambda layer, i, j: layer * nx * ny + j * nx + i  # noqa: E731
    faces = []
    for j in range(ny - 1):
        for i in range(nx - 1):
            a, b, c, d = idx(0, i, j), idx(0, i + 1, j), idx(0, i + 1, j + 1), idx(0, i, j + 1)
            faces += [a, b, c, a, c, d]
            a, b, c, d = idx(1, i, j), idx(1, i + 1, j), idx(1, i + 1, j + 1), idx(1, i, j + 1)
            faces += [a, c, b, a, d, c]
    rim = [(i, 0) for i in range(nx)] + [(nx - 1, j) for j in range(1, ny)] + \
          [(i, ny - 1) for i in range(nx - 2, -1, -1)] + [(0, j) for j in range(ny - 2, 0, -1)]
    for k in range(len(rim)):
        (i0, j0), (i1, j1) = rim[k], rim[(k + 1) % len(rim)]
        a, b = idx(0, i0, j0), idx(0, i1, j1)
        c, d = idx(1, i1, j1), idx(1, i0, j0)
        faces += [a, d, c, a, c, b]
    return [v for p in verts for v in p], faces


class Laundry(Grabbable):
    object_kind = "laundry"
    fields = (
        Field("mass", "float", 0.02, unit="kg", min=0.001, max=1.0, doc="a light cloth"),
        Field("size", "vec", [0.12, 0.1], length=2, unit="m", min=0.04, max=0.4,
              doc="cloth [x, y]"),
        Field("thickness", "float", 0.006, unit="m", min=0.002, max=0.03),
        Field("handles", "list", ["+x", "-x", "+y", "-y"],
              doc="top edges with a hem the beak grabs: +x, -x, +y, -y"),
        Field("hem", "float", 0.01, unit="m", min=0.004, max=0.05,
              doc="how far a hem sticks out of its edge"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.half = np.asarray(self.cfg["size"], dtype=float) / 2
        self.thick = float(self.cfg["thickness"])
        self.handles = face_list(name, self.cfg["handles"], SIDES)
        hem = float(self.cfg["hem"])
        self._local = np.array([[*(np.asarray(FACES[f][:2]) * (self.half + hem / 2)), self.thick / 2]
                                for f in self.handles])

    def _rest_z(self) -> float:
        return self.thick / 2

    def build(self, spec):
        hx, hy = self.half
        h = self.thick / 2
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, h])
        body.add_freejoint(name=self.body_name + "/free")
        _box(body, self.body_name + "/plate", (hx, hy, h), rgba=(0, 0, 0, 0), mass=self.mass,
             friction=[self.cfg["friction"], 0.005, 0.0001], group=3)
        mesh = spec.add_mesh(name=self.body_name + "/wrinkles")
        mesh.uservert, mesh.userface = wrinkled_slab(hx, hy, h)
        body.add_geom(name=self.body_name + "/cloth", type=mujoco.mjtGeom.mjGEOM_MESH,
                      meshname=mesh.name, rgba=LAUNDRY_RGBA, **_visual())
        hem = float(self.cfg["hem"])
        for f, pos in zip(self.handles, self._local):
            along_x = abs(FACES[f][0]) > 0
            half = (hem / 2, hy * 0.8, HANDLE_HALF / 2) if along_x else (hx * 0.8, hem / 2, HANDLE_HALF / 2)
            _box(body, f"{self.body_name}/handle{f}", half, pos, rgba=HANDLE_RGBA, mass=0.0)
        return body


# ---- key ----------------------------------------------------------------------------------
KEY_RGBA = (0.85, 0.7, 0.2, 1.0)
KEY_THICK = 0.01        # [m]


class Key(Grabbable):
    object_kind = "key"
    marked = True
    marker_anchor = "/head"
    marker_faces = markers.FACES_FLAT
    fields = (
        Field("mass", "float", 0.02, unit="kg", min=0.001, max=1.0),
        Field("head", "float", 0.04, unit="m", min=0.02, max=0.1,
              doc="side of the square head (the marker sits on it)"),
        Field("shaft", "float", 0.05, unit="m", min=0.01, max=0.2, doc="shaft length along +x"),
        Field("handle_size", "float", 0.015, unit="m", min=0.005, max=0.05,
              doc="how far the ring sticks out of the head's -x end"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.head = float(self.cfg["head"])
        self.handles = ["-x"]
        self._local = np.array([[-(self.head / 2 + float(self.cfg["handle_size"]) / 2), 0.0, 0.0]])

    def marker_face_size(self, face):
        return self.head

    def _rest_z(self) -> float:
        return KEY_THICK / 2

    def build(self, spec):
        h, t = self.head / 2, KEY_THICK / 2
        shaft = float(self.cfg["shaft"]) / 2
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, t])
        body.add_freejoint(name=self.body_name + "/free")
        fr = [self.cfg["friction"], 0.005, 0.0001]
        _box(body, self.body_name + "/head", (h, h, t), rgba=KEY_RGBA, mass=self.mass * 0.6, friction=fr)
        _box(body, self.body_name + "/shaft", (shaft, 0.006, t * 0.8), (h + shaft, 0, 0),
             rgba=KEY_RGBA, mass=self.mass * 0.4, friction=fr)
        for k in (0.4, 0.8):                                      # teeth (visual)
            _box(body, f"{self.body_name}/tooth{k}", (0.004, 0.005, t * 0.8),
                 (h + 2 * shaft * k, -0.009, 0), rgba=KEY_RGBA, **_visual())
        knobs(body, self.body_name, self.handles, self._local, float(self.cfg["handle_size"]))
        return body


# ---- handle ball --------------------------------------------------------------------------
BALL_RGBA = (0.9, 0.3, 0.45, 1.0)
RING_SEGMENTS = 8


class HandleBall(Grabbable):
    object_kind = "handle_ball"
    fields = (
        Field("radius", "float", 0.03, unit="m", min=0.015, max=0.1),
        Field("roll_friction", "float", 0.002, unit="m", min=0.0, max=0.05,
              doc="rolling friction (low: it rolls on once let go)"),
        Field("handles", "list", ["-x"], doc="faces with a ring handle: +x, -x, +y, -y"),
        Field("ring", "float", 0.012, unit="m", min=0.005, max=0.04, doc="ring radius"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.radius = float(self.cfg["radius"])
        self.handles = face_list(name, self.cfg["handles"], SIDES)
        out = self.radius + float(self.cfg["ring"])
        self._local = np.array([np.asarray(FACES[f]) * out for f in self.handles])

    def _rest_z(self) -> float:
        return self.radius

    def build(self, spec):
        c = self.cfg
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, self.radius])
        body.add_freejoint(name=self.body_name + "/free")
        body.add_geom(name=self.body_name + "/ball", type=mujoco.mjtGeom.mjGEOM_SPHERE,
                      size=[self.radius, 0, 0], mass=self.mass, condim=6,
                      friction=[c["friction"], 0.005, c["roll_friction"]], rgba=BALL_RGBA)
        r = float(c["ring"])
        for f, centre in zip(self.handles, self._local):
            n = np.asarray(FACES[f])
            for k in range(RING_SEGMENTS):                  # a ring in the plane of n and z
                a0, a1 = 2 * math.pi * k / RING_SEGMENTS, 2 * math.pi * (k + 1) / RING_SEGMENTS
                p0 = centre + r * (math.cos(a0) * n + math.sin(a0) * np.array([0, 0, 1.0]))
                p1 = centre + r * (math.cos(a1) * n + math.sin(a1) * np.array([0, 0, 1.0]))
                body.add_geom(name=f"{self.body_name}/handle{f}/{k}", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                              fromto=[*p0, *p1], size=[0.0025, 0, 0], rgba=HANDLE_RGBA, **_visual())
        return body


# ---- egg ----------------------------------------------------------------------------------
EGG_RGBA = (0.97, 0.94, 0.85, 1.0)
BROKEN_RGBA = (0.95, 0.75, 0.2, 1.0)


class Egg(Grabbable):
    object_kind = "egg"
    stage = "input"
    suffixes = (".broken",)
    state_keys = ("broken",)
    fields = (
        Field("mass", "float", 0.025, unit="kg", min=0.005, max=0.2),
        Field("radius", "float", 0.022, unit="m", min=0.01, max=0.06, doc="egg radius (x, y)"),
        Field("height", "float", 0.06, unit="m", min=0.02, max=0.15, doc="egg height (z)"),
        Field("handles", "list", ["+x", "-x", "+y", "-y"], doc="faces with a knob: +x, -x, +y, -y"),
        Field("handle_size", "float", 0.012, unit="m", min=0.005, max=0.05),
        Field("break_height", "float", BREAK_HEIGHT, unit="m", min=0.005, max=2.0, runtime=True,
              doc="a contact starting as fast as a drop from this height breaks it"),
        Field("on_break", "enum", "none", choices=("none", "fail"),
              doc="fail: a broken egg fails the episode"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        self.radius, self.height = float(c["radius"]), float(c["height"])
        self.handles = face_list(name, c["handles"], SIDES)
        out = self.radius + float(c["handle_size"]) / 2
        self._local = np.array([np.asarray(FACES[f]) * out for f in self.handles])
        self.broken = False
        self._touching: set[int] = set()
        self._v_prev = np.zeros(3)
        self._hit = 0.0                 # fastest contact start since the last update [m/s]
        self._vo = np.zeros(6)

    def _rest_z(self) -> float:
        return self.height / 2

    def build(self, spec):
        r, h = self.radius, self.height / 2
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, h])
        body.add_freejoint(name=self.body_name + "/free")
        _box(body, self.body_name + "/shape", (r * 0.85, r * 0.85, h), rgba=(0, 0, 0, 0), mass=self.mass,
             friction=[self.cfg["friction"], 0.005, 0.0001], group=3)
        body.add_geom(name=self.body_name + "/shell", type=mujoco.mjtGeom.mjGEOM_ELLIPSOID,
                      size=[r, r, h], rgba=EGG_RGBA, **_visual())
        knobs(body, self.body_name, self.handles, self._local, float(self.cfg["handle_size"]))
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._shell = model.geom(self.body_name + "/shell").id

    def reset(self, model, data):
        super().reset(model, data)
        self._set(model, False)
        self._touching, self._hit = set(), 0.0
        self._v_prev[:] = 0.0

    def _set(self, model, broken: bool) -> None:
        self.broken = broken
        model.geom_rgba[self._shell] = BROKEN_RGBA if broken else EGG_RGBA
        self.write(float(broken), self.name + ".broken")

    def break_speed(self) -> float:
        return math.sqrt(2.0 * 9.81 * float(self.live["break_height"]))

    def _contacts(self, model, data) -> dict[int, np.ndarray]:
        """Bodies touching the egg now, with the first contact normal of each."""
        now: dict[int, np.ndarray] = {}
        for i in range(data.ncon):
            c = data.contact[i]
            b1, b2 = int(model.geom_bodyid[c.geom1]), int(model.geom_bodyid[c.geom2])
            if b1 == self._bid:
                now.setdefault(b2, c.frame[:3].copy())
            elif b2 == self._bid:
                now.setdefault(b1, c.frame[:3].copy())
        return now

    def substep(self, model, data):
        now = self._contacts(model, data)
        for b, normal in now.items():
            if b in self._touching:
                continue
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, b, self._vo, 0)
            self._hit = max(self._hit, abs(float((self._v_prev - self._vo[3:]) @ normal)))
        self._touching = set(now)
        self._v_prev = data.qvel[self._v:self._v + 3].copy()

    def update(self, model, data, body):
        hit, self._hit = self._hit, 0.0
        if not self.broken and hit >= self.break_speed():
            self._set(model, True)

    def failed(self):
        return self.broken and self.cfg["on_break"] == "fail"

    def load_state(self, model, data, st):
        super().load_state(model, data, st)
        model.geom_rgba[self._shell] = BROKEN_RGBA if self.broken else EGG_RGBA
        mujoco.mj_forward(model, data)          # contacts of the restored state
        self._touching, self._hit = set(self._contacts(model, data)), 0.0
        self._v_prev = data.qvel[self._v:self._v + 3].copy()


# ---- stick --------------------------------------------------------------------------------
STICK_RGBA = (0.55, 0.4, 0.25, 1.0)


class Stick(Grabbable):
    object_kind = "stick"
    fields = (
        Field("mass", "float", 0.02, unit="kg", min=0.002, max=0.5),
        Field("length", "float", 0.2, unit="m", min=0.08, max=0.5, doc="rod length along +x"),
        Field("width", "float", 0.016, unit="m", min=0.006, max=0.04, doc="square rod side"),
        Field("handle_size", "float", 0.015, unit="m", min=0.005, max=0.05,
              doc="how far the handle sticks out of the -x end"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.length, self.width = float(self.cfg["length"]), float(self.cfg["width"])
        self.handles = ["-x"]
        self._local = np.array([[-(self.length / 2 + float(self.cfg["handle_size"]) / 2), 0.0, 0.0]])
        self._tip = np.array([self.length / 2, 0.0, 0.0])

    @classmethod
    def add_room_parts(cls, parts):
        """Wall buttons may be pressed from a stick's reach farther off (planning)."""
        super().add_room_parts(parts)
        reach = max(p.reach_extra() for p in parts.values() if isinstance(p, Stick))
        for p in parts.values():
            if getattr(p, "mount", None) == "wall":
                p.reach = max(p.reach, reach)

    def reach_extra(self) -> float:
        """How much farther than the beak tip the far end reaches when held."""
        return self.length + float(self.cfg["handle_size"]) / 2

    def _rest_z(self) -> float:
        return self.width / 2

    def build(self, spec):
        w = self.width / 2
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, w])
        body.add_freejoint(name=self.body_name + "/free")
        _box(body, self.body_name + "/rod", (self.length / 2, w, w), rgba=STICK_RGBA, mass=self.mass,
             friction=[self.cfg["friction"], 0.005, 0.0001])
        knobs(body, self.body_name, self.handles, self._local, float(self.cfg["handle_size"]))
        return body

    def tip_point(self, data) -> np.ndarray:
        """World position of the +x end (it presses wall buttons while held)."""
        return data.xpos[self._bid] + data.xmat[self._bid].reshape(3, 3) @ self._tip
