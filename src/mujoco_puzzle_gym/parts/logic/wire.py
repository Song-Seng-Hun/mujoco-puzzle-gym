"""Cable between circuit parts (R33, R34): a visible wire that can be cut.

A wire carries one channel (`carries`) to the parts named in `to`. Cutting it
replaces that channel **for those readers only** with the wire's `broken_value`
(`Part.cut`): every other reader of the channel is untouched, which is what
makes a wire a place in the room rather than a second channel.

`to` entries are `<part>` (every read of `carries` by that part) or
`<part>.<channel>`, where the channel must be `carries` (`part.enable` names
that part's power channel when it is the carried one). The ends of a wire sit
on fixed parts: a wire to a part that can be pushed around (`movable: true`, a
`relay` on a free joint) is refused by `mpg check`, and that link belongs to a
transceiver (R8) instead.

Shape: the cable is a chain of thin capsules from `pos` along `path` (world
points), drawn flat on the floor and never colliding, so the duck walks over
it. A marker (R26) goes on the wire's tag plate, not on the cable.

`breakable: true` adds the cable cover: a box that only shells and objects
collide with (the duck is excluded from it body by body), mounted on a wall
(`mount: wall`, a point on the wall face at `pos` facing `facing`) or on the
side of a part that blocks the duck (`mount: part`, `on: <part>`). Measured
(Open question 8): the cover protrudes `depth` 0.04 m from the face and reaches
`height` 0.18 m, and a peck counts within 0.02 m, because the duck can only
hold a place where something stops it. `by: [peck]` with a cover the duck
cannot stand at is a warning in `mpg check`, not a silent dead end.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from ... import markers
from ...schema import Field, SchemaError
from ..base import Part
from ..breakable import BREAK_KEYS, Breakable

CABLE_RGBA = (0.15, 0.55, 0.85, 1.0)
COVER_RGBA = (0.55, 0.5, 0.45, 1.0)
TAG_RGBA = (0.95, 0.95, 0.9, 1.0)
TAG = 0.05            # tag plate half-side [m]
BODY_PREFIX = "body/"  # the walking body's bodies (excluded from the cover)


class Wire(Breakable):
    type_name = "wire"
    role = "prop"
    stage = "logic"
    emits = False
    marked = True
    marker_anchor = "/tag"
    marker_faces = markers.FACES_FLAT
    state_keys = BREAK_KEYS
    fields = (
        Field("carries", "str", None, doc="the channel this wire carries"),
        Field("to", "list", [], doc="readers it feeds: `<part>` or `<part>.<channel>`"),
        Field("path", "any", None, doc="further cable points [[x, y], ...] after `pos`"),
        Field("thickness", "float", 0.012, unit="m", min=0.004, max=0.05,
              doc="cable radius"),
        Field("mount", "enum", "wall", choices=("wall", "part"),
              doc="`breakable`: the cover is on a wall face or on a part's side"),
        Field("on", "str", None, doc="`mount: part`: the part the cover is fixed to"),
        Field("facing", "float", 0.0, unit="deg", min=-180.0, max=180.0,
              doc="`breakable`: direction the cover faces (0 = +x, the side the duck comes from)"),
        Field("depth", "float", 0.04, unit="m", min=0.02, max=0.2,
              doc="`breakable`: how far the cover stands off the face (measured, R34)"),
        Field("height", "float", 0.18, unit="m", min=0.06, max=0.5,
              doc="`breakable`: how high the cover reaches (measured, R34)"),
        Field("width", "float", 0.12, unit="m", min=0.04, max=0.6,
              doc="`breakable`: cover width along the face"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        self.carries = c["carries"]
        if not self.carries:
            raise SchemaError(f"part {name!r}: a wire needs `carries: <channel>`")
        self.targets: list[tuple[str, str]] = _parse_to(name, c["to"], self.carries)
        if not self.targets:
            raise SchemaError(f"part {name!r}: a wire needs at least one reader in `to`")
        self.path = _parse_path(name, self.base_xy, c["path"])
        self.mount = c["mount"]
        self.on = c["on"]
        if self.mount == "part" and not self.on:
            raise SchemaError(f"part {name!r}: `mount: part` needs `on: <part>`")
        if self.mount == "wall" and self.on:
            raise SchemaError(f"part {name!r}: `on` only means something with `mount: part`")
        self.facing = math.radians(float(c["facing"]))
        self.depth = float(c["depth"])
        self.cover_height = float(c["height"])
        self.cover_width = float(c["width"])
        self.inputs = list(dict.fromkeys([*self.inputs, self.carries]))  # it reads what it carries
        self._readers: list[tuple[Part, str]] = []
        self._cover = -1
        self._cut_now = False

    # ---- room load ------------------------------------------------------------------
    @classmethod
    def add_room_parts(cls, parts: dict[str, Part]) -> None:
        """Resolve every wire's `to` entries to the parts they feed."""
        for wire in [p for p in parts.values() if isinstance(p, Wire)]:
            wire._readers = []
            for target, channel in wire.targets:
                part = parts.get(target)
                if part is None:
                    raise SchemaError(f"part {wire.name!r}: `to` names {target!r}, "
                                      f"which is not a part in this room")
                wire._readers.append((part, channel))

    def marker_face_size(self, face):
        return 2 * TAG

    # ---- scene ----------------------------------------------------------------------
    def build(self, spec):
        body = super().build(spec)
        pts = np.asarray(self.path, dtype=float) - self.base_xy        # body frame
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            body.add_geom(name=f"{self.body_name}/cable{i}", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                          fromto=[a[0], a[1], self.thickness, b[0], b[1], self.thickness],
                          size=[self.thickness, 0, 0], rgba=CABLE_RGBA,
                          contype=0, conaffinity=0)
        body.add_geom(name=self.body_name + "/tag", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[TAG, TAG, 0.004], pos=[0, 0, 0.004], rgba=TAG_RGBA,
                      contype=0, conaffinity=0)
        if self.breakable:
            cx, cy = self.cover_centre() - self.base_xy
            body.add_geom(name=self.body_name + "/cover", type=mujoco.mjtGeom.mjGEOM_BOX,
                          size=[self.depth / 2, self.cover_width / 2, self.cover_height / 2],
                          pos=[cx, cy, self.cover_height / 2],
                          quat=_yaw_quat(self.facing), rgba=COVER_RGBA)
        return body

    @property
    def thickness(self) -> float:
        return float(self.cfg["thickness"])

    def cover_centre(self) -> np.ndarray:
        """Cover centre in world xy: `depth`/2 in front of the face it is mounted on."""
        n = np.array([math.cos(self.facing), math.sin(self.facing)])
        return self.base_xy + n * (self.depth / 2)

    def attach_body(self, spec, body):
        """The duck does not collide with the cover (only shells and objects do, R33)."""
        if not self.breakable:
            return
        for b in spec.bodies:
            if (b.name or "").startswith(BODY_PREFIX):
                spec.add_exclude(bodyname1=self.body_name, bodyname2=b.name)

    def bind(self, model, data):
        super().bind(model, data)
        self._cover = model.geom(self.body_name + "/cover").id if self.breakable else -1
        self.break_bind(model, data)

    def break_geom(self, model):
        return self._cover

    def peck_normal(self):
        """The cover's front: the wall is behind it, so the duck pecks from here."""
        return np.array([math.cos(self.facing), math.sin(self.facing)])

    # ---- runtime --------------------------------------------------------------------
    def reset(self, model, data):
        self._clear_cut()
        self.break_reset(model)
        self._apply_cut()

    def update(self, model, data, body):
        self.break_step(model, data, body)
        self._apply_cut()

    def load_state(self, model, data, st):
        super().load_state(model, data, st)
        self.break_load(model)
        self._apply_cut()

    def _apply_cut(self) -> None:
        """A broken wire leaves `broken_value` on its readers' channel."""
        if self.broken == self._cut_now:
            return
        self._cut_now = self.broken
        for part, channel in self._readers:
            if self.broken:
                part.cut[channel] = self.broken_value
            else:
                part.cut.pop(channel, None)

    def _clear_cut(self) -> None:
        self._cut_now = False
        for part, channel in self._readers:
            part.cut.pop(channel, None)

    def state(self):
        return 0.0 if self.broken else 1.0

    def paint_break(self, model):
        """A cut cover turns dark and stops colliding (the cable is severed)."""
        if not self.breakable or self._cover < 0 or self._painted == self.broken:
            return
        super().paint_break(model)
        on = 0 if self.broken else 1
        model.geom_contype[self._cover] = on
        model.geom_conaffinity[self._cover] = on

    # ---- planning and checks ---------------------------------------------------------
    def plan_values(self, on, get):
        return self.break_plan_values({})

    def plan_broken_values(self, get):
        """Planning: a cut wire takes the channel it carries from its readers. The plan
        has one value per channel, so it drops the channel itself; where other parts
        read that channel past the wire, the plan is optimistic and the scripted solver
        is what proves the room."""
        out = super().plan_broken_values(get)
        out[self.carries] = self.broken_value
        return out

    def check_static(self, env):
        out = []
        for part, channel in self._readers:
            if channel not in list(part.inputs) + ([part.enable] if part.enable else []):
                out.append(f"wire {self.name!r} feeds {part.name!r} on channel {channel!r}, "
                           f"which that part does not read")
            if bool(part.cfg.get("movable", False)):
                out.append(f"wire {self.name!r} ends on {part.name!r}, which can be moved "
                           f"(`movable: true`); use a transceiver (R8) for that link")
        if self.mount == "part" and self.on not in env.parts:
            out.append(f"wire {self.name!r}: `on` names {self.on!r}, which is not a part")
        return out

    def check_warnings(self, env):
        out = list(super().check_warnings(env))
        if self.breakable and "peck" in self.by and self.mount == "part":
            part = env.parts.get(self.on)
            if part is not None and not part.blockers(False):
                out.append(f"wire {self.name!r} is pecked but its cover is on {self.on!r}, "
                           f"which does not block the duck: the duck cannot hold a place "
                           f"on open floor (R34, Open question 8)")
        return out


def _parse_to(name: str, raw, carries: str) -> list[tuple[str, str]]:
    out = []
    for entry in raw or []:
        if not isinstance(entry, str) or not entry:
            raise SchemaError(f"part {name!r}: `to` entries are `<part>` or "
                              f"`<part>.<channel>`, got {entry!r}")
        if "." in entry:
            target, channel = entry.split(".", 1)
            if channel != carries and channel != "enable":
                raise SchemaError(f"part {name!r}: `to` entry {entry!r} names channel "
                                  f"{channel!r}, but this wire carries {carries!r}")
            channel = carries
        else:
            target, channel = entry, carries
        out.append((target, channel))
    return out


def _parse_path(name: str, start: np.ndarray, raw) -> list[list[float]]:
    pts = [[float(start[0]), float(start[1])]]
    for p in raw or []:
        if (not isinstance(p, (list, tuple)) or len(p) != 2
                or not all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in p)):
            raise SchemaError(f"part {name!r}: `path` is a list of [x, y] points, got {p!r}")
        pts.append([float(p[0]), float(p[1])])
    if len(pts) == 1:
        pts.append([pts[0][0] + 0.2, pts[0][1]])        # a stub, so there is always a cable
    return pts


def _yaw_quat(yaw: float) -> list[float]:
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
