"""Floor tile (R10): a rectangle of floor with its own friction, tilt and height.

A room with any floor part (`floor_tile`, `trapdoor`) gets a tile floor
(floor.py): the floor parts own their rectangles and static boxes fill the
rest without gaps. Floor parts stay where room.yaml puts them (`random` must
be 0) so the floor has no gaps after a reset.

Values (all may change while running):
- `friction: [slide, torsion, roll]`;
- `tilt` [deg] about `tilt_axis` through the tile centre: positive raises the
  +y side (axis x) or the +x side (axis y);
- `height` [m]: the top face above (below) the room floor.
With `in` channels the tile moves between these (off) values and `on_friction`
/ `on_tilt` / `on_height` (on) values while the inputs are on, at
`friction_rate` [1/s], `tilt_speed` [deg/s] and `height_speed` [m/s]. Tilt is a
hinge joint and height a slide joint, each with a position actuator; a tile
whose tilt and height never change (no signal, setter or bind on them, R13) has
no joints. Friction is set on the geom directly. A setter or bind changing a
value moves the tile to it at the same rates.

The top height under a point (fall rule, R10) comes from these values (the
joints' current positions), not from rays.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from ... import markers
from ...floor import DEFAULT_FRICTION, TILE_THICKNESS, Rect
from ...schema import Field, SchemaError
from ..base import Part

MAX_TILT = 13.0      # deg; OQ5: walking down 15 deg fell 3/5, up to 13 deg none
MAX_HEIGHT = 0.1     # m
TILE_MASS = 2.0      # kg, with gravity compensation (the actuators hold loads only)
KEYS = ("friction", "tilt", "height")
RATES = {"friction": "friction_rate", "tilt": "tilt_speed", "height": "height_speed"}


class FloorPart(Part):
    """Base for parts that are a piece of the floor (tile floor, floor.py)."""

    floor_part = True
    fields = (
        Field("size", "vec", [0.3, 0.3], length=2, unit="m", min=0.05, max=5.0,
              doc="tile size [x, y]"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        if np.any(self.random != 0):
            raise SchemaError(f"part {name!r}: floor parts cannot move (`random` must be [0, 0])")
        self.half = np.asarray(self.cfg["size"], dtype=float) / 2

    def floor_rect(self) -> Rect:
        return Rect.around(self.base_xy, self.half)

    def place(self, model, xy):
        """Floor parts keep their place (the floor must stay gapless)."""

    def height_at(self, xy) -> tuple[float | None, float]:
        """(top height under `xy` or None over a hole, rim height)."""
        return 0.0, 0.0

    def target_mask(self, grid):
        return grid.rect(self.xy, tuple(self.half))


class FloorTile(FloorPart):
    marked = True
    marker_anchor = "/tile"
    marker_faces = markers.FACES_FLAT
    stateful = True
    state_keys = ("cur",)
    fields = (
        Field("friction", "vec", list(DEFAULT_FRICTION), length=3, min=0.0, max=5.0, unit="-",
              runtime=True, doc="slide, torsion, roll friction"),
        Field("tilt", "float", 0.0, unit="deg", min=-MAX_TILT, max=MAX_TILT, runtime=True,
              doc="tilt about tilt_axis (+: raises the +y side for axis x, +x side for axis y)"),
        Field("tilt_axis", "enum", "x", choices=("x", "y"), doc="hinge axis of the tilt"),
        Field("height", "float", 0.0, unit="m", min=-MAX_HEIGHT, max=MAX_HEIGHT, runtime=True,
              doc="top face height above the room floor"),
        Field("on_friction", "vec", None, length=3, min=0.0, max=5.0, unit="-", runtime=True,
              doc="friction while the inputs are on"),
        Field("on_tilt", "float", None, unit="deg", min=-MAX_TILT, max=MAX_TILT, runtime=True,
              doc="tilt while the inputs are on"),
        Field("on_height", "float", None, unit="m", min=-MAX_HEIGHT, max=MAX_HEIGHT, runtime=True,
              doc="height while the inputs are on"),
        Field("friction_rate", "float", 5.0, unit="1/s", min=0.01, max=100.0, runtime=True,
              doc="friction change per second"),
        Field("tilt_speed", "float", 10.0, unit="deg/s", min=0.5, max=90.0, runtime=True),
        Field("height_speed", "float", 0.05, unit="m/s", min=0.005, max=0.5, runtime=True),
    )

    def marker_face_size(self, face):
        return float(min(self.cfg["size"]))

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        # A value gets a joint when the signal changes it, or a setter or bind may (R13).
        self.moves = {k: (bool(self.inputs) and c["on_" + k] is not None
                          and not np.allclose(c["on_" + k], c[k])) or k in self.binds
                      for k in KEYS}
        self.axis = c["tilt_axis"]
        self.cur = {k: np.array(v, float) for k, v in self.off.items()}

    # Off / on values and rates read the run-time values (R13 setters may change them).
    @property
    def off(self) -> dict[str, object]:
        return {"friction": np.asarray(self.live["friction"], float), "tilt": float(self.live["tilt"]),
                "height": float(self.live["height"])}

    @property
    def on(self) -> dict[str, object]:
        off = self.off
        return {k: (off[k] if self.live["on_" + k] is None else
                    np.asarray(self.live["on_" + k], float) if k == "friction" else float(self.live["on_" + k]))
                for k in KEYS}

    @property
    def rate(self) -> dict[str, float]:
        return {k: float(self.live[r]) for k, r in RATES.items()}

    def claim_value(self, key):
        for k in KEYS:
            if key in (k, "on_" + k):
                self.moves[k] = True

    # ---- scene ------------------------------------------------------------------
    def _hinge_axis(self) -> list[float]:
        return [1.0, 0.0, 0.0] if self.axis == "x" else [0.0, -1.0, 0.0]

    def build(self, spec):
        pos = [*self.base_xy, 0.0 if self.moves["height"] else self.off["height"]]
        body = spec.worldbody.add_body(name=self.body_name, pos=pos)
        if not self.moves["tilt"] and self.off["tilt"]:
            ax = np.asarray(self._hinge_axis())
            a = math.radians(self.off["tilt"])
            body.quat = [math.cos(a / 2), *(math.sin(a / 2) * ax)]
        if self.moves["height"]:
            j = body.add_joint(name=self.body_name + "/lift", type=mujoco.mjtJoint.mjJNT_SLIDE,
                               axis=[0, 0, 1], range=[-MAX_HEIGHT - 0.01, MAX_HEIGHT + 0.01],
                               limited=mujoco.mjtLimited.mjLIMITED_TRUE, damping=50.0)
            act = spec.add_actuator(name=self.body_name + "/lift_motor", target=j.name,
                                    trntype=mujoco.mjtTrn.mjTRN_JOINT)
            act.set_to_position(kp=1.0e4, dampratio=1.0)
        if self.moves["tilt"]:
            lim = MAX_TILT + 1.0            # MjSpec joint ranges are in degrees
            j = body.add_joint(name=self.body_name + "/tilt", type=mujoco.mjtJoint.mjJNT_HINGE,
                               axis=self._hinge_axis(), range=[-lim, lim],
                               limited=mujoco.mjtLimited.mjLIMITED_TRUE, damping=5.0, armature=0.1)
            act = spec.add_actuator(name=self.body_name + "/tilt_motor", target=j.name,
                                    trntype=mujoco.mjtTrn.mjTRN_JOINT)
            act.set_to_position(kp=400.0, dampratio=1.0)
        if self.moves["height"] or self.moves["tilt"]:
            body.gravcomp = 1.0
            spec.add_exclude(bodyname1="world", bodyname2=self.body_name)
        body.add_geom(name=self.body_name + "/tile", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*self.half, TILE_THICKNESS / 2], pos=[0, 0, -TILE_THICKNESS / 2],
                      friction=list(self.off["friction"]), priority=1, mass=TILE_MASS,
                      rgba=(0.55, 0.7, 0.85, 1.0))
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._gid = model.geom(self.body_name + "/tile").id
        self._j: dict[str, tuple[int, int, int]] = {}
        for key, jn in (("height", "lift"), ("tilt", "tilt")):
            if self.moves[key]:
                j = model.joint(f"{self.body_name}/{jn}")
                self._j[key] = (int(j.qposadr[0]), int(j.dofadr[0]),
                                model.actuator(f"{self.body_name}/{jn}_motor").id)

    @staticmethod
    def _q(key: str, v) -> float:
        return math.radians(float(v)) if key == "tilt" else float(v)

    def reset(self, model, data):
        self.cur = {k: np.array(v, float) for k, v in self.off.items()}
        model.geom_friction[self._gid] = self.off["friction"]
        for key, (qa, va, act) in self._j.items():
            data.qpos[qa] = data.ctrl[act] = self._q(key, self.off[key])
            data.qvel[va] = 0.0

    # ---- runtime ----------------------------------------------------------------
    def update(self, model, data, body):
        if not self.enabled():
            return
        want = self.on if self.inputs and self.inputs_on() else self.off
        rate = self.rate
        dt = self.bus.dt
        for key in KEYS:
            if key != "friction" and key not in self._j:
                continue
            cur, tgt, step = self.cur[key], np.asarray(want[key], float), rate[key] * dt
            d = tgt - cur
            self.cur[key] = np.where(np.abs(d) <= step, tgt, cur + np.sign(d) * step)
            if key == "friction":
                model.geom_friction[self._gid] = self.cur[key]
            else:
                data.ctrl[self._j[key][2]] = self._q(key, self.cur[key])

    def values(self, data) -> dict[str, object]:
        """Current friction, tilt [deg] and height [m] (joint positions for moving ones)."""
        out: dict[str, object] = {"friction": self.cur["friction"].copy()}
        for key in ("tilt", "height"):
            if key in self._j:
                q = float(data.qpos[self._j[key][0]])
                out[key] = math.degrees(q) if key == "tilt" else q
            else:
                out[key] = float(self.cur[key])
        return out

    def height_at(self, xy):
        data = self.world.data if self.world is not None and getattr(self.world, "data", None) is not None else None
        v = self.values(data) if data is not None else {"tilt": float(self.cur["tilt"]),
                                                          "height": float(self.cur["height"])}
        d = (xy[1] - self.xy[1]) if self.axis == "x" else (xy[0] - self.xy[0])
        top = float(v["height"]) + d * math.tan(math.radians(float(v["tilt"])))
        return top, top

    def state(self):
        fr = []
        on, off = self.on, self.off
        for key in KEYS:
            span = np.abs(np.asarray(on[key]) - np.asarray(off[key])).max()
            if self.moves[key] and span > 0:
                fr.append(float(np.abs(self.cur[key] - off[key]).max() / span))
        return float(np.clip(np.mean(fr), 0.0, 1.0)) if fr else 0.0

    def save_state(self):
        return {"cur": {k: np.array(v) for k, v in self.cur.items()}}
