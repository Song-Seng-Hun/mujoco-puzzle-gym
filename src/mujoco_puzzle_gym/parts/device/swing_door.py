"""Hinged door (R9): a panel turning about a vertical hinge at one end.

Closed it spans a gap of `width` along `axis`, centred at `pos`. The hinge sits
at the `hinge` end (-1: the -axis end, 1: the +axis end) and the door swings
by `angle` degrees toward the `open_dir` side (+1: +x for a door along y, +y for
a door along x). `mode` and `speed` (deg/s) work as for the sliding door. The
panel collides with the body and objects, not with walls.

For planning, the closed panel blocks the gap; the open panel blocks the
segment where it rests (so leave room for it to swing).
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from ...layout import Segment
from ...schema import Field
from .door import OPEN_FRACTION, Opening


class SwingDoor(Opening):
    role = "gate"
    state_keys = ("q", "goal", "latched")
    fields = (
        Field("width", "float", 0.5, unit="m", min=0.1, max=2.0, doc="opening the panel spans"),
        Field("axis", "enum", "y", choices=("x", "y"), doc="direction the closed panel lies along"),
        Field("thickness", "float", 0.04, unit="m", min=0.01, max=0.2),
        Field("height", "float", 0.25, unit="m", min=0.05, max=1.0),
        Field("hinge", "enum", -1, choices=(-1, 1), doc="hinge at the -axis (-1) or +axis (1) end"),
        Field("open_dir", "enum", 1, choices=(1, -1), doc="side the door swings to (+1: +x or +y)"),
        Field("angle", "float", 90.0, unit="deg", min=10.0, max=170.0, doc="opening angle"),
        Field("speed", "float", None, unit="deg/s", min=5.0, max=1000.0,
              doc="opening speed (default: as fast as the motor turns it)"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        self.width = float(c["width"])
        self.axis = c["axis"]
        self.thickness = float(c["thickness"])
        self.height = float(c["height"])
        self.hinge_end = int(c["hinge"])
        self.open_side = int(c["open_dir"])
        self.angle = math.radians(float(c["angle"]))
        self.speed = math.radians(c["speed"]) if c["speed"] is not None else None
        a = self._axis_vec()
        self.panel_dir = -self.hinge_end * a              # hinge -> free end, closed
        n = np.array([1.0, 0.0]) if self.axis == "y" else np.array([0.0, 1.0])
        turned = np.array([-self.panel_dir[1], self.panel_dir[0]])   # +90 deg about z
        self.sign = 1.0 if turned @ (self.open_side * n) > 0 else -1.0
        self.q = self.goal = 0.0
        self._init_opening()

    def _axis_vec(self) -> np.ndarray:
        return np.array([0.0, 1.0]) if self.axis == "y" else np.array([1.0, 0.0])

    def hinge_xy(self, xy=None) -> np.ndarray:
        xy = self.xy if xy is None else np.asarray(xy, float)
        return xy + self.hinge_end * self._axis_vec() * (self.width / 2)

    def build(self, spec):
        hx, hy = self.hinge_xy(self.base_xy)
        body = spec.worldbody.add_body(name=self.body_name, pos=[hx, hy, 0.0])
        lo, hi = sorted((0.0, self.sign * math.degrees(self.angle)))
        # MjSpec joint ranges are in degrees (compiler default); qpos and ctrl are radians.
        joint = body.add_joint(name=self.body_name + "/hinge", type=mujoco.mjtJoint.mjJNT_HINGE,
                               axis=[0, 0, 1], range=[lo - 3.0, hi + 3.0],
                               limited=mujoco.mjtLimited.mjLIMITED_TRUE, damping=1.0, armature=0.02)
        half_len = self.width / 2
        c = self.panel_dir * half_len
        size = [half_len, self.thickness / 2] if self.axis == "x" else [self.thickness / 2, half_len]
        body.add_geom(name=self.body_name + "/panel", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*size, self.height / 2], pos=[c[0], c[1], self.height / 2 + 0.003],
                      rgba=(0.5, 0.3, 0.25, 1.0), density=300.0)
        body.add_geom(name=self.body_name + "/post", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[self.thickness * 0.6, self.height / 2], pos=[0, 0, self.height / 2 + 0.003],
                      rgba=(0.3, 0.3, 0.3, 1.0), contype=0, conaffinity=0, mass=0.01)
        act = spec.add_actuator(name=self.body_name + "/motor", target=joint.name,
                                trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.set_to_position(kp=600.0, dampratio=1.0)
        spec.add_exclude(bodyname1="world", bodyname2=self.body_name)
        return body

    def place(self, model, xy):
        self.xy = np.asarray(xy, dtype=float)
        model.body_pos[self._bid, :2] = self.hinge_xy()

    def bind(self, model, data):
        super().bind(model, data)
        j = model.joint(self.body_name + "/hinge")
        self._qadr = int(j.qposadr[0])
        self._vadr = int(j.dofadr[0])
        self._act = model.actuator(self.body_name + "/motor").id

    def reset(self, model, data):
        data.qpos[self._qadr] = 0.0
        data.qvel[self._vadr] = 0.0
        data.ctrl[self._act] = 0.0
        self.q = self.goal = 0.0
        self.latched = False

    def update(self, model, data, body):
        if self.enabled():
            target = self.sign * self.angle if self.want_open() else 0.0
            self.goal = self.ramp(self.goal, target, self.speed, self.bus.dt)
            data.ctrl[self._act] = self.goal
        self.q = float(data.qpos[self._qadr])

    def state(self):
        return float(np.clip(self.sign * self.q / self.angle, 0.0, 1.0))

    def is_open(self) -> bool:
        return self.state() >= OPEN_FRACTION

    def blockers(self, is_open):
        h = self.hinge_xy()
        d = self.panel_dir
        if is_open:
            a = self.sign * self.angle
            d = np.array([math.cos(a) * d[0] - math.sin(a) * d[1], math.sin(a) * d[0] + math.cos(a) * d[1]])
        return [Segment(tuple(h), tuple(h + d * self.width), self.thickness)]
