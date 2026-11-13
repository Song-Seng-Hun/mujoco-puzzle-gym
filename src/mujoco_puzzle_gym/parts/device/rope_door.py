"""Rope door (R31 `cleat_hold`): a passive door a tow rope lifts.

A panel on a vertical slide joint in a wall gap, like the sliding door (R9: `width`,
`axis`, `thickness`, `height`; collides with the walking body and objects, not with
walls or the floor). It has no actuator: gravity closes it (joint at 0, resting on its
lower limit) and only a rope pulls it up. Its hitch (`hitch_kind` "door", a fixed hitch:
warps untie it, rope.py) sits on the top edge of the panel, so a rope through a ceiling
pulley (`via:`) right above it lifts it when its other end is pulled away. A rope door
needs no `in` channels; R9's `mode`, `speed` and `enable` do not apply.

`mass` (default 0.05 kg) is at most 0.15 kg (weight 1.47 N, spec R31 at most 1.5 N):
standing at command 0 the body holds 2.2 N backward at its 0.04 m hitch (OQ9). The rope
that lifts it carries the panel and its 20 g knot (0.69 N at the default; walking it up
through a pulley, 0.78 N median, 0.89 N at the 95th percentile, measured in `cleat_hold`),
close to the default `max_tension` 0.9 N, so give that rope its own `max_tension`
(`cleat_hold`: 1.5 N).

Open and planning. `clearance` is the gap under the panel the walking body needs: the
default walking body's highest colliding geom tops out at 0.30 m walking (measured
2026-11-13, aabb of its geoms; head command -1 / 0 / 1: 0.300 / 0.303 / 0.295 m), so
the default 0.34 m leaves 4 cm.
`state()` (and channel `<name>.lift`) is the lift over `clearance`, 0..1; `<name>.open`
is 1 once the lift reaches `clearance`. The planners treat it as a gate: in the static
order (`mpg check`) it opens when a rope in the room can tie to it (it has no switch);
on live walks (`demo:` steps, the grid answer) it blocks until `state()` reaches 1
(`passable_state`), not at the 0.3 other gates pass at: a half-lifted panel hits the head.
`travel` is the most it lifts (joint range).
"""

from __future__ import annotations

import mujoco
import numpy as np

from ... import markers
from ...layout import Segment
from ...schema import Field, SchemaError
from ..base import Part

PANEL_RGBA = (0.45, 0.4, 0.3, 1.0)
CLEARANCE = 0.34      # [m] default walking body's top 0.30 m (measured) + 4 cm


class RopeDoor(Part):
    role = "gate"
    marked = True
    marker_anchor = "/panel"
    marker_faces = markers.FACES_PANEL
    stage = "device"
    hitch_kind = "door"          # a fixed rope hitch on the panel's top edge (rope.py)
    passable_state = 1.0         # live planning: passable only once lifted to `clearance`
    suffixes = (".open", ".lift")
    state_keys = ("q",)
    fields = (
        Field("width", "float", 0.5, unit="m", min=0.1, max=2.0, doc="opening the panel spans"),
        Field("axis", "enum", "y", choices=("x", "y"), doc="direction the panel lies along"),
        Field("thickness", "float", 0.04, unit="m", min=0.01, max=0.2),
        Field("height", "float", 0.25, unit="m", min=0.05, max=1.0),
        Field("mass", "float", 0.05, unit="kg", min=0.01, max=0.15,
              doc="panel mass; its weight (at most 1.47 N) hangs on the rope"),
        Field("clearance", "float", CLEARANCE, unit="m", min=0.1, max=1.0,
              doc="gap under the panel the body needs (open: lifted this far)"),
        Field("travel", "float", 0.45, unit="m", min=0.1, max=1.5, doc="most it lifts"),
        Field("damping", "float", 0.3, unit="N*s/m", min=0.0, max=10.0, doc="slide joint damping"),
    )

    def marker_face_size(self, face):
        return float(min(self.cfg["width"], self.cfg["height"]))

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        self.width = float(c["width"])
        self.axis = c["axis"]
        self.thickness = float(c["thickness"])
        self.height = float(c["height"])
        self.clearance = float(c["clearance"])
        self.travel = float(c["travel"])
        if self.travel < self.clearance:
            raise SchemaError(f"part {name!r}: travel {self.travel:g} m is less than clearance "
                              f"{self.clearance:g} m: it could never open")
        self.liftable = False         # a rope in the room can tie to it (planning)
        self.q = 0.0

    @classmethod
    def add_room_parts(cls, parts):
        from .rope import Rope

        ropes = [p for p in parts.values() if isinstance(p, Rope)]
        for p in parts.values():
            if isinstance(p, RopeDoor):
                p.liftable = any(p.name in r.ends_cfg or None in r.ends_cfg for r in ropes)

    def _axis_vec(self) -> np.ndarray:
        return np.array([0.0, 1.0]) if self.axis == "y" else np.array([1.0, 0.0])

    def hitch_offset(self) -> list[float]:
        """Rope hitch (R31): the middle of the panel's top edge, in the body frame."""
        return [0.0, 0.0, self.height + 0.003]

    def build(self, spec):
        body = super().build(spec)
        body.add_joint(name=self.body_name + "/slide", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 0, 1],
                       range=[0.0, self.travel], limited=mujoco.mjtLimited.mjLIMITED_TRUE,
                       damping=float(self.cfg["damping"]))
        half_len = self.width / 2 + 0.01
        size = [half_len, self.thickness / 2] if self.axis == "x" else [self.thickness / 2, half_len]
        body.add_geom(name=self.body_name + "/panel", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*size, self.height / 2], pos=[0, 0, self.height / 2 + 0.003],
                      rgba=PANEL_RGBA, mass=float(self.cfg["mass"]))
        spec.add_exclude(bodyname1="world", bodyname2=self.body_name)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        j = model.joint(self.body_name + "/slide")
        self._qadr = int(j.qposadr[0])
        self._vadr = int(j.dofadr[0])

    def reset(self, model, data):
        data.qpos[self._qadr] = 0.0
        data.qvel[self._vadr] = 0.0
        self.q = 0.0

    def update(self, model, data, body):
        self.q = float(data.qpos[self._qadr])
        s = self.state()
        self.write(s, self.name + ".lift")
        self.write(float(s >= 1.0), self.name + ".open")

    def lift(self, data) -> float:
        """Lift [m] now."""
        return float(data.qpos[self._qadr])

    def state(self):
        return float(np.clip(self.q / self.clearance, 0.0, 1.0))

    def plan_inputs_on(self, get) -> bool:
        return self.liftable

    def blockers(self, is_open):
        if is_open:
            return []
        a = self._axis_vec() * (self.width / 2 + 0.01)
        return [Segment(tuple(self.xy - a), tuple(self.xy + a), self.thickness)]

    def check_static(self, env) -> list[str]:
        if not self.liftable:
            return [f"rope door {self.name!r}: no rope in the room can tie to it (name it as a rope's "
                    f"`a` or `b`, or leave a rope end free for a hitch station), so it never opens"]
        return []
