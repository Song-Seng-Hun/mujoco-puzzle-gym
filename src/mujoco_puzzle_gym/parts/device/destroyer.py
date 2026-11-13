"""Destroyer (R34): a hammer or press that breaks circuit parts on a signal.

On every rising edge of `in` it strikes once: every breakable part (R34) named
in `targets`, or standing in the rectangle `size` around `pos` when `targets`
is empty, loses one hit. `cooldown` is the shortest time between strikes and
`charges` the number of strikes it has (none given: unlimited); `<name>.charges`
reports what is left, and `<name>.struck` pulses for the body step of a strike.

It acts only on breakable parts: the hammer is a shape, not a collider, so the
duck and the objects feel nothing from it even standing right under it. That is
what the spec asks for, and it also keeps a press from turning into a way to
launch the duck. A puzzle that wants the relay moved under the press pushes or
tows it there (`relay` with `movable: true`, R31), and the press still only
reaches what is breakable.
"""

from __future__ import annotations

import mujoco
import numpy as np

from ...schema import Field, SchemaError
from ..base import Part
from ..breakable import Breakable

FRAME_RGBA = (0.35, 0.35, 0.4, 1.0)
HEAD_RGBA = (0.7, 0.2, 0.2, 1.0)
HIT_RGBA = (1.0, 0.75, 0.2, 1.0)
HIT_SHOW = 0.2          # [s] the head stays lit after a strike


class Destroyer(Part):
    type_name = "destroyer"
    role = "prop"
    stage = "device"
    emits = False
    suffixes = (".charges", ".struck")
    state_keys = ("charges_left", "t", "last_hit", "prev_on", "struck")
    fields = (
        Field("size", "vec", [0.3, 0.3], length=2, unit="m", min=0.05, max=5.0,
              doc="area it reaches, around `pos` (ignored when `targets` is given)"),
        Field("targets", "list", [], doc="parts it strikes (default: whatever is in its area)"),
        Field("height", "float", 0.3, unit="m", min=0.1, max=1.0, doc="how high the frame stands"),
        Field("cooldown", "float", 0.5, unit="s", min=0.0, max=60.0, runtime=True,
              doc="shortest time between two strikes"),
        Field("charges", "int", None, min=1, max=100,
              doc="strikes it has (none: unlimited)"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        self.half = np.asarray(c["size"], dtype=float) / 2
        self.target_names: list[str] = list(c["targets"])
        self.height = float(c["height"])
        self.charges = c["charges"]
        if not self.inputs:
            raise SchemaError(f"part {name!r}: a destroyer needs an input channel (`in`) "
                              f"to strike on")
        self.charges_left = -1 if self.charges is None else int(self.charges)
        self.t = 0.0
        self.last_hit = -1e9
        self.prev_on = False
        self.struck = False
        self._targets: list[Breakable] = []
        self._head = -1
        self._lit = False

    @classmethod
    def add_room_parts(cls, parts: dict[str, Part]) -> None:
        """Resolve each destroyer's targets (all breakable parts when `targets` is empty)."""
        breakables = [p for p in parts.values() if isinstance(p, Breakable) and p.breakable]
        for d in [p for p in parts.values() if isinstance(p, Destroyer)]:
            if d.target_names:
                missing = [n for n in d.target_names if n not in parts]
                if missing:
                    raise SchemaError(f"part {d.name!r}: `targets` names {missing}, "
                                      f"which are not parts in this room")
                d._targets = [parts[n] for n in d.target_names]
            else:
                d._targets = breakables

    # ---- scene ----------------------------------------------------------------------
    def build(self, spec):
        body = super().build(spec)
        hx, hy = self.half
        for sx in (-1, 1):                      # two posts, so the shape reads as a press
            body.add_geom(name=f"{self.body_name}/post{'+' if sx > 0 else '-'}",
                          type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.015, 0.015, self.height / 2],
                          pos=[sx * hx, 0, self.height / 2], rgba=FRAME_RGBA,
                          contype=0, conaffinity=0)
        body.add_geom(name=self.body_name + "/head", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[hx, hy, 0.02], pos=[0, 0, self.height - 0.02], rgba=HEAD_RGBA,
                      contype=0, conaffinity=0)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._head = model.geom(self.body_name + "/head").id
        self._lit = False

    def paint(self, model) -> None:
        lit = self.t - self.last_hit < HIT_SHOW
        if lit != self._lit:
            self._lit = lit
            model.geom_rgba[self._head] = HIT_RGBA if lit else HEAD_RGBA

    # ---- runtime --------------------------------------------------------------------
    def reset(self, model, data):
        self.charges_left = -1 if self.charges is None else int(self.charges)
        self.t = 0.0
        self.last_hit = -1e9
        self.prev_on = False
        self.struck = False
        self._write()
        self.paint(model)

    def update(self, model, data, body):
        self.t += self.bus.dt
        self.struck = False
        on = self.enabled() and self.inputs_on()
        if on and not self.prev_on and self._ready():
            self.strike(model)
        self.prev_on = on
        self._write()
        self.paint(model)

    def _ready(self) -> bool:
        return (self.charges_left != 0
                and self.t - self.last_hit >= float(self.live["cooldown"]))

    def strike(self, model) -> int:
        """One strike now; returns how many parts it hit."""
        hits = sum(1 for p in self.reach() if p.hit(model, "destroyer"))
        self.last_hit = self.t
        self.struck = True
        if self.charges_left > 0:
            self.charges_left -= 1
        return hits

    def reach(self) -> list[Breakable]:
        """Breakable parts it strikes now (its area is read where the parts are)."""
        if self.target_names:
            return self._targets
        out = []
        for p in self._targets:
            xy = np.asarray(p.pos(self.world.data)[:2] if hasattr(p, "pos") else p.xy, dtype=float)
            if np.all(np.abs(xy - self.xy) <= self.half):
                out.append(p)
        return out

    def _write(self) -> None:
        left = float(self.charges_left) if self.charges_left >= 0 else float(len(self._targets) + 1)
        self.write(left, self.name + ".charges")
        self.write(float(self.struck), self.name + ".struck")

    def state(self):
        if self.charges is None:
            return 1.0
        return float(self.charges_left) / float(self.charges)

    def load_state(self, model, data, st):
        super().load_state(model, data, st)
        self._lit = not (self.t - self.last_hit < HIT_SHOW)
        self.paint(model)

    # ---- checks ----------------------------------------------------------------------
    def check_static(self, env):
        out = []
        bad = [p.name for p in self._targets
               if not (isinstance(p, Breakable) and p.breakable)]
        if bad:
            out.append(f"destroyer {self.name!r}: {bad} are not breakable parts "
                       f"(`breakable: true`, R34)")
        for p in self._targets:
            if isinstance(p, Breakable) and p.breakable and "destroyer" not in p.by:
                out.append(f"destroyer {self.name!r} targets {p.name!r}, which does not "
                           f"list `destroyer` in `by`")
        return out

    def check_warnings(self, env):
        if not self.target_names and not self.reach():
            return [f"destroyer {self.name!r} has no breakable part in its area; "
                    f"a movable relay (R31) has to be brought to it"]
        return []
