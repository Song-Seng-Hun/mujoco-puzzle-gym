"""Relay (R34): a box that passes a channel through, so a circuit has a place to cut.

It reads `in` and writes its own channel, like a one-input gate, but it always
stands in the room as a box with a lamp (`body: true` by default), because the
point of a relay is being somewhere the duck, a shell or a press can reach it.
`breakable: true` (R34) is the usual pairing: a broken relay freezes at
`broken_value`, so everything behind it goes dark.

`movable: true` makes the box a free body light enough to push, and with
`hitch: true` a tow rope (R31) can tie to it, so a puzzle can ask for the relay
to be dragged somewhere (under a press) instead of broken where it stands. A
movable relay is not a wire end: a wire to it is refused (R33), and that link
belongs to a transceiver (R8).
"""

from __future__ import annotations

import mujoco
import numpy as np

from ...schema import Field, SchemaError
from ..breakable import BREAK_KEYS, Breakable
from .base import BodiedLogic

__all__ = ["Relay"]


class Relay(BodiedLogic, Breakable):
    type_name = "relay"
    role = "prop"
    stage = "logic"
    emits = True
    state_keys = ("value",) + BREAK_KEYS
    fields = (
        Field("body", "bool", True,
              doc="a relay stands in the room (its whole point); `false` is a bare channel"),
        Field("movable", "bool", False,
              doc="free box: it can be pushed, and towed with `hitch: true` (R31)"),
        Field("mass", "float", 0.5, unit="kg", min=0.05, max=20.0,
              doc="`movable`: box mass"),
        Field("friction", "float", 1.0, min=0.0, max=2.0, doc="`movable`: sliding friction"),
        Field("hitch", "bool", False, doc="`movable`: a tow rope (R31) can tie to it"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        self.movable = bool(c["movable"])
        if self.movable and not self.has_body:
            raise SchemaError(f"part {name!r}: `movable` needs a body (`body: true`)")
        if c["hitch"] and not self.movable:
            raise SchemaError(f"part {name!r}: `hitch` needs `movable: true`")
        if not self.inputs:
            raise SchemaError(f"part {name!r}: a relay needs an input channel (`in`)")
        if self.movable:
            self.kind = "object"            # sensors and tow ropes see a free box
        self.value = 0.0
        self._q = self._v = -1

    # ---- scene ----------------------------------------------------------------------
    def build(self, spec):
        body = super().build(spec)
        if body is None:
            return None
        if self.movable:
            body.add_freejoint()
            # A free body's frame is its own, so the geoms sit around the box centre,
            # not above the floor as they do on a static box.
            for g in body.geoms:
                if g.name.endswith("/box"):
                    g.pos = [0.0, 0.0, 0.0]
                    g.mass = float(self.cfg["mass"])
                    g.friction = [float(self.cfg["friction"]), 0.005, 0.0001]
                else:
                    g.pos = [0.0, 0.0, self.height / 2 + 0.009]
                    g.mass = 0.0                        # the lamp weighs nothing
        return body

    def bind(self, model, data):
        super().bind(model, data)
        if self.movable:
            j = int(model.body_jntadr[self._bid])
            self._q = int(model.jnt_qposadr[j])
            self._v = int(model.jnt_dofadr[j])
        self.break_bind(model, data)

    def place(self, model, xy):
        if self.movable:
            self.xy = np.asarray(xy, dtype=float)       # a free body is placed in `reset`
        else:
            super().place(model, xy)

    def root_body_ids(self, model):
        return [self._bid] if self.movable and self._bid >= 0 else []

    # ---- runtime --------------------------------------------------------------------
    def reset(self, model, data):
        if self.movable and self._q >= 0:
            data.qpos[self._q:self._q + 7] = [*self.xy, self.height / 2, 1.0, 0.0, 0.0, 0.0]
            data.qvel[self._v:self._v + 6] = 0.0
        self.value = 0.0
        self.break_reset(model)
        if self.bus is not None:
            self.write(self.value)
        self.paint(model)

    def update(self, model, data, body):
        if self.break_step(model, data, body):
            self.value = self.broken_value
            self.paint(model)
            return
        if not self.enabled():
            self.value = 0.0
        else:
            self.value = float(self.read(self.inputs[0])) if len(self.inputs) == 1 \
                else float(self.inputs_on())
        self.write(self.value)
        self.paint(model)

    def state(self):
        return float(np.clip(self.value, 0.0, 1.0))

    def plan_values(self, on, get):
        """Planning: it passes its input through (it is taken as whole, R34)."""
        v = float(get(self.inputs[0])) if len(self.inputs) == 1 else float(self.plan_inputs_on(get))
        return self.break_plan_values({ch: v for ch in self.value_outputs()})

    def load_state(self, model, data, st):
        super().load_state(model, data, st)
        self.break_load(model)
        self.paint(model)

    # ---- the box's own geometry ------------------------------------------------------
    def pos(self, data) -> np.ndarray:
        """Where the box is now (a movable one has moved)."""
        if self.movable and self._q >= 0:
            return np.asarray(data.qpos[self._q:self._q + 3], dtype=float)
        return np.array([*self.xy, self.height / 2])

    def hitch_point(self, data) -> np.ndarray:
        """Centre of the box, where a tow rope ties (R31)."""
        p = self.pos(data)
        return np.array([p[0], p[1], max(p[2], self.height / 2)])

    def blockers(self, is_open):
        """A movable box is not a wall: the planner may push it, so it reports nothing."""
        return [] if self.movable else super().blockers(is_open)
