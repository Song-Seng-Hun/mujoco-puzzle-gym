"""Gate: a logic part that computes its channel from other channels (R7).

Exactly one of:
- `expr`: a safe expression (expr.py), e.g. `a and not b`, `delay(btn, 1.5)`.
  The channels it names are its inputs.
- `op` over `in`: preset form that does not name channels, so a preset can be
  reused: `and | or | xor` (all inputs), `not | delay | hold | pulse | timer |
  count` (one input; `seconds` for delay and hold), `latch` (in: [set, reset]).
- `sequence: [a, b, c]`: sequence lock. Each input's turn-on must come in this
  order. When the last one arrives the gate writes 1 for one body step and
  waits for the order again (`latch: true`: stays 1). A turn-on out of order
  writes `<name>.wrong` = 1 for one body step and starts the order again (an
  out-of-order `a` counts as a new start).
- `logic: <name>`: a Python function registered with `register_logic`
  (fn(inputs, state, dt) -> float); room.yaml never holds code. Set
  `stateful: true` when its output does not depend on this step's input.

The gate has no MuJoCo body unless `body: true` puts one in the room (R33,
`logic/base.py`). With `enable` off its output holds and its time functions stop.
"""

from __future__ import annotations

import copy

from ...expr import OPS, ExprError, compile_expr, op_expr
from ...registry import get_logic
from ...schema import Field, SchemaError
from ...signals import is_on
from ..breakable import BREAK_KEYS
from .base import BodiedLogic

MODES = ("expr", "op", "sequence", "logic")


class Gate(BodiedLogic):
    stage = "logic"
    emits = True
    fields = (
        Field("pos", "vec", [0.0, 0.0], length=2, unit="m",
              doc="where the gate stands (`body: true`) or where guides draw it"),
        Field("expr", "str", None, doc="safe expression over channels (R7)"),
        Field("op", "enum", None, choices=OPS, doc="preset logic over `in`"),
        Field("seconds", "float", 1.0, unit="s", min=0.0, max=3600.0,
              doc="time for op delay / hold"),
        Field("sequence", "list", None, doc="sequence lock: channels that must turn on in this order"),
        Field("latch", "bool", False, doc="sequence lock: stay on once the order is complete"),
        Field("logic", "str", None, doc="registered Python logic (register_logic)"),
        Field("stateful", "bool", False,
              doc="logic plugins: output does not depend on this step's input (may sit in a loop)"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        cfg = self.cfg
        modes = [m for m in MODES if cfg[m] is not None]
        if len(modes) != 1:
            raise SchemaError(f"part {name!r}: a gate needs exactly one of "
                              f"{', '.join(MODES)}; got {modes or 'none'}")
        self.mode = modes[0]
        self.expr = None
        self.fn = None
        self.plugin_state: dict = {}
        try:
            if self.mode == "expr":
                self.expr = compile_expr(cfg["expr"])
                self.inputs = list(dict.fromkeys(self.inputs + self.expr.channels))
            elif self.mode == "op":
                self.expr = op_expr(cfg["op"], self.inputs, cfg["seconds"])
        except ExprError as exc:
            raise SchemaError(f"part {name!r}: {exc}") from None
        if self.mode == "sequence":
            self.sequence = list(cfg["sequence"])
            self.inputs = list(dict.fromkeys(self.sequence))
            self.outputs.append(name + ".wrong")
            self.latch = cfg["latch"]
        elif self.mode == "logic":
            try:
                self.fn = get_logic(cfg["logic"])
            except KeyError as exc:
                raise SchemaError(f"part {name!r}: {exc.args[0]}") from None
            if not self.inputs:
                raise SchemaError(f"part {name!r}: logic {cfg['logic']!r} needs input channels in `in`")
        self.stateful = (self.expr.stateful if self.expr is not None
                         else True if self.mode == "sequence" else cfg["stateful"])
        self._reset_state()

    def _reset_state(self) -> None:
        self.value = 0.0
        if self.expr is not None:
            self.expr.reset()
        self.plugin_state = {}
        self.index = 0                      # sequence: next expected position
        self.prev = {ch: False for ch in self.inputs}
        self.done = False

    # ---- runtime -------------------------------------------------------------------
    def reset(self, model, data):
        self._reset_state()
        self.break_reset(model)
        self.paint(model)

    def _sequence(self) -> tuple[float, float]:
        rising = []
        for ch in self.inputs:
            now = is_on(self.read(ch))
            if now and not self.prev[ch]:
                rising.append(ch)
            self.prev[ch] = now
        if self.done:                       # latched
            return 1.0, 0.0
        out = wrong = 0.0
        for ch in sorted(rising, key=self.sequence.index):
            if ch == self.sequence[self.index]:
                self.index += 1
                if self.index == len(self.sequence):
                    out, self.index = 1.0, 0
                    self.done = self.latch
                    if self.done:
                        break
            else:
                wrong = 1.0
                self.index = 1 if ch == self.sequence[0] else 0
        return out, wrong

    def update(self, model, data, body):
        if self.break_step(model, data, body):          # broken: the output is frozen (R34)
            self.value = self.broken_value
            self.paint(model)
            return
        if not self.enabled():
            return
        if self.mode == "sequence":
            self.value, wrong = self._sequence()
            self.write(self.value, self.outputs[0])
            self.write(wrong, self.outputs[-1])
            self.paint(model)
            return
        if self.mode == "logic":
            vals = {ch: self.read(ch) for ch in self.inputs}
            self.value = float(self.fn(vals, self.plugin_state, self.bus.dt))
        else:
            self.value = self.expr.eval(self.read, self.bus.dt)
        for ch in self.outputs:
            self.write(self.value, ch)
        self.paint(model)

    def state(self):
        return float(is_on(self.value))

    # ---- planning and snapshots --------------------------------------------------------
    def plan_values(self, on, get):
        if self.mode == "sequence":
            v = float(all(is_on(get(ch)) for ch in self.inputs))
            return {self.outputs[0]: v, self.outputs[-1]: 0.0}
        if self.mode == "logic":            # unknown Python: assume it can turn on
            v = float(any(is_on(get(ch)) for ch in self.inputs))
        else:
            v = self.expr.plan(get)
        return {ch: v for ch in self.outputs}

    def save_state(self):
        return {"value": self.value, "index": self.index, "prev": dict(self.prev), "done": self.done,
                "expr": copy.deepcopy(self.expr.get_state()) if self.expr is not None else None,
                "plugin": copy.deepcopy(self.plugin_state),
                **{k: getattr(self, k) for k in BREAK_KEYS}}

    def load_state(self, model, data, st):
        self.value, self.index, self.done = st["value"], st["index"], st["done"]
        self.prev = dict(st["prev"])
        if self.expr is not None:
            self.expr.set_state(copy.deepcopy(st["expr"]))
        self.plugin_state = copy.deepcopy(st["plugin"])
        for k in BREAK_KEYS:
            if k in st:
                setattr(self, k, st[k])
        self.break_load(model)
        self.paint(model)
