"""Counter (R8b): counts the turn-ons of channel `up` (minus those of `down`).

Each turn-on (rising edge) of `up` adds 1 and of `down` subtracts 1; a held
input adds nothing more. A turn-on of `reset` sets the count back to `start`
(and wins over `up`/`down` in the same step). The count stays in [min, max];
with `wrap: true` going past `max` gives `min` and below `min` gives `max`.

`body: true` puts the counter in the room as a box with a lamp (R33, `base.py`).

Channels: `<name>.count` (the count, an integer) and `<name>.reached`
(1 when count >= target). The count is part of checkpoint snapshots (R6);
with `persist: true` it survives a restore (respawn, reset warp) instead of
going back to the snapshot's value. A new episode always starts at `start`.
With `enable` off the count holds and turn-ons are ignored.
"""

from __future__ import annotations

import numpy as np

from ...schema import Field, SchemaError
from ...signals import is_on
from ..breakable import BREAK_KEYS
from .base import BodiedLogic

LIMIT = 1_000_000


class Counter(BodiedLogic):
    stage = "logic"
    stateful = True
    suffixes = (".count", ".reached")
    fields = (
        Field("pos", "vec", [0.0, 0.0], length=2, unit="m",
              doc="where the counter stands (`body: true`) or where guides draw it"),
        Field("up", "str", doc="channel whose turn-ons add 1"),
        Field("down", "str", None, doc="channel whose turn-ons subtract 1"),
        Field("reset", "str", None, doc="channel whose turn-on sets the count to start"),
        Field("start", "int", 0, min=-LIMIT, max=LIMIT),
        Field("min", "int", 0, min=-LIMIT, max=LIMIT),
        Field("max", "int", 1000, min=-LIMIT, max=LIMIT),
        Field("wrap", "bool", False, doc="past max -> min, below min -> max"),
        Field("target", "int", 1, min=-LIMIT, max=LIMIT, doc="`<name>.reached` is 1 at count >= target"),
        Field("persist", "bool", False, doc="keep the count when a snapshot is restored"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        if not c["min"] <= c["start"] <= c["max"]:
            raise SchemaError(f"part {name!r}: needs min <= start <= max, got "
                              f"{c['min']} <= {c['start']} <= {c['max']}")
        self.up, self.down, self.reset_ch = c["up"], c["down"], c["reset"]
        self.inputs = [ch for ch in (self.up, self.down, self.reset_ch) if ch]
        self.lo, self.hi, self.start = c["min"], c["max"], c["start"]
        self.wrap, self.target, self.persist = c["wrap"], c["target"], c["persist"]
        self.count = self.start
        self.prev = {ch: False for ch in self.inputs}

    def reset(self, model, data):
        self.count = self.start
        self.prev = {ch: False for ch in self.inputs}
        self.break_reset(model)
        self._write()
        self.paint(model)

    def _rising(self, ch: str | None) -> bool:
        if ch is None:
            return False
        now = is_on(self.read(ch))
        up = now and not self.prev[ch]
        self.prev[ch] = now
        return up

    def _write(self) -> None:
        if self.bus is not None:
            self.write(float(self.count), self.outputs[0])
            self.write(float(self.count >= self.target), self.outputs[1])

    def update(self, model, data, body):
        if self.break_step(model, data, body):          # broken: the output is frozen (R34)
            self.paint(model)
            return
        r, u, d = self._rising(self.reset_ch), self._rising(self.up), self._rising(self.down)
        if not self.enabled():
            return
        if r:
            self.count = self.start
        else:
            n = self.count + int(u) - int(d)
            if n > self.hi:
                n = self.lo if self.wrap else self.hi
            elif n < self.lo:
                n = self.hi if self.wrap else self.lo
            self.count = n
        self._write()
        self.paint(model)

    def state(self):
        if self.target <= 0:
            return float(self.count >= self.target)
        return float(np.clip(self.count / self.target, 0.0, 1.0))

    def ratio(self) -> float:
        """Count over target (observation term `counters`)."""
        return self.count / self.target if self.target else float(self.count >= self.target)

    # ---- planning and snapshots --------------------------------------------------------
    def plan_values(self, on, get):
        can_count = is_on(get(self.up)) and self.target <= self.hi
        reached = self.start >= self.target or can_count
        return {self.outputs[0]: float(self.target if can_count else self.start),
                self.outputs[1]: float(reached)}

    def save_state(self):
        return {"count": self.count, "prev": dict(self.prev),
                **{k: getattr(self, k) for k in BREAK_KEYS}}

    def load_state(self, model, data, st):
        if not self.persist:
            self.count, self.prev = st["count"], dict(st["prev"])
        for k in BREAK_KEYS:
            if k in st:
                setattr(self, k, st[k])
        self.break_load(model)
        self._write()
        self.paint(model)
