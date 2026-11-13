"""Signal source (R33): a box in the room that drives a channel every body step.

Modes:
- `const`:   `value`, every step.
- `clock`:   on for `duty` of every `period` [s], shifted by `phase` [s].
- `pattern`: the values of `pattern`, one every `step` [s], repeating.
- `random`:  a value in [`low`, `high`], redrawn every `hold` [s].

`enable` off writes `off_value` (default 0) and stops the clock: the time inside
the source only advances on steps where it is enabled, so a gated clock keeps
its phase instead of running on in the dark.

The `random` mode is seeded from the part name and the room's `seed`, like the
cannon's spread (R16): the same room gives the same sequence in every episode, so
a `demo:` solution and `mpg check` are reproducible. Per-episode variety is R35's
job (randomisation), not this part's.

Its own time, its drawn value and its generator are in checkpoint snapshots (R6),
so a restored episode continues the phase instead of starting the period again.
`body: true` (the default here, unlike gates) puts it in the room with a lamp
that shows the current value; with `body: false` it is a channel with no place.
"""

from __future__ import annotations

import zlib

import numpy as np

from ...schema import Field, SchemaError
from .base import BodiedLogic

MODES = ("const", "clock", "pattern", "random")


class Source(BodiedLogic):
    type_name = "source"
    stage = "input"
    stateful = True
    emits = True
    state_keys = ("t", "value", "held", "until", "rng")
    fields = (
        Field("body", "bool", True,
              doc="put the source in the room as a box with a lamp (R33)"),
        Field("mode", "enum", "const", choices=MODES, doc="const | clock | pattern | random"),
        Field("value", "float", 1.0, min=-1000.0, max=1000.0, runtime=True,
              doc="`const`: the value it sends"),
        Field("period", "float", 2.0, unit="s", min=0.02, max=3600.0, runtime=True,
              doc="`clock`: length of one cycle"),
        Field("duty", "float", 0.5, min=0.0, max=1.0, runtime=True,
              doc="`clock`: share of the cycle that is on"),
        Field("phase", "float", 0.0, unit="s", min=0.0, max=3600.0,
              doc="`clock` / `pattern`: shift of the cycle at the episode start"),
        Field("pattern", "vec", None, min=-1000.0, max=1000.0,
              doc="`pattern`: the values, in order"),
        Field("step", "float", 1.0, unit="s", min=0.02, max=3600.0,
              doc="`pattern`: seconds per value"),
        Field("hold", "float", 1.0, unit="s", min=0.02, max=3600.0, runtime=True,
              doc="`random`: seconds a drawn value is held"),
        Field("low", "float", 0.0, min=-1000.0, max=1000.0, doc="`random`: lowest value"),
        Field("high", "float", 1.0, min=-1000.0, max=1000.0, doc="`random`: highest value"),
        Field("off_value", "float", 0.0, min=-1000.0, max=1000.0,
              doc="what it writes while `enable` is off"),
        Field("seed", "int", 0, min=0, max=2**31 - 1, doc="`random`: seed beside the part name"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        self.mode = c["mode"]
        self.phase = float(c["phase"])
        self.off_value = float(c["off_value"])
        self.pattern = [float(v) for v in c["pattern"]] if c["pattern"] else []
        if self.mode == "pattern" and not self.pattern:
            raise SchemaError(f"part {name!r}: mode `pattern` needs a non-empty `pattern` list")
        if self.mode != "pattern" and c["pattern"]:
            raise SchemaError(f"part {name!r}: `pattern` only means something in mode `pattern`, "
                              f"not {self.mode!r}")
        if c["low"] > c["high"]:
            raise SchemaError(f"part {name!r}: needs low <= high, got {c['low']} > {c['high']}")
        self.low, self.high = float(c["low"]), float(c["high"])
        self.seed = int(c["seed"])
        self._reset_state()

    def _reset_state(self) -> None:
        self.t = 0.0
        self.rng = np.random.default_rng(zlib.crc32(self.name.encode()) + self.seed)
        self.held = self.low
        self.until = 0.0
        self.value = self.off_value

    # ---- runtime ------------------------------------------------------------------
    def reset(self, model, data):
        self._reset_state()
        self.value = self._value_now()
        if self.bus is not None:
            self.write(self.value)
        self.paint(model)

    def _value_now(self) -> float:
        """The value for the source's own time `t` (the drawn value for `random`)."""
        if self.mode == "const":
            return float(self.live["value"])
        if self.mode == "clock":
            period = float(self.live["period"])
            duty = float(self.live["duty"])
            return float((self.t + self.phase) % period < duty * period)
        if self.mode == "pattern":
            step = float(self.cfg["step"])
            i = int((self.t + self.phase) / step) % len(self.pattern)
            return self.pattern[i]
        while self.t >= self.until:                  # random: redraw on the hold boundary
            self.held = float(self.rng.uniform(self.low, self.high))
            self.until += float(self.live["hold"])
        return self.held

    def update(self, model, data, body):
        if not self.enabled():
            self.value = self.off_value
            self.write(self.value)
            self.paint(model)
            return
        self.t += self.bus.dt
        self.value = self._value_now()
        self.write(self.value)
        self.paint(model)

    def state(self):
        return float(np.clip(self.value, 0.0, 1.0))

    # ---- planning and snapshots --------------------------------------------------------
    def plan_values(self, on, get):
        """Planning takes the source as "eventually at its highest": a clock door is
        open at some point, a pattern reaches its largest value."""
        if self.mode == "const":
            v = float(self.live["value"])
        elif self.mode == "clock":
            v = float(float(self.live["duty"]) > 0.0)
        elif self.mode == "pattern":
            v = max(self.pattern)
        else:
            v = self.high
        return {ch: v for ch in self.outputs}

    def load_state(self, model, data, st):
        super().load_state(model, data, st)
        self.paint(model)
