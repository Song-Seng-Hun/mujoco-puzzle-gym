"""Budget (R91): a conserved scalar that only ever goes down, and stops its loads at zero.

Nothing in this package knew about running out. `counter` counts up and can be compared, a
`source` can be gated, `timer(x)` reports how long a channel has been on - but every one of
those is a reading, and none of them is *spent*. A room could ask the body to press a button
three times; it could not ask it to decide what to spend one tank of something on.

Two of the three ways to spend were already expressible, and are deliberately not here:

- **By count.** `count(pass) < 3` is a use budget today, so `spend: use` would be a second
  spelling of an existing one (checked 2026-11-17).
- **By unbroken time.** `timer(load) < 10` is a time limit today - but only while the load
  stays on. `timer` is "seconds on without a break" and drops to 0 the moment the channel
  goes off, so a room built on it is solved by switching the load off and on again.

What is left is what this part is for: time that **accumulates across breaks**, and one pool
**several loads draw from**, which no expression can reach because expressions refuse
arithmetic between channels (`a - b` is "BinOp is not allowed") and so cannot add two timers.

`loads` names the channels that draw from the pool. It is a declaration, not a guess: the
room says `door.enable: tank` to make the door depend on the pool and `tank.loads: [open_cmd]`
to make it cost something, exactly as a body declares its own standoff rather than letting
shared code infer one. Writing only one of the two is a room bug this part cannot see, and
`check_static` says so when a budget has no loads at all.

- `spend: time` - each load that is on costs `rate` per second, so two open doors drain it
  twice as fast.
- `spend: flow` - any load being on costs `rate` per second in total: one shared tap.

Channels: its own (or `out`) 1 while anything is left, and `<name>.left` [the amount]. At
zero the part writes 0 for good; a `refill` channel's turn-on puts `amount` back, and without
one the room has exactly what it started with. The remaining amount is part of checkpoint
snapshots, so a restore does not refill it.

`body: true` puts it in the room as a box whose lamp is on while the pool is not empty.
"""

from __future__ import annotations

from ...schema import Field, SchemaError
from ...signals import is_on
from ..breakable import BREAK_KEYS
from .base import BodiedLogic

SPENDS = ("time", "flow")


class Budget(BodiedLogic):
    stage = "logic"
    emits = True          # the bare `<name>` channel: 1 while anything is left
    stateful = True
    suffixes = (".left",)
    state_keys = ("left", "_prev_refill", "_t")
    fields = (
        Field("pos", "vec", [0.0, 0.0], length=2, unit="m",
              doc="where the budget stands (`body: true`) or where guides draw it"),
        Field("amount", "float", 10.0, min=0.0, max=1e6, runtime=True,
              doc="what the pool starts an episode with"),
        Field("spend", "enum", "time", choices=SPENDS,
              doc="time: `rate`/s per load that is on; flow: `rate`/s while any load is on"),
        Field("rate", "float", 1.0, min=1e-4, max=1e4,
              doc="units spent per second (see `spend`)"),
        Field("loads", "list", None, doc="channels that draw from the pool"),
        Field("refill", "str", None, doc="channel whose turn-on puts `amount` back"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        self.loads = [str(ch) for ch in (c["loads"] or [])]
        self.refill_ch = c["refill"]
        if self.refill_ch in self.loads:
            raise SchemaError(f"part {name!r}: {self.refill_ch!r} is both a load and the "
                              f"refill, so spending it would also fill it")
        self.inputs = [*self.loads, *([self.refill_ch] if self.refill_ch else [])]
        self.spend = c["spend"]
        self.rate = float(c["rate"])
        self.left = float(self.live["amount"])
        self._prev_refill = False
        self._t = None            # `data.time` of the last update, so the step length is measured

    def check_static(self, env) -> list[str]:
        """A budget with no loads can never be spent, so every room holding one is a room
        where it reads 1 for ever. Said rather than warned: it makes the part a no-op."""
        if not self.loads:
            return [f"part {self.name!r}: no `loads`, so nothing ever draws from it and it "
                    f"stays full; name the channels that should cost something"]
        return []

    def reset(self, model, data):
        self.left = float(self.live["amount"])
        self._prev_refill = False
        self._t = float(data.time)
        self.break_reset(model)
        self._write()
        self.paint(model)

    def _write(self) -> None:
        if self.bus is not None:
            for ch in self.outputs[:-1]:
                self.write(float(self.left > 0.0), ch)
            self.write(round(self.left, 4), self.outputs[-1])

    def draw(self) -> float:
        """Units per second the loads are asking for right now."""
        on = sum(1 for ch in self.loads if is_on(self.read(ch)))
        if not on:
            return 0.0
        return self.rate * (on if self.spend == "time" else 1)

    def update(self, model, data, body):
        if self.break_step(model, data, body):          # broken: the output is frozen (R34)
            self.paint(model)
            return
        if self.refill_ch is not None:
            filling = is_on(self.read(self.refill_ch))
            if filling and not self._prev_refill:
                self.left = float(self.live["amount"])
            self._prev_refill = filling
        if not self.enabled():
            return
        now = float(data.time)
        dt = 0.0 if self._t is None else max(0.0, now - self._t)
        self._t = now
        if self.left > 0.0:
            self.left = max(0.0, self.left - self.draw() * dt)
        self._write()
        self.paint(model)

    def state(self):
        amount = float(self.live["amount"])
        return float(self.left / amount) if amount > 0 else 0.0

    # ---- planning and snapshots --------------------------------------------------------
    def plan_values(self, on, get):
        """The static `order` stage asks what this could be. A plan cannot know how long a
        load will be held, so it answers "there is some left", the way the sequence gate
        answers an AND of its inputs: `order` passing does not promise the scripted answer
        stays inside the budget, and a room that is tight about it says so in its header."""
        return {self.outputs[0]: 1.0, self.outputs[-1]: float(self.live["amount"])}

    def save_state(self):
        return {"left": self.left, "_prev_refill": self._prev_refill, "_t": self._t,
                **{k: getattr(self, k) for k in BREAK_KEYS}}

    def load_state(self, model, data, st):
        self.left = float(st["left"])
        self._prev_refill = bool(st.get("_prev_refill", False))
        self._t = st.get("_t")
        for k in BREAK_KEYS:
            if k in st:
                setattr(self, k, st[k])
        self.break_load(model)
        self._write()
        self.paint(model)
