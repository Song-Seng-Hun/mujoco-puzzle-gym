"""Setter (R13): changes another part's value when a signal turns on.

    - {name: boost, type: setter, in: [b1], target: launcher.velocity, value: [0.8, 0, 2.2]}

On the rising edge of its `in` channels (`require` all / any) it sets
`target: <part>.<value>` to `value`; with `revert: true` it puts the old value
back when they turn off. `enable` off ignores edges.

Only values the target's schema marks run-time (`mpg parts`: "실행 중 변경")
can be targets; size or shape values (a door's `width`) are rejected while the
room loads, and so is a `value` outside the value's range. A value that is
also the target of a `bind` (base.py) is rejected too: the bind writes it every
body step, so the setter would be silently overwritten. `claimed_values` lists
who claims which value; difficulty steps (R21) and randomisation (R35) add
their claims there when they come.

The setter has no body. It writes its own channel (1 while its value is
applied) and the target watches that channel, so the bus updates the setter
before its target and a new value acts in the same body step.
"""

from __future__ import annotations

import copy
from typing import Any

from ...schema import Field, SchemaError
from ..base import Part


def claimed_values(parts: dict[str, Part]) -> dict[tuple[str, str], list[str]]:
    """(part, value) -> who changes it while running: `bind`, setter names."""
    out: dict[tuple[str, str], list[str]] = {}
    for n, p in parts.items():
        for key in p.binds:
            out.setdefault((n, key), []).append("bind")
    for n, p in parts.items():
        if isinstance(p, Setter):
            out.setdefault((p.target_part, p.target_key), []).append(n)
    return out


class Setter(Part):
    stage = "device"
    has_body = False
    emits = True
    state_keys = ("was_on", "applied", "saved")
    fields = (
        Field("pos", "vec", [0.0, 0.0], length=2, unit="m", doc="unused (a setter has no body)"),
        Field("target", "str", doc="<part>.<value>: the run-time value to change"),
        Field("value", "any", doc="new value (checked against the target value's schema)"),
        Field("revert", "bool", False, doc="put the old value back when the signal turns off"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        owner = f"part {name!r}"
        part, dot, key = self.cfg["target"].rpartition(".")
        if not dot or not part or not key:
            raise SchemaError(f"{owner}: target must be <part>.<value>, got {self.cfg['target']!r}")
        if not self.inputs:
            raise SchemaError(f"{owner}: a setter needs `in` channels")
        self.target_part, self.target_key = part, key
        self.value: Any = self.cfg["value"]
        self.revert = bool(self.cfg["revert"])
        self.target: Part | None = None
        self.was_on = self.applied = False
        self.saved: Any = None

    @classmethod
    def add_room_parts(cls, parts):
        """Resolve every setter's target and reject values that cannot change at run time."""
        for n, p in parts.items():
            if not isinstance(p, Setter):
                continue
            owner = f"part {n!r}"
            t = parts.get(p.target_part)
            if t is None:
                raise SchemaError(f"{owner}: target {p.target_part!r} is not a part "
                                  f"(parts: {sorted(parts)})")
            f = t.full_schema().get(p.target_key)
            if f is None or not f.runtime:
                ok = sorted(k for k, g in t.full_schema().items() if g.runtime)
                what = "has no value" if f is None else "cannot change while running:"
                raise SchemaError(f"{owner}: target {p.cfg['target']!r}: {t.type_name} {p.target_part!r} "
                                  f"{what} {p.target_key!r} (run-time values: {ok})")
            p.value = f.check(f"{owner}: value for {p.cfg['target']}", p.value)
            p.target = t
            t.claim_value(p.target_key)
            t.watch = list(dict.fromkeys([*t.watch, *p.outputs[:1]]))   # setter updates first
        for (part, key), who in claimed_values(parts).items():
            if "bind" in who and len(who) > 1:
                raise SchemaError(f"value {part}.{key} is bound (`bind`) and also set by "
                                  f"{[w for w in who if w != 'bind']}; the bind would overwrite "
                                  f"the setter every step")

    def reset(self, model, data):
        self.was_on = self.applied = False
        self.saved = None
        for ch in self.outputs:
            self.write(0.0, ch)

    def update(self, model, data, body):
        on = self.inputs_on()
        rising, falling = on and not self.was_on, self.was_on and not on
        self.was_on = on
        if not self.enabled() or self.target is None:
            return
        if rising and not self.applied:
            self.saved = copy.deepcopy(self.target.live[self.target_key])
            self.target.set_value(model, data, self.target_key, copy.deepcopy(self.value))
            self.applied = True
        elif falling and self.applied and self.revert:
            self.target.set_value(model, data, self.target_key, self.saved)
            self.applied = False
        for ch in self.outputs:
            self.write(float(self.applied), ch)

    def state(self):
        return float(self.applied)
