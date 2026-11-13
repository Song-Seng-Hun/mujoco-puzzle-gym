"""Breaking a circuit part (R34): the `breakable` mixin and its four causes.

A part with a body (R33 `body: true` logic parts, a `wire`'s cable cover, a
`relay`) can be given `breakable: true` and a list of causes in `by`:

- `shell`     an R16 cannon shell hits it at `min_speed` or faster. One contact
              of one shell counts once, as the target's does (`pool.Impacts`).
- `crush`     an object (R17/R28) falls onto it or is pressed against it with a
              contact force of at least `crush_force`. A shell never counts
              here (its contact is only ever judged as `shell`), and, as with a
              shell, one contact of one object counts once: an object left
              resting on the part does not drain a hit every step.
- `peck`      the beak tip is within `peck_dist` of the part and the grab action
              goes on and then off again. The target has to sit where the duck
              is stopped (a wall, or a part whose body blocks it): the walking
              policy drifts forward at command 0 and cannot walk backwards, so
              it cannot hold a place on open floor (Open question 8).
- `destroyer` a `destroyer` part strikes it (`hit`).

Each counted cause costs one hit; at `hp` hits the part is broken: its outputs
freeze at `broken_value`, `<name>.broken` turns on, `<name>.hp` falls to 0 and
the shape changes (colour, and a cable cover stops colliding). `on_break: fail`
ends the episode as a failure (a wire that must not be cut). While the `repair`
channel is on the part is whole again with its `hp` back at the start value.

Measured (16f step 0, 20 seeds a condition, default 0.03 kg cube on a 0.08 x
0.08 x 0.12 m logic box): dropped 0.05 m it peaks at 0.87 N, pushed by the duck
at 3.1-12.2 N, and merely resting on the part at 0.30 N, its weight. Hence the
`crush_force` default of 0.5 N. It is only 1.7x a 30 g weight, so an object over
about 51 g crushes a part by sitting on it; `mpg check` warns about such a room.

A part is not broken by this file alone: it mixes `Breakable` in, lists the
state keys, and calls four hooks, so that nothing happens in a room that does
not ask for it:
- `break_bind(model, data)` in `bind`, `break_reset(model)` in `reset`,
  `break_load(model)` at the end of `load_state`;
- `break_step(model, data, body)` at the top of `update`: it judges the causes
  and returns True when the part is broken, which means "write nothing more".
`break_bodies` / `break_geom` say which body the contacts count on and which
geom is measured and painted (a wire points them at its cover).
"""

from __future__ import annotations

from typing import Any

import mujoco
import numpy as np

from ..schema import Field, SchemaError
from ..signals import is_on
from .base import Part
from .pool import Impacts

CAUSES = ("shell", "crush", "peck", "destroyer")
BREAK_KEYS = ("broken", "hp_left")          # state_keys every breakable part adds (R6)
BROKEN_RGBA = (0.35, 0.18, 0.18, 1.0)       # a broken part's colour


class Breakable(Part):
    """Mixin for a part that `breakable: true` can break (R34)."""

    fields = (
        Field("breakable", "bool", False,
              doc="R34: this part can be broken (it needs a body)"),
        Field("by", "list", ["shell"],
              doc=f"causes that count: {', '.join(CAUSES)}"),
        Field("hp", "int", 1, min=1, max=20, doc="hits it takes to break"),
        Field("broken_value", "float", 0.0, min=-1000.0, max=1000.0,
              doc="what its outputs read once broken"),
        Field("on_break", "enum", "none", choices=("none", "fail"),
              doc="fail: breaking it ends the episode as a failure"),
        Field("repair", "str", None, doc="channel: while on, the part is whole again"),
        Field("crush_force", "float", 0.5, unit="N", min=0.05, max=200.0, runtime=True,
              doc="`crush`: contact force of an object that counts (measured, R34)"),
        Field("min_speed", "float", 1.0, unit="m/s", min=0.0, max=30.0, runtime=True,
              doc="`shell`: slowest hit that counts"),
        Field("peck_dist", "float", 0.02, unit="m", min=0.005, max=0.1,
              doc="`peck`: beak tip this close to the part counts (measured, R34)"),
    )

    def __init__(self, name: str, cfg: dict[str, Any]):
        super().__init__(name, cfg)
        c = self.cfg
        self.breakable = bool(c["breakable"])
        self.by: list[str] = list(c["by"])
        self.hp = int(c["hp"])
        self.broken_value = float(c["broken_value"])
        self.on_break = c["on_break"]
        self.repair_channel: str | None = c["repair"]
        bad = [b for b in self.by if b not in CAUSES]
        if bad:
            raise SchemaError(f"part {name!r}: `by` causes {bad} are not among {list(CAUSES)}")
        if self.breakable and not self.by:
            raise SchemaError(f"part {name!r}: `breakable` needs at least one cause in `by`")
        # `body: true` parts set `has_body` after this runs (BodiedLogic), so ask the
        # setting when there is one.
        has_body = bool(c["body"]) if "body" in c else self.has_body
        if self.breakable and not has_body:
            raise SchemaError(f"part {name!r}: only a part with a body can be breakable "
                              f"(R34); set `body: true`")
        if not self.breakable:
            for key in ("by", "hp", "on_break", "repair", "broken_value"):
                if c[key] != self._schema[key].default:
                    raise SchemaError(f"part {name!r}: {key!r} only means something with "
                                      f"`breakable: true`")
        if self.breakable:
            self.outputs += [name + ".broken", name + ".hp"]
            if self.repair_channel:
                self.watch = list(dict.fromkeys([*self.watch, self.repair_channel]))
        self.broken = False
        self.hp_left = self.hp
        self._break_reset_transients()
        self._geom = -1
        self._bodies: set[int] = set()
        self._impacts: Impacts | None = None
        self._painted: bool | None = None

    def _break_reset_transients(self) -> None:
        self._touch: dict[int, dict[str, Any]] = {}     # object root -> this contact's peak
        self._events: list[str] = []                    # causes counted since the last update
        self._grab_prev = False
        self._peck_armed = False

    # ---- what the contacts and the shape belong to --------------------------------
    def break_bodies(self, model: mujoco.MjModel) -> set[int]:
        """Bodies whose contacts count (default: the part's own body)."""
        return {self._bid} if self._bid >= 0 else set()

    def break_geom(self, model: mujoco.MjModel) -> int:
        """Geom measured for `peck` and repainted when broken (default: its box)."""
        try:
            return model.geom(self.body_name + "/box").id
        except KeyError:
            return -1

    # ---- hooks the part calls ------------------------------------------------------
    def break_bind(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        if not self.breakable:
            return
        self._bodies = {int(b) for b in self.break_bodies(model)}
        self._geom = int(self.break_geom(model))
        self._impacts = Impacts() if "shell" in self.by else None
        self._painted = None

    def break_reset(self, model: mujoco.MjModel) -> None:
        if not self.breakable:
            return
        self.broken = False
        self.hp_left = self.hp
        self._break_reset_transients()
        if self._impacts is not None:
            self._impacts = Impacts()
        self.paint_break(model)

    def break_load(self, model: mujoco.MjModel) -> None:
        """After `load_state`: the shape is a model value, so put it back from the state."""
        if self.breakable:
            self._break_reset_transients()
            self.paint_break(model)

    def hit(self, model: mujoco.MjModel, cause: str = "destroyer") -> bool:
        """One strike from outside (a `destroyer`). True when it counted."""
        if not self.breakable or self.broken or cause not in self.by:
            return False
        self._events.append(cause)
        return True

    def substep(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Record shell contacts and object contact forces (R5 `substep` hook)."""
        if not self.breakable or self.broken or not self._bodies:
            return
        if self._impacts is not None and self.world is not None:
            self._impacts.substep(model, data, self.world, self._bodies)
        if "crush" in self.by:
            self._crush_substep(model, data)

    def _crush_substep(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Peak contact force per object contact; one contact counts once."""
        force = np.zeros(6)
        now: set[int] = set()
        for i in range(data.ncon):
            c = data.contact[i]
            b1, b2 = int(model.geom_bodyid[c.geom1]), int(model.geom_bodyid[c.geom2])
            if b1 in self._bodies:
                other = int(model.body_rootid[b2])
            elif b2 in self._bodies:
                other = int(model.body_rootid[b1])
            else:
                continue
            if self.world is None or self.world.kinds.get(other) != "object":
                continue                       # the duck and shells never count as `crush`
            mujoco.mj_contactForce(model, data, i, force)
            f = float(np.linalg.norm(force[:3]))
            now.add(other)
            seen = self._touch.setdefault(other, {"peak": 0.0, "counted": False})
            seen["peak"] = max(seen["peak"], f)
            if not seen["counted"] and f >= float(self.live["crush_force"]):
                seen["counted"] = True
                self._events.append("crush")
        for root in list(self._touch):
            if root not in now:
                del self._touch[root]          # the contact ended: it may count again
        return None

    def break_step(self, model: mujoco.MjModel, data: mujoco.MjData, body) -> bool:
        """Judge the causes and write `<name>.broken` / `<name>.hp`.

        True: the part is broken, so the caller writes nothing but `broken_value`
        (this method has already written it to every value output)."""
        if not self.breakable:
            return False
        if self.repair_channel and is_on(self.read(self.repair_channel)):
            if self.broken or self.hp_left != self.hp:
                self.broken = False
                self.hp_left = self.hp
                self._break_reset_transients()
                self.paint_break(model)
        elif not self.broken:
            hits = len(self._events)
            if self._impacts is not None:
                speed = float(self.live["min_speed"])
                hits += sum(1 for v in self._impacts.events if v >= speed)
                self._impacts.events.clear()
            if "peck" in self.by:
                hits += self._peck_step(model, data)
            self._events.clear()
            if hits:
                self.hp_left = max(0, self.hp_left - hits)
                if self.hp_left == 0:
                    self.broken = True
                    self.paint_break(model)
        self._events.clear()
        self.write(float(self.broken), self.name + ".broken")
        self.write(float(self.hp_left), self.name + ".hp")
        if self.broken:
            for ch in self.value_outputs():
                self.write(self.broken_value, ch)
            return True
        return False

    def value_outputs(self) -> list[str]:
        """Outputs that carry the part's value (not `.broken` / `.hp`)."""
        extra = (self.name + ".broken", self.name + ".hp")
        return [ch for ch in self.outputs if ch not in extra]

    def _peck_step(self, model: mujoco.MjModel, data: mujoco.MjData) -> int:
        """1 when the grab action went on and then off with the tip close enough."""
        w = self.world
        if w is None or w.beak_site < 0 or self._geom < 0:
            return 0
        near = self.beak_distance(model, data) <= float(self.cfg["peck_dist"])
        grab = is_on(w.grab)
        hits = 0
        if grab and not self._grab_prev:
            self._peck_armed = near
        elif self._grab_prev and not grab:
            if self._peck_armed and near:
                hits = 1
            self._peck_armed = False
        elif grab and not near:
            self._peck_armed = False           # it drifted off while the beak was closed
        self._grab_prev = grab
        return hits

    def beak_distance(self, model: mujoco.MjModel, data: mujoco.MjData) -> float:
        """Distance from the beak tip to the measured geom (0 inside it)."""
        w = self.world
        if w is None or w.beak_site < 0 or self._geom < 0:
            return float("inf")
        return geom_distance(model, data, self._geom, data.site_xpos[w.beak_site])

    # ---- shape and episode end -----------------------------------------------------
    def paint_break(self, model: mujoco.MjModel) -> None:
        """Colour (and, for a cover, collision) from `broken`; a model value, so it is
        applied again after a reset and after a snapshot restore."""
        if not self.breakable or self._geom < 0 or self._painted == self.broken:
            return
        self._painted = self.broken
        if self.broken:
            self._whole_rgba = tuple(model.geom_rgba[self._geom])
            model.geom_rgba[self._geom] = BROKEN_RGBA
        elif getattr(self, "_whole_rgba", None) is not None:
            model.geom_rgba[self._geom] = self._whole_rgba

    def failed(self) -> bool:
        return bool(self.breakable and self.broken and self.on_break == "fail")

    # ---- planning ------------------------------------------------------------------
    def break_plan_values(self, values: dict[str, float]) -> dict[str, float]:
        """Planning (R6a): a breakable part is taken as whole, and reports hp and
        `.broken` as they start (the solution order is not allowed to need a break;
        the circuit check of 3c reads the graph instead)."""
        if not self.breakable:
            return values
        out = dict(values)
        out[self.name + ".broken"] = 0.0
        out[self.name + ".hp"] = float(self.hp)
        return out


def geom_distance(model: mujoco.MjModel, data: mujoco.MjData, gid: int, point) -> float:
    """Distance from `point` to box geom `gid` (its own frame), 0 inside it."""
    p = np.asarray(data.geom_xpos[gid], dtype=float)
    rot = np.asarray(data.geom_xmat[gid], dtype=float).reshape(3, 3)
    local = rot.T @ (np.asarray(point, dtype=float) - p)
    outside = np.maximum(np.abs(local) - np.asarray(model.geom_size[gid][:3], dtype=float), 0.0)
    return float(np.linalg.norm(outside))
