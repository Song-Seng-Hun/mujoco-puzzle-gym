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
channel is on the part is whole again with its `hp` back at the start value, and
so it is while the `repair_with: <marker id>` fuse object (R26) sits in its
socket, i.e. within `socket_dist` of the part (the beak sets the fuse down
against it; there is no reachable top on a 0.2 m box). `has_repair()` says
whether the room has either way, which the circuit check (3c) reads.

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

from .. import markers
from ..schema import Field, SchemaError
from ..signals import is_on
from .base import Part
from .pool import Impacts

CAUSES = ("shell", "crush", "peck", "destroyer")
BREAK_KEYS = ("broken", "hp_left")          # state_keys every breakable part adds (R6)
BROKEN_RGBA = (0.35, 0.18, 0.18, 1.0)       # a broken part's colour
PLACE_TOL = 0.03                            # [m] scripted `place` default (carry.py)


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
        Field("repair_with", "int", None, min=0, max=markers.N_IDS - 1,
              doc="marker id of the fuse object that repairs it in its socket (R34)"),
        Field("socket_dist", "float", 0.06, unit="m", min=0.02, max=0.3,
              doc="`repair_with`: how close to the socket the fuse counts"),
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
        self.repair_with: int | None = None if c["repair_with"] is None else int(c["repair_with"])
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
            for key in ("by", "hp", "on_break", "repair", "repair_with", "broken_value"):
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
        self._geom_half = 0.0           # `repair_with`: the measured geom's widest half side

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
        self._geom_half = (float(np.max(model.geom_size[self._geom][:2])) if self._geom >= 0 else 0.0)
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
        if self._repaired(model, data):
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

    # ---- repair --------------------------------------------------------------------
    def has_repair(self) -> bool:
        """Whether the room gives any way to make this part whole again (R34; the
        circuit check reads it)."""
        return bool(self.breakable and (self.repair_channel or self.repair_with is not None))

    def _repaired(self, model: mujoco.MjModel, data: mujoco.MjData) -> bool:
        if self.repair_channel and is_on(self.read(self.repair_channel)):
            return True
        return self.fuse_in(model, data)

    def fuse_in(self, model: mujoco.MjModel, data: mujoco.MjData) -> bool:
        """`repair_with`: a free object with that marker id (R26) is in the socket, i.e.
        within `socket_dist` of the part's measured geom (the beak sets it down against
        the part: on a 0.2 m box there is no top the duck can reach)."""
        w = self.world
        if self.repair_with is None or self._geom < 0 or w is None:
            return False
        for rid, obj in w.objects.items():
            if w.kinds.get(rid) != "object" or rid in w.held or obj.parked:
                continue
            mark = getattr(obj, "marker", None)
            if mark is None or int(mark.id) != self.repair_with:
                continue
            if geom_distance(model, data, self._geom, data.xpos[rid]) <= float(self.cfg["socket_dist"]):
                return True
        return False

    def place_tolerance(self, obj=None) -> float:
        """Scripted `place` / `drop` (carry.py): let the beak set the fuse (or the object
        that crushes the part) down anywhere against it, its own width plus the socket
        reach, since there is no reachable top on a part the duck runs into."""
        if not self.breakable:
            return PLACE_TOL
        return self._geom_half + float(self.cfg["socket_dist"])

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

    def break_xy(self, data: mujoco.MjData) -> np.ndarray:
        """Where the measured geom is (x, y): a wire's cover stands off the wall, so this
        is what the duck walks at to peck it."""
        if self._geom < 0:
            return np.asarray(self.xy, dtype=float)
        return np.asarray(data.geom_xpos[self._geom][:2], dtype=float)

    def peck_normal(self):
        """Unit (x, y) the part must be pecked from, or None when any side will do: a
        wire's cover is on a wall face, so the duck has to come round to its front."""
        return None

    def break_top(self) -> float:
        """How high the part reaches [m] (a wire's `height` is its cover's): `mpg check`
        asks for the `head` action when the beak must come down to peck it."""
        return float(self.cfg.get("height", 0.0) or 0.0)

    # ---- aiming a cannon at it (R16 + R34) -----------------------------------------
    def break_board(self, model: mujoco.MjModel, data: mujoco.MjData, toward) -> "ShellBoard | None":
        """The measured geom's face that looks toward `toward` (x, y), as a board the
        cannon's ballistics can aim at (`Cannon.hits_board` / `can_hit`)."""
        if self._geom < 0:
            return None
        p = np.asarray(data.geom_xpos[self._geom], dtype=float)
        rot = np.asarray(data.geom_xmat[self._geom], dtype=float).reshape(3, 3)
        size = np.asarray(model.geom_size[self._geom][:3], dtype=float)
        d = np.asarray(toward, dtype=float)[:2] - p[:2]
        best, sign, axis = -2.0, 1.0, 0
        for i in (0, 1):
            for s in (1.0, -1.0):
                score = float(s * rot[:2, i] @ d)
                if score > best:
                    best, sign, axis = score, s, i
        n = sign * rot[:, axis]
        other = 1 - axis
        u = rot[:, other]
        return ShellBoard(self, p + n * size[axis], n, u, (float(size[other]), float(size[2])))

    # ---- static checks (R34) --------------------------------------------------------
    def check_warnings(self, env) -> list[str]:
        """Causes in `by` the room cannot deliver, and objects heavy enough to crush the
        part by resting on it (R34 static checks; `mpg check` prints them without failing)."""
        out = list(super().check_warnings(env))
        if not self.breakable:
            return out
        if "shell" in self.by and not any(hasattr(p, "hits_board") for p in env.parts.values()):
            out.append(f"part {self.name!r} can be broken `by: [shell]` but the room has no cannon")
        if "peck" in self.by and not self.blockers(True) and getattr(self, "mount", None) not in ("wall", "part"):
            out.append(f"part {self.name!r} can be broken `by: [peck]` but it does not stop the duck "
                       f"(put it on a wall or give it a body the duck runs into: the walking policy "
                       f"drifts forward and cannot hold a place on open floor)")
        if "crush" in self.by:
            force = float(self.cfg["crush_force"])
            heavy = sorted(p.name for p in env.parts.values()
                           if hasattr(p, "handle_points") and float(getattr(p, "mass", 0.0) or 0.0) * 9.81 > force)
            if heavy:
                out.append(f"part {self.name!r} has crush_force {force:g} N, so merely setting "
                           f"{heavy} on it breaks it (their weight is more than that)")
        return out

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

    def plan_broken_values(self, get) -> dict[str, float]:
        """Planning (R6a) for a part the room's `demo:` breaks: its value outputs read
        `broken_value` and `<name>.broken` is on, so the planner sees what the break
        opens (logic.py `plan_channels`, circuit.py `demo_breaks`)."""
        out = {ch: self.broken_value for ch in self.value_outputs()}
        out[self.name + ".broken"] = 1.0
        out[self.name + ".hp"] = 0.0
        return out


class ShellBoard:
    """One face of a breakable part, shaped like an R16 target board: the cannon's
    `hits_board` / `can_hit` read `face()` and `live["min_speed"]`, nothing else."""

    def __init__(self, part: Breakable, centre, normal, u, half: tuple[float, float]):
        self.part = part
        self.centre = np.asarray(centre, dtype=float)
        self.normal = np.asarray(normal, dtype=float)
        self.u = np.asarray(u, dtype=float)
        self.half = half
        self.live = part.live

    def face(self):
        return self.centre, self.normal, self.u, self.half


def geom_distance(model: mujoco.MjModel, data: mujoco.MjData, gid: int, point) -> float:
    """Distance from `point` to box geom `gid` (its own frame), 0 inside it."""
    p = np.asarray(data.geom_xpos[gid], dtype=float)
    rot = np.asarray(data.geom_xmat[gid], dtype=float).reshape(3, 3)
    local = rot.T @ (np.asarray(point, dtype=float) - p)
    outside = np.maximum(np.abs(local) - np.asarray(model.geom_size[gid][:3], dtype=float), 0.0)
    return float(np.linalg.norm(outside))
