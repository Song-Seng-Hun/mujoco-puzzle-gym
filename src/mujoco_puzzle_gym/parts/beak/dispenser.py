"""Dispenser (R28, category `beak`): a cabinet that hands out one item per rising edge.

The cabinet stands with its front face at `pos`, facing `facing` (deg; the side
the body stands on), its top `height` up (a stand height the beak reaches).
Each rising edge of its trigger hands out one copy of `item` while the
dispenser is enabled (`enable`), has stock left, fewer than `max_active` copies
are out and `cooldown` s passed since the last one. The trigger (`button`):
- `floor`: a floor plate `button_at` [out, left] m from `pos` (cabinet frame;
  default 0.35 m in front), stepped on by the walking body;
- `wall_beak`: a plate on the cabinet front at `button_height`, pressed with
  the beak tip (R4 wall window; below the top, so the body lowers its head);
- `none`: the `in` channels (another part's signal).
The built-in plates use the button's press test (input/button.py,
`momentary`); `in` channels count too with a plate.

`item`: a grabbable object preset (R28, e.g. `beak/cube_weight`) or an NPC
preset (R29), as a name or `{preset: ..., <value>: ...}` (or `{type: ...}`)
with values to override. A copy appears at `exit` [out, left] m from `pos`
(default: objects on the top, 2 cm behind the front edge; robots on the floor
0.15 m in front), on the top when that point is over the cabinet, turned so
that its -x side (a single handle's side) faces out, with `exit_vel` [out,
left, up] m/s (roll it out). Outputs: `<name>.stock` (copies left to hand
out), `<name>.empty` (1 when none). `refill` (a channel): each rising edge puts
every stored copy back in stock.

Copies (MuJoCo cannot add bodies while running): `stock` copies (at most
MAX_STOCK) are built up front as parts `<name>/<i>` of the item's type (class
`copy_class`), sharing one marker id (`marker_clone_of`, R26). In storage a
copy is parked (`Part.parked`): an object PARK_Z below the floor, collisions
off and gravity compensated (as the cannon's shell pool, R16); a robot (no
vertical joint) at floor height NPC_PARK m outside the room, collisions and
motors off. Sensors, observations, the gaze sensor and the planners leave
parked copies out. Bits and positions are snapshot fields
(snapshot.MODEL_FIELDS); which copies are out (`out`, since when), spent and
the trigger memory are in `save_state`; reset parks every copy (full stock).

Back to storage: a receptacle with `consume: true` (R5) makes an accepted
copy vanish, and an object copy below the floor (it fell through a pit or off
a cliff onto the catch plane) goes back too. Either way it no longer counts
as out (`max_active`); with `recycle: true` it is in stock again, otherwise it
stays stored (spent) until `refill`.

Planning: the grid answer takes a copy for a receptacle in the solution order
that accepts the item (solver.py); `demo:` has `take <dispenser>` (room.py).
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from ... import markers
from ...floor import CATCH_DEPTH
from ...layout import Segment
from ...schema import Field, SchemaError
from ..base import Part
from ..input.button import THICK, Button
from ..pool import PARK_Z
from ..trap.base import yaw_quat

MAX_STOCK = 20
DEPTH, HALF = 0.12, 0.1         # [m] cabinet depth (behind `pos`) and half width
PARK_GAP = 0.5                  # [m] between parked object copies
NPC_PARK = 100.0                # [m] parked robot copies this far off along +x
FLOOR_AT = (0.35, 0.0)          # [m] default floor plate [out, left] from `pos`
FLOOR_SIZE = (0.2, 0.2)
WALL_SIZE = (0.1, 0.05)         # [m] wall plate [width, height]
OBJECT_EXIT = (-0.02, 0.0)      # [m] default exit: on the top, 2 cm behind the front edge
NPC_EXIT = (0.15, 0.0)          # [m] default exit of a robot: the floor in front
CAUGHT_Z = -CATCH_DEPTH + 0.2  # [m] a copy lower than this lies on the catch plane (floors >= -0.2 m)
DROP = 0.002                    # [m] copies appear this far above where they rest
BODY_RGBA = (0.4, 0.5, 0.6, 1.0)
OFF_RGBA = (0.9, 0.35, 0.2, 1.0)
ON_RGBA = (0.2, 0.8, 0.3, 1.0)


class Dispensed:
    """Mixin of a dispenser's copies (`copy_class`): parked on reset and while stored."""

    dispenser: "Dispenser"
    index: int

    def build(self, spec):
        body = super().build(spec)
        body.gravcomp = 1.0          # stored at reset; the model gets ngravcomp > 0 (ball.py)
        return body

    def reset(self, model, data):
        super().reset(model, data)
        self.park(model, data, self.dispenser.park_spot(self.index))

    def update(self, model, data, body):
        if not self.parked:
            super().update(model, data, body)

    def substep(self, model, data):
        if not self.parked:
            super().substep(model, data)

    def vanish(self, model, data) -> None:
        """A receptacle consumed it (R5 `consume`): back to the dispenser's storage."""
        self.dispenser.store(model, data, self.index)


_COPY_CLASSES: dict[type, type] = {}


def copy_class(cls: type) -> type:
    """The class of a dispenser's copies of part class `cls` (one per class)."""
    if cls not in _COPY_CLASSES:
        _COPY_CLASSES[cls] = type(f"Dispensed{cls.__name__}", (Dispensed, cls), {"__module__": __name__})
    return _COPY_CLASSES[cls]


def resolve_item(owner: str, raw) -> tuple[type, dict]:
    """(part class, values) of `item`: a preset name, or a mapping with `preset` or `type`."""
    from ... import catalog
    from ...registry import get_part
    from ..npc import Npc
    from ..object.grabbable import Grabbable

    entry = {"preset": raw} if isinstance(raw, str) else raw
    if not isinstance(entry, dict) or not ("preset" in entry or "type" in entry) or "name" in entry:
        raise SchemaError(f"{owner}: item must be a preset name or {{preset: <name>, <value>: ...}}, "
                          f"got {raw!r}")
    try:
        entry, _ = catalog.expand_part(dict(entry))
        cls = get_part(entry["type"])
    except (catalog.PresetError, KeyError) as exc:
        raise SchemaError(f"{owner}: item: {exc.args[0]}") from None
    if not issubclass(cls, (Grabbable, Npc)):
        raise SchemaError(f"{owner}: item must be a grabbable object (R28) or an npc robot (R29), "
                          f"not a {entry['type']!r}")
    return cls, {k: v for k, v in entry.items() if k not in ("type", "pos", "random")}


class Dispenser(Part):
    type_name = "dispenser"
    stateful = True                  # its channels come from its stock, not this step's input
    adds_parts = True                # room.make_parts runs its add_room_parts first
    floor_z = 0.0                    # floor top at `pos`, set by the env before reset (R37 heights)
    suffixes = (".stock", ".empty")
    state_keys = ("out", "spent", "last", "was_on", "was_refill", "dispensed")
    fields = (
        Field("button", "enum", "floor", choices=("floor", "wall_beak", "none"),
              doc="floor: a plate the body steps on; wall_beak: a plate on the front the beak "
                  "presses; none: the `in` channels"),
        Field("item", "any", "beak/cube_weight",
              doc="what it hands out: an object (R28) or npc (R29) preset, or {preset, <value>: ...}"),
        Field("stock", "int", 5, min=1, max=MAX_STOCK, doc="copies built (all in stock at reset)"),
        Field("max_active", "int", 1, min=1, max=MAX_STOCK, runtime=True,
              doc="copies out at once at most"),
        Field("cooldown", "float", 1.0, unit="s", min=0.0, max=60.0, runtime=True,
              doc="least time between two copies"),
        Field("refill", "str", None, doc="channel: each rising edge restocks every stored copy"),
        Field("recycle", "bool", False,
              doc="a consumed or fallen copy goes back in stock (else it stays stored until refill)"),
        Field("facing", "float", 0.0, unit="deg", min=-180.0, max=180.0,
              doc="the way the front faces (where the body stands)"),
        Field("height", "float", 0.17, unit="m", min=0.1, max=0.25,
              doc="cabinet top (a stand height the beak reaches)"),
        Field("exit", "vec", None, length=2, unit="m", min=-0.5, max=1.0,
              doc="[out, left] from pos where a copy appears; default objects [-0.02, 0] (top), "
                  "npc [0.15, 0] (floor)"),
        Field("exit_vel", "vec", [0.0, 0.0, 0.0], length=3, unit="m/s", min=-3.0, max=3.0,
              doc="[out, left, up] velocity of a new copy"),
        Field("button_at", "vec", None, length=2, unit="m", min=-1.0, max=1.0,
              doc="[out, left] of the plate from pos; default floor [0.35, 0], wall_beak [0, 0]"),
        Field("button_size", "vec", None, length=2, unit="m", min=0.02, max=1.0,
              doc="plate size; default floor [0.2, 0.2], wall_beak [0.1, 0.05] (width, height)"),
        Field("button_height", "float", 0.14, unit="m", min=0.05, max=0.25,
              doc="wall_beak: plate centre above the floor (below the top)"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        owner = f"part {name!r}"
        self.mode = c["button"]
        if self.mode == "none" and not self.inputs:
            raise SchemaError(f"{owner}: button: none needs `in` channels")
        self.item_cls, self.item_cfg = resolve_item(owner, c["item"])
        self.npc_item = bool(getattr(self.item_cls, "planar", False))
        self.n = int(c["stock"])
        self.h = float(c["height"])
        self.recycle = bool(c["recycle"])
        a = math.radians(float(c["facing"]))
        self.f = np.array([math.cos(a), math.sin(a)])
        self.side = np.array([-self.f[1], self.f[0]])           # cabinet frame +y (left of `f`)
        self.exit = np.asarray(c["exit"] if c["exit"] is not None
                               else (NPC_EXIT if self.npc_item else OBJECT_EXIT), dtype=float)
        self.refill = c["refill"]
        if self.refill:
            self.watch.append(self.refill)
        self.stage = "device" if self.mode == "none" else "input"
        self.marked = self.mode != "none"
        self.press_plate: Button | None = None
        if self.mode != "none":
            wall = self.mode == "wall_beak"
            self.marker_anchor = "/plate"
            self.marker_faces = ("front",) if wall else markers.FACES_FLAT
            self.plate_at = np.asarray(c["button_at"] if c["button_at"] is not None
                                       else ((0.0, 0.0) if wall else FLOOR_AT), dtype=float)
            size = c["button_size"] if c["button_size"] is not None else (WALL_SIZE if wall else FLOOR_SIZE)
            self.press_plate = Button(name + "/button", {
                "pos": self._world(self.plate_at).tolist(), "mount": "wall" if wall else "floor",
                "kind": "momentary", "trigger": "body", "size": list(size),
                "height": float(c["button_height"]), "facing": float(c["facing"]), "marker": "none"})
            self.needs_body("walk", *(["beak_grab"] if wall else []))
        self.copies: list[Part] = []
        self._reset_state()

    def _reset_state(self) -> None:
        self.out: dict[int, float] = {}      # copy index -> time handed out
        self.spent: set[int] = set()         # stored, not in stock (consumed or fallen, no recycle)
        self.last = -math.inf                # time of the last copy
        self.was_on = self.was_refill = False
        self.dispensed = 0                   # copies handed out this episode

    def _world(self, local) -> np.ndarray:
        """World xy of cabinet-frame [out, left] from the present `xy`."""
        return self.xy + self.f * float(local[0]) + self.side * float(local[1])

    def marker_face_size(self, face):
        return float(min(self.press_plate.half)) * 2 if self.press_plate is not None else 0.0

    @classmethod
    def add_room_parts(cls, parts):
        """Build every dispenser's copies as parts `<name>/<i>` (after the room's parts)."""
        for d in [p for p in parts.values() if isinstance(p, Dispenser)]:
            klass = copy_class(d.item_cls)
            for i in range(d.n):
                name = f"{d.name}/{i}"
                if name in parts:
                    raise SchemaError(f"part name {name!r} is taken by dispenser {d.name!r}'s copies")
                c = klass(name, {**d.item_cfg, "pos": d.exit_xy().tolist()})
                c.dispenser, c.index, c.zone = d, i, getattr(d, "zone", 0)
                c.marker_clone_of = d.name
                c.marker = markers.parse(c, d.item_cfg.get("marker"))
                d.copies.append(c)
                parts[name] = c

    # ---- scene ------------------------------------------------------------------
    def build(self, spec):
        c = self.cfg
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, 0.0],
                                       quat=yaw_quat(float(c["facing"])))
        body.add_geom(name=self.body_name + "/cabinet", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[DEPTH / 2, HALF, self.h / 2], pos=[-DEPTH / 2, 0.0, self.h / 2], rgba=BODY_RGBA)
        b = self.press_plate
        if b is not None and b.mount == "wall":
            body.add_geom(name=self.body_name + "/plate", type=mujoco.mjtGeom.mjGEOM_BOX,
                          size=[THICK / 2, *b.half], pos=[float(self.plate_at[0]) + THICK / 2, float(self.plate_at[1]),
                                                             b.cfg["height"]],
                          rgba=OFF_RGBA, contype=0, conaffinity=0)
        elif b is not None:                       # world-axis plate (the press test is axis-aligned)
            body.add_geom(name=self.body_name + "/plate", type=mujoco.mjtGeom.mjGEOM_BOX,
                          size=[*b.half, 0.002], pos=[*self.plate_at, 0.002],
                          quat=yaw_quat(-float(c["facing"])), rgba=OFF_RGBA, contype=0, conaffinity=0)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        if self.press_plate is not None:
            self.press_plate._bid = self._bid
            self.press_plate._gid = model.geom(self.body_name + "/plate").id

    def place(self, model, xy):
        self.xy = np.asarray(xy, dtype=float)
        model.body_pos[self._bid, :2] = self.xy
        if self.press_plate is not None:
            self.press_plate.xy = self._world(self.plate_at)

    # ---- storage ----------------------------------------------------------------
    def exit_xy(self) -> np.ndarray:
        return self._world(self.exit)

    def park_spot(self, i: int) -> np.ndarray:
        """Where stored copy `i` waits (see the module docstring)."""
        if self.npc_item:
            return np.array([self.xy[0] + NPC_PARK + i, self.xy[1], self.floor_z])
        return np.array([self.xy[0] + i * PARK_GAP, self.xy[1], PARK_Z])

    def stock_left(self) -> int:
        return self.n - len(self.out) - len(self.spent)

    def store(self, model, data, i: int) -> None:
        """Copy `i` back to storage (consumed or fallen); in stock again with `recycle`."""
        self.out.pop(i, None)
        if not self.recycle:
            self.spent.add(i)
        self.copies[i].park(model, data, self.park_spot(i))
        self._write()

    def _hand_out(self, model, data, now: float) -> None:
        i = next(k for k in range(self.n) if k not in self.out and k not in self.spent)
        copy = self.copies[i]
        local = self.exit
        on_top = -DEPTH <= local[0] <= 0.0 and abs(local[1]) <= HALF
        xy = self.exit_xy()
        z = self.floor_z + (self.h if on_top else 0.0)
        yaw = float(self.cfg["facing"]) + 180.0           # -x side (a lone handle) faces out
        v = np.asarray(self.cfg["exit_vel"], dtype=float)
        copy.reset(model, data)                 # per-life state as new (a recycled broken egg
                                                # comes out whole); parked until unpark below
        vel = [*(self.f * v[0] + self.side * v[1]), v[2]]
        if self.npc_item:
            copy.unpark(model, [*xy, z])
            copy.teleport(data, (*xy, math.radians(yaw)))
            copy.kick(data, vel[:2])
        else:
            copy.unpark(model)
            copy.place(model, xy)
            copy.put(data, [*xy, z + copy._rest_z() + DROP], vel)
            data.qpos[copy._q + 3:copy._q + 7] = yaw_quat(yaw)
        self.out[i] = now
        self.last = now
        self.dispensed += 1

    # ---- runtime ----------------------------------------------------------------
    def reset(self, model, data):
        self._reset_state()
        if self.press_plate is not None:
            self.press_plate.on = False
            model.geom_rgba[self.press_plate._gid] = OFF_RGBA
        self._write()

    def _write(self) -> None:
        left = self.stock_left()
        self.write(float(left), self.name + ".stock")
        self.write(float(left == 0), self.name + ".empty")

    def update(self, model, data, body):
        now = float(data.time)
        if not self.npc_item:                  # fell through a hole onto the catch plane
            for i in list(self.out):
                if self.copies[i].pos(data)[2] < CAUGHT_Z:
                    self.store(model, data, i)
        if self.refill:
            r = self.bus.on(self.refill)
            if r and not self.was_refill:
                self.spent.clear()
            self.was_refill = r
        b = self.press_plate
        pressed = False
        if b is not None:
            b.world = self.world
            pressed = b.on = b.pressed(data, body)
            model.geom_rgba[b._gid] = ON_RGBA if pressed else OFF_RGBA
        trig = pressed or self.inputs_on()
        if trig and not self.was_on and self.ready(now):
            self._hand_out(model, data, now)
        self.was_on = trig
        self._write()

    def ready(self, now: float) -> bool:
        """A rising edge at time `now` would hand out a copy."""
        return (self.enabled() and self.stock_left() > 0 and len(self.out) < int(self.live["max_active"])
                and now - self.last >= float(self.live["cooldown"]) - 1e-9)

    def state(self):
        return float(self.stock_left()) / self.n

    def last_copy(self) -> Part | None:
        """The copy handed out last that is still out, or None."""
        if not self.out:
            return None
        return self.copies[max(self.out, key=self.out.get)]

    # ---- planning ---------------------------------------------------------------
    def blockers(self, is_open):
        back = self.xy - self.f * (DEPTH / 2)
        return [Segment(tuple(back - self.side * HALF), tuple(back + self.side * HALF), DEPTH)]
