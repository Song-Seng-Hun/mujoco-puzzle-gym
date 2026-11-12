"""Puzzle part base class (R4, R17).

A part adds its own body to the scene spec, reads MuJoCo data each body step,
and may write only its own actuator targets. It never touches the walking body
except through the `BodyState` the environment hands it.

Roles drive the generic logic in `check` and `PuzzleEnv`:
- "switch": has an on/off state; `satisfied()` is True when on. Can be a link source.
- "gate":   blocks movement until its linked sources are on. Can be a link target.
- "goal":   `satisfied()` is True when the body reached it; ends the episode.
- "checkpoint": a flag (R6); the env saves a snapshot when one turns on.
- "prop":   anything else (logic parts, decoration, obstacles via `blockers`).

Signals (R1): a part reads channels `in`, writes channels `out` and has an
optional power channel `enable` (see `signals.py`). `stage` (input, logic,
device) sets when it updates in a body step; `stateful` parts (delay, latch)
may sit in a channel loop. `emits` parts write the channel named after the
part when `out` is not given; `suffixes` add channels `<name><suffix>`
(a counter writes `<name>.count` and `<name>.reached`).

Bodies (R5): a part whose class sets `kind` ("object", "npc") owns a free
body that sensors classify by that kind (`World.kind_of`); `trigger: body |
object | npc | any` on input parts picks which kinds count.

State (R6): `save_state` / `load_state` copy the attributes named in
`state_keys` for checkpoint snapshots; `failed()` True ends the episode.
Planning (R6a): `plan_values` gives the channels this part writes when the
input parts in `on` are on (logic parts compute through, see logic.py).

Markers (R26): `marked` classes get an ArUco marker unless room.yaml says
`marker: none`; `marker_anchor` names the geom (suffix of `body_name`) it sits
on, `marker_faces` the faces allowed (first = default) and `marker_face_size`
the side of the square that fits on a face (markers/__init__.py).

Settings (R3): each class lists its values as `fields` (`schema.Field`); the
class schema is the union over its base classes (`full_schema`). The checked
values, defaults filled in, are `self.cfg`; an out-of-range value raises
`SchemaError` (a ValueError) naming the part, the value and the allowed range.

Run-time values (R13): fields marked `runtime` live in `self.live` (start: the
room value). Parts read them from there every step (none caches one), and
`set_value` changes one, checked against its schema. The env puts them back to the room values on reset
(`reset_values`) and snapshots keep them (snapshot.py). `bind: {<value>:
{from: <channel>, map: [in0, in1, out0, out1]}}` maps a channel onto a
number value linearly every body step, clipped to the schema range (the bus
calls `apply_binds` before `update`). Setters (device/setter.py) change values
on signal edges; difficulty steps (R21) and randomisation (R35) are meant to
use `set_value` too and join setter.py's bind-conflict check (`claimed_values`).
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, ClassVar

import mujoco
import numpy as np

from ..bodies import BodyState
from ..layout import Grid, Segment
from ..schema import Field, Schema, SchemaError, resolve
from ..signals import is_on

if TYPE_CHECKING:
    from ..signals import SignalBus

BODY_PREFIX = "part/"

#: Which body kinds each `trigger` value counts (R4, R5). Shells and rope knots
#: never count (they have their own kinds).
TRIGGERS: dict[str, frozenset[str]] = {
    "body": frozenset({"body"}),
    "object": frozenset({"object"}),
    "npc": frozenset({"npc"}),
    "any": frozenset({"body", "object", "npc"}),
}


@dataclass
class World:
    """What parts may know about the scene beyond their own body (set by the env).

    kinds:   root body id -> "body" (the walking body), "object", "npc", ...
    objects: root body id -> the part owning that free body
    held:    root body ids the body is holding now (not counted by sensors, R5)
    """

    dt: float = 0.02
    kinds: dict[int, str] = field(default_factory=dict)
    objects: dict[int, "Part"] = field(default_factory=dict)
    held: set[int] = field(default_factory=set)
    checkpoints: list["Part"] = field(default_factory=list)
    #: The env's MjData (floor parts read joint positions for the fall rule, R10).
    data: Any = None
    #: Requests parts make of the env this body step: ("warp", part, key, dest) (R14),
    #: ("move", part, key, delta) (conveyor, R12), ("kick", part, key, velocity) (spring, R15).
    requests: list[tuple] = field(default_factory=list)
    #: Sim time until which a warped body / object ignores warp pads (R14 `cooldown`).
    warp_until: dict[Any, float] = field(default_factory=dict)
    #: The walking body's adapter (its eye for the gaze sensor, R27).
    body: Any = None
    #: Grab command this body step (R18/R19 `grab` action; above 0.5 closes the beak).
    grab: float = -1.0
    #: Site id of the beak tip (`BodyAdapter.grab_site`), -1 when the body has none.
    beak_site: int = -1
    #: Far end of a held tool (the stick, R28): presses wall buttons as the beak tip does.
    tool_tip: Any = None

    def kind_of(self, model: mujoco.MjModel, body_id: int) -> str | None:
        return self.kinds.get(int(model.body_rootid[body_id]))

    def next_checkpoint(self) -> int | None:
        """Lowest `order` among flags not reached yet (R6)."""
        left = [c.order for c in self.checkpoints if not c.reached]
        return min(left) if left else None


class Part:
    type_name: ClassVar[str] = "part"
    role: ClassVar[str] = "prop"
    stage: ClassVar[str] = "device"      # input | logic | device (R1 evaluation order)
    stateful: bool = False               # output does not depend on this step's input
    emits: ClassVar[bool] = False        # default `out` is the part's own name
    suffixes: ClassVar[tuple[str, ...]] = ()   # extra channels <name><suffix>
    has_body: ClassVar[bool] = True      # False: no MuJoCo body (logic parts)
    kind: ClassVar[str | None] = None    # body kind for sensors ("object", "npc")
    state_keys: ClassVar[tuple[str, ...]] = ()  # attributes saved in snapshots (R6)
    marked: ClassVar[bool] = False       # gets a marker unless `marker: none` (R26)
    marker_anchor: ClassVar[str] = ""    # geom (suffix of body_name) the marker sits on
    marker_faces: ClassVar[tuple[str, ...]] = ()   # allowed faces; first is the default
    marker = None                        # markers.Marker, set when the room loads
    plan_driver = None                   # planning: a part that turns this input on itself (R29)
    #: Values this class adds to its base classes' schema (R3).
    fields: ClassVar[tuple[Field, ...]] = (
        Field("pos", "vec", length=2, unit="m", doc="position [x, y] on the floor"),
        Field("random", "vec", [0.0, 0.0], length=2, unit="m", min=0.0, max=5.0,
              doc="uniform +- range added to pos on every reset"),
        Field("require", "enum", "all", choices=("all", "any"),
              doc="gates: open when all / any of their inputs are on"),
        Field("requires_body", "list", [],
              doc="body capabilities this part needs (R2b), checked by `mpg check`"),
        Field("in", "list", [], doc="channels this part reads (R1)"),
        Field("out", "list", None, doc="channels this part writes (default: its name if it emits)"),
        Field("enable", "str", None, doc="power channel; off (<= 0.5) freezes the part's output"),
        Field("marker", "any", None,
              doc="ArUco marker (R26): `none` or {id, size, face}; default: a free id on parts that carry one"),
        Field("bind", "any", None,
              doc="R13: {<run-time value>: {from: <channel>, map: [in0, in1, out0, out1]}}, "
                  "applied every body step, clipped to the value's range"),
    )

    @classmethod
    def full_schema(cls) -> Schema:
        """Fields of this class and its bases (subclass entries override)."""
        schema: Schema = {}
        for klass in reversed(cls.__mro__):
            for f in vars(klass).get("fields", ()):
                schema[f.name] = f
        return schema

    def __init__(self, name: str, cfg: dict[str, Any]):
        self.name = name
        self.cfg = cfg = resolve(f"part {name!r}", self.full_schema(), cfg)
        self.base_xy = np.asarray(cfg["pos"], dtype=float)
        self.random = np.asarray(cfg["random"], dtype=float)
        self.xy = self.base_xy.copy()
        self.sources: list[Part] = []          # filled from room links (gates)
        self.require = cfg["require"]
        self.requires_body: list[str] = list(cfg["requires_body"])
        self.inputs: list[str] = list(cfg["in"])
        out = cfg["out"]
        self.outputs: list[str] = list(out) if out is not None else ([name] if self.emits else [])
        self.outputs += [name + s for s in self.suffixes]
        self.enable: str | None = cfg["enable"]
        self.bus: SignalBus | None = None      # set by signals.wire
        self.world: World | None = None        # set by the env
        self.pending_snapshot = False          # checkpoint asks the env for a snapshot
        self._bid = -1
        self._schema = schema = self.full_schema()
        self.live: dict[str, Any] = {k: copy.deepcopy(cfg[k]) for k, f in schema.items()
                                     if f.runtime or f.episode}
        self._live0 = copy.deepcopy(self.live)
        self.binds = self._parse_binds(schema, cfg["bind"])
        #: Extra channels this part reads besides `in` and `enable` (bind sources).
        self.watch: list[str] = list(dict.fromkeys(b[0] for b in self.binds.values()))

    def _parse_binds(self, schema, raw) -> dict[str, tuple[str, list[float]]]:
        owner = f"part {self.name!r}"
        if raw is None:
            return {}
        if not isinstance(raw, dict):
            raise SchemaError(f"{owner}: bind must be {{<value>: {{from: <channel>, map: "
                              f"[in0, in1, out0, out1]}}}}, got {raw!r}")
        out = {}
        for key, b in raw.items():
            f = schema.get(key)
            if f is None or not f.runtime:
                ok = sorted(k for k, g in schema.items() if g.runtime)
                raise SchemaError(f"{owner}: bind {key!r}: not a run-time value of this part "
                                  f"(run-time values: {ok})")
            if f.kind not in ("float", "int"):
                raise SchemaError(f"{owner}: bind {key!r}: only number values can be bound "
                                  f"({key} is a {f.kind})")
            m = b.get("map") if isinstance(b, dict) else None
            if (not isinstance(b, dict) or not isinstance(b.get("from"), str) or set(b) != {"from", "map"}
                    or not isinstance(m, (list, tuple)) or len(m) != 4
                    or not all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in m)):
                raise SchemaError(f"{owner}: bind {key!r} must be {{from: <channel>, "
                                  f"map: [in0, in1, out0, out1]}}, got {b!r}")
            if m[0] == m[1]:
                raise SchemaError(f"{owner}: bind {key!r}: map needs in0 != in1, got {list(m)}")
            out[key] = (b["from"], [float(v) for v in m])
        return out

    @classmethod
    def add_room_parts(cls, parts: dict[str, "Part"]) -> None:
        """Room load hook, once per part class in the room: add helper parts
        (coin groups, R36). Runs before the channels are wired."""

    def marker_face_size(self, face: str) -> float:
        """Side [m] of the largest square marker (white margin included) on `face`."""
        return 0.0

    def attach_body(self, spec: mujoco.MjSpec, body) -> None:
        """Scene hook after the walking body is in `spec` (the beak welds, grab.py)."""

    def needs_body(self, *caps: str) -> None:
        """Add capabilities this part needs whatever room.yaml says (R2b)."""
        self.requires_body = list(dict.fromkeys([*caps, *self.requires_body]))

    @property
    def body_name(self) -> str:
        return BODY_PREFIX + self.name

    # ---- scene ------------------------------------------------------------
    def build(self, spec: mujoco.MjSpec) -> mujoco.MjsBody | None:
        """Add this part's body (named `body_name`) at `base_xy`; subclasses add geoms."""
        if not self.has_body:
            return None
        return spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, 0.0])

    def bind(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        if self.has_body:
            self._bid = model.body(self.body_name).id

    def place(self, model: mujoco.MjModel, xy) -> None:
        """Move the part (randomised layouts). Call `mj_forward` afterwards."""
        self.xy = np.asarray(xy, dtype=float)
        if self.has_body:
            model.body_pos[self._bid, :2] = self.xy

    def root_body_ids(self, model: mujoco.MjModel) -> list[int]:
        """Free bodies this part owns, classified as `kind` for sensors."""
        return [self._bid] if self.kind and self._bid >= 0 else []

    @classmethod
    def shift_cfg(cls, cfg: dict[str, Any], dx: float, dy: float) -> dict[str, Any]:
        """`cfg` moved by (dx, dy): a `stages:` room placed in its zone (R14). Parts with
        other world coordinates in their settings shift those too."""
        out = copy.deepcopy(cfg)
        if isinstance(out.get("pos"), (list, tuple)) and len(out["pos"]) == 2:
            out["pos"] = [float(out["pos"][0]) + dx, float(out["pos"][1]) + dy]
        return out

    def sample_xy(self, rng: np.random.Generator) -> np.ndarray:
        return self.base_xy + rng.uniform(-self.random, self.random)

    # ---- runtime ----------------------------------------------------------
    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        pass

    def update(self, model: mujoco.MjModel, data: mujoco.MjData, body: BodyState) -> None:
        """Called after every body step (50 Hz for the default body)."""

    def state(self) -> float:
        """Observable state in [0, 1]."""
        return 0.0

    def satisfied(self) -> bool:
        return False

    def failed(self) -> bool:
        """True ends the episode as a failure (receptacle `wrong: fail`, flag `skip: fail`)."""
        return False

    def substep(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Called after every physics step when a part overrides it (R5 `substep` hook):
        record contact starts or peak forces here and judge them in `update`."""

    # ---- run-time values (R13) ---------------------------------------------------
    def set_value(self, model: mujoco.MjModel, data: mujoco.MjData, key: str, value: Any,
                  episode: bool = False) -> None:
        """Change run-time value `key` (schema-checked; SchemaError when out of range).
        `episode=True` (episode start, R21 difficulty) also takes values marked `episode`."""
        f = self._schema.get(key)
        if f is None or not (f.runtime or (episode and f.episode)):
            when = "at episode start" if episode else "while running"
            raise SchemaError(f"part {self.name!r}: {key!r} cannot change {when}")
        self.live[key] = f.check(f"part {self.name!r}", value)

    def claim_value(self, key: str) -> None:
        """A setter targets `key` (called while the room loads, before `build`): parts
        that build moving joints only for changing values build them for it."""

    def why_not_stepped(self, key: str) -> str | None:
        """Why run-time value `key` cannot be a difficulty step (R21) here, or None."""
        return None

    def reset_values(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Run-time values back to the room values (env reset, before `reset`)."""
        self.load_values(model, data, self._live0)

    def load_values(self, model: mujoco.MjModel, data: mujoco.MjData, values: dict[str, Any]) -> None:
        self.live = copy.deepcopy(values)

    def apply_binds(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Set every bound value from its channel (linear map, clipped to the schema range)."""
        schema = self._schema
        for key, (ch, (i0, i1, o0, o1)) in self.binds.items():
            f = schema[key]
            v = o0 + (self.read(ch) - i0) * (o1 - o0) / (i1 - i0)
            lo = -np.inf if f.min is None else f.min
            hi = np.inf if f.max is None else f.max
            v = float(np.clip(v, lo, hi))
            if f.kind == "int":
                v = int(round(v))
            self.live[key] = v

    # ---- snapshots (R6) -------------------------------------------------------------
    def save_state(self) -> dict[str, Any]:
        return {k: copy.deepcopy(getattr(self, k)) for k in self.state_keys}

    def load_state(self, model: mujoco.MjModel, data: mujoco.MjData, st: dict[str, Any]) -> None:
        """Restore `save_state` output. Called after the bus values are restored."""
        for k, v in st.items():
            setattr(self, k, copy.deepcopy(v))

    # ---- signals (R1) --------------------------------------------------------
    def read(self, channel: str) -> float:
        return self.bus.get(channel)

    def write(self, value: float, channel: str | None = None) -> None:
        """Write `value` to `channel` (default: this part's first output)."""
        self.bus.set(channel if channel is not None else self.outputs[0], value)

    def enabled(self) -> bool:
        return self.enable is None or self.bus.on(self.enable)

    def inputs_on(self) -> bool:
        """Whether `all` / `any` (per `require`) of the `in` channels are on now."""
        if not self.inputs:
            return False
        flags = [is_on(self.bus.get(ch)) for ch in self.inputs]
        return all(flags) if self.require == "all" else any(flags)

    def plan_values(self, on: set[str], get) -> dict[str, float]:
        """Planning: channels this part writes when the input parts named in `on` are
        on and other channels read `get(channel)`. Inputs: on iff named in `on` or
        their `plan_driver` (a seek_button robot, R29) drives onto them; logic parts
        compute through (logic.py); devices write nothing."""
        if self.stage == "input":
            v = self.name in on or (self.plan_driver is not None and self.plan_driver.plan_drives(get))
            return {ch: float(v) for ch in self.outputs}
        return {}

    def plan_inputs_on(self, get) -> bool:
        """Planning: `inputs_on` with channel values from `get`."""
        if not self.inputs:
            return False
        flags = [is_on(get(ch)) for ch in self.inputs]
        return all(flags) if self.require == "all" else any(flags)

    def sources_on(self, on: set[str]) -> bool:
        """Planning: whether this part's sources (writers of its `in`) are all / any in `on`."""
        if not self.sources:
            return False
        flags = [s.name in on for s in self.sources]
        return all(flags) if self.require == "all" else any(flags)

    # ---- planning (grid) ---------------------------------------------------
    def check_static(self, env) -> list[str]:
        """Reasons this part makes the room unsolvable, found without simulating
        (`mpg check` stage `parts`); empty when fine."""
        return []

    def check_warnings(self, env) -> list[str]:
        """Things `mpg check` stage `parts` reports without failing (an NPC as fast
        as the body, R29); empty when none."""
        return []

    def blockers(self, is_open: bool) -> list[Segment]:
        """Segments that block walking; gates return theirs while not open."""
        return []

    def hole_mask(self, grid: Grid, get) -> np.ndarray | None:
        """Planning: cells with no floor (open trapdoor, pit) once channels read
        `get(channel)`; the planner keeps the body off them. None: solid floor."""
        return None

    def hole_now(self, grid: Grid) -> np.ndarray | None:
        """`hole_mask` from the part's state now (the scripted solver's `demo:` walks)."""
        return None

    def target_key(self) -> Any:
        """Changes when `target_mask` changes (coin groups, R36); distance fields are cached per key."""
        return None

    def target_mask(self, grid: Grid) -> np.ndarray:
        """Cells where the body centre must be to satisfy this part."""
        return grid.disk(self.xy, 0.05)
