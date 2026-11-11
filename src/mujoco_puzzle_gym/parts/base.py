"""Puzzle part base class (R4, R17).

A part adds its own body to the scene spec, reads MuJoCo data each body step,
and may write only its own actuator targets. It never touches the walking body
except through the `BodyState` the environment hands it.

Roles drive the generic logic in `check` and `PuzzleEnv`:
- "switch": has an on/off state; `satisfied()` is True when on. Can be a link source.
- "gate":   blocks movement until its linked sources are on. Can be a link target.
- "goal":   `satisfied()` is True when the body reached it; ends the episode.
- "prop":   anything else (decoration, obstacles via `blockers`).

Signals (R1): a part reads channels `in`, writes channels `out` and has an
optional power channel `enable` (see `signals.py`). `stage` (input, logic,
device) sets when it updates in a body step; `stateful` parts (delay, latch)
may sit in a channel loop. `emits` parts write the channel named after the
part when `out` is not given.

Settings (R3): each class lists its values as `fields` (`schema.Field`); the
class schema is the union over its base classes (`full_schema`). The checked
values, defaults filled in, are `self.cfg`; an out-of-range value raises
`SchemaError` (a ValueError) naming the part, the value and the allowed range.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, ClassVar

import mujoco
import numpy as np

from ..bodies import BodyState
from ..layout import Grid, Segment
from ..schema import Field, Schema, resolve
from ..signals import is_on

if TYPE_CHECKING:
    from ..signals import SignalBus

BODY_PREFIX = "part/"


class Part:
    type_name: ClassVar[str] = "part"
    role: ClassVar[str] = "prop"
    stage: ClassVar[str] = "device"      # input | logic | device (R1 evaluation order)
    stateful: bool = False               # output does not depend on this step's input
    emits: ClassVar[bool] = False        # default `out` is the part's own name
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
        self.requires_body: list[str] = cfg["requires_body"]
        self.inputs: list[str] = list(cfg["in"])
        out = cfg["out"]
        self.outputs: list[str] = list(out) if out is not None else ([name] if self.emits else [])
        self.enable: str | None = cfg["enable"]
        self.bus: SignalBus | None = None      # set by signals.wire
        self._bid = -1

    @property
    def body_name(self) -> str:
        return BODY_PREFIX + self.name

    # ---- scene ------------------------------------------------------------
    def build(self, spec: mujoco.MjSpec) -> mujoco.MjsBody:
        """Add this part's body (named `body_name`) at `base_xy`; subclasses add geoms."""
        return spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, 0.0])

    def bind(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._bid = model.body(self.body_name).id

    def place(self, model: mujoco.MjModel, xy) -> None:
        """Move the part (randomised layouts). Call `mj_forward` afterwards."""
        self.xy = np.asarray(xy, dtype=float)
        model.body_pos[self._bid, :2] = self.xy

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

    def sources_on(self, on: set[str]) -> bool:
        """Planning: whether this part's sources (writers of its `in`) are all / any in `on`."""
        if not self.sources:
            return False
        flags = [s.name in on for s in self.sources]
        return all(flags) if self.require == "all" else any(flags)

    # ---- planning (grid) ---------------------------------------------------
    def blockers(self, is_open: bool) -> list[Segment]:
        """Segments that block walking; gates return theirs while not open."""
        return []

    def target_mask(self, grid: Grid) -> np.ndarray:
        """Cells where the body centre must be to satisfy this part."""
        return grid.disk(self.xy, 0.05)
