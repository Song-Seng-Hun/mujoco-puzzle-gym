"""Puzzle part base class (R4, R17).

A part adds its own body to the scene spec, reads MuJoCo data each body step,
and may write only its own actuator targets. It never touches the walking body
except through the `BodyState` the environment hands it.

Roles drive the generic logic in `check` and `PuzzleEnv`:
- "switch": has an on/off state; `satisfied()` is True when on. Can be a link source.
- "gate":   blocks movement until its linked sources are on. Can be a link target.
- "goal":   `satisfied()` is True when the body reached it; ends the episode.
- "prop":   anything else (decoration, obstacles via `blockers`).
"""

from __future__ import annotations

from typing import Any, ClassVar

import mujoco
import numpy as np

from ..bodies import BodyState
from ..layout import Grid, Segment

BODY_PREFIX = "part/"


class Part:
    type_name: ClassVar[str] = "part"
    role: ClassVar[str] = "prop"

    def __init__(self, name: str, cfg: dict[str, Any]):
        if "pos" not in cfg or len(cfg["pos"]) != 2:
            raise ValueError(f"part {name!r}: `pos: [x, y]` is required")
        self.name = name
        self.cfg = cfg
        self.base_xy = np.asarray(cfg["pos"], dtype=float)
        self.random = np.asarray(cfg.get("random", [0.0, 0.0]), dtype=float)
        if self.random.shape != (2,) or (self.random < 0).any():
            raise ValueError(f"part {name!r}: `random` must be [dx, dy] >= 0")
        self.xy = self.base_xy.copy()
        self.sources: list[Part] = []          # filled from room links (gates)
        self.require = cfg.get("require", "all")
        if self.require not in ("all", "any"):
            raise ValueError(f"part {name!r}: `require` must be 'all' or 'any'")
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

    def sources_on(self, on: set[str] | None = None) -> bool:
        """Whether this part's link sources are on (from live state, or the set `on`)."""
        if not self.sources:
            return False
        flags = [(s.name in on) if on is not None else s.satisfied() for s in self.sources]
        return all(flags) if self.require == "all" else any(flags)

    # ---- planning (grid) ---------------------------------------------------
    def blockers(self, is_open: bool) -> list[Segment]:
        """Segments that block walking; gates return theirs while not open."""
        return []

    def target_mask(self, grid: Grid) -> np.ndarray:
        """Cells where the body centre must be to satisfy this part."""
        return grid.disk(self.xy, 0.05)
