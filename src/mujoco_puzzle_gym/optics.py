"""Ray tracing for the optical parts (R99): where an emitter's light goes.

One emitter casts one ray at a fixed height. Each `mj_ray` hit is looked up in a
map of geom id -> optical part; a part that answers `interact` sends the light on
(a mirror reflects it, a filter dims it, a splitter does both), a photocell takes
the power that arrived, and anything else - a wall, a door, a box, the body -
ends that path. There is no renderer and no optical solver here: a path is a list
of straight segments carrying one scalar power.

Two bounds keep a room honest. `bounces` limits how often one path may turn, and
`MAX_SEGMENTS` limits the whole tree, because a splitter branches and depth alone
does not bound a branching walk (the point Codex made about `BEAM_SPLITTER`).
Paths beyond either bound are dropped rather than truncated, so a room that needs
them reads as dark instead of as nearly right.
"""

from __future__ import annotations

from typing import Iterable, Protocol

import mujoco
import numpy as np

EPS = 3e-4                  # [m] step past a surface before casting again
MAX_SEGMENTS = 16           # segments in the whole tree, branches included
MIN_POWER = 1e-4            # below this a path is dark and stops


class Surface(Protocol):
    """An optical part: it owns geoms and decides what leaves them."""

    def optical_geoms(self, model) -> list[int]:
        ...

    def interact(self, model, data, gid: int, point: np.ndarray, direction: np.ndarray,
                 power: float) -> list[tuple[np.ndarray, np.ndarray, float, int]]:
        """Outgoing (origin, direction, power, turns) for a ray arriving at `point`.
        `turns` is 1 for a path that changed direction and 0 for one that went straight
        on, because `bounces` counts turns: a filter dims the light without bending it
        and must not spend the budget (measured: with `bounces: 1` a beam through one
        filter died at the filter and the cell behind it read 0.000 at every setting,
        which made a room's control vacuous)."""


class Segment:
    """One straight piece of a path, with the power it carries."""

    __slots__ = ("a", "b", "power")

    def __init__(self, a: np.ndarray, b: np.ndarray, power: float):
        self.a, self.b, self.power = np.asarray(a, float), np.asarray(b, float), float(power)

    @property
    def length(self) -> float:
        return float(np.linalg.norm(self.b - self.a))


def normal_of(data, bid: int) -> np.ndarray:
    """World normal (unit, xy) of a panel body: its local +x axis."""
    n = np.asarray(data.xmat[bid], float).reshape(3, 3)[:, 0]
    n = np.array([n[0], n[1], 0.0])
    d = float(np.linalg.norm(n))
    return n / d if d > 1e-9 else np.array([1.0, 0.0, 0.0])


def reflect(direction: np.ndarray, normal: np.ndarray) -> np.ndarray:
    """`direction` mirrored in the plane with `normal`, which is flipped to face the ray."""
    n = normal if float(np.dot(direction, normal)) < 0 else -normal
    out = direction - 2.0 * float(np.dot(direction, n)) * n
    d = float(np.linalg.norm(out))
    return out / d if d > 1e-9 else direction


def quat_for(direction: np.ndarray) -> np.ndarray:
    """Quaternion turning +z onto `direction` (a capsule's axis is its z)."""
    q = np.zeros(4)
    mujoco.mju_quatZ2Vec(q, np.asarray(direction, float))
    return q


def trace(model, data, origin, direction, power: float, bounces: int,
          surfaces: dict[int, Surface], cells: dict[int, str],
          exclude: int = -1) -> tuple[list[Segment], dict[str, float]]:
    """Follow one emitter's light. Returns the segments to draw and the power each
    photocell took (by part name; a cell hit twice sums, which is what a splitter
    feeding one cell from both sides should do)."""
    segs: list[Segment] = []
    lit: dict[str, float] = {}
    gid = np.zeros((1, 1), dtype=np.int32)
    stack = [(np.asarray(origin, float), np.asarray(direction, float), float(power), 0)]
    while stack:
        if len(segs) >= MAX_SEGMENTS:
            break
        start, vec, p, depth = stack.pop()
        if p < MIN_POWER:
            continue
        dist = mujoco.mj_ray(model, data, start, vec, None, 1, exclude, gid)
        if dist < 0:                                   # nothing out there: draw the ray short
            segs.append(Segment(start, start + vec * 2.0, p))
            continue
        point = start + vec * float(dist)
        segs.append(Segment(start, point, p))
        hit = int(gid[0, 0])
        if hit in cells:
            name = cells[hit]
            lit[name] = lit.get(name, 0.0) + p
            continue
        surf = surfaces.get(hit)
        if surf is None:
            continue
        for out_origin, out_dir, out_power, turns in surf.interact(model, data, hit, point,
                                                                   vec, p):
            if depth + turns > bounces:
                continue
            stack.append((out_origin, out_dir, out_power, depth + turns))
    return segs, lit


def collect(parts: Iterable) -> tuple[dict[int, Surface], dict[int, str]]:
    """Geom id -> surface, and geom id -> photocell name, over a room's parts. Called
    after `bind`, because the ids come from the compiled model."""
    surfaces: dict[int, Surface] = {}
    cells: dict[int, str] = {}
    for part in parts:
        if hasattr(part, "interact"):
            for g in part.optical_geoms(None):
                surfaces[g] = part
        elif getattr(part, "is_photocell", False):
            for g in part.optical_geoms(None):
                cells[g] = part.name
    return surfaces, cells
