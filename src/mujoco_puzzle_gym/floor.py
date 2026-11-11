"""Tile floors (R10) and the floor height under the body (fall rule).

A room uses the M1 plane floor unless it has floor parts (`floor_tile`,
`trapdoor`) or a map (R37). Then the floor is boxes whose top faces are the
walking surface, laid without gaps: floor parts own their rectangles and the
rest of each zone is covered by as few static boxes as possible (`cover`).
A catch plane lies `CATCH_DEPTH` below so that whatever falls through a hole
lands somewhere.

Floor tiles get geom priority 1 so their friction, not the larger of the two
touching geoms', sets the contact friction (MuJoCo's default combines with
max, and the body's feet have friction 1.0, so a slippery plane alone does
nothing; measured 2026-11-11).

`FloorMap.height_at(xy)` gives the top height of the tile under `xy` from the
layout and the floor parts' current joint values (no rays), or `None` over a
hole (open trapdoor, map void, outside every tile) together with the rim
height of the hole. The env compares the trunk height against it (R10): on a
tile the body has fallen when the trunk is lower than the body's
`fall_height` above that tile; over a hole when the trunk is below the rim.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Iterable

import mujoco
import numpy as np

TILE_THICKNESS = 0.05      # [m] below the top face
CATCH_DEPTH = 0.5          # [m] catch plane below z = 0
DEFAULT_FRICTION = (1.0, 0.005, 0.0001)   # MuJoCo defaults (slide, torsion, roll)
TILE_RGBA = ((0.82, 0.82, 0.78, 1.0), (0.74, 0.74, 0.70, 1.0))
EPS = 1e-9


@dataclass(frozen=True)
class Rect:
    """Axis-aligned rectangle [x0, x1] x [y0, y1]."""

    x0: float
    x1: float
    y0: float
    y1: float

    @classmethod
    def around(cls, centre, half) -> "Rect":
        return cls(centre[0] - half[0], centre[0] + half[0], centre[1] - half[1], centre[1] + half[1])

    def contains(self, xy, margin: float = 0.0) -> bool:
        return (self.x0 - margin <= xy[0] <= self.x1 + margin
                and self.y0 - margin <= xy[1] <= self.y1 + margin)

    def overlaps(self, o: "Rect") -> bool:
        return (min(self.x1, o.x1) - max(self.x0, o.x0) > 1e-6
                and min(self.y1, o.y1) - max(self.y0, o.y0) > 1e-6)

    @property
    def centre(self) -> tuple[float, float]:
        return ((self.x0 + self.x1) / 2, (self.y0 + self.y1) / 2)

    @property
    def half(self) -> tuple[float, float]:
        return ((self.x1 - self.x0) / 2, (self.y1 - self.y0) / 2)

    def shifted(self, dx: float, dy: float = 0.0) -> "Rect":
        return Rect(self.x0 + dx, self.x1 + dx, self.y0 + dy, self.y1 + dy)


def cover(area: Rect, holes: Iterable[Rect]) -> list[Rect]:
    """Rectangles that tile `area` minus `holes` exactly (no gaps, no overlaps).

    Cuts the area along every hole edge, then merges free cells into runs per
    row and runs with the same x extent across rows.
    """
    holes = [h for h in holes if h.overlaps(area)]
    xs = sorted({area.x0, area.x1, *(min(max(v, area.x0), area.x1) for h in holes for v in (h.x0, h.x1))})
    ys = sorted({area.y0, area.y1, *(min(max(v, area.y0), area.y1) for h in holes for v in (h.y0, h.y1))})
    xs = _dedup(xs)
    ys = _dedup(ys)
    nx, ny = len(xs) - 1, len(ys) - 1
    free = np.ones((ny, nx), dtype=bool)
    for j in range(ny):
        for i in range(nx):
            c = ((xs[i] + xs[i + 1]) / 2, (ys[j] + ys[j + 1]) / 2)
            free[j, i] = not any(h.contains(c) for h in holes)
    open_runs: dict[tuple[int, int], Rect] = {}
    out: list[Rect] = []
    for j in range(ny):
        runs, i = [], 0
        while i < nx:
            if not free[j, i]:
                i += 1
                continue
            k = i
            while k + 1 < nx and free[j, k + 1]:
                k += 1
            runs.append((i, k))
            i = k + 1
        nxt: dict[tuple[int, int], Rect] = {}
        for run in runs:
            prev = open_runs.pop(run, None)
            y0 = prev.y0 if prev else ys[j]
            nxt[run] = Rect(xs[run[0]], xs[run[1] + 1], y0, ys[j + 1])
        out += open_runs.values()
        open_runs = nxt
    out += open_runs.values()
    return out


def _dedup(vals: list[float]) -> list[float]:
    out: list[float] = []
    for v in vals:
        if not out or v - out[-1] > 1e-6:
            out.append(v)
    return out


@dataclass
class StaticTile:
    rect: Rect
    top: float = 0.0
    friction: tuple[float, float, float] = DEFAULT_FRICTION


HeightFn = Callable[[Any], "tuple[float | None, float]"]


class FloorMap:
    """Top height of the floor under a point (R10, R37).

    Entries are (rect, fn) with fn(xy) -> (top or None, rim); later entries win
    (floor parts are added after the static tiles). Outside every entry there
    is no floor (rim `outside_rim`).
    """

    def __init__(self, outside_rim: float = 0.0):
        self.entries: list[tuple[Rect, HeightFn]] = []
        self.outside_rim = outside_rim

    def add(self, rect: Rect, fn: HeightFn) -> None:
        self.entries.append((rect, fn))

    def add_static(self, tile: StaticTile) -> None:
        top = float(tile.top)
        self.add(tile.rect, lambda xy, top=top: (top, top))

    def height_at(self, xy) -> tuple[float | None, float]:
        for rect, fn in reversed(self.entries):
            if rect.contains(xy, EPS):
                return fn(xy)
        return None, self.outside_rim


def add_tiles(spec: mujoco.MjSpec, tiles: list[StaticTile], prefix: str = "floor/tile") -> None:
    """Static floor boxes (top face at `tile.top`) on the world body."""
    for i, t in enumerate(tiles):
        (cx, cy), (hx, hy) = t.rect.centre, t.rect.half
        spec.worldbody.add_geom(
            name=f"{prefix}/{i}", type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[hx, hy, TILE_THICKNESS / 2 + t.top / 2 if t.top > 0 else TILE_THICKNESS / 2],
            pos=[cx, cy, (t.top - TILE_THICKNESS) / 2 if t.top > 0 else t.top - TILE_THICKNESS / 2],
            friction=list(t.friction), priority=1, rgba=TILE_RGBA[i % 2])


def lower_catch_plane(spec: mujoco.MjSpec) -> None:
    """Move the world's plane (`floor`) down to catch what falls through holes."""
    spec.geom("floor").pos = [0.0, 0.0, -CATCH_DEPTH]
    spec.geom("floor").rgba = [0.25, 0.25, 0.3, 1.0]
