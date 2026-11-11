"""Letter maps (R37): a room's shape, floor heights, low walls, ramps and cliffs as text.

    map: |
      #########
      #...#...#
      #.......#
      #...#111#
      #########
    cell: 0.1                 # cell side [m]
    step_height: 0.01         # height of one level ('1'..'9')
    legend:                   # extra symbols
      "=": {type: wall, height: 0.06}
      "~": {type: floor, height: 0.0, friction: [0.2, 0.005, 0.0001]}
      "/": {type: ramp, dir: +x, from: 0.0, to: 0.03}

The map centre is the origin; the top line is +y, the left column -x. Symbols:
`#` wall (`wall.height`), `.` floor at 0, `1`-`9` floor at n x `step_height`,
space no floor (cliff; outside the map too). Legend kinds: `wall` (height),
`floor` (height, friction), `ramp` (rising direction `dir`, height `from` at
the low end, `to` at the high end). A run of one ramp symbol along its `dir`
is one ramp. Cells are reported 1-based as (row, col) from the top left.

Built as: wall boxes and floor boxes (neighbouring cells of one kind merged
into one box), one tilted box per ramp run, no box for cliffs. The planner
grid takes walls, cliffs, steps higher than the body's `max_step` and ramps
steeper than its `max_slope` (R2b capability values) as blocked (`terrain`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np

from .floor import DEFAULT_FRICTION, TILE_THICKNESS, Rect, StaticTile

DIRS = {"+x": (1.0, 0.0), "-x": (-1.0, 0.0), "+y": (0.0, 1.0), "-y": (0.0, -1.0)}
RESERVED = set("#. 123456789")
DEFAULT_CELL = 0.1
DEFAULT_STEP_HEIGHT = 0.01     # OQ5: the walker climbs 1 cm steps (10/10), 1.2 cm 8/10
EPS = 1e-6


class MapError(ValueError):
    """Invalid map; the message says what to fix."""


@dataclass(frozen=True)
class Ramp:
    rect: Rect
    dir: str
    lo: float          # height at the low end (`from`)
    hi: float          # height at the high end (`to`)
    cells: tuple[tuple[int, int], ...]

    @property
    def length(self) -> float:
        return self.rect.x1 - self.rect.x0 if self.dir in ("+x", "-x") else self.rect.y1 - self.rect.y0

    @property
    def slope_deg(self) -> float:
        return math.degrees(math.atan2(abs(self.hi - self.lo), self.length))

    def height_at(self, xy) -> float:
        d = DIRS[self.dir]
        start = (self.rect.x0 if d[0] > 0 else self.rect.x1) if d[0] else (self.rect.y0 if d[1] > 0 else self.rect.y1)
        s = abs((xy[0] if d[0] else xy[1]) - start)
        t = min(max(s / self.length, 0.0), 1.0)
        return self.lo + (self.hi - self.lo) * t


def _quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


def _rotate(q, v):
    w, x, y, z = q
    u = np.array([x, y, z])
    v = np.asarray(v, float)
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def merge_cells(mask: np.ndarray) -> list[tuple[int, int, int, int]]:
    """Rectangles (r0, r1, c0, c1), inclusive, covering the True cells of `mask` exactly."""
    nr, nc = mask.shape
    open_runs: dict[tuple[int, int], int] = {}      # (c0, c1) -> r0
    out = []
    for r in range(nr):
        runs, c = [], 0
        while c < nc:
            if not mask[r, c]:
                c += 1
                continue
            k = c
            while k + 1 < nc and mask[r, k + 1]:
                k += 1
            runs.append((c, k))
            c = k + 1
        nxt = {}
        for run in runs:
            nxt[run] = open_runs.pop(run, r)
        out += [(r0, r - 1, c0, c1) for (c0, c1), r0 in open_runs.items()]
        open_runs = nxt
    out += [(r0, nr - 1, c0, c1) for (c0, c1), r0 in open_runs.items()]
    return out


class RoomMap:
    def __init__(self, text: Any, cell: float = DEFAULT_CELL, step_height: float = DEFAULT_STEP_HEIGHT,
                 wall_height: float = 0.25, legend: dict | None = None):
        if not isinstance(text, str) or not text.strip("\n"):
            raise MapError("`map` must be a multi-line string (use `map: |`)")
        try:
            self.cell = float(cell)
            self.step_height = float(step_height)
        except (TypeError, ValueError):
            raise MapError("`cell` and `step_height` must be numbers") from None
        if not 0.02 <= self.cell <= 1.0:
            raise MapError(f"cell = {self.cell} m is out of range [0.02, 1]")
        if not 0.0 < self.step_height <= 0.2:
            raise MapError(f"step_height = {self.step_height} m is out of range (0, 0.2]")
        lines = text.split("\n")
        while lines and not lines[-1].strip():
            lines.pop()
        while lines and not lines[0].strip():
            lines.pop(0)
        self.rows = [ln.rstrip("\r") for ln in lines]
        self.nrows = len(self.rows)
        self.ncols = max(len(r) for r in self.rows)
        self.rows = [r.ljust(self.ncols) for r in self.rows]
        self.wall_height = float(wall_height)
        self.legend = self._legend(legend or {})
        self.kinds: dict[tuple[int, int], tuple] = {}
        for r, row in enumerate(self.rows):
            for c, ch in enumerate(row):
                self.kinds[(r, c)] = self._kind(ch, r, c)
        self.ramps = self._ramps()
        self._ramp_of = {cell: rp for rp in self.ramps for cell in rp.cells}
        floors = [k[1] for k in self.kinds.values() if k[0] == "floor"]
        floors += [h for rp in self.ramps for h in (rp.lo, rp.hi)]
        self.rim = min([0.0, *floors])

    # ---- parsing ----------------------------------------------------------------
    def _legend(self, legend: dict) -> dict[str, dict]:
        if not isinstance(legend, dict):
            raise MapError("`legend` must map one-character symbols to {type: wall|floor|ramp, ...}")
        out = {}
        for sym, d in legend.items():
            sym = str(sym)
            if len(sym) != 1 or sym in RESERVED:
                raise MapError(f"legend symbol {sym!r}: use one character other than "
                               f"'#', '.', ' ' and digits")
            if not isinstance(d, dict) or d.get("type") not in ("wall", "floor", "ramp"):
                raise MapError(f"legend {sym!r}: needs `type: wall | floor | ramp`, got {d!r}")
            try:
                if d["type"] == "wall":
                    e = {"type": "wall", "height": float(d.get("height", self.wall_height))}
                    if not 0.005 <= e["height"] <= 2.0:
                        raise MapError(f"legend {sym!r}: wall height {e['height']} m is out of range "
                                       "[0.005, 2]")
                elif d["type"] == "floor":
                    fr = tuple(float(v) for v in d.get("friction", DEFAULT_FRICTION))
                    if len(fr) != 3 or not all(0 <= v <= 5 for v in fr):
                        raise MapError(f"legend {sym!r}: friction must be 3 numbers in [0, 5]")
                    e = {"type": "floor", "height": float(d.get("height", 0.0)), "friction": fr}
                    if not -0.2 <= e["height"] <= 0.5:
                        raise MapError(f"legend {sym!r}: floor height {e['height']} m is out of range "
                                       "[-0.2, 0.5]")
                else:
                    if d.get("dir") not in DIRS:
                        raise MapError(f"legend {sym!r}: ramp `dir` must be one of {sorted(DIRS)}")
                    e = {"type": "ramp", "dir": d["dir"], "from": float(d["from"]), "to": float(d["to"])}
            except (KeyError, TypeError, ValueError):
                raise MapError(f"legend {sym!r}: bad values {d!r}") from None
            out[sym] = e
        return out

    def _kind(self, ch: str, r: int, c: int) -> tuple:
        if ch == "#":
            return ("wall", self.wall_height)
        if ch == ".":
            return ("floor", 0.0, DEFAULT_FRICTION)
        if ch == " ":
            return ("void",)
        if ch.isdigit():
            return ("floor", int(ch) * self.step_height, DEFAULT_FRICTION)
        if ch in self.legend:
            e = self.legend[ch]
            if e["type"] == "wall":
                return ("wall", e["height"])
            if e["type"] == "floor":
                return ("floor", e["height"], e["friction"])
            return ("ramp", ch)
        raise MapError(f"map cell {self.label(r, c)}: unknown symbol {ch!r} (define it in `legend`)")

    def _ramps(self) -> list[Ramp]:
        out = []
        seen: set[tuple[int, int]] = set()
        for (r, c), k in sorted(self.kinds.items()):
            if k[0] != "ramp" or (r, c) in seen:
                continue
            sym = k[1]
            e = self.legend[sym]
            along_x = e["dir"] in ("+x", "-x")
            cells = []
            rr, cc = r, c
            while (rr, cc) in self.kinds and self.kinds[(rr, cc)] == ("ramp", sym):
                cells.append((rr, cc))
                if along_x:
                    cc += 1
                else:
                    rr += 1
            seen.update(cells)
            rects = [self.cell_rect(*x) for x in cells]
            rect = Rect(min(q.x0 for q in rects), max(q.x1 for q in rects),
                        min(q.y0 for q in rects), max(q.y1 for q in rects))
            out.append(Ramp(rect, e["dir"], e["from"], e["to"], tuple(cells)))
        return out

    # ---- geometry ---------------------------------------------------------------
    def label(self, r: int, c: int) -> str:
        return f"(row {r + 1}, col {c + 1})"

    def centre(self, r: int, c: int) -> tuple[float, float]:
        return ((c - (self.ncols - 1) / 2) * self.cell, ((self.nrows - 1) / 2 - r) * self.cell)

    def cell_rect(self, r: int, c: int) -> Rect:
        x, y = self.centre(r, c)
        h = self.cell / 2
        return Rect(x - h, x + h, y - h, y + h)

    def cell_of(self, xy) -> tuple[int, int] | None:
        c = int(math.floor(xy[0] / self.cell + self.ncols / 2))
        r = int(math.floor(self.nrows / 2 - xy[1] / self.cell))
        return (r, c) if 0 <= r < self.nrows and 0 <= c < self.ncols else None

    def extent(self) -> Rect:
        hx, hy = self.ncols * self.cell / 2, self.nrows * self.cell / 2
        return Rect(-hx, hx, -hy, hy)

    def _rect_of(self, r0, r1, c0, c1) -> Rect:
        a, b = self.cell_rect(r0, c0), self.cell_rect(r1, c1)
        return Rect(a.x0, b.x1, b.y0, a.y1)

    def wall_rects(self) -> list[tuple[Rect, float]]:
        out = []
        for h in sorted({k[1] for k in self.kinds.values() if k[0] == "wall"}):
            mask = self._mask(lambda k, h=h: k[0] == "wall" and k[1] == h)
            out += [(self._rect_of(*b), h) for b in merge_cells(mask)]
        return out

    def floor_tiles(self) -> list[StaticTile]:
        out = []
        keys = sorted({(k[1], k[2]) for k in self.kinds.values() if k[0] == "floor"})
        for h, fr in keys:
            mask = self._mask(lambda k, h=h, fr=fr: k[0] == "floor" and k[1] == h and k[2] == fr)
            out += [StaticTile(self._rect_of(*b), h, fr) for b in merge_cells(mask)]
        return out

    def _mask(self, pred) -> np.ndarray:
        m = np.zeros((self.nrows, self.ncols), dtype=bool)
        for (r, c), k in self.kinds.items():
            m[r, c] = pred(k)
        return m

    def kind_at(self, xy) -> tuple:
        rc = self.cell_of(xy)
        return ("void",) if rc is None else self.kinds[rc]

    def height_at(self, xy) -> tuple[float | None, float]:
        """(top of the floor at `xy` or None over a cliff or wall, rim height)."""
        rc = self.cell_of(xy)
        if rc is None:
            return None, self.rim
        k = self.kinds[rc]
        if k[0] == "floor":
            return k[1], k[1]
        if k[0] == "ramp":
            h = self._ramp_of[rc].height_at(xy)
            return h, h
        if k[0] == "wall":
            return k[1], k[1]
        return None, self.rim

    # ---- scene ------------------------------------------------------------------
    def build(self, spec: mujoco.MjSpec, wall_rgba, tile_rgba) -> None:
        for i, (rect, h) in enumerate(self.wall_rects()):
            (cx, cy), (hx, hy) = rect.centre, rect.half
            spec.worldbody.add_geom(name=f"wall/{i}", type=mujoco.mjtGeom.mjGEOM_BOX,
                                    size=[hx, hy, (h + TILE_THICKNESS) / 2],
                                    pos=[cx, cy, (h - TILE_THICKNESS) / 2], rgba=wall_rgba)
        for i, rp in enumerate(self.ramps):
            d = np.array([*DIRS[rp.dir], 0.0])
            dh = rp.hi - rp.lo
            L = rp.length
            a = math.atan2(dh, L)
            yaw = math.atan2(d[1], d[0])
            q_yaw = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
            axis = np.cross(d, [0.0, 0.0, 1.0])          # +a raises the +d end
            q_tilt = [math.cos(a / 2), *(math.sin(a / 2) * axis)]
            q = _quat_mul(q_tilt, q_yaw)
            width = (rp.rect.y1 - rp.rect.y0) if rp.dir in ("+x", "-x") else (rp.rect.x1 - rp.rect.x0)
            top = np.array([*rp.rect.centre, (rp.lo + rp.hi) / 2])
            centre = top - _rotate(q, [0.0, 0.0, TILE_THICKNESS / 2])
            spec.worldbody.add_geom(name=f"floor/ramp/{i}", type=mujoco.mjtGeom.mjGEOM_BOX,
                                    size=[math.hypot(L, dh) / 2, width / 2, TILE_THICKNESS / 2],
                                    pos=centre.tolist(), quat=q, priority=1,
                                    rgba=tile_rgba)

    # ---- planning ---------------------------------------------------------------
    def terrain(self, grid, max_step: float, max_slope: float) -> tuple[np.ndarray, np.ndarray]:
        """(blocked, heights) on `grid`: walls, cliffs, steep ramps, and the higher side of
        every neighbouring pair of cells whose heights differ by more than `max_step`."""
        heights = np.full((grid.ny, grid.nx), np.nan)
        blocked = np.zeros((grid.ny, grid.nx), dtype=bool)
        for iy in range(grid.ny):
            for ix in range(grid.nx):
                xy = (grid.X[iy, ix], grid.Y[iy, ix])
                rc = self.cell_of(xy)
                k = ("void",) if rc is None else self.kinds[rc]
                if k[0] in ("void", "wall"):
                    blocked[iy, ix] = True
                    continue
                if k[0] == "ramp" and self._ramp_of[rc].slope_deg > max_slope + EPS:
                    blocked[iy, ix] = True
                heights[iy, ix] = self.height_at(xy)[0]
        h = np.where(np.isnan(heights), -np.inf, heights)
        with np.errstate(invalid="ignore"):
            for lo, hi, axis in ((h[:-1, :], h[1:, :], 0), (h[:, :-1], h[:, 1:], 1)):
                step = np.isfinite(hi - lo) & (np.abs(hi - lo) > max_step + EPS)
                up = step & (hi > lo)          # the second cell of the pair is higher
                down = step & (lo > hi)
                if axis == 0:
                    blocked[1:, :] |= up
                    blocked[:-1, :] |= down
                else:
                    blocked[:, 1:] |= up
                    blocked[:, :-1] |= down
        return blocked, heights

    def step_reasons(self, reach_cells: set[tuple[int, int]], max_step: float, max_slope: float,
                     limit: int = 6) -> list[str]:
        """Why floor next to the reached cells is out of reach: steps and steep ramps."""
        out = []
        for (r, c) in sorted(reach_cells):
            for dr, dc in ((0, 1), (1, 0), (0, -1), (-1, 0)):
                n = (r + dr, c + dc)
                if n in reach_cells or n not in self.kinds or self.kinds[n][0] in ("void", "wall"):
                    continue
                k = self.kinds[n]
                if k[0] == "ramp" and self._ramp_of[n].slope_deg > max_slope + EPS:
                    out.append(f"ramp at cell {self.label(*n)} is {self._ramp_of[n].slope_deg:.1f} deg, "
                               f"steeper than max_slope {max_slope:g} deg")
                    continue
                h0 = self._edge_height((r, c), n)
                h1 = self._edge_height(n, (r, c))
                if abs(h1 - h0) > max_step + EPS:
                    out.append(f"step from cell {self.label(r, c)} ({h0:.3f} m) to cell {self.label(*n)} "
                               f"({h1:.3f} m) is higher than max_step {max_step:g} m")
                if len(out) >= limit:
                    return out
        return list(dict.fromkeys(out))

    def _edge_height(self, rc, toward) -> float:
        """Floor height of cell `rc` at its edge facing cell `toward`."""
        k = self.kinds[rc]
        if k[0] == "floor":
            return k[1]
        x, y = self.centre(*rc)
        tx, ty = self.centre(*toward)
        return self._ramp_of[rc].height_at(((x + tx) / 2, (y + ty) / 2))
