"""2-D occupancy grid of a room: flood fill and geodesic distance fields.

Used by `mpg check` (solution order, scripted solver) and by the progress
reward. Obstacles are thick line segments (walls, closed doors); a cell is
blocked when its centre is within `thickness / 2 + inflate` of a segment.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

import numpy as np

DEFAULT_RES = 0.04  # [m]


@dataclass(frozen=True)
class Segment:
    p0: tuple[float, float]
    p1: tuple[float, float]
    thickness: float


class Grid:
    def __init__(self, size: tuple[float, float], res: float = DEFAULT_RES,
                 center: tuple[float, float] = (0.0, 0.0)):
        self.size = (float(size[0]), float(size[1]))
        self.res = res
        self.origin = (float(center[0]), float(center[1]))   # centre of the grid area
        self.nx = int(math.ceil(self.size[0] / res))
        self.ny = int(math.ceil(self.size[1] / res))
        self.x0 = self.origin[0] - self.nx * res / 2
        self.y0 = self.origin[1] - self.ny * res / 2
        xs = self.x0 + (np.arange(self.nx) + 0.5) * res
        ys = self.y0 + (np.arange(self.ny) + 0.5) * res
        self.X, self.Y = np.meshgrid(xs, ys)          # shape (ny, nx)

    # ---- geometry -----------------------------------------------------------
    def cell(self, xy) -> tuple[int, int]:
        ix = int(np.clip((xy[0] - self.x0) // self.res, 0, self.nx - 1))
        iy = int(np.clip((xy[1] - self.y0) // self.res, 0, self.ny - 1))
        return iy, ix

    def center(self, iy: int, ix: int) -> np.ndarray:
        return np.array([self.X[iy, ix], self.Y[iy, ix]])

    def blocked(self, segments: list[Segment], inflate: float = 0.0) -> np.ndarray:
        """Cells whose centre lies inside any (inflated) segment, plus the border."""
        out = np.zeros((self.ny, self.nx), dtype=bool)
        # Half a cell diagonal keeps thin walls watertight on the grid.
        pad = inflate + self.res * 0.75
        for s in segments:
            out |= self._seg_dist(s) <= s.thickness / 2 + pad
        out[0, :] = out[-1, :] = out[:, 0] = out[:, -1] = True
        return out

    def _seg_dist(self, s: Segment) -> np.ndarray:
        p0, p1 = np.asarray(s.p0, float), np.asarray(s.p1, float)
        d = p1 - p0
        L2 = float(d @ d)
        px, py = self.X - p0[0], self.Y - p0[1]
        t = np.clip((px * d[0] + py * d[1]) / L2, 0.0, 1.0) if L2 > 0 else np.zeros_like(px)
        return np.hypot(px - t * d[0], py - t * d[1])

    def disk(self, xy, radius: float) -> np.ndarray:
        return np.hypot(self.X - xy[0], self.Y - xy[1]) <= radius

    def rect(self, xy, half: tuple[float, float]) -> np.ndarray:
        return (np.abs(self.X - xy[0]) <= half[0]) & (np.abs(self.Y - xy[1]) <= half[1])

    # ---- search -------------------------------------------------------------
    def nearest_free(self, free: np.ndarray, xy) -> tuple[int, int] | None:
        if not free.any():
            return None
        iy, ix = self.cell(xy)
        if free[iy, ix]:
            return iy, ix
        d = np.where(free, np.hypot(self.X - xy[0], self.Y - xy[1]), np.inf)
        k = int(np.argmin(d))
        return divmod(k, self.nx)

    def flood(self, free: np.ndarray, start_xy) -> np.ndarray:
        """Cells reachable from the free cell nearest `start_xy` (4-connected)."""
        reach = np.zeros_like(free)
        start = self.nearest_free(free, start_xy)
        if start is None:
            return reach
        reach[start] = True
        while True:
            grown = reach.copy()
            grown[1:, :] |= reach[:-1, :]
            grown[:-1, :] |= reach[1:, :]
            grown[:, 1:] |= reach[:, :-1]
            grown[:, :-1] |= reach[:, 1:]
            grown &= free
            if (grown == reach).all():
                return reach
            reach = grown

    def distance(self, free: np.ndarray, target: np.ndarray) -> np.ndarray:
        """Geodesic distance [m] from every free cell to the target cells (8-connected).

        Unreachable or blocked cells get inf. If no target cell is free, the
        free cells nearest the target act as the target, so a field always exists.
        """
        seeds = target & free
        if not seeds.any() and target.any() and free.any():
            ty, tx = np.nonzero(target)
            d = np.full(free.shape, np.inf)
            for y, x in zip(ty, tx):
                d = np.minimum(d, np.hypot(self.X - self.X[y, x], self.Y - self.Y[y, x]))
            d = np.where(free, d, np.inf)
            seeds = d <= d.min() + self.res
        dist = np.full(free.shape, np.inf)
        heap: list[tuple[float, int, int]] = []
        for y, x in zip(*np.nonzero(seeds)):
            dist[y, x] = 0.0
            heap.append((0.0, int(y), int(x)))
        heapq.heapify(heap)
        r, rd = self.res, self.res * math.sqrt(2.0)
        steps = ((1, 0, r), (-1, 0, r), (0, 1, r), (0, -1, r),
                 (1, 1, rd), (1, -1, rd), (-1, 1, rd), (-1, -1, rd))
        ny, nx = free.shape
        while heap:
            d0, y, x = heapq.heappop(heap)
            if d0 > dist[y, x]:
                continue
            for dy, dx, c in steps:
                yy, xx = y + dy, x + dx
                if 0 <= yy < ny and 0 <= xx < nx and free[yy, xx]:
                    if dx and dy and not (free[y + dy, x] and free[y, x + dx]):
                        continue  # no corner cutting
                    nd = d0 + c
                    if nd < dist[yy, xx]:
                        dist[yy, xx] = nd
                        heapq.heappush(heap, (nd, yy, xx))
        return dist

    def lookup(self, field: np.ndarray, xy) -> float:
        """Field value at `xy`: its cell, or the nearest finite cell plus the gap."""
        iy, ix = self.cell(xy)
        v = field[iy, ix]
        if np.isfinite(v):
            return float(v)
        finite = np.isfinite(field)
        if not finite.any():
            return float("inf")
        gap = np.where(finite, np.hypot(self.X - xy[0], self.Y - xy[1]), np.inf)
        k = int(np.argmin(gap))
        return float(field.flat[k] + gap.flat[k])

    def descend(self, field: np.ndarray, xy, length: float) -> np.ndarray:
        """Follow steepest descent of `field` from `xy` for about `length` metres."""
        free = np.isfinite(field)
        start = self.nearest_free(free, xy)
        if start is None:
            return np.asarray(xy, float)
        y, x = start
        travelled = 0.0
        while travelled < length and field[y, x] > 0:
            best = None
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    yy, xx = y + dy, x + dx
                    if (dy or dx) and 0 <= yy < self.ny and 0 <= xx < self.nx and field[yy, xx] < field[y, x]:
                        if best is None or field[yy, xx] < field[best]:
                            best = (yy, xx)
            if best is None:
                break
            travelled += self.res * (math.sqrt(2.0) if best[0] != y and best[1] != x else 1.0)
            y, x = best
        return self.center(y, x)
