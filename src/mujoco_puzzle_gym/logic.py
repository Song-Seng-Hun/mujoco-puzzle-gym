"""Puzzle logic on the room grid: which order of parts solves the room (R6a).

Walls and closed gates block; a part counts as reachable when its target area,
grown by one grid cell, touches a cell reachable from the spawn point. There is
no body clearance here: whether the body physically fits is what the scripted
solver, check (c), tests.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .layout import Grid, Segment


def free_mask(grid: Grid, walls: list[Segment], parts: dict[str, Any], on: set[str],
              inflate: float = 0.0) -> np.ndarray:
    """Free cells with every gate closed unless its sources are in `on`."""
    segs = list(walls)
    for p in parts.values():
        segs += p.blockers(p.role == "gate" and p.sources_on(on))
    return ~grid.blocked(segs, inflate)


def _grow(mask: np.ndarray) -> np.ndarray:
    out = mask.copy()
    out[1:, :] |= mask[:-1, :]
    out[:-1, :] |= mask[1:, :]
    out[:, 1:] |= mask[:, :-1]
    out[:, :-1] |= mask[:, 1:]
    return out


def solution_order(grid: Grid, walls: list[Segment], parts: dict[str, Any],
                   start_xy) -> dict[str, Any]:
    """Find an order of switches (and the gates they open) that reaches a goal.

    Returns {"ok", "order": [names], "targets": [non-gate names], "reason"}.
    """
    on: set[str] = set()
    order: list[str] = []
    opened: set[str] = set()
    switches = [p for p in parts.values() if p.role == "switch"]
    gates = [p for p in parts.values() if p.role == "gate"]
    goals = [p for p in parts.values() if p.role == "goal"]
    while True:
        reach = grid.flood(free_mask(grid, walls, parts, on), start_xy)
        for g in goals:
            if (_grow(g.target_mask(grid)) & reach).any():
                order.append(g.name)
                targets = [n for n in order if parts[n].role != "gate"]
                return {"ok": True, "order": order, "targets": targets, "reason": ""}
        new = [s for s in switches if s.name not in on and (_grow(s.target_mask(grid)) & reach).any()]
        trial = on | {s.name for s in new}
        newly_open = [g for g in gates if g.name not in opened and g.sources_on(trial)]
        if not newly_open:
            closed = [g for g in gates if g.name not in opened]
            why = []
            for g in closed:
                if not g.sources:
                    why.append(f"{g.name}: no switch is linked to it")
                else:
                    missing = [s.name for s in g.sources if s.name not in trial]
                    why.append(f"{g.name}: needs {g.require} of {[s.name for s in g.sources]}, "
                               f"unreachable: {missing}")
            reason = "no goal is reachable from spawn"
            if why:
                reason += "; closed gates: " + "; ".join(why)
            return {"ok": False, "order": order, "targets": [], "reason": reason}
        # Keep only switches that a newly opened gate needs.
        needed = {s.name for g in newly_open for s in g.sources} & {s.name for s in new}
        for s in new:
            if s.name in needed:
                order.append(s.name)
        on |= needed
        for g in newly_open:
            order.append(g.name)
            opened.add(g.name)
