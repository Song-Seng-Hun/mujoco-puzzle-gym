"""Puzzle logic on the room grid: which order of parts solves the room (R6a).

Walls and closed gates block; a part counts as reachable when its target area,
grown by one grid cell, touches a cell reachable from the spawn point. There is
no body clearance here: whether the body physically fits is what the scripted
solver, check (c), tests.

Gates open through the signal graph: `plan_channels` runs every part's
`plan_values` in bus order with the input parts in `on` switched on, so a door
fed by gates, counters, transceivers or delays opens when the inputs behind
them are on (time functions taken as "eventually": a delay passes its input,
a counter of a reachable button reaches its target). It assumes every `enable`
is on and that Python logic plugins can turn on when any input is on.

Trapdoors and pits (R20): a hole a part opens with the inputs in `on`
(`Part.hole_mask`) is blocked like a wall, so the order goes round it.

Warp pads (R14): when no reachable switch opens a gate, a reachable warp pad
whose destination lies outside the reached area is taken: it joins the order
(the body must step on it) and the search goes on from its destination.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .layout import Grid, Segment


def plan_channels(parts: dict[str, Any], on: set[str]) -> dict[str, float]:
    """Channel values once the input parts named in `on` are on (planning)."""
    order = parts.bus.order if hasattr(parts, "bus") else list(parts.values())
    vals: dict[str, float] = {}
    get = lambda ch: vals.get(ch, 0.0)  # noqa: E731
    for p in order:
        vals.update(p.plan_values(on, get))
    return vals


def gate_open(gate: Any, vals: dict[str, float]) -> bool:
    return gate.plan_inputs_on(lambda ch: vals.get(ch, 0.0))


def upstream_inputs(part: Any) -> list[Any]:
    """Input-stage parts behind `part`, traced back through logic parts (room order kept)."""
    seen: list[Any] = []
    out: list[Any] = []
    stack = list(reversed(part.sources))
    while stack:
        p = stack.pop()
        if any(p is q for q in seen):
            continue
        seen.append(p)
        if p.stage == "input":
            out.append(p)
        else:
            stack.extend(reversed(p.sources))
    return out


def free_mask(grid: Grid, walls: list[Segment], parts: dict[str, Any], on: set[str],
              inflate: float = 0.0) -> np.ndarray:
    """Free cells with every gate closed unless the inputs in `on` open it, and no
    cell over a hole (trapdoor, pit) that the inputs in `on` open."""
    vals = plan_channels(parts, on)
    get = lambda ch: vals.get(ch, 0.0)  # noqa: E731
    segs = list(walls)
    for p in parts.values():
        segs += p.blockers(p.role == "gate" and gate_open(p, vals))
    free = ~grid.blocked(segs, inflate)
    for p in parts.values():
        if (hole := p.hole_mask(grid, get)) is not None:
            free &= ~grid.dilate(hole, inflate)
    return free


def live_free(grid: Grid, walls: list[Segment], parts: dict[str, Any], passable: float,
              inflate: float = 0.0) -> np.ndarray:
    """Free cells from the parts' state now: gates whose `state()` reached `passable`
    are open, holes (`hole_now`) are blocked. Used by the `demo:` executor."""
    segs = list(walls)
    for p in parts.values():
        segs += p.blockers(p.role == "gate" and p.state() >= passable)
    free = ~grid.blocked(segs, inflate)
    for p in parts.values():
        if (hole := p.hole_now(grid)) is not None:
            free &= ~grid.dilate(hole, inflate)
    return free


def _grow(mask: np.ndarray) -> np.ndarray:
    out = mask.copy()
    out[1:, :] |= mask[:-1, :]
    out[:-1, :] |= mask[1:, :]
    out[:, 1:] |= mask[:, :-1]
    out[:, :-1] |= mask[:, 1:]
    return out


def _take_warp(grid: Grid, parts: dict[str, Any], on: set[str], reach: np.ndarray,
               used: set[str]):
    """A reachable, unused warp pad leading out of `reach`: (part, destination xy) or None."""
    vals = plan_channels(parts, on)
    for p in parts.values():
        if p.role != "warp" or p.name in used or not (_grow(p.target_mask(grid)) & reach).any():
            continue
        dest = p.plan_dest(lambda ch: vals.get(ch, 0.0))
        if dest is not None and not reach[grid.cell(dest)]:
            return p, dest
    return None


def reach_with_warps(grid: Grid, walls: list[Segment], parts: dict[str, Any], on: set[str],
                     start_xy, inflate: float = 0.0) -> np.ndarray:
    """Cells reachable from `start_xy` with the inputs in `on`, also through warp pads
    (R14: a `stages:` room's next zone, a pad to another spot) found on the way."""
    free = free_mask(grid, walls, parts, on, inflate)
    reach = grid.flood(free, start_xy)
    used: set[str] = set()
    while (warp := _take_warp(grid, parts, on, reach, used)) is not None:
        used.add(warp[0].name)
        reach |= grid.flood(free, warp[1])
    return reach


def solution_order(grid: Grid, walls: list[Segment], parts: dict[str, Any],
                   start_xy) -> dict[str, Any]:
    """Find an order of switches (and the gates they open) that reaches a goal.

    Each round turns on reachable switches that open a closed gate: first none
    (gates already open), then one switch at a time, then all reachable ones;
    with all of them, the switches kept are those behind the gate in the signal
    graph. Returns {"ok", "order": [names], "targets": [non-gate names], "reason"}.
    """
    on: set[str] = set()
    order: list[str] = []
    opened: set[str] = set()
    warps_used: set[str] = set()
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
        new_names = {s.name for s in new}
        trial = on | new_names
        candidates = [set()] + [{s.name} for s in new] + ([new_names] if len(new) > 1 else [])
        vals = [plan_channels(parts, on | c) for c in candidates]
        newly_open, needed = [], set()
        for g in gates:
            if g.name in opened:
                continue
            for added, v in zip(candidates, vals):
                if gate_open(g, v):
                    newly_open.append(g)
                    if added is new_names and len(new) > 1:
                        behind = {p.name for p in upstream_inputs(g)} & new_names
                        needed |= behind or new_names
                    else:
                        needed |= added
                    break
        if not newly_open:
            warp = _take_warp(grid, parts, on, reach, warps_used)
            if warp is not None:
                order.append(warp[0].name)
                warps_used.add(warp[0].name)
                start_xy = warp[1]
                continue
            closed = [g for g in gates if g.name not in opened]
            why = []
            for g in closed:
                inputs = upstream_inputs(g)
                if not inputs:
                    why.append(f"{g.name}: no switch is linked to it")
                    continue
                missing = [s.name for s in inputs if s.name not in trial]
                via = [s.name for s in g.sources]
                names = [s.name for s in inputs]
                path = f"{g.require} of {via}" + (f" (inputs {names})" if via != names else "")
                why.append(f"{g.name}: needs {path}, unreachable: {missing}")
            reason = "no goal is reachable from spawn"
            if why:
                reason += "; closed gates: " + "; ".join(why)
            return {"ok": False, "order": order, "targets": [], "reason": reason}
        for s in new:
            if s.name in needed:
                order.append(s.name)
        on |= needed
        for g in newly_open:
            order.append(g.name)
            opened.add(g.name)
