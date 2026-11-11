"""`mpg check room.yaml` (R6): is the room solvable, stable and actually solved?

body:      every part's `requires_body` capabilities are on the body (R2b, static);
parts:     no part reports a static problem (`Part.check_static`: a ball too big
           for its goal net, a ball in a wall or out of reach, R38);
(a) order:     a switch/door order that reaches a goal exists (link graph on the room grid),
               and every coin is reachable (R36);
               in letter-map rooms (R37) also: parts, spawn and their random ranges lie on
               floor cells, and a failure names the steps / steep ramps (cells) in the way;
(b) stability: zero command for 10 s at the nominal layout: no NaN, no fall;
(c) solver:    the scripted privileged solver succeeds on >= `need` of `seeds`
               random layouts (seeds 0..seeds-1); skipped when body or (a) fails.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import numpy as np

from .env import PuzzleEnv
from .logic import free_mask, solution_order
from .parts.floor.coin import unreachable_coins
from .solver import ScriptedSolver, run_episode

STABILITY_SECONDS = 10.0
STAGES = ("body", "parts", "order", "stability", "solver")


def check_body(env: PuzzleEnv) -> dict[str, Any]:
    """Static: parts whose `requires_body` names a capability the body lacks (R2b)."""
    caps = env.body.capabilities
    missing = {n: m for n, p in env.parts.items() if (m := caps.missing(p.requires_body))}
    res: dict[str, Any] = {"ok": not missing, "capabilities": sorted(caps.names)}
    if missing:
        res["missing"] = missing
        res["reason"] = "; ".join(f"part {n!r} needs body capabilities {m} the body lacks"
                                  for n, m in missing.items())
    return res


def check_parts(env: PuzzleEnv) -> dict[str, Any]:
    """Static: reasons parts give through `check_static` (R38)."""
    why = [r for p in env.parts.values() for r in p.check_static(env)]
    return {"ok": not why, **({"reason": "; ".join(why)} if why else {})}


def check_order(env: PuzzleEnv) -> dict[str, Any]:
    layout = env.sample_layout(None)
    for n, xy in layout["parts"].items():
        env.parts[n].place(env.model, xy)
    res = solution_order(env.grid, env.walls, env.parts, layout["spawn"][:2])
    if res["ok"] and (coins := unreachable_coins(env, res["order"], layout["spawn"][:2])):
        res = {**res, "ok": False, "reason": "; ".join(coins)}
    out = {"ok": res["ok"], "order": res["order"], **({"reason": res["reason"]} if not res["ok"] else {})}
    if env.room.map is not None:
        places = map_placement(env)
        if places:
            out["ok"] = False
            out["placement"] = places
            out["reason"] = "; ".join(([out["reason"]] if "reason" in out else []) + places)
        elif not res["ok"]:
            steps = map_steps(env, layout["spawn"][:2])
            if steps:
                out["steps"] = steps
                out["reason"] += "; in the way: " + "; ".join(steps)
    return out


def map_placement(env: PuzzleEnv) -> list[str]:
    """Letter maps (R37): parts, spawn and their random ranges must lie on floor cells."""
    m = env.room.map
    out: list[str] = []

    def probe(what: str, xy, rnd, half=(0.0, 0.0)) -> None:
        ex, ey = abs(rnd[0]) + half[0], abs(rnd[1]) + half[1]
        for sx, sy in ((0, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)):
            p = (xy[0] + sx * ex, xy[1] + sy * ey)
            k = m.kind_at(p)
            if k[0] in ("wall", "void"):
                rc = m.cell_of(p)
                where = f"cell {m.label(*rc)}" if rc is not None else "outside the map"
                what_k = "is on a wall" if k[0] == "wall" else "has no floor under it"
                span = " (with its random range and size)" if (sx or sy) else ""
                out.append(f"{what} at ({p[0]:.2f}, {p[1]:.2f}){span} {what_k}: {where}")
                return

    probe("spawn", env.room.spawn_pos, env.room.spawn_random[:2])
    for n, p in env.parts.items():
        if not p.has_body or getattr(p, "floor_part", False):
            continue
        half = tuple(getattr(p, "half", (0.0, 0.0))) if np.ndim(getattr(p, "half", 0.0)) == 1 else (0.0, 0.0)
        probe(f"part {n!r}", p.base_xy, p.random, half)
    return out


def map_steps(env: PuzzleEnv, start_xy) -> list[str]:
    """Steps and ramps between the reachable floor (all switches on) and the rest (R37)."""
    m = env.room.map
    on = {p.name for p in env.parts.values() if p.role in ("switch", "warp")}
    reach = env.grid.flood(free_mask(env.grid, env.walls, env.parts, on), start_xy)
    cells = {m.cell_of(env.grid.center(iy, ix)) for iy, ix in zip(*np.nonzero(reach))}
    cells.discard(None)
    vals = env.body.capabilities.values
    return m.step_reasons(cells, float(vals.get("max_step", np.inf)), float(vals.get("max_slope", 90.0)))


def check_stability(env: PuzzleEnv, seconds: float = STABILITY_SECONDS) -> dict[str, Any]:
    env.reset(options={"layout": env.sample_layout(None)})
    start = env.body_state.pos.copy()
    steps = int(round(seconds * env.hz))
    sane, fallen = True, False
    for _ in range(steps):
        sane &= env.advance(np.zeros(3))
        fallen |= env.body_state.fallen
        if not sane:
            break
    drift = float(np.linalg.norm(env.body_state.pos[:2] - start[:2]))
    return {"ok": sane and not fallen, "seconds": seconds, "nan": not sane, "fallen": fallen,
            "drift_m": round(drift, 3)}


def check_solver(env: PuzzleEnv, seeds: int = 20, need: int = 18) -> dict[str, Any]:
    solver = ScriptedSolver(env)
    episodes = [run_episode(env, s, solver) for s in range(seeds)]
    wins = sum(e["success"] for e in episodes)
    failures = [{k: e[k] for k in ("seed", "fallen", "nan", "target", "seconds")}
                for e in episodes if not e["success"]]
    times = [e["seconds"] for e in episodes if e["success"]]
    return {"ok": wins >= need, "successes": wins, "seeds": seeds, "need": need,
            "mean_success_seconds": round(float(np.mean(times)), 2) if times else None,
            "failures": failures}


def check(room: str | Path, task: str | Path | None = None, seeds: int = 20,
          need: int = 18) -> dict[str, Any]:
    t0 = time.perf_counter()
    env = PuzzleEnv(task=task, room=room)
    result: dict[str, Any] = {"room": env.room.name, "episode_seconds": env.max_steps / env.hz}
    result["body"] = check_body(env)
    result["parts"] = check_parts(env)
    result["order"] = check_order(env)
    result["stability"] = check_stability(env)
    if result["body"]["ok"] and result["order"]["ok"] and result["parts"]["ok"]:
        result["solver"] = check_solver(env, seeds, need)
    else:
        why = ("no solution order (a)" if not result["order"]["ok"] else
               "missing body capabilities" if not result["body"]["ok"] else "part static check")
        result["solver"] = {"ok": False, "skipped": why, "successes": 0,
                            "seeds": seeds, "need": need}
    result["ok"] = all(result[k]["ok"] for k in STAGES)
    result["failed"] = [k for k in STAGES if not result[k]["ok"]]
    result["wall_seconds"] = round(time.perf_counter() - t0, 1)
    env.close()
    return result
