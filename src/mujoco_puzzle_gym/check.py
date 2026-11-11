"""`mpg check room.yaml` (R6): is the room solvable, stable and actually solved?

body:      every part's `requires_body` capabilities are on the body (R2b, static);
(a) order:     a switch/door order that reaches a goal exists (link graph on the room grid);
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
from .logic import solution_order
from .solver import ScriptedSolver, run_episode

STABILITY_SECONDS = 10.0
STAGES = ("body", "order", "stability", "solver")


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


def check_order(env: PuzzleEnv) -> dict[str, Any]:
    layout = env.sample_layout(None)
    for n, xy in layout["parts"].items():
        env.parts[n].place(env.model, xy)
    res = solution_order(env.grid, env.walls, env.parts, layout["spawn"][:2])
    return {"ok": res["ok"], "order": res["order"], **({"reason": res["reason"]} if not res["ok"] else {})}


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
    result["order"] = check_order(env)
    result["stability"] = check_stability(env)
    if result["body"]["ok"] and result["order"]["ok"]:
        result["solver"] = check_solver(env, seeds, need)
    else:
        why = "no solution order (a)" if not result["order"]["ok"] else "missing body capabilities"
        result["solver"] = {"ok": False, "skipped": why, "successes": 0,
                            "seeds": seeds, "need": need}
    result["ok"] = all(result[k]["ok"] for k in STAGES)
    result["failed"] = [k for k in STAGES if not result[k]["ok"]]
    result["wall_seconds"] = round(time.perf_counter() - t0, 1)
    env.close()
    return result
