"""`mpg check room.yaml` (R6, R20): is the room solvable, stable and actually solved?

Two phases (R20). The static stages need no simulation:
values:    every part value is in its schema range (R3);
channels:  every channel a part reads is written by a part, no channel has two
           writers, no loop runs through stateless parts only, links name parts (R1);
demo:      the `demo:` sub-goals name known commands and fitting parts;
           (these three fail while the room loads: the result then holds only
           the failing one, and the solver is skipped)
body:      every part's `requires_body` capabilities are on the body (R2b);
parts:     no part reports a static problem (`Part.check_static`: a ball too big
           for its goal net, a ball in a wall or out of reach, R38);
overlap:   at the nominal layout no solid part geometry sits in a wall or in
           another part (`OVERLAP_DEPTH`; floor pads and doors in their wall gap
           do not count);
(a) order: a signal path to a goal exists: a switch/door order that reaches a
           goal (link graph on the room grid), and every coin is reachable (R36);
           in letter-map rooms (R37) also: parts, spawn and their random ranges lie on
           floor cells, and a failure names the steps / steep ramps (cells) in the way.
The dynamic stages simulate:
(b) stability: zero command for 10 s at the nominal layout: no NaN, no fall;
(c) solver:    the scripted privileged solver (with the room's `demo:`) succeeds
               on >= `need` of `seeds` random layouts (seeds 0..seeds-1); skipped
               when a static stage fails. The snapshots of the checkpoint flags it
               reached are written to `<snap_dir>/<room>/checkpoints/<flag>.pkl`
               (R20; from the first successful episode, else the first that reached each).
`static` and `dynamic` in the result sum up the two phases.
"""

from __future__ import annotations

import pickle
import time
from pathlib import Path
from typing import Any

import numpy as np

import mujoco

from .env import PuzzleEnv
from .logic import free_mask, solution_order
from .parts.base import BODY_PREFIX
from .parts.floor.coin import unreachable_coins
from .room import RoomError
from .solver import ScriptedSolver, run_episode

STABILITY_SECONDS = 10.0
LOAD_STAGES = ("values", "channels", "demo")
STATIC = (*LOAD_STAGES, "body", "parts", "overlap", "order")
DYNAMIC = ("stability", "solver")
STAGES = STATIC + DYNAMIC
OVERLAP_DEPTH = 0.005    # [m] penetration that counts as an overlap
SNAP_DIR = Path("runs/check")


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


def check_overlap(env: PuzzleEnv) -> dict[str, Any]:
    """Static: solid part geoms in a wall or in another part at the nominal layout.

    Only colliding geoms count (floor pads, flags and goal areas have none),
    floor parts are skipped, and gates (doors) are not tested against walls:
    they sit in a wall gap and slide into the wall."""
    m = env.model
    env.reset(options={"layout": env.sample_layout(None)})
    d = env.data
    owner: dict[int, str] = {}
    walls: list[int] = []
    for g in range(m.ngeom):
        if not (m.geom_contype[g] or m.geom_conaffinity[g]):
            continue
        b = int(m.geom_bodyid[g])
        bname = m.body(b).name
        if bname.startswith(BODY_PREFIX):
            pname = bname[len(BODY_PREFIX):].split("/")[0]
            if pname in env.parts and not getattr(env.parts[pname], "floor_part", False):
                owner[g] = pname
        elif b == 0 and m.geom(g).name.startswith("wall"):
            walls.append(g)
    hits: dict[tuple[str, str], float] = {}

    def probe(g1: int, g2: int, key: tuple[str, str]) -> None:
        dist = mujoco.mj_geomDistance(m, d, g1, g2, 0.05, None)
        if dist < -OVERLAP_DEPTH:
            hits[key] = min(hits.get(key, 0.0), dist)

    geoms = sorted(owner)
    for i, g1 in enumerate(geoms):
        a = owner[g1]
        for g2 in geoms[i + 1:]:
            if owner[g2] != a:
                probe(g1, g2, tuple(sorted((a, owner[g2]))))
        if env.parts[a].role != "gate":
            for w in walls:
                probe(g1, w, (a, ""))
    why = [f"parts {a!r} and {b!r} overlap by {-v * 100:.1f} cm" if b else
           f"part {a!r} overlaps a wall by {-v * 100:.1f} cm" for (a, b), v in hits.items()]
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


def check_solver(env: PuzzleEnv, seeds: int = 20, need: int = 18,
                 snap_dir: str | Path | None = None) -> dict[str, Any]:
    solver = ScriptedSolver(env)
    episodes = [run_episode(env, s, solver) for s in range(seeds)]
    wins = sum(e["success"] for e in episodes)
    keys = ("seed", "fallen", "nan", "target", "seconds") + (("demo_step",) if solver.demo else ())
    failures = [{k: e[k] for k in keys} for e in episodes if not e["success"]]
    times = [e["seconds"] for e in episodes if e["success"]]
    res = {"ok": wins >= need, "successes": wins, "seeds": seeds, "need": need,
           "mean_success_seconds": round(float(np.mean(times)), 2) if times else None,
           "failures": failures}
    snaps: dict = {}
    for e in sorted(episodes, key=lambda e: not e["success"]):      # successful first
        for flag, snap in e["snaps"].items():
            snaps.setdefault(flag, snap)
    if snaps and snap_dir is not None:
        out = Path(snap_dir) / env.room.name / "checkpoints"
        out.mkdir(parents=True, exist_ok=True)
        res["checkpoints"] = {}
        for flag, snap in snaps.items():
            path = out / f"{flag}.pkl"
            path.write_bytes(pickle.dumps(snap))
            res["checkpoints"][flag] = str(path)
    return res


def check_static(env: PuzzleEnv) -> dict[str, Any]:
    """The static stages after the room loaded (values, channels and demo passed)."""
    out: dict[str, Any] = {k: {"ok": True} for k in LOAD_STAGES}
    out["body"] = check_body(env)
    out["parts"] = check_parts(env)
    out["overlap"] = check_overlap(env)
    out["order"] = check_order(env)
    return out


def _summary(result: dict[str, Any]) -> None:
    for phase, names in (("static", STATIC), ("dynamic", DYNAMIC)):
        failed = [k for k in names if k in result and not result[k]["ok"]]
        if any(k in result for k in names):
            result[phase] = {"ok": not failed, "failed": failed}
        else:
            result[phase] = {"ok": None, "failed": [], "skipped": True}
    result["ok"] = all(result[k]["ok"] for k in STAGES if k in result)
    result["failed"] = [k for k in STAGES if k in result and not result[k]["ok"]]


def check(room: str | Path, task: str | Path | None = None, seeds: int = 20,
          need: int = 18, static_only: bool = False,
          snap_dir: str | Path | None = SNAP_DIR) -> dict[str, Any]:
    """All stages; `static_only` stops after the static phase (no simulation)."""
    t0 = time.perf_counter()
    skipped = {"ok": False, "successes": 0, "seeds": seeds, "need": need}
    try:
        env = PuzzleEnv(task=task, room=room)
    except RoomError as exc:
        if exc.stage not in LOAD_STAGES:
            raise
        result = {"room": Path(room).stem, exc.stage: {"ok": False, "reason": str(exc)},
                  "error": str(exc)}
        if not static_only:
            result["solver"] = {**skipped, "skipped": f"static stage {exc.stage} failed"}
        _summary(result)
        result["wall_seconds"] = round(time.perf_counter() - t0, 1)
        return result
    result: dict[str, Any] = {"room": env.room.name, "episode_seconds": env.max_steps / env.hz}
    result.update(check_static(env))
    if not static_only:
        result["stability"] = check_stability(env)
        failed = [k for k in STATIC if not result[k]["ok"]]
        if not failed:
            result["solver"] = check_solver(env, seeds, need, snap_dir)
        else:
            why = {"order": "no solution order (a)", "body": "missing body capabilities",
                   "parts": "part static check"}.get(failed[0], f"static stage {failed[0]} failed")
            result["solver"] = {**skipped, "skipped": why}
    _summary(result)
    result["wall_seconds"] = round(time.perf_counter() - t0, 1)
    env.close()
    return result
