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
           for its goal net, a ball in a wall or out of reach, R38); warnings
           (`Part.check_warnings`: an NPC as fast as the body, R29) do not fail it;
overlap:   at the nominal layout no solid part geometry sits in a wall or in
           another part (`OVERLAP_DEPTH`; floor pads and doors in their wall gap
           do not count);
actions:   the task actions cover what the plan needs (R19): `grab` when an
           object must be carried (a `grab` sub-goal in `demo:`, or an input in
           the solution order that only a grabbable object sets off, unless a
           `drag` sub-goal drags something onto it, R31) or a device
           held by its knob (a `pull` sub-goal, a pull cord or grab lever in the
           order, R28), `head` when a wall button
           the plan presses sits wholly below the level beak tip (the body's
           `beak_height`; a dispenser's wall plate counts when the plan takes
           from it: a `take` sub-goal, or its item for a receptacle in the
           order, R28); `crouch` for a `glide` sub-goal (R30); without them a policy
           can never do it. Checked after
           `order`, on its solution order;
circuit:   the break graph (R34, circuit.py): what breaking each breakable part
           reaches, against `demo:` (nothing broken that ends the episode, nothing
           broken that changes nothing) with warnings when the answer leans on a
           part that can be broken with no repair; checked after `order`, on its
           solution order;
resample:  a randomized room (R35 `randomize:`) is drawn once per seed (100) and
           the draws are counted: a seed that used up `max_resample` without a
           solvable draw fails the stage, and more than `RESAMPLE_WARN` of the
           draws thrown away is a warning. Skipped when the room declares no
           randomization or the task turns every layer off; it runs last because
           it leaves the room on a drawn layout. A room drawing its room (R35
           `rooms`) also reports how often each room came up and warns about one
           that never did and is not evaluation-only;
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
from .circuit import check_circuit, demo_breaks
from .logic import free_mask, solution_order
from .parts.base import BODY_PREFIX
from .parts.floor.coin import unreachable_coins
from .parts.object.grabbable import Grabbable, takes
from .parts.base import TRIGGERS
from . import community as community_mod
from .room import RoomError
from .solver import ScriptedSolver, run_episode

STABILITY_SECONDS = 10.0
RESAMPLE_SEEDS = 100     # R35: seeds the `resample` stage draws the room with
RESAMPLE_WARN = 0.3      # R35: thrown-away share that earns a warning
LOAD_STAGES = ("values", "channels", "demo")
STATIC = ("community", *LOAD_STAGES, "body", "parts", "overlap", "order", "circuit", "actions",
          "resample")  # community: --community only
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
    """Static: reasons parts give through `check_static` (R38); `warnings` from
    `check_warnings` (R29 NPC speed) do not fail the stage."""
    why = [r for p in env.parts.values() for r in p.check_static(env)]
    warn = [w for p in env.parts.values() for w in p.check_warnings(env)]
    return {"ok": not why, **({"reason": "; ".join(why)} if why else {}),
            **({"warnings": warn} if warn else {})}


def check_overlap(env: PuzzleEnv) -> dict[str, Any]:
    """Static: solid part geoms in a wall or in another part at the nominal layout.

    Only colliding geoms count (floor pads, flags and goal areas have none),
    floor parts are skipped, and a gate (door) panel is not tested against its
    own wall (`own_wall`): it sits in that wall's gap and slides into it. It is
    tested against every other wall."""
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
        for w in walls:
            if not (env.parts[a].role == "gate" and own_wall(m, d, g1, w)):
                probe(g1, w, (a, ""))
    why = [f"parts {a!r} and {b!r} overlap by {-v * 100:.1f} cm" if b else
           f"part {a!r} overlaps a wall by {-v * 100:.1f} cm" for (a, b), v in hits.items()]
    return {"ok": not why, **({"reason": "; ".join(why)} if why else {})}


def _box_line(m, d, g: int):
    """(centre xy, unit long-axis xy, half thickness) of a box geom seen from above."""
    rot = d.geom_xmat[g].reshape(3, 3)
    size = m.geom_size[g]
    k = 0 if size[0] >= size[1] else 1
    axis = rot[:2, k]
    n = float(np.linalg.norm(axis))
    return d.geom_xpos[g][:2], axis / (n if n > 1e-9 else 1.0), float(size[1 - k])


def own_wall(m, d, panel: int, wall: int) -> bool:
    """Whether box geom `wall` is the wall a gate's box geom `panel` closes a gap of:
    the two lie along one line (parallel within 5 deg, the panel centre on the
    wall's centre line). Non-box geoms have no own wall."""
    box = mujoco.mjtGeom.mjGEOM_BOX
    if m.geom_type[panel] != box or m.geom_type[wall] != box:
        return False
    pc, pa, pt = _box_line(m, d, panel)
    wc, wa, wt = _box_line(m, d, wall)
    if abs(float(pa @ wa)) < np.cos(np.radians(5.0)):
        return False
    off = pc - wc
    return abs(float(off @ np.array([-wa[1], wa[0]]))) <= wt + pt


def check_order(env: PuzzleEnv) -> dict[str, Any]:
    layout = env.sample_layout(None)
    for n, xy in layout["parts"].items():
        env.parts[n].place(env.model, xy)
    res = solution_order(env.grid, env.walls, env.parts, layout["spawn"][:2],
                         broken=demo_breaks(env.room))
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
    """Letter maps (R37): parts, spawn and everywhere a draw can put them (the `random:`
    shorthand and the R35 `randomize:` samplers) must lie on floor cells."""
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

    def drawn(what: str, owner: str, nominal, half=(0.0, 0.0)) -> None:
        """Every edge position the R35 samplers can draw must be on floor too."""
        for xy in env.randomizer.extremes_of(owner, nominal):
            probe(what, xy, (0.0, 0.0), half)

    probe("spawn", env.room.spawn_pos, env.room.spawn_random[:2])
    drawn("spawn", "spawn", env.room.spawn_pos)
    for n, p in env.parts.items():
        if not p.has_body or getattr(p, "floor_part", False):
            continue
        half = tuple(getattr(p, "half", (0.0, 0.0))) if np.ndim(getattr(p, "half", 0.0)) == 1 else (0.0, 0.0)
        if getattr(p, "mount", None) == "wall":      # a wall plate's size is [width, height] on the wall
            half = (0.0, 0.0)
        probe(f"part {n!r}", p.base_xy, p.random, half)
        drawn(f"part {n!r}", n, p.base_xy, half)
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


def check_resample(env: PuzzleEnv, seeds: int = RESAMPLE_SEEDS) -> dict[str, Any]:
    """Static: how much of a randomized room's drawing is thrown away (R35).

    Draws the room once per seed and counts the draws each one took. The stage
    warns above `RESAMPLE_WARN` thrown-away draws and fails when a seed used up
    the room's `max_resample` without a solvable draw, because that episode would
    start from an unsolvable room. A room that declares no randomization (or whose
    task turns every layer off) is skipped."""
    if not env.randomizer.entries:
        return {"ok": True, "skipped": "the room declares no `randomize:`"}
    if not env.randomizer.active():
        return {"ok": True, "skipped": "the task's `randomize:` turns every layer off"}
    draws, stuck = 0, []
    zones: dict[int, int] = {}
    for s in range(seeds):
        layout = env.sample_layout(np.random.default_rng(s))
        draws += int(layout.get("draws", 1))
        zones[int(layout.get("zone", 0) or 0)] = zones.get(int(layout.get("zone", 0) or 0), 0) + 1
        if layout.get("why"):
            stuck.append(f"seed {s}: {layout['why'][0]}")
    env.reset(options={"layout": env.sample_layout(None)})      # back to the nominal room
    ratio = (draws - seeds) / draws
    out: dict[str, Any] = {"ok": not stuck, "seeds": seeds, "draws": draws,
                           "ratio": round(ratio, 3), "max_resample": env.randomizer.max_resample}
    if stuck:
        out["reason"] = (f"{len(stuck)} of {seeds} seeds found no solvable room within "
                         f"max_resample {env.randomizer.max_resample}: " + "; ".join(stuck[:3]))
    warn = [f"{where}: the link cue is hidden, so the room needs memory or search; "
            f"a policy without one has a success-rate ceiling"
            for where in env.randomizer.hidden_cues()]
    rooms = env.randomizer.rooms
    if rooms is not None:              # R35 `rooms`: which room each seed played in
        names = rooms.zones
        out["rooms"] = {names[k]: n for k, n in sorted(zones.items())}
        missed = [names[k] for k in range(len(names))
                  if k not in zones and k not in rooms.holdout]
        if missed:
            warn.append(f"{seeds} seeds never drew {missed}; a room the training never sees "
                        f"is an evaluation-only room (`randomize.rooms.holdout`)")
    if ratio > RESAMPLE_WARN:
        warn.append(f"{ratio:.0%} of the draws are thrown away (over "
                    f"{RESAMPLE_WARN:.0%}): narrow the ranges or widen the room")
    if warn:
        out["warnings"] = warn
    return out


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


def check_actions(env: PuzzleEnv, order: list[str]) -> dict[str, Any]:
    """Static: task actions the plan needs (R19). `grab`: a `grab` / `pull` sub-goal, a part
    in `order` that a grabbable object sets off and the body cannot (unless a `drag`
    sub-goal drags something there, R31), or a held device
    (`grab_handle`, R28) in `order`; `head`: a wall button in
    `order` (or a `press` sub-goal) whose top edge is below the level beak tip; `crouch`:
    a `glide` sub-goal or a low bar the order stage lets a gliding body pass (R30)."""
    demo = list(getattr(env.room, "demo", []) or [])
    objs = [p for p in env.parts.values() if isinstance(p, Grabbable)]
    why = []
    if "grab" not in env.actions:
        need = [e[1] for e in demo if e[0] in ("grab", "pull")]
        need += [e[1] for e in demo if e[0] == "break" and e[2] == "peck"]      # the beak cuts it (R34)
        need += [e[1] for e in demo if e[0] == "break" and e[2] == "crush"]     # an object is carried
        need += [e[1] for e in demo if e[0] == "repair"
                 and env.parts[e[1]].repair_with is not None]                    # the fuse is carried
        need += [n for n in order if getattr(env.parts[n], "grab_handle", False)]     # held devices (R28)
        dragged = {e[2] for e in demo if e[0] == "drag"}           # a rope drags it there (R31)
        need += [n for n in order if any(takes(env.parts[n], o) for o in objs) and n not in dragged
                 and "body" not in TRIGGERS.get(getattr(env.parts[n], "trigger", None), ())]
        if need:
            why.append(f"{sorted(set(need))} need the beak grab but the task actions {env.actions} "
                       "have no `grab`")
    if "crouch" not in env.actions:
        bars = {e[1] for e in demo if e[0] == "glide"}
        bars |= {p.name for p in env.parts.values() if getattr(p, "glide_ok", False)}  # passable only gliding
        bars = sorted(bars)
        if bars:
            why.append(f"`glide` under {bars} needs the crouch but the task actions {env.actions} "
                       "have no `crouch`")
    level = env.body.capabilities.values.get("beak_height")
    if "head" not in env.actions and level is not None:
        pressed = set(order) | {e[1] for e in demo if e[0] in ("press", "take")}
        pressed |= {d.name for d in env.parts.values() if hasattr(d, "copies") and isinstance(d.copies[0], Grabbable)
                    and any(takes(env.parts[n], d.copies[0]) for n in order)}     # dispensers (R28)
        plate = {n: getattr(env.parts[n], "press_plate", None) or env.parts[n] for n in pressed}
        low = [n for n in sorted(pressed) if getattr(plate[n], "mount", None) == "wall"
               and plate[n].cfg["height"] + plate[n].half[1] < level]
        if low:
            why.append(f"wall buttons {low} are below the level beak tip ({level} m) but the task "
                       f"actions {env.actions} have no `head`")
        pecked = sorted(e[1] for e in demo if e[0] == "break" and e[2] == "peck"
                        and env.parts[e[1]].break_top() < level)                  # R34
        if pecked:
            why.append(f"pecking {pecked} needs the beak below the level tip ({level} m) but the "
                       f"task actions {env.actions} have no `head`")
    return {"ok": not why, "actions": list(env.actions), **({"reason": "; ".join(why)} if why else {})}


def check_static(env: PuzzleEnv) -> dict[str, Any]:
    """The static stages after the room loaded (values, channels and demo passed)."""
    out: dict[str, Any] = {k: {"ok": True} for k in LOAD_STAGES}
    out["body"] = check_body(env)
    out["parts"] = check_parts(env)
    out["overlap"] = check_overlap(env)
    out["order"] = check_order(env)
    out["circuit"] = check_circuit(env, out["order"]["order"])
    out["actions"] = check_actions(env, out["order"]["order"])
    out["resample"] = check_resample(env)      # last: it leaves the room on a drawn layout
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
          snap_dir: str | Path | None = SNAP_DIR, community: bool = False) -> dict[str, Any]:
    """All stages; `static_only` stops after the static phase (no simulation);
    `community` adds the shared-room limits (community.py) and stops on a failure."""
    t0 = time.perf_counter()
    skipped = {"ok": False, "successes": 0, "seeds": seeds, "need": need}
    if community:
        problems = community_mod.check_raw(room)
        if problems:
            result = {"room": Path(room).stem, "community": community_mod.stage(problems),
                      "error": problems[0]}
            if not static_only:
                result["solver"] = {**skipped, "skipped": "static stage community failed"}
            _summary(result)
            result["wall_seconds"] = round(time.perf_counter() - t0, 1)
            return result
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
    if community:
        result["community"] = community_mod.stage(community_mod.check_loaded(env.room))
    result.update(check_static(env))
    if not static_only:
        result["stability"] = check_stability(env)
        failed = [k for k in STATIC if k in result and not result[k]["ok"]]
        if not failed:
            result["solver"] = check_solver(env, seeds, need, snap_dir)
        else:
            why = {"order": "no solution order (a)", "body": "missing body capabilities",
                   "parts": "part static check", "resample": "a drawn room has no solution", "actions": "task actions miss what the plan needs"}.get(failed[0], f"static stage {failed[0]} failed")
            result["solver"] = {**skipped, "skipped": why}
    _summary(result)
    result["wall_seconds"] = round(time.perf_counter() - t0, 1)
    env.close()
    return result
