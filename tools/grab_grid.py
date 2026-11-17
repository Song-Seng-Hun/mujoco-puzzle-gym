#!/usr/bin/env python
"""Sweep the object a room's solution picks up over a grid, and print what each spot does.

Why this exists. `mpg check` answers "does this room solve where its parts are", and for a
room that draws nothing that is one episode however many seeds it runs (which is what
`check_solver`'s `distinct_outcomes` now says out loud). It was read here as a success rate
for `puzzles/tabletop/pick_and_place`, and swept afterwards by hand the solution held at 3 of
25 block positions over 80x80 mm - the grab failing on one side of the working spot and the
place on the other. That sweep lived in a scratch directory, so the number it produced could
not be reproduced or compared against. This is that sweep, kept.

Kept, and it has already moved twice: 3 of 25 when the room shipped, 7 of 25 after the M6
carry work (A5, A9), and **25 of 25** since the box place stopped pressing the wall
(R108, `Carrier._lower_in_act`). Widen it with `--span 100 --step 50` once it saturates:
at 200x200 mm the same room is 19 of 25, and five of the six failures are `grab`, which is
where the next piece of work is.

It is a measuring tool, not a check: it fails nothing, it is not wired into `mpg check`, and
a low score is a fact about the body's reach and the scripted solution, not a broken room.
Use it as the objective when changing either - a change that raises the shipped room's score
is an improvement, and one that leaves it while passing the room's own check has changed
nothing that matters.

**This grid is one room's objective, not the project's.** Measured: sweeping the arm's home
pose took it from 7 of 25 to **13 of 25** - nearly double, with the shipped position still
solving - and the same pose took `puzzles/tabletop/weigh_it_down` from 20/20 to **0/20**. On
the grid alone that change looks like the best result on this page. So read it beside the
other rooms the body solves, and ship nothing on this number by itself.

    python tools/grab_grid.py --room src/mujoco_puzzle_gym/presets/puzzles/tabletop/pick_and_place.yaml \
        --task examples/task_pick_and_place.yaml --part block

Each cell runs the scripted solution once (the rooms this is for draw nothing, so one seed is
the episode) and is printed as:

    ok      the solution finished
    grab    it stalled on a `grab` step
    place   it stalled on a `place` or `drop` step
    s<n>    it stalled on demo step n (any other step)
    x       the room was refused before the solver ran (out of reach, overlap, ...)
"""

from __future__ import annotations

import argparse
import sys
import tempfile
import warnings
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def _outcome(room: Path, task: Path | None, seeds: int) -> str:
    """One cell: run the room's own check stages, then its scripted solution."""
    from mujoco_puzzle_gym.check import check_solver, check_static
    from mujoco_puzzle_gym.env import PuzzleEnv

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        env = PuzzleEnv(task=str(task) if task else None, room=str(room))
    try:
        static = check_static(env)
        bad = [k for k, v in static.items() if isinstance(v, dict) and not v.get("ok", True)]
        if bad:
            return "x"
        res = check_solver(env, seeds=seeds, need=seeds)
        if res["ok"]:
            return "ok"
        steps = [f["demo_step"] for f in res["failures"] if f.get("demo_step") is not None]
        if not steps:
            return "s?"
        demo = list(getattr(env.room, "demo", []) or [])
        verb = demo[min(steps[0], len(demo) - 1)][0] if demo else ""
        if verb == "grab":
            return "grab"
        if verb in ("place", "drop"):
            return "place"
        return f"s{steps[0]}"
    finally:
        env.close()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--room", required=True, help="room.yaml to sweep")
    p.add_argument("--task", default=None, help="task.yaml the room is checked with")
    p.add_argument("--part", required=True, help="name of the object part to move")
    p.add_argument("--span", type=float, default=40.0, help="half-width of the sweep [mm]")
    p.add_argument("--step", type=float, default=20.0, help="grid spacing [mm]")
    p.add_argument("--seeds", type=int, default=1, help="episodes per cell")
    a = p.parse_args()

    raw = yaml.safe_load(Path(a.room).read_text())
    parts = raw.get("parts") or []
    try:
        part = next(q for q in parts if q.get("name") == a.part)
    except StopIteration:
        p.error(f"room {a.room} has no part named {a.part!r}")
    x0, y0 = (float(v) for v in part["pos"][:2])

    n = int(a.span // a.step)
    offs = [i * a.step for i in range(-n, n + 1)]
    out: dict[tuple[float, float], str] = {}
    with tempfile.TemporaryDirectory() as tmp:
        for dx in offs:
            for dy in offs:
                part["pos"] = [round(x0 + dx / 1000.0, 5), round(y0 + dy / 1000.0, 5)]
                room = Path(tmp) / f"g{dx:+05.0f}{dy:+05.0f}.yaml"
                room.write_text(yaml.safe_dump(raw, sort_keys=False, allow_unicode=True))
                out[(dx, dy)] = _outcome(room, Path(a.task) if a.task else None, a.seeds)
                print(".", end="", flush=True)
    print("\r", end="")

    print(f"{a.part} swept +-{a.span:g} mm in {a.step:g} mm steps, {a.seeds} episode(s) per cell")
    print("        " + "".join(f"{dy:+6.0f}" for dy in offs) + "   [dy, mm]")
    for dx in offs:
        print(f"dx{dx:+5.0f} " + "".join(f"{out[(dx, dy)]:>6s}" for dy in offs))
    good = sum(1 for v in out.values() if v == "ok")
    print(f"\n{good} of {len(out)} positions solve "
          f"(nominal position: {'ok' if out[(0.0, 0.0)] == 'ok' else out[(0.0, 0.0)]})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
