"""Where each room's scripted solution stops under actuator latency, per sub-goal.

R142: `docs/challenges.md` claimed for five sessions that five of the ten arm rooms fail
at 40 ms of latency "for one reason", and every attempt to fix it aimed at that one reason
(the jaw height held while travelling). Printing the stalled sub-goal took one run and
showed four reasons: three of the five never finish their `grab` at all, and only two of
them reach the `goto goal` the five fixes were written for. Nothing was measuring which
verb stopped, so a wrong single cause survived five rounds of work.

So this prints it. One line per room x latency x seed:

    .venv/bin/python tools/latency_stall.py --rooms dead_wedge,lever_arm --latency 0,40

`demo_step` is the sub-goal the solver was on when the episode ended, `at` that sub-goal
written out, `unfinished` the `ScriptedSolver._unfinished` reports (R78, R141) for sub-goals
that ended without their effect. A room that passes prints its time, so the control rooms
in the same run show the latency is not globally fatal.

The latency knob is `env.actuator_latency_steps` in the task file, in whole control steps
of 0.02 s; this takes milliseconds and converts, and refuses a figure that is not a
multiple of 20 ms rather than rounding one silently.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
EXAMPLES = ROOT / "examples"
#: The ten arm rooms, in the order `docs/training-log.md` reports them.
ARM_ROOMS = ("reach_target", "reach_the_marker", "pick_and_place", "weigh_it_down",
             "build_a_tower", "dead_wedge", "lever_arm", "conveyor_triage",
             "fetch_the_steel", "pick_by_hand")
STEP_MS = 20


def task_for(name: str, steps: int) -> dict:
    """The example task for `name` with the latency overridden, room path made absolute."""
    path = EXAMPLES / f"task_{name}.yaml"
    if not path.exists():
        raise SystemExit(f"no task file for {name!r}: {path}")
    raw = yaml.safe_load(path.read_text())
    raw["room"] = str((EXAMPLES / raw["room"]).resolve())
    raw["env"] = dict(raw["env"], actuator_latency_steps=steps)
    return raw


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--rooms", default=",".join(ARM_ROOMS),
                   help="comma-separated example task names (default: the ten arm rooms)")
    p.add_argument("--latency", default="0,40",
                   help=f"comma-separated latencies in ms, multiples of {STEP_MS}")
    p.add_argument("--seeds", type=int, default=3, help="seeds per room and latency")
    args = p.parse_args(argv)

    sys.path.insert(0, str(ROOT / "src"))
    from mujoco_puzzle_gym.env import PuzzleEnv
    from mujoco_puzzle_gym.solver import ScriptedSolver, run_episode

    lats = []
    for raw in args.latency.split(","):
        ms = int(raw)
        if ms % STEP_MS or ms < 0:
            raise SystemExit(f"--latency {ms}: not a non-negative multiple of {STEP_MS} ms")
        lats.append(ms)

    for name in args.rooms.split(","):
        for ms in lats:
            env = PuzzleEnv(task=task_for(name, ms // STEP_MS))
            solver = ScriptedSolver(env)
            demo = list(solver.demo) if solver.demo else []
            for seed in range(args.seeds):
                e = run_episode(env, seed, solver)
                step = e["demo_step"]
                at = demo[step] if step is not None and step < len(demo) else None
                print(f"{name:18s} {ms:3d}ms s{seed} "
                      f"{'ok ' if e['success'] else 'FAIL'} {e['seconds']:5.1f}s "
                      f"step={step}/{len(demo)} at={at} unfinished={e['demo_unfinished']}",
                      flush=True)
            del env
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
