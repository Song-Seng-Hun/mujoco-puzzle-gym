"""`mpg` command line. Every command supports --json and exits non-zero on failure (R12)."""

from __future__ import annotations

import argparse
import json
import math
import sys
from typing import Any, Callable

from . import __version__

Handler = Callable[[argparse.Namespace], dict[str, Any]]


class CommandError(Exception):
    """A failure with a reason the user (or an agent) can act on."""


def emit(result: dict[str, Any], as_json: bool) -> int:
    ok = bool(result.get("ok", True))
    result = {"ok": ok, **{k: v for k, v in result.items() if k != "ok"}}
    if as_json:
        print(json.dumps(result, ensure_ascii=False, default=str))
    else:
        for key, value in result.items():
            print(f"{key}: {value}")
    return 0 if ok else 1


def _version(_: argparse.Namespace) -> dict[str, Any]:
    return {"version": __version__}


def _fetch_assets(args: argparse.Namespace) -> dict[str, Any]:
    from . import assets

    try:
        return assets.fetch(root=args.cache_dir)
    except (assets.AssetError, OSError) as exc:
        raise CommandError(str(exc)) from exc


def _configure_fetch(p: argparse.ArgumentParser) -> None:
    p.add_argument("--cache-dir", default=None,
                   help="cache root (default: $MPG_CACHE_DIR or ~/.cache/mujoco-puzzle-gym)")


def _bench(args: argparse.Namespace) -> dict[str, Any]:
    from . import bench

    try:
        procs = [int(x) for x in args.procs.split(",") if x.strip()] if args.procs else []
    except ValueError as exc:
        raise CommandError(f"--procs must be comma-separated integers: {args.procs!r}") from exc
    try:
        return bench.run(seconds=args.seconds, procs=procs,
                         room=None if args.no_env else (args.room or bench.DEFAULT_ROOM))
    except (FileNotFoundError, ValueError) as exc:
        raise CommandError(str(exc)) from exc


def _configure_bench(p: argparse.ArgumentParser) -> None:
    p.add_argument("--seconds", type=float, default=5.0, help="wall time per measurement")
    p.add_argument("--procs", default="4,8,16",
                   help="comma-separated process counts for the parallel runs ('' = none)")
    p.add_argument("--room", default=None,
                   help="room for the PuzzleEnv measurement (default: examples/switch_door.yaml)")
    p.add_argument("--no-env", action="store_true", help="walker only, skip the PuzzleEnv runs")


def _build(args: argparse.Namespace) -> dict[str, Any]:
    from .room import RoomError, build

    try:
        return build(args.room, out_dir=args.out)
    except (RoomError, FileNotFoundError) as exc:
        raise CommandError(str(exc)) from exc


def _configure_build(p: argparse.ArgumentParser) -> None:
    p.add_argument("room", help="room.yaml")
    p.add_argument("--out", default=None, help="output directory (default: build/<room name>/)")


def _check(args: argparse.Namespace) -> dict[str, Any]:
    from .check import check
    from .room import RoomError
    from .task import TaskError

    need = args.need if args.need is not None else math.ceil(0.9 * args.seeds)
    try:
        return check(args.room, task=args.task, seeds=args.seeds, need=need)
    except (RoomError, TaskError, FileNotFoundError) as exc:
        raise CommandError(str(exc)) from exc


def _configure_check(p: argparse.ArgumentParser) -> None:
    p.add_argument("room", help="room.yaml")
    p.add_argument("--task", default=None, help="task.yaml for env settings (default: built-in)")
    p.add_argument("--seeds", type=int, default=20, help="random layouts for the scripted solver")
    p.add_argument("--need", type=int, default=None,
                   help="successes required (default: 90%% of --seeds, 18 of 20)")


def _train(args: argparse.Namespace) -> dict[str, Any]:
    from .room import RoomError
    from .task import TaskError
    from .train import train

    try:
        return train(args.task, minutes=args.minutes, algo=args.algo, n_envs=args.n_envs,
                     runs_dir=args.runs_dir, run_id=args.id, max_steps=args.max_steps)
    except (TaskError, RoomError, KeyError, FileNotFoundError) as exc:
        raise CommandError(str(exc.args[0]) if exc.args else str(exc)) from exc


def _configure_train(p: argparse.ArgumentParser) -> None:
    p.add_argument("task", help="task.yaml")
    p.add_argument("--minutes", type=float, required=True, help="wall-clock training budget")
    p.add_argument("--algo", default=None,
                   help="ppo | sac | a2c | an entry point in group mujoco_puzzle_gym.algos "
                        "(default: task `algo:`)")
    p.add_argument("--n-envs", type=int, default=None, help="parallel envs (default: task `n_envs:`)")
    p.add_argument("--runs-dir", default="runs", help="parent of the run directory")
    p.add_argument("--id", default=None, help="run id (default: <time>-<task>-<algo>)")
    p.add_argument("--max-steps", type=int, default=None, help="also stop after this many env steps")


def _run_errors():
    from .room import RoomError
    from .task import TaskError

    return (TaskError, RoomError, KeyError, FileNotFoundError, ValueError)


def _eval(args: argparse.Namespace) -> dict[str, Any]:
    from .evaluate import evaluate

    try:
        return evaluate(args.run, episodes=args.episodes, policy=args.policy,
                        deterministic=not args.stochastic, workers=args.workers)
    except _run_errors() as exc:
        raise CommandError(str(exc)) from exc


def _configure_eval(p: argparse.ArgumentParser) -> None:
    p.add_argument("run", help="runs/<id>")
    p.add_argument("--episodes", type=int, default=100, help="held-out layouts (eval seeds)")
    p.add_argument("--policy", choices=["trained", "random"], default="trained",
                   help="'random' = uniform random baseline on the same seeds")
    p.add_argument("--stochastic", action="store_true", help="sample actions instead of the mean")
    p.add_argument("--workers", type=int, default=12, help="parallel processes")


def _diagnose(args: argparse.Namespace) -> dict[str, Any]:
    from .diagnose import diagnose

    try:
        return diagnose(args.run, episodes=args.episodes, video=not args.no_video,
                        workers=args.workers)
    except _run_errors() as exc:
        raise CommandError(str(exc)) from exc


def _configure_diagnose(p: argparse.ArgumentParser) -> None:
    p.add_argument("run", help="runs/<id>")
    p.add_argument("--episodes", type=int, default=20, help="rollouts of the saved policy")
    p.add_argument("--no-video", action="store_true", help="skip the failure MP4")
    p.add_argument("--workers", type=int, default=12, help="parallel processes")


def _render(args: argparse.Namespace) -> dict[str, Any]:
    from .evaluate import render

    try:
        return render(args.run, out=args.out, seed=args.seed, tries=args.tries)
    except _run_errors() as exc:
        raise CommandError(str(exc)) from exc


def _configure_render(p: argparse.ArgumentParser) -> None:
    p.add_argument("run", help="runs/<id>")
    p.add_argument("--out", default=None, help="MP4 path (default: runs/<id>/success.mp4)")
    p.add_argument("--seed", type=int, default=None,
                   help="render this env seed whatever happens (default: first eval-seed success)")
    p.add_argument("--tries", type=int, default=20, help="eval seeds to try for a success")


COMMANDS: dict[str, tuple[str, Handler, Callable[[argparse.ArgumentParser], None]]] = {
    "version": ("Print the package version.", _version, lambda p: None),
    "fetch-assets": ("Download and SHA256-verify the pinned robot and policy files.",
                     _fetch_assets, _configure_fetch),
    "build": ("Build body + room MJCF and a top-view PNG from room.yaml.", _build, _configure_build),
    "check": ("Check a room: solution order, physics stability, scripted solver success.",
              _check, _configure_check),
    "bench": ("Measure walker and PuzzleEnv speed: steps/s, single and parallel.",
              _bench, _configure_bench),
    "train": ("Train a policy on task.yaml for --minutes; writes runs/<id>/.", _train, _configure_train),
    "diagnose": ("Classify why a run fails (env bug, body, exploration, reward hacking) + failure MP4.",
                 _diagnose, _configure_diagnose),
    "eval": ("Success rate and mean time on held-out layouts.", _eval, _configure_eval),
    "render": ("Offscreen MP4 of a success episode.", _render, _configure_render),
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="mpg", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    for name, (help_text, _, configure) in COMMANDS.items():
        p = sub.add_parser(name, help=help_text)
        p.add_argument("--json", action="store_true", help="machine-readable output")
        configure(p)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    _, handler, _ = COMMANDS[args.command]
    try:
        result = handler(args)
    except CommandError as exc:
        result = {"ok": False, "error": str(exc)}
    return emit(result, args.json)


if __name__ == "__main__":
    sys.exit(main())
