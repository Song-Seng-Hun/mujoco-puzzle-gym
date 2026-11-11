"""`mpg` command line. Every command supports --json and exits non-zero on failure (R12)."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Callable

from . import __version__

Handler = Callable[[argparse.Namespace], dict[str, Any]]


class CommandError(Exception):
    """A failure with a reason the user (or an agent) can act on."""


TEXT = "__text__"     # a handler result printed as is without --json (`mpg parts --markdown`)


def emit(result: dict[str, Any], as_json: bool) -> int:
    if TEXT in result:
        text = result.pop(TEXT)
        if not as_json:
            print(text, end="")
            return 0
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
                         room=None if args.no_env else (args.room or bench.DEFAULT_ROOM),
                         camera=not args.no_camera)
    except (FileNotFoundError, ValueError) as exc:
        raise CommandError(str(exc)) from exc


def _configure_bench(p: argparse.ArgumentParser) -> None:
    p.add_argument("--seconds", type=float, default=5.0, help="wall time per measurement")
    p.add_argument("--procs", default="4,8,16",
                   help="comma-separated process counts for the parallel runs ('' = none)")
    p.add_argument("--room", default=None,
                   help="room for the PuzzleEnv measurement (default: examples/switch_door.yaml)")
    p.add_argument("--no-env", action="store_true", help="walker only, skip the PuzzleEnv runs")
    p.add_argument("--no-camera", action="store_true",
                   help="skip the PuzzleEnv runs with a 64x64 head camera (R39)")


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
    if args.community and args.task:
        raise CommandError("--community takes no --task: a task's `plugins:` import code; "
                           "shared rooms are checked with the built-in task")
    try:
        return check(args.room, task=args.task, seeds=args.seeds, need=need,
                     static_only=args.static, community=args.community)
    except (RoomError, TaskError, FileNotFoundError) as exc:
        raise CommandError(str(exc)) from exc


def _configure_check(p: argparse.ArgumentParser) -> None:
    p.add_argument("room", help="room.yaml")
    p.add_argument("--task", default=None, help="task.yaml for env settings (default: built-in)")
    p.add_argument("--seeds", type=int, default=20, help="random layouts for the scripted solver")
    p.add_argument("--need", type=int, default=None,
                   help="successes required (default: 90%% of --seeds, 18 of 20)")
    p.add_argument("--static", action="store_true",
                   help="static stages only (no simulation): values, channels, demo, body, "
                        "parts, overlap, order")
    p.add_argument("--community", action="store_true",
                   help="also check the limits for shared rooms (community/rooms/): built-in "
                        "parts only, stages inside the room's folder, size caps; no --task")


def _parts(args: argparse.Namespace) -> dict[str, Any]:
    from . import catalog
    from .task import TaskError, import_plugins

    try:
        import_plugins(args.plugin or [])
    except TaskError as exc:
        raise CommandError(str(exc)) from exc
    if args.markdown:
        text = catalog.markdown()
        return {"markdown": text, TEXT: text}
    return catalog.catalog()


def _configure_parts(p: argparse.ArgumentParser) -> None:
    p.add_argument("--markdown", action="store_true",
                   help="value tables of the built-in parts and the preset list "
                        "(the table part of docs/parts-guide.md)")
    p.add_argument("--plugin", action="append", default=None,
                   help="also import this module first (parts it registers are listed)")


def _train(args: argparse.Namespace) -> dict[str, Any]:
    from .room import RoomError
    from .task import TaskError
    from .train import train

    try:
        return train(args.task, minutes=args.minutes, algo=args.algo, n_envs=args.n_envs,
                     runs_dir=args.runs_dir, run_id=args.id, max_steps=args.max_steps,
                     curriculum=args.curriculum, advance_at=args.advance_at, replay=args.replay)
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
    p.add_argument("--curriculum", choices=["forward", "reverse", "none"], default=None,
                   help="train over checkpoint segments (R21; needs `mpg check` snapshots; "
                        "default: task `curriculum.mode`, none)")
    p.add_argument("--advance-at", type=float, default=None,
                   help="segment success rate that opens the next segment (default 0.8)")
    p.add_argument("--replay", type=float, default=None,
                   help="share of episodes started at earlier open segments (default 0.2)")
    p.add_argument("--trust", action="store_true",
                   help="open run folders and snapshots outside ./runs/ (pickle can run code)")


def _run_errors():
    from .room import RoomError
    from .task import TaskError

    return (TaskError, RoomError, KeyError, FileNotFoundError, ValueError)


def _eval(args: argparse.Namespace) -> dict[str, Any]:
    from .evaluate import evaluate, evaluate_onnx

    if args.onnx:
        if args.run or not args.task:
            raise CommandError("--onnx takes --task task.yaml and no run folder")
        if args.full or args.segments or args.policy != "trained" or args.stochastic:
            raise CommandError("--onnx runs the deterministic policy from the start; "
                               "--full, --segments, --policy and --stochastic are for run folders")
        try:
            return evaluate_onnx(args.onnx, args.task, episodes=args.episodes, workers=args.workers)
        except _run_errors() as exc:
            raise CommandError(str(exc)) from exc
    if not args.run:
        raise CommandError("give a run folder (runs/<id>) or --onnx policy.onnx --task task.yaml")
    try:
        return evaluate(args.run, episodes=args.episodes, policy=args.policy,
                        deterministic=not args.stochastic, workers=args.workers,
                        full=args.full, segments=args.segments)
    except _run_errors() as exc:
        raise CommandError(str(exc)) from exc


def _configure_eval(p: argparse.ArgumentParser) -> None:
    p.add_argument("run", nargs="?", default=None, help="runs/<id> (or use --onnx)")
    p.add_argument("--onnx", default=None,
                   help="evaluate an exported policy.onnx (+ policy.json next to it) instead of "
                        "a run folder; needs --task; no pickle is read")
    p.add_argument("--task", default=None, help="task.yaml for --onnx")
    p.add_argument("--episodes", type=int, default=100, help="held-out layouts (eval seeds)")
    p.add_argument("--policy", choices=["trained", "random"], default="trained",
                   help="'random' = uniform random baseline on the same seeds")
    p.add_argument("--stochastic", action="store_true", help="sample actions instead of the mean")
    p.add_argument("--workers", type=int, default=12, help="parallel processes")
    p.add_argument("--full", action="store_true",
                   help="report full_success_rate: start to goal, no snapshots (R22)")
    p.add_argument("--segments", action="store_true",
                   help="also run every checkpoint segment from its snapshot: segment_success_rates; "
                        "with --full also chain_gap_rate (R22)")
    p.add_argument("--trust", action="store_true",
                   help="open run folders and snapshots outside ./runs/ (pickle can run code)")


def _export_onnx(args: argparse.Namespace) -> dict[str, Any]:
    from .onnx_policy import export

    try:
        return export(args.run, out_dir=args.out)
    except ImportError as exc:
        raise CommandError(f"{exc}; install the export extra: "
                           "uv pip install 'mujoco-puzzle-gym[export]'") from exc
    except _run_errors() as exc:
        raise CommandError(str(exc)) from exc


def _configure_export_onnx(p: argparse.ArgumentParser) -> None:
    p.add_argument("run", help="runs/<id> (ppo, a2c or sac; vector observations)")
    p.add_argument("--out", default=None, help="output folder (default: runs/<id>/onnx/)")
    p.add_argument("--trust", action="store_true",
                   help="open a run folder outside ./runs/ (pickle can run code)")


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
    p.add_argument("--trust", action="store_true",
                   help="open run folders and snapshots outside ./runs/ (pickle can run code)")


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
    p.add_argument("--trust", action="store_true",
                   help="open run folders and snapshots outside ./runs/ (pickle can run code)")


DEMO_RUN = Path(__file__).resolve().parents[2] / "examples" / "pretrained" / "switch_door"


def _demo(args: argparse.Namespace) -> dict[str, Any]:
    from .evaluate import render

    if not (DEMO_RUN / "summary.json").exists():
        raise CommandError(f"bundled demo policy missing at {DEMO_RUN} (run from a git checkout)")
    try:
        return render(DEMO_RUN, out=args.out, seed=args.seed)
    except _run_errors() as exc:
        raise CommandError(str(exc)) from exc


def _configure_demo(p: argparse.ArgumentParser) -> None:
    p.add_argument("--out", default="demo.mp4", help="MP4 path (default: ./demo.mp4)")
    p.add_argument("--seed", type=int, default=None, help="layout seed (default: first success)")


COMMANDS: dict[str, tuple[str, Handler, Callable[[argparse.ArgumentParser], None]]] = {
    "version": ("Print the package version.", _version, lambda p: None),
    "fetch-assets": ("Download and SHA256-verify the pinned robot and policy files.",
                     _fetch_assets, _configure_fetch),
    "build": ("Build body + room MJCF and a top-view PNG from room.yaml.", _build, _configure_build),
    "check": ("Check a room: solution order, physics stability, scripted solver success.",
              _check, _configure_check),
    "bench": ("Measure walker and PuzzleEnv speed: steps/s, single and parallel.",
              _bench, _configure_bench),
    "parts": ("List registered parts with their value schemas, and the presets (R3).",
              _parts, _configure_parts),
    "train": ("Train a policy on task.yaml for --minutes; writes runs/<id>/.", _train, _configure_train),
    "diagnose": ("Classify why a run fails (env bug, body, exploration, reward hacking) + failure MP4.",
                 _diagnose, _configure_diagnose),
    "eval": ("Success rate and mean time on held-out layouts.", _eval, _configure_eval),
    "export-onnx": ("Write policy.onnx + policy.json from a run, to share without pickle.",
                    _export_onnx, _configure_export_onnx),
    "render": ("Offscreen MP4 of a success episode.", _render, _configure_render),
    "demo": ("Render the bundled trained policy solving the example room to demo.mp4.", _demo, _configure_demo),
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
    if getattr(args, "trust", False):
        from .trust import set_trust
        set_trust(True)
    _, handler, _ = COMMANDS[args.command]
    try:
        result = handler(args)
    except CommandError as exc:
        result = {"ok": False, "error": str(exc)}
    return emit(result, args.json)


if __name__ == "__main__":
    sys.exit(main())
