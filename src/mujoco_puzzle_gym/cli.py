"""`mpg` command line. Every command supports --json and exits non-zero on failure (R12)."""

from __future__ import annotations

import argparse
import json
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
        return bench.run(seconds=args.seconds, procs=procs)
    except FileNotFoundError as exc:
        raise CommandError(str(exc)) from exc


def _configure_bench(p: argparse.ArgumentParser) -> None:
    p.add_argument("--seconds", type=float, default=5.0, help="wall time per measurement")
    p.add_argument("--procs", default="4,8,16",
                   help="comma-separated process counts for the parallel runs ('' = none)")


COMMANDS: dict[str, tuple[str, Handler, Callable[[argparse.ArgumentParser], None]]] = {
    "version": ("Print the package version.", _version, lambda p: None),
    "fetch-assets": ("Download and SHA256-verify the pinned robot and policy files.",
                     _fetch_assets, _configure_fetch),
    "bench": ("Measure walker speed: physics and high-level steps/s, single and parallel.",
              _bench, _configure_bench),
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
