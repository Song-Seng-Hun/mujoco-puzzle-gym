"""task.yaml (R17): which room, env timing, obs terms, reward terms and weights,
termination, algorithm and network. Missing keys take DEFAULTS; the filled-in
task is saved as `task.resolved.yaml` next to a run.

    room: switch_door.yaml        # relative to the task file
    env: {hz: 10, episode_seconds: 40, command_scale: 1.0}   # scale > 1: unclipped
                                                            # commands (broken tests)
    plugins: [my_terms]           # modules imported first (here and in every worker)
    obs: [gravity, ang_vel, lin_vel, part_pos, part_state]   # items may take
                                  # parameters: {item: coins, k: 5} (R39, obs.py)
    critic_obs: [part_pos, part_state]   # optional: value function only (R39)
    randomize: {body: 1.0}        # optional: scales obs-item `noise` (R35 body layer)
    curriculum: {mode: none, advance_at: 0.8, replay: 0.2, window: 50, snapshots: null,
                 order: difficulty_first}
                                  # R21 segments over checkpoint flags (curriculum.py);
                                  # `snapshots` is relative to the task file
    difficulty: []                # R21 difficulty steps, e.g.
                                  # [{param: cannon_a.power, from: 1, to: 3, steps: 3}]
    reward: {progress: 10.0, stage: 2.0, success: 10.0, fall: -5.0}
    termination: {success: true, fall: true}
    algo: ppo
    algo_kwargs: {learning_rate: 0.0003}
    net: [128, 128]
    n_envs: 12
    seed: 0
"""

from __future__ import annotations

import copy
import importlib
from pathlib import Path
from typing import Any

import yaml

from . import curriculum
from .registry import REWARDS, load_builtins

RESOLVED_NAME = "task.resolved.yaml"
HZ_RANGE = (5.0, 25.0)

DEFAULTS: dict[str, Any] = {
    "room": None,
    "body": "default",
    "env": {"hz": 10.0, "episode_seconds": 40.0, "command_scale": 1.0},
    "plugins": [],
    "obs": ["gravity", "ang_vel", "lin_vel", "part_pos", "part_state"],
    "reward": {"progress": 10.0, "stage": 2.0, "success": 10.0, "fall": -5.0},
    "termination": {"success": True, "fall": True},
    "algo": "ppo",
    "algo_kwargs": {},
    "net": [128, 128],
    "n_envs": 12,
    "seed": 0,
    "curriculum": dict(curriculum.DEFAULTS),
    "difficulty": [],
}


class TaskError(ValueError):
    pass


def _merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict) and k != "reward":
            out[k] = _merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def load_task(task: str | Path | dict | None = None, room: str | Path | None = None) -> dict[str, Any]:
    """Read task.yaml (or a dict), fill defaults, resolve the room path, validate."""
    base_dir = Path.cwd()
    if isinstance(task, (str, Path)):
        path = Path(task)
        try:
            raw = yaml.safe_load(path.read_text()) or {}
        except (OSError, yaml.YAMLError) as exc:
            raise TaskError(f"cannot read task file {path}: {exc}") from exc
        base_dir = path.resolve().parent
    else:
        raw = dict(task or {})
    if not isinstance(raw, dict):
        raise TaskError("task file must be a mapping")
    out = _merge(DEFAULTS, raw)
    import_plugins(out["plugins"])
    if room is not None:
        out["room"] = str(Path(room).resolve())
    elif out["room"] is None:
        raise TaskError("task needs `room: path/to/room.yaml`")
    else:
        out["room"] = str((base_dir / out["room"]).resolve())
    cur = out.get("curriculum")
    if isinstance(cur, dict) and cur.get("snapshots"):
        cur["snapshots"] = str((base_dir / cur["snapshots"]).resolve())
    validate(out)
    return out


def import_plugins(modules: list[str]) -> None:
    """Import user modules that register parts, terms or algorithms (R17).

    Called by `load_task`, so it runs in every process that builds an env from
    the task, including training worker processes.
    """
    if not isinstance(modules, list) or not all(isinstance(m, str) for m in modules):
        raise TaskError("`plugins` must be a list of module names")
    for name in modules:
        try:
            importlib.import_module(name)
        except ImportError as exc:
            raise TaskError(f"cannot import plugin module {name!r}: {exc}") from exc


def validate(task: dict[str, Any]) -> None:
    load_builtins()
    hz = float(task["env"]["hz"])
    if not HZ_RANGE[0] <= hz <= HZ_RANGE[1]:
        raise TaskError(f"env.hz must be within {HZ_RANGE}, got {hz}")
    if float(task["env"]["episode_seconds"]) <= 0:
        raise TaskError("env.episode_seconds must be > 0")
    if float(task["env"]["command_scale"]) <= 0:
        raise TaskError("env.command_scale must be > 0")
    from .obs import ObsItemError, check_items

    try:
        check_items(task["obs"], "obs")
        if task.get("critic_obs") is not None:
            critic = check_items(task["critic_obs"], "critic_obs")
            if not critic:
                raise ObsItemError("`critic_obs` must not be empty (leave it out instead)")
            if any(n == "camera" for n, _ in critic):
                raise ObsItemError("`critic_obs` cannot hold a camera item")
    except ObsItemError as exc:
        raise TaskError(str(exc)) from None
    rnd = task.get("randomize")
    if rnd is not None:
        if not isinstance(rnd, dict):
            raise TaskError("`randomize` must be a mapping, e.g. {body: 1.0}")
        body = rnd.get("body", 1.0)
        if isinstance(body, bool) or not isinstance(body, (int, float)) or body < 0:
            raise TaskError(f"randomize.body must be a number >= 0, got {body!r}")
    try:
        curriculum.validate(task["curriculum"])
    except curriculum.CurriculumError as exc:
        raise TaskError(str(exc)) from None
    if not isinstance(task["reward"], dict) or not task["reward"]:
        raise TaskError("`reward` must map term names to weights")
    unknown = [r for r in task["reward"] if r not in REWARDS]
    if unknown:
        raise TaskError(f"unknown reward terms {unknown}; registered: {sorted(REWARDS)}")


def write_resolved(task: dict[str, Any], out_dir: str | Path) -> Path:
    """Save the filled-in task as `<out_dir>/task.resolved.yaml`."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    path = out / RESOLVED_NAME
    path.write_text(yaml.safe_dump(task, sort_keys=False, allow_unicode=True))
    return path
