"""Registries for puzzle parts, observation terms and reward terms (R17).

Built-in parts and terms register through the same functions users call, so a
new part or term needs no edit to library code:

    from mujoco_puzzle_gym.registry import register_part, register_reward

    @register_reward("upright")
    def upright(env):            # returns the unweighted term value
        return -float(env.body_state.gravity[2])

Weights and which terms are active come from task.yaml (`obs:`, `reward:`).

Python logic for gates (R7) goes in only as a registered function; room.yaml
names it (`{type: gate, logic: my_rule, in: [a, b]}`) and never holds code:

    @register_logic("majority")
    def majority(inputs, state, dt):   # inputs: channel -> value; state: dict kept per gate
        return float(sum(v > 0.5 for v in inputs.values()) * 2 > len(inputs))

Training algorithms (R8) are `algos.base.Trainer` subclasses, found by name in
this order: built-ins (ppo, sac, a2c), `register_algo` calls (e.g. from a
task.yaml `plugins:` module), then the Python entry point group
`mujoco_puzzle_gym.algos`:

    [project.entry-points."mujoco_puzzle_gym.algos"]
    my_algo = "my_package.trainer:MyTrainer"
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Callable

import numpy as np

if TYPE_CHECKING:
    from .algos.base import Trainer
    from .env import PuzzleEnv
    from .parts.base import Part

ObsFn = Callable[["PuzzleEnv"], np.ndarray]
RewardFn = Callable[["PuzzleEnv"], float]
LogicFn = Callable[[dict[str, float], dict[str, Any], float], float]

PARTS: dict[str, type["Part"]] = {}
OBS: dict[str, ObsFn] = {}
REWARDS: dict[str, RewardFn] = {}
ALGOS: dict[str, type["Trainer"]] = {}
LOGIC: dict[str, LogicFn] = {}
ALGO_ENTRY_POINTS = "mujoco_puzzle_gym.algos"


def _register(table: dict[str, Any], kind: str, name: str, obj: Any, replace: bool) -> Any:
    if not replace and name in table and table[name] is not obj:
        raise ValueError(f"{kind} {name!r} is already registered (pass replace=True to override)")
    table[name] = obj
    return obj


def register_part(name: str, cls: type["Part"] | None = None, *, replace: bool = False):
    """Register a Part subclass under the `type:` name used in room.yaml.

    Usable as `register_part("pad", PadClass)` or as a class decorator.
    """
    if cls is None:
        return lambda c: register_part(name, c, replace=replace)
    cls.type_name = name
    return _register(PARTS, "part", name, cls, replace)


def register_obs(name: str, fn: ObsFn | None = None, *, replace: bool = False):
    """Register an observation term: fn(env) -> 1-D float array of fixed length."""
    if fn is None:
        return lambda f: register_obs(name, f, replace=replace)
    return _register(OBS, "obs term", name, fn, replace)


def register_reward(name: str, fn: RewardFn | None = None, *, replace: bool = False):
    """Register a reward term: fn(env) -> float, multiplied by its task.yaml weight."""
    if fn is None:
        return lambda f: register_reward(name, f, replace=replace)
    return _register(REWARDS, "reward term", name, fn, replace)


def register_logic(name: str, fn: LogicFn | None = None, *, replace: bool = False):
    """Register a gate logic function fn(inputs, state, dt) -> float (R7)."""
    if fn is None:
        return lambda f: register_logic(name, f, replace=replace)
    return _register(LOGIC, "logic", name, fn, replace)


def get_logic(name: str) -> LogicFn:
    try:
        return LOGIC[name]
    except KeyError:
        raise KeyError(f"unknown logic {name!r}; registered: {sorted(LOGIC)}") from None


def load_builtins() -> None:
    """Import the modules whose import registers the built-in parts and terms."""
    from . import obs, parts, rewards  # noqa: F401


def get_part(name: str) -> type["Part"]:
    load_builtins()
    try:
        return PARTS[name]
    except KeyError:
        raise KeyError(f"unknown part type {name!r}; registered: {sorted(PARTS)}") from None


def register_algo(name: str, cls: type["Trainer"] | None = None, *, replace: bool = False):
    """Register a `Trainer` subclass under the name used by `--algo` / task `algo:`."""
    if cls is None:
        return lambda c: register_algo(name, c, replace=replace)
    cls.name = name
    return _register(ALGOS, "algo", name, cls, replace)


def get_algo(name: str) -> type["Trainer"]:
    """Built-in or registered algorithm, else the entry point of that name."""
    from importlib.metadata import entry_points

    from .algos import sb3  # noqa: F401  registers ppo, sac, a2c

    if name in ALGOS:
        return ALGOS[name]
    found = [ep for ep in entry_points(group=ALGO_ENTRY_POINTS) if ep.name == name]
    if not found:
        eps = sorted(ep.name for ep in entry_points(group=ALGO_ENTRY_POINTS))
        raise KeyError(f"unknown algo {name!r}; registered: {sorted(ALGOS)}, "
                       f"entry points ({ALGO_ENTRY_POINTS}): {eps}")
    cls = found[0].load()
    return register_algo(name, cls, replace=True)
