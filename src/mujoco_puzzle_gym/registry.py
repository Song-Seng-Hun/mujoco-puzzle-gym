"""Registries for puzzle parts, observation terms and reward terms (R17).

Built-in parts and terms register through the same functions users call, so a
new part or term needs no edit to library code:

    from mujoco_puzzle_gym.registry import register_part, register_reward

    @register_reward("upright")
    def upright(env):            # returns the unweighted term value
        return -float(env.body_state.gravity[2])

Weights and which terms are active come from task.yaml (`obs:`, `reward:`).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Callable

import numpy as np

if TYPE_CHECKING:
    from .env import PuzzleEnv
    from .parts.base import Part

ObsFn = Callable[["PuzzleEnv"], np.ndarray]
RewardFn = Callable[["PuzzleEnv"], float]

PARTS: dict[str, type["Part"]] = {}
OBS: dict[str, ObsFn] = {}
REWARDS: dict[str, RewardFn] = {}


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


def load_builtins() -> None:
    """Import the modules whose import registers the built-in parts and terms."""
    from . import obs, parts, rewards  # noqa: F401


def get_part(name: str) -> type["Part"]:
    load_builtins()
    try:
        return PARTS[name]
    except KeyError:
        raise KeyError(f"unknown part type {name!r}; registered: {sorted(PARTS)}") from None
