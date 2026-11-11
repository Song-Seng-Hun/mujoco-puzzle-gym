"""Built-in observation terms (R7). Each maps the env to a fixed-length 1-D array.

task.yaml `obs:` items are term names or mappings with parameters (R39):

    obs: [gravity, {item: coins, k: 5}, {item: rangefinder, rays: 9}]

A function term receives the parameters as keyword arguments (`coins(env, k=5)`);
a class term (`sensors.py`) receives them when it is built, before the model is
compiled. Every item takes `noise`: a standard deviation of Gaussian noise on the
policy's copy of the item (R35 `body` layer, scaled by task `randomize.body`,
default 1); sensor classes interpret it in their own units.
"""

from __future__ import annotations

import inspect
from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np

from .registry import OBS, register_obs


def _to_body_frame(env, xy) -> np.ndarray:
    s = env.body_state
    d = np.asarray(xy) - s.pos[:2]
    c, sn = np.cos(s.yaw), np.sin(s.yaw)
    return np.array([c * d[0] + sn * d[1], -sn * d[0] + c * d[1]])


@register_obs("gravity", replace=True)
def gravity(env):
    """Projected gravity in the body frame (3)."""
    return env.body_state.gravity


@register_obs("ang_vel", replace=True)
def ang_vel(env):
    """Body angular velocity (3)."""
    return env.body_state.ang_vel


@register_obs("lin_vel", replace=True)
def lin_vel(env):
    """Body linear velocity in the heading frame (3)."""
    return env.body_state.lin_vel


@register_obs("part_pos", replace=True)
def part_pos(env):
    """Privileged: every part's position relative to the body, heading frame (2 per part)."""
    return np.concatenate([_to_body_frame(env, p.xy) for p in env.parts.values()])


@register_obs("part_state", replace=True)
def part_state(env):
    """Privileged: every part's state in [0, 1] (switch on, door open fraction, goal reached)."""
    return np.array([p.state() for p in env.parts.values()])


@register_obs("target_pos", replace=True)
def target_pos(env):
    """Privileged: the current target (next part in the solution order), heading frame (2)."""
    t = env.current_target()
    return _to_body_frame(env, t.xy) if t is not None else np.zeros(2)


@register_obs("counters", replace=True)
def counters(env):
    """Every counter's count over its target (R8b), room order (1 per counter)."""
    return np.array([p.ratio() for p in env.parts.values() if hasattr(p, "ratio")], dtype=float)


#: Remaining coins the `coins` term lists (nearest first).
COIN_OBS_K = 3


def _count(name: str, k) -> int:
    if isinstance(k, bool) or not isinstance(k, int) or k < 1:
        raise ValueError(f"{name}: k must be a whole number >= 1, got {k!r}")
    return k


@register_obs("coins", replace=True)
def coins(env, k: int = COIN_OBS_K):
    """Coins (R36): per coin group (room order) the fraction picked, then the
    `k` (default `COIN_OBS_K`) nearest coins not picked yet as (x, y, 1) in the
    heading frame; empty slots are (0, 0, 0)."""
    k = _count("coins", k)
    groups = [p for p in env.parts.values() if hasattr(p, "picked_ratio")]
    left = [c for g in groups for c in g.remaining()]
    pos = env.body_state.pos[:2]
    left.sort(key=lambda c: float(np.hypot(*(c.xy - pos))))
    slots = np.zeros((k, 3))
    for i, c in enumerate(left[:k]):
        slots[i] = [*_to_body_frame(env, c.xy), 1.0]
    return np.concatenate([[g.picked_ratio() for g in groups], slots.ravel()])


BALLS_K = 3   # nearest balls in the `balls` term


@register_obs("balls", replace=True)
def balls(env, k: int = BALLS_K):
    """Nearest `k` (default `BALLS_K`) balls (R38): position and velocity relative
    to the body, heading frame, plus a present flag; empty slots are zeros (5 per slot)."""
    k = _count("balls", k)
    s = env.body_state
    c, sn = np.cos(s.yaw), np.sin(s.yaw)
    rows = []
    for p in env.parts.values():
        if getattr(p, "object_kind", None) != "ball":
            continue
        pos = p.pos(env.data)[:2]
        v = p.vel(env.data)[:2]
        v = np.array([c * v[0] + sn * v[1], -sn * v[0] + c * v[1]]) - s.lin_vel[:2]
        rows.append((float(np.linalg.norm(pos - s.pos[:2])),
                     np.concatenate([_to_body_frame(env, pos), v, [1.0]])))
    rows.sort(key=lambda r: r[0])
    out = np.zeros((k, 5))
    for i, (_, row) in enumerate(rows[:k]):
        out[i] = row
    return out.ravel()


# ---- obs items with parameters (R39) ---------------------------------------------
from . import sensors  # noqa: E402,F401  registers `rangefinder` and `camera`
from . import gaze  # noqa: E402,F401  registers `markers_seen` (R27)

#: Items a real robot has (proprioception and sensors). A task whose `obs:` uses
#: only these is "sensors only" (`obs_kind`, R39); plugin terms count as privileged.
SENSOR_TERMS = frozenset({"gravity", "ang_vel", "lin_vel", "rangefinder", "camera", "markers_seen"})


class ObsItemError(ValueError):
    pass


def parse_items(items: Any, key: str = "obs") -> list[tuple[str, dict[str, Any]]]:
    """`[name | {item: name, **params}]` -> [(name, params)]."""
    if not isinstance(items, list):
        raise ObsItemError(f"`{key}` must be a list of term names or {{item: name, ...}} mappings")
    out = []
    for it in items:
        if isinstance(it, str):
            out.append((it, {}))
        elif isinstance(it, dict) and isinstance(it.get("item"), str):
            out.append((it["item"], {k: v for k, v in it.items() if k != "item"}))
        else:
            raise ObsItemError(f"`{key}` item {it!r}: use a term name or {{item: name, ...}}")
    return out


def check_items(items: Any, key: str = "obs") -> list[tuple[str, dict[str, Any]]]:
    """Parse and validate names and parameters (without an env)."""
    parsed = parse_items(items, key)
    unknown = [n for n, _ in parsed if n not in OBS]
    if unknown:
        raise ObsItemError(f"unknown {key} terms {unknown}; registered: {sorted(OBS)}")
    for name, params in parsed:
        entry = OBS[name]
        try:
            if isinstance(entry, type):
                entry(None, None, **params)
            else:
                rest = dict(params)
                if "noise" in rest:
                    sensors._noise(rest.pop("noise"))
                inspect.signature(entry).bind(None, **rest)
        except TypeError as exc:
            raise ObsItemError(f"`{key}` item {name!r}: bad parameters {sorted(params)}: {exc}") from None
        except ValueError as exc:
            raise ObsItemError(f"`{key}` item {name!r}: {exc}") from None
    if sum(n == "camera" for n, _ in parsed) > 1:
        raise ObsItemError(f"`{key}` may hold at most one camera item")
    return parsed


@dataclass
class ObsItem:
    name: str
    fn: Callable[[Any], np.ndarray]
    noise: float = 0.0          # generic additive std (function terms)
    image: bool = False
    shape: tuple[int, ...] = field(default_factory=tuple)


def make_items(env, spec, items: Any, key: str = "obs") -> list[ObsItem]:
    """Build the env's items; class terms add what they need to `spec` (uncompiled)."""
    out = []
    for i, (name, params) in enumerate(check_items(items, key)):
        entry = OBS[name]
        if isinstance(entry, type):
            env.obs_prefix = f"{key}{i}_"
            obj = entry(env, spec, **params)
            out.append(ObsItem(name, obj, image=bool(getattr(obj, "image", False)),
                               shape=tuple(getattr(obj, "shape", ()))))
        else:
            rest = dict(params)
            noise = float(rest.pop("noise", 0.0))
            fn = (lambda e, f=entry, kw=rest: f(e, **kw)) if rest else entry
            out.append(ObsItem(name, fn, noise=noise))
    return out


def obs_kind(task: dict[str, Any]) -> str:
    """`sensors_only` when `obs:` has no privileged item, else `privileged` (R39)."""
    names = [n for n, _ in parse_items(task["obs"])]
    return "sensors_only" if set(names) <= SENSOR_TERMS else "privileged"
