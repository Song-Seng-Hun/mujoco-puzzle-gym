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


def _obs_parts(env):
    """Parts the `part_*` terms list: all but the hidden `_look_` planning switches (gaze.py)."""
    from .gaze import LOOK_PREFIX

    return [p for n, p in env.parts.items() if not n.startswith(LOOK_PREFIX)]


@register_obs("addon", replace=True)
def addon(env):
    """The body's addon mode, one-hot over (none, roller) (R30, 2)."""
    from .bodies import ADDON_MODES

    out = np.zeros(len(ADDON_MODES))
    out[ADDON_MODES.index(env.body.addon)] = 1.0
    return out


@register_obs("tow", replace=True)
def tow(env):
    """Tow rope (R31): the body's hitch tied (0/1), that rope's tension (0..1 of its
    `max_tension`) and its other end relative to the body (its hitch when tied, else
    its knot; heading frame, 2); 4 values, zeros while no rope is tied to the body
    (first such rope, room order)."""
    for r in env.world.ropes:
        if not r.on_body:
            continue
        e = 1 - r.tied.index(r._body_h)
        far = r._end_point(env.data, e)
        t = float(np.clip(r.tension / float(r.cfg["max_tension"]), 0.0, 1.0))
        return np.array([1.0, t, *_to_body_frame(env, far[:2])])
    return np.zeros(4)


@register_obs("part_pos", replace=True)
def part_pos(env):
    """Privileged: every part's position relative to the body, heading frame (2 per part;
    zeros for a parked part, a dispenser's stored copy, R28)."""
    return np.concatenate([np.zeros(2) if p.parked else _to_body_frame(env, p.xy) for p in _obs_parts(env)])


@register_obs("part_state", replace=True)
def part_state(env):
    """Privileged: every part's state in [0, 1] (switch on, door open fraction, goal reached;
    0 for a parked part)."""
    return np.array([0.0 if p.parked else p.state() for p in _obs_parts(env)])


@register_obs("target_pos", replace=True)
def target_pos(env):
    """Privileged: the current target (next part in the solution order), heading frame (2)."""
    t = env.current_target()
    return _to_body_frame(env, t.target_xy()) if t is not None else np.zeros(2)


@register_obs("counters", replace=True)
def counters(env):
    """Every counter's count over its target (R8b), room order (1 per counter)."""
    return np.array([p.ratio() for p in env.parts.values() if hasattr(p, "ratio")], dtype=float)


@register_obs("checkpoints", replace=True)
def checkpoints(env):
    """Checkpoint flags (R21), in `order`: the next flag to reach as a one-hot over
    the K flags plus a last slot for "all reached, the goal is next" (K + 1), then
    each flag's reached bit (K). Length 2K + 1; a room without flags gives [1]."""
    flags = sorted(env.world.checkpoints, key=lambda c: c.order)
    reached = np.array([float(c.reached) for c in flags])
    nxt = np.zeros(len(flags) + 1)
    left = [i for i, c in enumerate(flags) if not c.reached]
    nxt[left[0] if left else len(flags)] = 1.0
    return np.concatenate([nxt, reached])


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


@register_obs("circuit", replace=True)
def circuit(env):
    """Breakable circuit parts (R34), room order: each one's position relative to the
    body (heading frame) and whether it is broken (3 per part). A room without a
    breakable part gives an empty array."""
    from .parts.breakable import Breakable

    rows = []
    for p in env.parts.values():
        if not (isinstance(p, Breakable) and p.breakable) or p.parked:
            continue
        xy = p.pos(env.data)[:2] if hasattr(p, "pos") else p.xy
        rows.append(np.concatenate([_to_body_frame(env, xy), [float(p.broken)]]))
    return np.concatenate(rows) if rows else np.zeros(0)


@register_obs("part_links", replace=True)
def part_links(env):
    """The link cue (R35 `shuffle: {links: ...}`): per shuffled reader a one-hot over
    its candidate sources, so a policy can see which part feeds it this episode. An
    entry with `hidden: true` gives zeros, and a room that shuffles no link gives an
    empty array."""
    return env.randomizer.link_observation(env.layout)


@register_obs("part_group", replace=True)
def part_group(env):
    """Which member of each drawn group is in the room (R35 `type` / `count`): per group
    one bit per member, 1 for the ones in play. A room that draws no group gives an
    empty array."""
    return env.randomizer.used_observation(env.layout)


@register_obs("part_used", replace=True)
def part_used(env):
    """Whether each part slot is in the room this episode (R35 structure layer), room
    order: 1 for a part in play, 0 for one off stage (a storage shelf, a resting
    `rooms` zone, a dispenser's stored copy). The slots never move, so the item keeps
    its length, which is how a policy reads which room it woke up in (R35 `rooms`)."""
    return np.array([0.0 if p.parked else 1.0 for p in _obs_parts(env)])


@register_obs("randomized", replace=True)
def randomized(env):
    """The run-time values this episode drew (R35 physics layer), normalized to about
    [-1, 1] against the range the room declares, part and value name in order. Off by
    default: without it the policy has to cope without knowing what was drawn. A room
    that randomizes no value gives an empty array."""
    return env.randomizer.observation(env.layout, env.parts)


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
        if getattr(p, "object_kind", None) != "ball" or p.parked:
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


@register_obs("npcs", replace=True)
def npcs(env):
    """NPC robots (R29), room order: position and velocity relative to the body
    (heading frame), moving (0/1) and acting (its rule reacting now, 0/1); 6 per robot
    (zeros for a dispenser's stored copy, R28)."""
    s = env.body_state
    c, sn = np.cos(s.yaw), np.sin(s.yaw)
    rows = []
    for p in env.parts.values():
        if not getattr(p, "planar", False):
            continue
        if p.parked:
            rows.append(np.zeros(6))
            continue
        v = p.vel(env.data)
        v = np.array([c * v[0] + sn * v[1], -sn * v[0] + c * v[1]]) - s.lin_vel[:2]
        rows.append(np.concatenate([_to_body_frame(env, p.pos(env.data)), v,
                                    [float(p.moving(env.data)), float(p.acting)]]))
    return np.concatenate(rows) if rows else np.zeros(0)


#: Object kinds the `beak` term numbers (1 = first); 0 = nothing held. New kinds
#: go at the end so the numbers of the old ones stay.
GRAB_KINDS = ("cube", "laundry", "key", "handle_ball", "egg", "stick", "lever", "pull_cord", "drawer", "knot")


@register_obs("beak", replace=True)
def beak(env):
    """Beak grab (R18): holding (0/1), held object kind (`GRAB_KINDS` index + 1, 0:
    none) and the nearest handle relative to the beak tip (heading frame, 3); 5 values.
    Zeros (and no handle) in a room without grabbable objects. Handles not grabbable now
    (a latched drawer's strap, a dispenser's stored copies) are left out."""
    from .grab import BEAK_PART

    b = env.parts.get(BEAK_PART)
    if b is None or not b._eq:
        return np.zeros(5)
    kind = 0.0
    if b.held is not None:
        kind = float(GRAB_KINDS.index(b.objects[b.held].object_kind) + 1)
    tip = env.data.site_xpos[b._site]
    pts = [o.handle_points(env.data) for o in b.objects if getattr(o, "grabbable_now", True)]
    if not pts:                                         # e.g. only a latched drawer: no handle
        return np.array([float(b.held is not None), kind, 0.0, 0.0, 0.0])
    pts = np.concatenate(pts) - tip
    d = pts[int(np.argmin(np.linalg.norm(pts, axis=1)))]
    yaw = env.body_state.yaw
    c, sn = np.cos(yaw), np.sin(yaw)
    return np.array([float(b.held is not None), kind, c * d[0] + sn * d[1], -sn * d[0] + c * d[1], d[2]])


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
