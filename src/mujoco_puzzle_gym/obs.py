"""Built-in observation terms (R7). Each maps the env to a fixed-length 1-D array."""

from __future__ import annotations

import numpy as np

from .registry import register_obs


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


@register_obs("coins", replace=True)
def coins(env):
    """Coins (R36): per coin group (room order) the fraction picked, then the
    `COIN_OBS_K` nearest coins not picked yet as (x, y, 1) in the heading frame;
    empty slots are (0, 0, 0)."""
    groups = [p for p in env.parts.values() if hasattr(p, "picked_ratio")]
    left = [c for g in groups for c in g.remaining()]
    pos = env.body_state.pos[:2]
    left.sort(key=lambda c: float(np.hypot(*(c.xy - pos))))
    slots = np.zeros((COIN_OBS_K, 3))
    for i, c in enumerate(left[:COIN_OBS_K]):
        slots[i] = [*_to_body_frame(env, c.xy), 1.0]
    return np.concatenate([[g.picked_ratio() for g in groups], slots.ravel()])


BALLS_K = 3   # nearest balls in the `balls` term


@register_obs("balls", replace=True)
def balls(env):
    """Nearest `BALLS_K` balls (R38): position and velocity relative to the body,
    heading frame, plus a present flag; empty slots are zeros (5 per slot)."""
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
    out = np.zeros((BALLS_K, 5))
    for i, (_, row) in enumerate(rows[:BALLS_K]):
        out[i] = row
    return out.ravel()
