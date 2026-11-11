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
