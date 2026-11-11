"""Built-in reward terms (R7). Each returns an unweighted value; task.yaml gives weights.

Terms read `env.events`, filled once per env step:
- progress: metres of geodesic distance (around walls, through open doors) gained
  toward the current target this step;
- new_on:   switches that turned on this step;
- success:  a goal was reached;
- fallen:   the body fell;
- respawned: a fall or failure went back to the last checkpoint (R6).
"""

from __future__ import annotations

from .registry import register_reward


@register_reward("progress", replace=True)
def progress(env):
    return env.events["progress"]


@register_reward("stage", replace=True)
def stage(env):
    return float(env.events["new_on"])


@register_reward("success", replace=True)
def success(env):
    return float(env.events["success"])


@register_reward("fall", replace=True)
def fall(env):
    return float(env.events["fallen"])


@register_reward("respawn_penalty", replace=True)
def respawn_penalty(env):
    """1 when the body went back to a checkpoint this step; the default weight is -1."""
    return float(env.events.get("respawned", False))


@register_reward("coin_pick", replace=True)
def coin_pick(env):
    """Coins picked since the last env step (R36); a coin put back by a checkpoint
    restore pays again when picked again."""
    n = 0
    for p in env.parts.values():
        if hasattr(p, "picked_ratio"):
            n += p.picks - p.rewarded
            p.rewarded = p.picks
    return float(n)


@register_reward("ball_progress", replace=True)
def ball_progress(env):
    """Metres per env step the ball is moving toward the current push target
    (a goal net or pin, R38): ball velocity along ball -> target, over hz.
    The ball is the accepted one nearest the target that is not in it yet."""
    import numpy as np

    t = env.current_target()
    if t is None or not hasattr(t, "push_target"):
        return 0.0
    counted = set(getattr(t, "inside", []))
    best, best_d = None, np.inf
    for p in env.parts.values():
        if getattr(p, "object_kind", None) == "ball" and t.push_accepts(p) and p.name not in counted:
            d = float(np.linalg.norm(p.pos(env.data)[:2] - t.xy))
            if d < best_d:
                best, best_d = p, d
    if best is None:
        return 0.0
    b = best.pos(env.data)[:2]
    line = np.asarray(t.push_target(env, b)) - b
    n = float(np.linalg.norm(line))
    return float(best.vel(env.data)[:2] @ line / n) / env.hz if n > 1e-6 else 0.0
