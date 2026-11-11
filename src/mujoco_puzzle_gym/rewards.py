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
