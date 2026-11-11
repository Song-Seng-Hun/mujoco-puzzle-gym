"""Built-in reward terms (R7). Each returns an unweighted value; task.yaml gives weights.

Terms read `env.events`, filled once per env step:
- progress: metres of geodesic distance (around walls, through open doors) gained
  toward the current target this step;
- new_on:   switches that turned on this step;
- success:  a goal was reached;
- fallen:   the body fell.
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
