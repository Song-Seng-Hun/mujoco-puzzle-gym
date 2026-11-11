"""`mpg diagnose runs/<id>` (R9): why does a run fail? Numbers as evidence.

Evidence comes from two places:
- the training log (`episodes.jsonl`): first vs last 20% of episodes;
- fresh rollouts of the saved policy on held-out seeds (DIAG_SEED_BASE + k).

Causes (zero or more, most fundamental first; `primary` is the first):
- env_bug:        any NaN episode, or a part-state contradiction (state outside
                  [0, 1], or success while a gate on the solution path never opened);
- body_failure:   fall rate >= FALL_RATE in the last window or the rollouts;
- exploration:    switches pressed in <= SWITCH_RATE of all training episodes
                  and of the rollouts (the room has a switch on its solution path);
- reward_hacking: return rose (last vs first window, by >= RISE_FRAC of |first|
                  and >= RISE_STD first-window std) while success did not
                  (rise < FLAT_SUCCESS and last-window success < HACK_SUCCESS_MAX),
                  and neither body_failure nor exploration explains the flat success.
A run whose rollouts succeed >= TARGET_SUCCESS reports no cause.

One failed rollout is saved as `runs/<id>/diagnose/failure.mp4`.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .evaluate import DEFAULT_WORKERS, make_policy, record, run_episodes
from .train import load_run, split_windows, window_stats

DIAG_SEED_BASE = 2_000_000
TARGET_SUCCESS = 0.8
FALL_RATE = 0.3
SWITCH_RATE = 0.02
RISE_FRAC = 0.2
RISE_STD = 0.5
FLAT_SUCCESS = 0.1
HACK_SUCCESS_MAX = 0.5
CAUSES = ("env_bug", "body_failure", "exploration", "reward_hacking")
ADVICE = {
    "env_bug": "fix the env or part code (NaN or impossible part states); training results are not trustworthy",
    "body_failure": "the body falls: check env.command_scale / action mapping, add or raise the fall penalty",
    "exploration": "the switch is never found: move it closer, enlarge it, add part_pos/target_pos obs, "
                   "or a shaped reward toward it",
    "reward_hacking": "return rises without success: rebalance weights (raise success/stage, lower "
                      "progress) or make progress potential-based",
}


def classify(first: dict[str, Any], last: dict[str, Any], roll: dict[str, Any],
             all_train: dict[str, Any], has_switch: bool) -> tuple[list[str], dict[str, Any]]:
    """Return (causes, evidence) from window statistics (see window_stats)."""
    causes: list[str] = []
    ev: dict[str, Any] = {}
    nan = all_train.get("nan_episodes", 0) + roll.get("nan_episodes", 0)
    contra = all_train.get("contradiction_episodes", 0) + roll.get("contradiction_episodes", 0)
    ev["env_bug"] = {"nan_episodes": nan, "contradiction_episodes": contra}
    if nan or contra:
        causes.append("env_bug")

    fall = max(last.get("fall_rate", 0.0), roll.get("fall_rate", 0.0))
    ev["body_failure"] = {"fall_rate_last_window": last.get("fall_rate"),
                          "fall_rate_rollouts": roll.get("fall_rate"), "threshold": FALL_RATE}
    if fall >= FALL_RATE:
        causes.append("body_failure")

    ev["exploration"] = {"switch_rate_training": all_train.get("switch_rate"),
                         "switch_rate_rollouts": roll.get("switch_rate"),
                         "threshold": SWITCH_RATE, "room_has_switch": has_switch}
    if has_switch and all_train.get("switch_rate", 0.0) <= SWITCH_RATE \
            and roll.get("switch_rate", 0.0) <= SWITCH_RATE:
        causes.append("exploration")

    r0, r1 = first.get("mean_return", 0.0), last.get("mean_return", 0.0)
    s0, s1 = first.get("success_rate", 0.0), last.get("success_rate", 0.0)
    rise = r1 - r0
    need = max(RISE_FRAC * abs(r0), RISE_STD * first.get("std_return", 0.0), 1e-6)
    ev["reward_hacking"] = {"return_first": r0, "return_last": r1, "rise": round(rise, 3),
                            "rise_needed": round(need, 3), "success_first": s0, "success_last": s1,
                            "success_rollouts": roll.get("success_rate")}
    explained = {"body_failure", "exploration"} & set(causes)   # these already explain flat success
    if rise >= need and (s1 - s0) < FLAT_SUCCESS and s1 < HACK_SUCCESS_MAX and not explained:
        causes.append("reward_hacking")
    return causes, ev


def diagnose(run_dir: str | Path, episodes: int = 20, video: bool = True,
             workers: int = DEFAULT_WORKERS) -> dict[str, Any]:
    run = load_run(run_dir)
    run_dir = Path(run_dir)
    hz = float(run["task"]["env"]["hz"])
    eps = run["episodes"]
    first, last = split_windows(eps)
    w_first, w_last, w_all = window_stats(first, hz), window_stats(last, hz), window_stats(eps, hz)

    seeds = list(range(DIAG_SEED_BASE, DIAG_SEED_BASE + episodes))
    rolls = run_episodes(run_dir, seeds, workers=workers)
    w_roll = window_stats(rolls, hz)

    from .room import Room

    parts = Room.load(run["task"]["room"]).make_parts()
    has_switch = any(p.role == "switch" for p in parts.values())
    if w_roll["success_rate"] >= TARGET_SUCCESS:
        causes, evidence = [], {}
        verdict = "ok"
    else:
        causes, evidence = classify(w_first, w_last, w_roll, w_all, has_switch)
        verdict = causes[0] if causes else "undetermined"

    result: dict[str, Any] = {
        "run_id": run["summary"]["run_id"],
        "verdict": verdict,
        "causes": causes,
        "advice": [ADVICE[c] for c in causes] if causes else
                  (["no failure pattern matched: train longer or check the curves"]
                   if verdict == "undetermined" else []),
        "evidence": evidence,
        "training": {"episodes": len(eps), "first_window": w_first, "last_window": w_last,
                     "all": {k: w_all[k] for k in ("success_rate", "fall_rate", "switch_rate",
                                                   "nan_episodes", "contradiction_episodes")}
                     if eps else {}},
        "rollouts": {"seeds": [seeds[0], seeds[-1]], **w_roll},
    }
    failed = [e for e in rolls if not e["success"]]
    if video and failed:
        seed = failed[0]["seed"]
        _, path = record(run, make_policy(run), seed, run_dir / "diagnose" / "failure.mp4")
        result["failure_video"] = str(path.resolve())
        result["failure_seed"] = seed
    out = run_dir / "diagnose" / "diagnose.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2))
    result["report"] = str(out.resolve())
    return result
