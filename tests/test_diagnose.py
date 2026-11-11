"""R9: mpg diagnose classifies failures with numbers as evidence.

Fast: the classification rules on hand-made statistics.
Slow: three deliberately broken configs (examples/broken/task_*.yaml), each
trained briefly and diagnosed; each must give its expected primary cause.
"""

import json
import subprocess
import sys

import pytest

from mujoco_puzzle_gym.diagnose import classify


def stats(**kw):
    base = {"episodes": 50, "mean_return": 0.0, "std_return": 1.0, "success_rate": 0.0,
            "fall_rate": 0.0, "switch_rate": 0.5, "nan_episodes": 0, "contradiction_episodes": 0}
    return {**base, **kw}


@pytest.mark.parametrize("first, last, roll, expected", [
    (stats(), stats(), stats(nan_episodes=2), ["env_bug"]),
    (stats(), stats(), stats(contradiction_episodes=1), ["env_bug"]),
    (stats(), stats(fall_rate=0.6), stats(fall_rate=0.7), ["body_failure"]),
    (stats(switch_rate=0.0), stats(switch_rate=0.0, mean_return=5.0), stats(switch_rate=0.0),
     ["exploration"]),
    (stats(mean_return=2.0), stats(mean_return=40.0, success_rate=0.02), stats(), ["reward_hacking"]),
    # return rose with success: learning, not hacking
    (stats(mean_return=2.0), stats(mean_return=40.0, success_rate=0.6), stats(success_rate=0.5), []),
    # flat return, no other sign: nothing to blame
    (stats(mean_return=2.0), stats(mean_return=2.1), stats(), []),
])
def test_rules(first, last, roll, expected):
    every = stats(switch_rate=min(first["switch_rate"], last["switch_rate"]))
    causes, evidence = classify(first, last, roll, every, has_switch=True)
    assert causes == expected, evidence
    assert set(evidence) == {"env_bug", "body_failure", "exploration", "reward_hacking"}


def test_no_switch_no_exploration_blame():
    s = stats(switch_rate=0.0)
    causes, _ = classify(s, s, s, s, has_switch=False)
    assert "exploration" not in causes


def mpg(*args):
    out = subprocess.run([sys.executable, "-m", "mujoco_puzzle_gym.cli", *args, "--json"],
                         capture_output=True, text=True)
    assert out.returncode == 0, (args, out.stdout, out.stderr)
    return json.loads(out.stdout)


BROKEN = {  # task -> (expected primary cause, training minutes)
    "task_progress_hack.yaml": ("reward_hacking", 1.5),
    "task_far_switch.yaml": ("exploration", 0.5),
    "task_wild_commands.yaml": ("body_failure", 0.3),
}


@pytest.mark.slow
@pytest.mark.parametrize("task", sorted(BROKEN))
def test_broken_config_is_classified(asset_dir, repo_root, tmp_path, task):
    expected, minutes = BROKEN[task]
    mpg("train", str(repo_root / "examples/broken" / task), "--minutes", str(minutes),
        "--runs-dir", str(tmp_path), "--id", "run")
    res = mpg("diagnose", str(tmp_path / "run"), "--episodes", "12")
    assert res["verdict"] == expected, json.dumps(res, indent=1)
    assert res["causes"][0] == expected and res["evidence"][expected]
    video = res.get("failure_video")
    assert video and (tmp_path / "run" / "diagnose" / "failure.mp4").stat().st_size > 0
