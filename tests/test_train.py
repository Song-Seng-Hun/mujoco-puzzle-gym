"""R8 train, R10 eval, R11 render on a real (tiny) run; slow: 5-min PPO and 1-min SAC."""

import json
import subprocess
import sys

import imageio.v3 as iio
import pytest
import yaml

from mujoco_puzzle_gym.evaluate import evaluate, render
from mujoco_puzzle_gym.train import EVAL_SEED_BASE, TRAIN_SEED_LIMIT


def mpg(*args):
    out = subprocess.run([sys.executable, "-m", "mujoco_puzzle_gym.cli", *args, "--json"],
                         capture_output=True, text=True)
    return out.returncode, json.loads(out.stdout)


def test_run_dir_artifacts(quick_run):
    summary = json.loads((quick_run / "summary.json").read_text())
    assert summary["algo"] == "ppo" and summary["n_envs"] == 2
    assert summary["env_steps"] >= 2048 and summary["episodes"] >= 1
    for name in ("checkpoint.zip", "curves.png", "episodes.jsonl", "task.resolved.yaml"):
        assert (quick_run / name).stat().st_size > 0, name
    ep = json.loads((quick_run / "episodes.jsonl").read_text().splitlines()[0])
    assert set(ep["terms"]) == {"progress", "stage", "success", "fall"}
    assert ep["return"] == pytest.approx(sum(ep["terms"].values()), abs=1e-2)
    task = yaml.safe_load((quick_run / "task.resolved.yaml").read_text())
    assert task["n_envs"] == 2 and task["env"]["hz"] == 10


def test_eval_uses_held_out_seeds(quick_run):
    res = evaluate(quick_run, episodes=3, workers=1)
    assert res["seeds"] == [EVAL_SEED_BASE, EVAL_SEED_BASE + 2]
    assert EVAL_SEED_BASE > TRAIN_SEED_LIMIT
    assert 0.0 <= res["success_rate"] <= 1.0 and res["episodes"] == 3


def test_render_writes_mp4(quick_run, tmp_path):
    res = render(quick_run, out=tmp_path / "ep.mp4", seed=EVAL_SEED_BASE)
    frames = iio.imread(res["mp4"])
    assert res["bytes"] > 0 and len(frames) == res["frames"] > 1
    assert frames.shape[1:] == (480, 640, 3)


@pytest.mark.slow
def test_five_minute_ppo_beats_random(asset_dir, repo_root, tmp_path):
    code, summary = mpg("train", str(repo_root / "examples/task_ppo.yaml"), "--minutes", "5",
                        "--runs-dir", str(tmp_path), "--id", "ppo5")
    assert code == 0, summary
    run = str(tmp_path / "ppo5")
    _, trained = mpg("eval", run, "--episodes", "50")
    _, rand = mpg("eval", run, "--episodes", "50", "--policy", "random")
    assert trained["mean_return"] > rand["mean_return"], (trained, rand)


@pytest.mark.slow
def test_sac_one_minute(asset_dir, repo_root, tmp_path):
    code, summary = mpg("train", str(repo_root / "examples/task_ppo.yaml"), "--algo", "sac",
                        "--minutes", "1", "--runs-dir", str(tmp_path), "--id", "sac1")
    assert code == 0, summary
    assert summary["algo"] == "sac" and (tmp_path / "sac1" / "checkpoint.zip").exists()
