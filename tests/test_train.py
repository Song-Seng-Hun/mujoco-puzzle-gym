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
    for name in ("checkpoint.zip", "curves.png", "episodes.jsonl.gz", "task.resolved.yaml"):
        assert (quick_run / name).stat().st_size > 0, name
    from mujoco_puzzle_gym.train import read_episode_log

    ep = read_episode_log(quick_run)[0]
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


def test_the_episode_log_is_gzipped_and_a_pre_r139_run_still_reads(quick_run, tmp_path):
    """R139: one line per episode, nothing rotating it, and a 15-minute run of the arm room
    wrote 227,663 episodes and 59.0 MB. The content is load-bearing (`diagnose` reads the
    first and last fifth, `curves.png` reads all of it), so it is compressed rather than
    trimmed - measured 14.9x on a one-minute run here and 20.6x on that 59 MB file.

    The second half of the test is the part that matters for anyone with runs on disk: a run
    written before this change has a plain `episodes.jsonl` and has to keep working.
    """
    import gzip

    from mujoco_puzzle_gym.train import EPISODES_NAME, EPISODES_PLAIN, episode_log, read_episode_log

    gz = quick_run / EPISODES_NAME
    assert gz.exists() and not (quick_run / EPISODES_PLAIN).exists()
    plain_bytes = gzip.decompress(gz.read_bytes())
    # The ratio is a property of a real run, not of this four-episode fixture, where gzip's
    # own header dominates (1,048 bytes -> 271). Measured where it matters: 14.9x on a
    # one-minute run and 20.6x on the 59 MB file this change exists for. Here the claim is
    # the structural one - the log on disk is compressed and smaller than its contents.
    assert gz.stat().st_size < len(plain_bytes)

    eps = read_episode_log(quick_run)
    assert eps and {"return", "length", "success"} <= set(eps[0])
    assert episode_log(quick_run) == gz

    legacy = tmp_path / "old_run"
    legacy.mkdir()
    (legacy / EPISODES_PLAIN).write_bytes(plain_bytes)
    assert episode_log(legacy) == legacy / EPISODES_PLAIN
    assert read_episode_log(legacy) == eps
