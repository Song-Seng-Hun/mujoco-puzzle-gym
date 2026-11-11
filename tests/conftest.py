from pathlib import Path

import pytest

from mujoco_puzzle_gym import assets


@pytest.fixture(scope="session")
def asset_dir():
    """The verified asset cache; downloads it (network) only when missing."""
    return assets.fetch()["cache_dir"]


@pytest.fixture(scope="session")
def repo_root():
    from pathlib import Path
    return Path(__file__).resolve().parents[1]


@pytest.fixture(scope="session")
def quick_run(asset_dir, repo_root, tmp_path_factory):
    """A few seconds of PPO on the example room with 2 envs: a real run dir to eval/render."""
    from mujoco_puzzle_gym.train import train

    runs = tmp_path_factory.mktemp("runs")
    summary = train(repo_root / "examples/task_ppo.yaml", minutes=1.0, n_envs=2,
                    runs_dir=runs, run_id="quick", max_steps=2048)
    return Path(summary["run_dir"])
