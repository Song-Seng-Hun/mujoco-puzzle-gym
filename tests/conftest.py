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


@pytest.fixture(autouse=True)
def _trust_tmp_runs(monkeypatch):
    """Tests train into tmp folders outside ./runs/; trust them. tests/test_trust.py unsets it."""
    monkeypatch.setenv("MPG_TRUST", "1")


def pytest_addoption(parser):
    parser.addoption("--shard", default=None, metavar="i/n",
                     help="run only shard i of n (0-based), interleaved by collection order")


def pytest_collection_modifyitems(config, items):
    """`--shard i/n` keeps every n-th test from i. Interleaving (not slicing) spreads the
    slow and memory-hungry files over all shards, so CI's shards finish together and no
    single process reaches the whole suite's peak memory. The suite's resident set grows
    to about 12 GB over a full run, close to a GitHub runner's 16 GB; two shards keep each
    process well under that. Without the option nothing changes."""
    spec = config.getoption("--shard")
    if not spec:
        return
    try:
        i, n = (int(x) for x in spec.split("/"))
    except ValueError:
        raise pytest.UsageError(f"--shard {spec}: write it as i/n, e.g. 0/2") from None
    if not 0 <= i < n:
        raise pytest.UsageError(f"--shard {spec}: need 0 <= i < n")
    items[:] = [it for k, it in enumerate(items) if k % n == i]
