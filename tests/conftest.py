import pytest

from mujoco_puzzle_gym import assets


@pytest.fixture(scope="session")
def asset_dir():
    """The verified asset cache; downloads it (network) only when missing."""
    return assets.fetch()["cache_dir"]
