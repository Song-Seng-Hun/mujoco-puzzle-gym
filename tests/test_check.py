"""R6: mpg check — (a) solution order, (b) stability, (c) scripted solver."""

import json
import subprocess
import sys

import pytest

from mujoco_puzzle_gym.check import check_order, check_solver, check_stability
from mujoco_puzzle_gym.env import PuzzleEnv

ROOM = "examples/switch_door.yaml"
UNLINKED = "examples/broken/door_unlinked.yaml"
IN_WALL = "examples/broken/switch_in_wall.yaml"


def mpg_check(path, *extra):
    out = subprocess.run([sys.executable, "-m", "mujoco_puzzle_gym.cli", "check", str(path),
                          "--json", *extra], capture_output=True, text=True)
    return out.returncode, json.loads(out.stdout)


@pytest.fixture(scope="module")
def example(asset_dir, repo_root):
    env = PuzzleEnv(room=repo_root / ROOM)
    yield env
    env.close()


def test_example_order(example):
    res = check_order(example)
    assert res["ok"] and res["order"] == ["switch_a", "door_b", "goal"]


def test_example_stable(example):
    res = check_stability(example)
    assert res["ok"] and not res["nan"] and not res["fallen"]
    assert res["drift_m"] < 0.1


def test_example_solver_few_seeds(example):
    res = check_solver(example, seeds=3, need=3)
    assert res["ok"], res


def test_unlinked_door_fails_order(asset_dir, repo_root):
    code, res = mpg_check(repo_root / UNLINKED, "--seeds", "2")
    assert code != 0 and res["ok"] is False
    assert res["order"]["ok"] is False and "door_b" in res["order"]["reason"]
    assert "order" in res["failed"]


def test_switch_in_wall_passes_order(asset_dir, repo_root):
    env = PuzzleEnv(room=repo_root / IN_WALL)
    assert check_order(env)["ok"]


@pytest.mark.slow
def test_example_room_passes_full_check(asset_dir, repo_root):
    code, res = mpg_check(repo_root / ROOM)
    assert code == 0 and res["ok"] is True, res
    assert res["solver"]["seeds"] == 20 and res["solver"]["successes"] >= 18


@pytest.mark.slow
def test_switch_in_wall_fails_solver_only(asset_dir, repo_root):
    code, res = mpg_check(repo_root / IN_WALL)
    assert code != 0 and res["failed"] == ["solver"], res
    assert res["solver"]["successes"] < 18
