"""R19: task `actions` — `move` (3) plus `head` and / or `grab` (1 each). The action
size follows the setting, the env passes gymnasium's checker, the head value
reaches the body's head command and the grab value the beak."""

import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

from mujoco_puzzle_gym.bodies.microduck import HEAD_LOW
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.task import TaskError

ROOM = "examples/rooms/cube_plate.yaml"


@pytest.fixture
def make(asset_dir, repo_root):
    envs = []

    def _make(actions):
        env = PuzzleEnv(task={"actions": actions, "obs": ["gravity", "beak"]}, room=repo_root / ROOM)
        envs.append(env)
        return env

    yield _make
    for e in envs:
        e.close()


@pytest.mark.parametrize("actions, n", [(["move"], 3), (["move", "head"], 4), (["move", "grab"], 4),
                                        (["move", "head", "grab"], 5)])
def test_action_size_follows_the_setting(make, actions, n):
    env = make(actions)
    assert env.action_space.shape == (n,)


def test_check_env_with_head_and_grab(make):
    check_env(make(["move", "head", "grab"]), skip_render_check=True)


def test_head_and_grab_values_reach_the_body_and_beak(make):
    env = make(["move", "head", "grab"])
    env.reset(seed=0)
    env.step(np.array([-1.0, 0.0, 0.0, 1.0, 1.0], dtype=np.float32))
    assert np.allclose(env.body.head, HEAD_LOW)       # head 1 = lowest
    assert env.world.grab == 1.0
    env.step(np.array([-1.0, 0.0, 0.0, -1.0, -1.0], dtype=np.float32))
    assert np.allclose(env.body.head, 0.0) and env.world.grab == -1.0


@pytest.mark.parametrize("actions", [["head"], ["move", "grab", "head"], ["move", "jump"], "move"])
def test_bad_actions_are_rejected(asset_dir, repo_root, actions):
    with pytest.raises(TaskError, match="actions"):
        PuzzleEnv(task={"actions": actions}, room=repo_root / ROOM)
