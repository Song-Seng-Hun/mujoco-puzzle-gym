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


# ---- `mpg check` stage `actions`: the task must have what the plan needs ----------------
def test_check_asks_for_grab_in_a_cube_room(asset_dir, repo_root):
    from mujoco_puzzle_gym.check import check

    res = check(repo_root / ROOM, static_only=True)
    assert res["failed"] == ["actions"] and "`grab`" in res["actions"]["reason"]
    res = check(repo_root / ROOM, task=repo_root / "examples/task_cube_plate.yaml", static_only=True)
    assert res["ok"], res


@pytest.mark.parametrize("height, actions, ok", [(0.15, ["move"], False), (0.15, ["move", "head"], True),
                                                 (0.21, ["move"], True)])
def test_check_asks_for_head_for_a_low_wall_button(asset_dir, tmp_path, height, actions, ok):
    import yaml

    from mujoco_puzzle_gym.check import check

    room = {"name": "wall_low", "size": [2.6, 1.8], "wall": {"thickness": 0.06, "height": 0.25},
            "walls": [{"from": [0.3, -0.9], "to": [0.3, -0.2]}, {"from": [0.3, 0.2], "to": [0.3, 0.9]}],
            "spawn": {"pos": [-0.6, 0.0], "yaw": 3.14},
            "parts": [{"name": "btn", "type": "button", "mount": "wall", "pos": [-1.27, 0.0], "facing": 0,
                       "height": height, "size": [0.12, 0.06], "kind": "oneshot"},
                      {"name": "door", "type": "door", "pos": [0.3, 0.0], "width": 0.4, "axis": "y",
                       "in": ["btn"]},
                      {"name": "goal", "type": "goal", "pos": [0.9, 0.0], "radius": 0.12}]}
    (tmp_path / "wall_low.yaml").write_text(yaml.safe_dump(room))
    res = check(tmp_path / "wall_low.yaml", task={"actions": actions}, static_only=True)
    assert res["actions"]["ok"] is ok and res["ok"] is ok, res
    if not ok:
        assert "`head`" in res["actions"]["reason"]
