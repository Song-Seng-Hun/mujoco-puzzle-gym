"""R7: PuzzleEnv passes gymnasium's checker; reward = sum of weighted terms in info."""

import dataclasses

import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.solver import ScriptedSolver
from mujoco_puzzle_gym.task import TaskError, load_task

TASK = "examples/task_ppo.yaml"


@pytest.fixture(scope="module")
def env(asset_dir, repo_root):
    e = PuzzleEnv(task=repo_root / TASK)
    yield e
    e.close()


def test_check_env(env):
    check_env(env, skip_render_check=True)


def test_spaces_and_timing(env):
    assert env.action_space.shape == (3,)
    # gravity 3 + ang_vel 3 + lin_vel 3 + part_pos 2x3 + part_state 3
    assert env.observation_space.shape == (18,)
    assert env.hz == 10 and env.body_steps == 5 and env.max_steps == 400


def test_reward_is_sum_of_terms(env):
    env.reset(seed=1)
    for _ in range(20):
        _, reward, *_, info = env.step(env.action_space.sample())
        assert set(info["reward_terms"]) == {"progress", "stage", "success", "fall"}
        assert reward == pytest.approx(sum(info["reward_terms"].values()))


def test_progress_stage_success_along_solver(env):
    env.reset(seed=3)
    solver = ScriptedSolver(env)
    total = {"progress": 0.0, "stage": 0.0, "success": 0.0, "fall": 0.0}
    first_progress = None
    while True:
        _, _, terminated, truncated, info = env.step(solver.act())
        for k, v in info["reward_terms"].items():
            total[k] += v
        if first_progress is None and env.steps == 30:
            first_progress = total["progress"]
        if terminated or truncated:
            break
    assert info["success"]
    assert first_progress > 0, "walking to the switch must earn progress"
    assert total["stage"] == pytest.approx(2.0)        # one switch x weight 2
    assert total["success"] == pytest.approx(10.0)
    assert total["fall"] == 0.0
    assert total["progress"] > 0.5 * 10.0                # > 0.5 m net at weight 10


def test_fall_penalty_terminates(env, monkeypatch):
    env.reset(seed=0)
    real_state = env.body.state
    monkeypatch.setattr(env.body, "state",
                        lambda data: dataclasses.replace(real_state(data), fallen=True))
    _, _, terminated, _, info = env.step(np.zeros(3, dtype=np.float32))
    assert terminated and info["fallen"]
    assert info["reward_terms"]["fall"] == pytest.approx(-5.0)


def test_weights_and_term_selection_come_from_task(asset_dir, repo_root):
    e = PuzzleEnv(task={"room": str(repo_root / "examples/switch_door.yaml"),
                        "obs": ["gravity", "target_pos"], "reward": {"progress": 1.0}})
    assert e.observation_space.shape == (5,)
    e.reset(seed=0)
    *_, info = e.step(np.zeros(3, dtype=np.float32))
    assert set(info["reward_terms"]) == {"progress"}


@pytest.mark.parametrize("task, needle", [
    ({"reward": {"nope": 1.0}}, "unknown reward"),
    ({"obs": ["nope"]}, "unknown obs"),
    ({"env": {"hz": 50}}, "env.hz"),
])
def test_bad_task_is_rejected(repo_root, task, needle):
    with pytest.raises(TaskError, match=needle):
        load_task({"room": str(repo_root / "examples/switch_door.yaml"), **task})


def test_hz_must_divide_body_steps(asset_dir, repo_root):
    with pytest.raises(TaskError, match="whole number"):
        PuzzleEnv(task={"room": str(repo_root / "examples/switch_door.yaml"), "env": {"hz": 7}})
