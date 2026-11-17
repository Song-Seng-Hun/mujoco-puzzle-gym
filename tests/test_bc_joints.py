"""R131: proprioception the arm reads off itself (`joints`), a task that declares actuator
latency, and `algo: bc` - the three things it took to put a trained policy on a carry room.

The measurement behind them is in `docs/training-log.md` and
`docs/specs/2026-11-17-arm-steady-state` R131: PPO left `pick_and_place` at success 0.0
after about 25 M steps over six reward designs, two observation kinds and a curriculum;
cloning the room's own scripted solution solves it at **1.0** over 50 held-out seeds.
"""

import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.obs import SENSOR_TERMS, obs_kind
from mujoco_puzzle_gym.task import TaskError, load_task

ROOT_ROOM = "src/mujoco_puzzle_gym/presets/puzzles/tabletop/pick_by_hand.yaml"


def arm_task(repo_root, **over):
    task = {"room": str(repo_root / ROOT_ROOM), "body": "so101",
            "actions": ["move", "head", "grab"],
            "env": {"hz": 10, "episode_seconds": 40},
            "obs": ["gravity", "ang_vel", "lin_vel", "joints", "markers_seen"],
            "reward": {"progress": 10.0, "stage": 2.0, "success": 10.0},
            "termination": {"success": True, "fall": False}}
    task.update(over)
    return task


# ---- `joints`: the arm's own encoders ------------------------------------------------
def test_joints_is_a_sensor_term_not_a_privileged_one():
    """It is proprioception, so a task using it stays `sensors_only` (R131)."""
    assert "joints" in SENSOR_TERMS
    assert obs_kind({"obs": ["gravity", "joints"]}) == "sensors_only"
    assert obs_kind({"obs": ["joints", "part_pos"]}) == "privileged"


def test_the_arm_reports_six_angles_and_six_rates_inside_their_ranges(so101_asset_dir, repo_root):
    env = PuzzleEnv(task=arm_task(repo_root))
    try:
        env.reset(seed=0)
        q = env.body.joint_state(env.data)
        assert q.shape == (12,)
        assert np.all(np.abs(q[:6]) <= 1.0 + 1e-9)          # angles are scaled to the range
        assert np.allclose(q[6:], 0.0, atol=1e-6)           # at rest
    finally:
        env.close()


def test_the_jaw_angle_is_what_says_which_phase_a_grasp_is_in(so101_asset_dir, repo_root):
    """The reason this term exists. Open the jaw, step, and the sixth value moves - without
    it, "going in to grasp" and "holding and lifting" are the same observation."""
    env = PuzzleEnv(task=arm_task(repo_root))
    try:
        env.reset(seed=0)
        shut = float(env.body.joint_state(env.data)[5])
        env.body.set_grab(1.0)                               # open
        for _ in range(10):
            env.step(np.array([0.0, 0.0, 0.0, -1.0, 1.0], dtype=np.float32))
        assert abs(float(env.body.joint_state(env.data)[5]) - shut) > 0.05
    finally:
        env.close()


def test_a_body_with_no_joints_to_report_gives_an_empty_slot(asset_dir, tmp_path):
    """The default keeps every other body and task unchanged: the term is zero-length,
    the way `beak` is in a room with nothing to grab."""
    room = {"name": "r", "size": [4.0, 4.0], "wall": {"thickness": 0.1, "height": 0.4},
            "spawn": {"pos": [0.0, 0.0], "yaw": 0.0},
            "parts": [{"name": "goal", "type": "goal", "pos": [1.0, 0.0], "radius": 0.3}]}
    path = tmp_path / "room.yaml"
    path.write_text(yaml.safe_dump(room))
    env = PuzzleEnv(task={"room": str(path), "obs": ["gravity", "joints"]})
    try:
        obs, _ = env.reset(seed=0)
        assert env.body.joint_state(env.data) is None
        assert obs.shape == (3,)
    finally:
        env.close()


# ---- declared actuator latency -------------------------------------------------------
def test_a_task_can_declare_actuator_latency_and_it_survives_reset(so101_asset_dir, repo_root):
    env = PuzzleEnv(task=arm_task(repo_root, env={"hz": 10, "episode_seconds": 40,
                                                  "actuator_latency_steps": 2}))
    try:
        for seed in (0, 1):
            env.reset(seed=seed)
            assert env.body.latency_steps == 2
    finally:
        env.close()


def test_declaring_latency_on_a_body_without_any_is_refused(asset_dir, tmp_path):
    room = {"name": "r", "size": [4.0, 4.0], "wall": {"thickness": 0.1, "height": 0.4},
            "spawn": {"pos": [0.0, 0.0], "yaw": 0.0},
            "parts": [{"name": "goal", "type": "goal", "pos": [1.0, 0.0], "radius": 0.3}]}
    path = tmp_path / "room.yaml"
    path.write_text(yaml.safe_dump(room))
    with pytest.raises(TaskError, match="actuator latency"):
        PuzzleEnv(task={"room": str(path), "env": {"actuator_latency_steps": 1}})


# ---- `algo: bc` ----------------------------------------------------------------------
def test_bc_refuses_unknown_algo_kwargs():
    from mujoco_puzzle_gym.algos.bc import BehaviourCloning

    with pytest.raises(TaskError, match="unknown algo_kwargs"):
        BehaviourCloning(None, {"algo_kwargs": {"nope": 1}})


@pytest.mark.slow
def test_bc_clones_the_scripted_solution_of_a_room(so101_asset_dir, repo_root, tmp_path):
    """End to end on the cheapest arm room: collect, fit, score with the policy **alone**,
    save, load, and solve held-out seeds. The score is its own pass for a measured reason -
    `ScriptedSolver.act()` runs the carrier, so a pass with the solver in the loop scored a
    model 100/100 that `mpg eval` scored 0.0 (R131)."""
    from mujoco_puzzle_gym.evaluate import evaluate
    from mujoco_puzzle_gym.train import train

    task = {"room": str(repo_root / "src/mujoco_puzzle_gym/presets/puzzles/tabletop/reach_target.yaml"),
            "body": "so101", "env": {"hz": 10, "episode_seconds": 10},
            "obs": ["gravity", "ang_vel", "lin_vel", "joints", "part_pos", "part_state"],
            "reward": {"progress": 10.0, "stage": 2.0, "success": 10.0},
            "termination": {"success": True, "fall": False},
            "algo": "bc", "n_envs": 2, "net": [64, 64],
            "algo_kwargs": {"demo_episodes": 20, "epochs": 60, "round_episodes": 5,
                            "rollout_share": 0.2}}
    path = tmp_path / "task.yaml"
    path.write_text(yaml.safe_dump(task))
    summary = train(path, minutes=3.0, runs_dir=tmp_path / "runs", run_id="bc")
    assert summary["algo"] == "bc"
    assert evaluate(summary["run_dir"], episodes=5, workers=1)["success_rate"] == 1.0
