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


# ---- marker detection noise (R131) ----------------------------------------------------
def marker_task(repo_root, **item):
    return {"room": str(repo_root / "src/mujoco_puzzle_gym/presets/puzzles/tabletop/reach_the_marker.yaml"),
            "body": "so101", "env": {"hz": 10, "episode_seconds": 8},
            "obs": ["gravity", {"item": "markers_seen", **item}] if item
                   else ["gravity", "markers_seen"],
            "reward": {"progress": 10.0, "success": 10.0},
            "termination": {"success": True, "fall": False},
            "randomize": {"layout": 1.0}}


@pytest.mark.parametrize("item,match", [({"noise": -1.0}, "noise"), ({"noise": "x"}, "noise"),
                                        ({"dropout": 2.0}, "probability")])
def test_marker_noise_params_are_checked(item, match):
    from mujoco_puzzle_gym.gaze import MarkersSeen
    from mujoco_puzzle_gym.obs import ObsItemError

    with pytest.raises(ObsItemError, match=match):
        MarkersSeen(None, None, **item)


def in_view(envs, seed: int = 1_000_000):
    """Step both envs with a zero command until the clean one has the marker in view.

    The noise only touches the observation, so the two envs stay in the same state; the
    first element of each `markers_seen` slot is the seen flag.
    """
    obs = [e.reset(seed=seed)[0] for e in envs]
    for _ in range(20):
        if obs[0][3] == 1.0:
            return obs
        obs = [e.step(np.zeros(e.action_space.shape, dtype=np.float32))[0] for e in envs]
    raise AssertionError("the marker never came into view for this layout")


def test_a_dropped_marker_reads_as_out_of_view(so101_asset_dir, repo_root):
    """`dropout: 1.0` is the test case: the marker is in view and still never reported."""
    clean = PuzzleEnv(task=marker_task(repo_root))
    blind = PuzzleEnv(task=marker_task(repo_root, dropout=1.0))
    try:
        seen, lost = in_view([clean, blind])
        assert seen[3] == 1.0
        assert np.allclose(lost[3:], 0.0)
    finally:
        clean.close()
        blind.close()


def test_position_noise_moves_the_reported_marker_but_not_the_seen_flag(so101_asset_dir, repo_root):
    clean = PuzzleEnv(task=marker_task(repo_root))
    noisy = PuzzleEnv(task=marker_task(repo_root, noise=0.05))
    try:
        a, b = in_view([clean, noisy])
        assert a[3] == b[3] == 1.0
        assert not np.allclose(a[4:6], b[4:6], atol=1e-4)
    finally:
        clean.close()
        noisy.close()


# ---- the latency fix (R131) -----------------------------------------------------------
@pytest.mark.slow
@pytest.mark.parametrize("latency", [0, 2])
def test_the_scripted_solution_survives_declared_actuator_latency(so101_asset_dir, repo_root,
                                                                  latency):
    """R118 measured 40 ms (2 control steps) as fatal to every arm room. It was the
    controller, not the delay: the solve integrated from the **measured** pose, so while a
    command was in flight it kept re-applying the same error. Integrating from the newest
    queued target instead (`So101Adapter.step`) leaves 0 ms untouched and carries
    `pick_and_place` at 40 ms - measured 5/5 at 8.5 s against 8.2 s with no delay."""
    from mujoco_puzzle_gym.solver import ScriptedSolver

    task = load_task(repo_root / "examples/task_pick_and_place.yaml")
    task["env"]["actuator_latency_steps"] = latency
    env = PuzzleEnv(task=task)
    try:
        for seed in (0, 1):
            env.reset(seed=seed)
            solver = ScriptedSolver(env)
            solver.reset()
            while True:
                *_, term, trunc, info = env.step(solver.act())
                if term or trunc:
                    assert info.get("success"), f"latency {latency}, seed {seed}"
                    break
    finally:
        env.close()


def test_calibration_bias_is_one_draw_held_for_the_whole_episode(so101_asset_dir, repo_root):
    """What makes `bias` different from `noise`, and harder: it cannot be averaged away.

    Within an episode the offset is the same every step (so the clean and biased readings
    differ by a constant); a different episode draws a different one. Same seed, same draw -
    an episode stays reproducible.
    """
    clean = PuzzleEnv(task=marker_task(repo_root))
    biased = PuzzleEnv(task=marker_task(repo_root, bias=0.05))
    try:
        a, b = in_view([clean, biased])
        off = b[4:6] - a[4:6]
        assert np.linalg.norm(off) > 1e-3, "the bias should move the reported position"
        for _ in range(3):
            zero = np.zeros(clean.action_space.shape, dtype=np.float32)
            a = clean.step(zero)[0]
            b = biased.step(zero)[0]
            if a[3] == 1.0 and b[3] == 1.0:
                assert np.allclose(b[4:6] - a[4:6], off, atol=1e-5)
        a2, b2 = in_view([clean, biased], seed=1_000_007)
        assert not np.allclose(b2[4:6] - a2[4:6], off, atol=1e-4)
    finally:
        clean.close()
        biased.close()


def test_lens_distortion_grows_with_distance_from_the_centre(so101_asset_dir, repo_root):
    """The radial model: `r (1 + k r^2)`, so a marker further out moves further."""
    from mujoco_puzzle_gym.gaze import MarkersSeen

    item = MarkersSeen(None, None, distortion=-0.5)
    near = np.array([[1.0, 0.1, 0.0, 0.0, 0.0]])
    far = np.array([[1.0, 0.4, 0.0, 0.0, 0.0]])
    moved = []
    for row in (near, far):
        xy = row[0, 1:3]
        r2 = float(np.sum(xy ** 2))
        moved.append(abs(float(xy[0]) * item.distortion * r2))
    assert moved[1] > moved[0] * 5          # 0.4^3 vs 0.1^3 is 64x before the shared factor


def test_distortion_must_be_a_number():
    from mujoco_puzzle_gym.gaze import MarkersSeen
    from mujoco_puzzle_gym.obs import ObsItemError

    with pytest.raises(ObsItemError, match="distortion"):
        MarkersSeen(None, None, distortion="barrel")
