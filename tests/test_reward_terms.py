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


# ---- progress_free (R35): no solution order -----------------------------------------
FREE = {"progress_free": 10.0, "stage": 2.0, "success": 10.0, "fall": -5.0}


@pytest.fixture(scope="module")
def free_env(asset_dir, repo_root):
    """The order-reasoning room: four plates, one of them wired to the door."""
    e = PuzzleEnv(task=repo_root / "examples/task_order_cue.yaml")
    yield e
    e.close()


def test_the_term_is_tracked_only_when_the_task_pays_it(asset_dir, repo_root, free_env):
    plain = PuzzleEnv(task={"room": str(repo_root / "examples/switch_door.yaml")})
    try:
        plain.reset(seed=0)
        *_, info = plain.step(np.zeros(3, dtype=np.float32))
        assert not plain.free_targets                  # no field is built for nothing
        assert plain.events["progress_free"] == 0.0
        assert "progress_free" not in info["reward_terms"]
    finally:
        plain.close()
    free_env.reset(seed=0)
    *_, info = free_env.step(np.zeros(3, dtype=np.float32))
    assert free_env.free_targets and "progress_free" in info["reward_terms"]


def test_every_plate_counts_not_only_the_one_that_works(free_env):
    """The reward must not say which plate is wired this episode: the distance is to the
    nearest plate or goal, which is usually not the wired one."""
    nearer = 0
    for seed in range(6):
        free_env.reset(seed=seed)
        names = {n for n, _ in free_env.free_key()[1]}
        assert names == {"plate_a", "plate_b", "plate_c", "plate_d", "goal"}
        assert [t.name for t in free_env.targets] == [free_env.layout["links"]["door"], "goal"]
        free = free_env.free_distance()
        assert free <= free_env.target_distance() + 1e-9
        nearer += free < free_env.target_distance() - 0.05
    assert nearer, "a decoy plate is the nearest target in at least one of these layouts"


def test_walking_to_a_decoy_plate_earns_it(free_env):
    """Driving at the nearest plate pays `progress_free` even when it is not the wired one
    (then `progress`, which follows the solution order, pays less)."""
    free_env.reset(seed=4)
    wired = free_env.layout["links"]["door"]
    near = min((n for n, _ in free_env.free_key()[1] if n != "goal"),
               key=lambda n: np.linalg.norm(free_env.parts[n].xy - free_env.body_state.pos[:2]))
    assert near != wired, "this seed is meant to put a decoy nearest"
    gained = 0.0
    for _ in range(30):
        to = free_env.parts[near].xy - free_env.body_state.pos[:2]
        yaw = free_env.body_state.yaw
        turn = np.arctan2(to[1], to[0]) - yaw
        cmd = np.array([0.8, 0.0, np.clip(np.arctan2(np.sin(turn), np.cos(turn)), -1, 1)],
                       dtype=np.float32)
        *_, info = free_env.step(cmd)
        gained += info["reward_terms"]["progress_free"]
    assert gained > 0.0


def test_a_solved_plate_drops_out_and_the_goal_takes_over(free_env):
    """Once a plate is on it is no longer a target, and the door it opened is open in the
    distance field, so the goal behind it is what the reward leads to."""
    free_env.reset(seed=2)
    solver = ScriptedSolver(free_env)
    on = None
    for _ in range(free_env.max_steps):
        _, _, terminated, truncated, info = free_env.step(solver.act())
        if on is None and info["reward_terms"]["stage"] > 0:
            on = {n for n, _ in free_env.free_key()[1]}
            assert free_env.layout["links"]["door"] not in on and "goal" in on
            after = free_env.free_distance()
        if terminated or truncated:
            break
    assert on is not None and info["success"]
    assert after < 3.0                               # the goal is reachable, not infinite
    left = {n for n, _ in free_env.free_key()[1]}     # the goal is in; the decoys are not
    assert "goal" not in left and left <= {"plate_a", "plate_b", "plate_c", "plate_d"}


def test_the_cue_off_room_draws_the_same_room_without_the_cue(asset_dir, repo_root):
    """The control room differs only in the cue: same plates, same wiring draw."""
    blind = PuzzleEnv(task=repo_root / "examples/task_order_blind.yaml")
    cue = PuzzleEnv(task=repo_root / "examples/task_order_cue.yaml")
    try:
        assert blind.observation_space.shape == cue.observation_space.shape
        for seed in range(3):
            blind.reset(seed=seed)
            cue.reset(seed=seed)
            assert blind.layout["links"] == cue.layout["links"]        # the same draw
            assert not np.any(blind.randomizer.link_observation(blind.layout))
            assert np.any(cue.randomizer.link_observation(cue.layout))
        assert blind.randomizer.hidden_cues() and not cue.randomizer.hidden_cues()
    finally:
        blind.close()
        cue.close()


@pytest.mark.parametrize("mode, drawn", [("nominal", None), ("train", {"plate_a", "plate_b",
                                                                      "plate_c"}),
                                         ("wide", {"plate_d"})])
def test_the_evaluation_only_plate_is_the_wide_draw(free_env, mode, drawn):
    got = set()
    for seed in range(5):
        free_env.reset(seed=seed, options={"randomize": mode})
        got |= set(free_env.layout["links"].values())
    assert got <= (drawn or set()) and (got or drawn is None)
    assert free_env.randomizer.wide_ok()


def test_hz_must_divide_body_steps(asset_dir, repo_root):
    with pytest.raises(TaskError, match="whole number"):
        PuzzleEnv(task={"room": str(repo_root / "examples/switch_door.yaml"), "env": {"hz": 7}})
