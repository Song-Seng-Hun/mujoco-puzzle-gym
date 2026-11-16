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


def test_seeds_that_differ_say_nothing(example):
    """The quiet side of the guard below: this room's seeds really do differ, so the stage
    must not claim its count is one outcome repeated."""
    res = check_solver(example, seeds=3, need=1)
    assert "distinct_outcomes" not in res and "note" not in res


def test_seeds_that_all_came_out_the_same_say_so(so101_asset_dir, repo_root):
    """`'successes': N, 'seeds': N` reads as N trials, and on this project it was read that
    way and reported that way for a solution that works at one object position and fails a
    few millimetres off it. The stage now says when its own episodes did not differ.

    Measured per room rather than inferred from the room's `randomize:`, because a room that
    declares none can still answer the seed - `examples/rooms/laundry.yaml` has no
    `randomize:` and seed 1 of 5 fails where the others pass - and the room used here
    declares none either way.
    """
    from mujoco_puzzle_gym.task import load_task

    env = PuzzleEnv(task=load_task(repo_root / "examples/task_pick_and_place.yaml"))
    try:
        res = check_solver(env, seeds=2, need=2)
        assert res["distinct_outcomes"] == 1
        assert "one outcome repeated" in res["note"]
        assert check_solver(env, seeds=1, need=1).get("note") is None, "one seed says nothing"
    finally:
        env.close()


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


# ---- R78: a sub-goal that ends without its effect fails the stage --------------------
def test_an_unfinished_subgoal_is_named(tmp_path, so101_asset_dir):
    """The predicate itself. A `demo:` sub-goal ends when its act returns None, which is
    meant to mean "done" - and `grab` and `place` return None when they give up too,
    because the carrier's stuck counter walks it off and the next step starts as if the
    object had moved. Three false claims on this project came from that, so the effect is
    checked where the step ends.

    Tested as a unit rather than through an episode on purpose: the failure it guards
    against is no longer reachable in the arm's rooms (that was the bug, and it is fixed),
    and a room crafted to fake it either fails for an ordinary reason or stops advancing.
    """
    import warnings

    import yaml

    from mujoco_puzzle_gym.env import PuzzleEnv
    from mujoco_puzzle_gym.solver import ScriptedSolver

    room = tmp_path / "unfinished.yaml"
    room.write_text(yaml.safe_dump(
        {"name": "unfinished", "size": [1.2, 1.0], "wall": {"thickness": 0.02, "height": 0.1},
         "spawn": {"pos": [-0.3, 0.0], "yaw": 0.0},
         "parts": [
             {"name": "block", "type": "cube", "pos": [-0.05, -0.12], "size": 0.03, "mass": 0.02},
             {"name": "box", "type": "receptacle", "pos": [0.0, 0.14], "size": [0.12, 0.12],
              "accept": ["block"]},
             {"name": "goal", "type": "goal", "pos": [0.05, -0.25], "radius": 0.05}],
         "demo": ["grab block", "place block box", "goto goal"]}))
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        env = PuzzleEnv(task={"room": str(room), "body": "so101",
                              "actions": ["move", "head", "grab"],
                              "env": {"hz": 10, "episode_seconds": 40}})
    try:
        env.reset(seed=0)
        solver = ScriptedSolver(env)
        solver.reset()
        # nothing held, nothing in the box: both sub-goals would be lying if they ended now
        why = solver._unfinished(("grab", "block"))
        assert why and "grab block" in why and "without holding" in why
        why = solver._unfinished(("place", "block", "box"))
        assert why and "place block box" in why and "not in 'box'" in why
        # the box reports what is physically in it, so putting the name there clears it
        env.parts["box"].inside.append("block")
        assert solver._unfinished(("place", "block", "box")) is None
        # a verb with no postcondition is never reported: guessing one for the other
        # nineteen would be a rule nobody measured
        assert solver._unfinished(("goto", "goal")) is None
        assert solver.demo_unfinished == []
    finally:
        env.close()
