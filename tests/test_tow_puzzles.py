"""R31 tow puzzles: the `puzzles/tow/` presets and the pit fill check.

- the tow presets are listed, load and pass the static stages with no warnings, and
  their `demo:` uses the rope (`tow`, `drag`, `untow` or a button that unties);
- a box the `demo:` drags into a pit must be as tall as the pit is deep (top flush
  with the floor within 1 cm), else the static stage `parts` fails;
- slow: without the rope steps (or walking straight at the goal) the presets are not
  solved. Their 20-seed `mpg check` runs in test_examples.py (room presets).
"""

import math

import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym import catalog
from mujoco_puzzle_gym.check import check
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.floor import CATCH_DEPTH
from mujoco_puzzle_gym.solver import ScriptedSolver, run_episode

TOW = catalog.PRESET_ROOT / "puzzles/tow"
NAMES = ["sled_delivery", "drop_and_go", "chain_dropoff", "tethered_start"]


def test_tow_presets_listed():
    assert {f"puzzles/tow/{n}" for n in NAMES} <= set(catalog.puzzle_preset_names())


@pytest.mark.parametrize("name", NAMES)
def test_tow_presets_pass_static_stages(asset_dir, name):
    res = check(TOW / f"{name}.yaml", static_only=True)
    assert res["static"]["ok"] and "warnings" not in res["parts"], res
    raw = yaml.safe_load((TOW / f"{name}.yaml").read_text())
    ropes = {p["name"] for p in raw["parts"] if p.get("type") == "rope"}
    uses = [d for d in raw["demo"] if d.split()[0] in ("tow", "drag", "untow") and d.split()[1] in ropes]
    unties = [p for p in raw["parts"] if p.get("type") == "rope" and (p.get("detach_a") or p.get("detach_b"))]
    assert uses or unties


def pit_room(tmp_path, size: float) -> PuzzleEnv:
    raw = {"name": "pit_fill", "size": [4.0, 1.5], "spawn": {"pos": [-1.2, 0.0], "yaw": 0.0},
           "parts": [{"name": "box", "type": "cube", "pos": [-1.5, 0.0], "size": size, "mass": 0.1,
                      "friction": 0.1, "hitch": True},
                     {"name": "rope", "type": "rope", "pos": [-1.2, 0.5], "a": "body", "b": "box", "length": 0.4},
                     {"name": "pit", "preset": "trap/pit", "pos": [0.0, 0.0], "size": [0.4, 0.4]},
                     {"name": "goal", "type": "goal", "pos": [1.5, 0.0], "radius": 0.1}],
           "demo": ["drag rope pit", "goto goal"]}
    path = tmp_path / "pit_fill.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


def test_box_dragged_into_a_pit_must_be_flush(asset_dir, tmp_path):
    res = check(pit_room(tmp_path, 0.1), static_only=True)
    assert not res["parts"]["ok"]
    why = res["parts"]["reason"]
    assert "'box' (0.10 m high) dragged into pit 'pit'" in why and f"({CATCH_DEPTH:.2f} m deep)" in why
    assert "0.40 m below the floor" in why
    # the tallest cube (0.3 m) still falls short of the catch plane depth: no box fills a pit yet
    assert not check(pit_room(tmp_path, 0.3), static_only=True)["parts"]["ok"]


# ---- slow: the rope steps are needed ------------------------------------------------------
def without(path, tmp_path, demo):
    raw = yaml.safe_load(path.read_text())
    raw["demo"] = demo
    out = tmp_path / path.name
    out.write_text(yaml.safe_dump(raw))
    return PuzzleEnv(room=out)


@pytest.mark.slow
@pytest.mark.parametrize("name, demo", [("sled_delivery", ["goto goal"]),
                                        ("drop_and_go", ["drag rope zone", "goto goal"]),
                                        ("chain_dropoff", ["drag front drop_a", "goto goal"]),
                                        ("tethered_start", ["goto goal"])])
def test_tow_presets_need_the_rope_steps(asset_dir, tmp_path, name, demo):
    env = without(TOW / f"{name}.yaml", tmp_path, demo)
    solver = ScriptedSolver(env)
    wins = [run_episode(env, s, solver)["success"] for s in range(3)]
    env.close()
    assert not any(wins)


def walk_straight(env, seed: int) -> dict:
    """Walk at the goal at full speed, heading held, ignoring doors and ropes."""
    env.reset(seed=seed)
    goal = env.parts["goal"].xy
    while True:
        b = env.body_state
        d = goal - b.pos[:2]
        err = (math.atan2(d[1], d[0]) - b.yaw + math.pi) % (2 * math.pi) - math.pi
        move = np.array([1.0 if abs(err) < 0.4 else 0.0, 0.0, float(np.clip(2.5 * err, -1, 1))], dtype=np.float32)
        _, _, term, trunc, info = env.step(env.scripted_action(move, -1.0, -1.0, -1.0))
        if term or trunc:
            return info


@pytest.mark.slow
@pytest.mark.parametrize("name", NAMES)
def test_walking_straight_does_not_solve(asset_dir, name):
    env = PuzzleEnv(room=TOW / f"{name}.yaml")
    wins = [walk_straight(env, s)["success"] for s in range(3)]
    env.close()
    assert not any(wins)
