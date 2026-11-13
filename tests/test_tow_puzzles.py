"""R31 tow puzzles: the `puzzles/tow/` presets and the pit fill check.

- the tow presets are listed, load and pass the static stages with no warnings, and
  their `demo:` uses the rope (`tow`, `drag`, `untow` or a button that unties);
- a box the `demo:` drags into a pit must be as tall as the pit is deep (top flush
  with the floor within 1 cm) and fit its opening (0-4 cm wider), else the static stage
  `parts` fails;
- slow: without the rope steps (or walking straight at the goal) the presets are not
  solved; `cleat_hold` without its handoff station (lift the door, walk back) and
  `npc_leash` by herding instead of towing are not solved either; `pit_bridge` without
  the drag or without untying is not solved; `overload` towed without unloading the
  cart is not solved (the rope snaps). Their 20-seed `mpg check` runs in
  test_examples.py (room presets); `overload` needs the grab actions (its task).
"""

import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym import catalog
from mujoco_puzzle_gym.check import check
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.floor import CATCH_DEPTH
from mujoco_puzzle_gym.solver import ScriptedSolver, run_episode

TOW = catalog.PRESET_ROOT / "puzzles/tow"
NAMES = ["sled_delivery", "drop_and_go", "chain_dropoff", "tethered_start", "cleat_hold", "npc_leash",
         "pit_bridge", "overload"]
TASKS = {"overload": Path(__file__).resolve().parents[1] / "examples/task_overload.yaml"}    # grab actions


def test_tow_presets_listed():
    assert {f"puzzles/tow/{n}" for n in NAMES} <= set(catalog.puzzle_preset_names())


@pytest.mark.parametrize("name", NAMES)
def test_tow_presets_pass_static_stages(asset_dir, name):
    res = check(TOW / f"{name}.yaml", task=TASKS.get(name), static_only=True)
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
    # the tallest cube (0.3 m) falls short of the catch plane depth: only a pit with `depth` can be filled
    assert not check(pit_room(tmp_path, 0.3), static_only=True)["parts"]["ok"]


def deep_pit_room(tmp_path, opening) -> "Path":
    raw = yaml.safe_load(pit_room(tmp_path, 0.3).read_text())
    for p in raw["parts"]:
        if p["name"] == "box":
            p.update({"handles": ["-x"], "handle_size": 0.005, "pos": [-1.6, 0.0]})
        if p["name"] == "pit":
            p.update({"size": opening, "depth": 0.3})
    raw["parts"] = [p if p["name"] != "rope" else {**p, "pos": [-1.2, 0.5], "length": 0.5} for p in raw["parts"]]
    path = tmp_path / "deep_pit.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


def test_box_dragged_into_a_deep_pit_must_fit_its_opening(asset_dir, tmp_path):
    """With `depth` the 0.3 m box is flush; the opening must be 0-4 cm wider than the box's
    largest horizontal size (0.305 m with its handle), each side."""
    assert check(deep_pit_room(tmp_path, [0.31, 0.345]), static_only=True)["parts"]["ok"]
    for opening in ([0.30, 0.345], [0.31, 0.35], [0.4, 0.4]):
        res = check(deep_pit_room(tmp_path, opening), static_only=True)
        assert not res["parts"]["ok"] and "must be 0-4 cm wider than the box (0.305 m across" in res["parts"]["reason"]


def test_cleat_hold_without_its_pulley_fails_static(tmp_path):
    """A rope door opens in the static order only with a pulley over it (cleat_hold passes above)."""
    raw = yaml.safe_load((TOW / "cleat_hold.yaml").read_text())
    for p in raw["parts"]:
        if p.get("type") == "rope":
            p.pop("via")
    out = tmp_path / "cleat_hold.yaml"
    out.write_text(yaml.safe_dump(raw))
    res = check(out, static_only=True)
    assert not res["parts"]["ok"] and "no rope that can tie to it has a `via` pulley" in res["parts"]["reason"]


# ---- slow: the rope steps are needed ------------------------------------------------------
def without(path, tmp_path, demo, drop=()):
    raw = yaml.safe_load(path.read_text())
    raw["demo"] = demo
    raw["parts"] = [p for p in raw["parts"] if p["name"] not in drop]
    out = tmp_path / path.name
    out.write_text(yaml.safe_dump(raw))
    return PuzzleEnv(room=out, task=TASKS.get(path.stem))


@pytest.mark.slow
@pytest.mark.parametrize("name, demo", [("sled_delivery", ["goto goal"]),
                                        ("drop_and_go", ["drag rope zone", "goto goal"]),
                                        ("chain_dropoff", ["drag front drop_a", "goto goal"]),
                                        ("tethered_start", ["goto goal"]),
                                        ("cleat_hold", ["goto goal"]),
                                        ("npc_leash", ["goto goal"]),
                                        ("npc_leash", ["herd bot zone", "goto goal"]),
                                        ("pit_bridge", ["untow rope", "goto goal"]),        # no drag: the pit stays open
                                        ("pit_bridge", ["drag rope pit", "goto goal"]),     # no untow: the rope holds
                                        ("overload", ["tow rope cart", "drag rope zone", "goto goal"])])   # loaded: snaps
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
    env = PuzzleEnv(room=TOW / f"{name}.yaml", task=TASKS.get(name))
    wins = [walk_straight(env, s)["success"] for s in range(3)]
    env.close()
    assert not any(wins)


# ---- review: demo legs and the towed end down a chain --------------------------------------
def legs_of(env, rope):
    from mujoco_puzzle_gym.parts.device.rope import reset_towed, tow_legs

    r = env.parts[rope]
    return [(np.round(a, 2).tolist(), np.round(b, 2).tolist(), t) for a, b, t in tow_legs(env, r, reset_towed(env, r))]


def test_chain_dropoff_legs(asset_dir):
    env = PuzzleEnv(room=TOW / "chain_dropoff.yaml")
    env.reset(seed=0)
    try:
        front, rear = legs_of(env, "front"), legs_of(env, "rear")
    finally:
        env.close()
    # front tows cart_a past drop_b (the rear rope's drag moves the body too) and on to drop_a,
    # where drop_a unties it: no towing to the goal
    assert front == [([-2.1, 0.0], [-0.6, 0.35], "cart_a"), ([-0.6, 0.35], [0.6, -0.35], "cart_a")]
    assert rear == [([-2.1, 0.0], [-0.6, 0.35], "cart_b")]      # untied at drop_b


def test_reversed_chain_order_picks_the_far_cart(asset_dir, tmp_path):
    from mujoco_puzzle_gym.parts.device.rope import demo_far_end, reset_towed

    raw = yaml.safe_load((TOW / "chain_dropoff.yaml").read_text())
    for p in raw["parts"]:
        if p["name"] == "rear":                     # written cart_b -> cart_a, untied at end a
            p.update({"a": "cart_b", "b": "cart_a", "detach_a": "drop_b"})
            p.pop("detach_b")
    path = tmp_path / "chain_reversed.yaml"
    path.write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(room=path)
    env.reset(seed=0)
    try:
        rear = env.parts["rear"]
        assert reset_towed(env, rear) == "cart_b" and demo_far_end(env, rear) == "cart_b"
        assert legs_of(env, "rear") == [([-2.1, 0.0], [-0.6, 0.35], "cart_b")]
    finally:
        env.close()


@pytest.mark.slow
def test_cleat_hold_needs_the_handoff(asset_dir, tmp_path):
    """Lifting the door by walking to the anchor and walking back to it still tied lets
    the door down in front of the duck (measured: shut, lift < 1 mm, within 0.2 m)."""
    env = without(TOW / "cleat_hold.yaml", tmp_path, ["goto post", "goto goal"], drop=("station",))
    solver = ScriptedSolver(env)
    door = env.parts["door"]
    runs, lifted = [], []
    for s in range(3):
        runs.append(run_episode(env, s, solver)["success"])
        lifted.append(door.lift(env.data))
    env.close()
    assert not any(runs) and max(lifted) < door.clearance
