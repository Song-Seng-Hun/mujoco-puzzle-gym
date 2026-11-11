"""R20: `mpg check` in two phases (static, dynamic), the `demo:` sub-goal
executor, the four broken rooms, checkpoint snapshots, and the planner keeping
off trapdoors that open. The 20-seed dynamic stage of every example room is slow."""

import json
import pickle
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym.check import check, check_overlap
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.logic import free_mask, solution_order
from mujoco_puzzle_gym.room import Room, RoomError
from mujoco_puzzle_gym.snapshot import Snapshot
from mujoco_puzzle_gym.solver import ScriptedSolver, run_episode

ROOT = Path(__file__).resolve().parents[1]
SWITCH_DOOR = ROOT / "examples/switch_door.yaml"
MINI_SOCCER = ROOT / "examples/rooms/mini_soccer.yaml"
EXAMPLES = [SWITCH_DOOR, *sorted((ROOT / "examples/rooms").glob("*.yaml"))]


def write_room(tmp_path, raw, name="room.yaml") -> Path:
    path = tmp_path / name
    path.write_text(yaml.safe_dump(raw))
    return path


def switch_door(**changes) -> dict:
    raw = yaml.safe_load(SWITCH_DOOR.read_text())
    raw.update(changes)
    return raw


# ---- the four broken rooms ------------------------------------------------------------
@pytest.mark.parametrize("room, stage, needle", [
    ("channel_cut", "channels", "reads channel 'lever_a' that no part writes"),
    ("stateless_loop", "channels", "signal loop through stateless parts only"),
    ("button_unreachable", "order", "unreachable: ['button']"),
    ("value_out_of_range", "values", "width = 5.0 m is out of range"),
])
def test_broken_rooms_fail_for_their_reason(asset_dir, room, stage, needle):
    res = check(ROOT / f"examples/broken/{room}.yaml", seeds=2, need=2)
    assert res["ok"] is False and res["static"]["ok"] is False
    assert res["failed"][0] == stage and res["static"]["failed"] == [stage]
    assert needle in res[stage]["reason"]
    assert res["solver"]["skipped"] and res["solver"]["successes"] == 0


def test_broken_room_through_cli(asset_dir):
    out = subprocess.run([sys.executable, "-m", "mujoco_puzzle_gym.cli", "check",
                          str(ROOT / "examples/broken/channel_cut.yaml"), "--json"],
                         capture_output=True, text=True)
    res = json.loads(out.stdout)
    assert out.returncode != 0 and res["failed"] == ["channels", "solver"]


# ---- static phase -----------------------------------------------------------------------
@pytest.mark.parametrize("room", EXAMPLES, ids=lambda p: p.stem)
def test_example_rooms_pass_static_phase(asset_dir, room):
    res = check(room, static_only=True)
    assert res["ok"] and res["static"] == {"ok": True, "failed": []}, res
    assert res["dynamic"]["skipped"] and "solver" not in res


def test_overlapping_parts_fail_static_phase(asset_dir, tmp_path):
    raw = yaml.safe_load(MINI_SOCCER.read_text())
    raw["parts"].insert(1, {"name": "ball2", "type": "ball", "pos": [-0.33, 0.05]})
    raw["parts"][2]["pos"] = [-0.35, 0.86]                  # net back wall into the outer wall
    env = PuzzleEnv(room=write_room(tmp_path, raw))
    res = check_overlap(env)
    assert not res["ok"]
    assert "parts 'ball' and 'ball2' overlap" in res["reason"]
    assert "part 'net' overlaps a wall" in res["reason"]
    env.close()


# ---- demo: parsing -------------------------------------------------------------------------
@pytest.mark.parametrize("demo, needle", [
    (["jump goal"], "unknown sub-goal 'jump'"),
    (["goto nowhere"], "no part named 'nowhere'"),
    (["grab cube"], "comes with cubes"),
    (["wait soon"], "`wait <seconds>`"),
    (["press door_b"], "cannot press 'door_b'"),
    (["push goal to door_b"], "'goal' is not a ball"),
])
def test_bad_demo_rejected(tmp_path, demo, needle):
    path = write_room(tmp_path, switch_door(demo=demo))
    with pytest.raises(RoomError, match="demo") as exc:
        Room.load(path)
    assert needle in str(exc.value) and exc.value.stage == "demo"
    res = check(path, static_only=True)
    assert res["failed"] == ["demo"] and needle in res["demo"]["reason"]


def test_demo_parsed(tmp_path):
    path = write_room(tmp_path, switch_door(demo=["press switch_a", "wait 0.5", "goto goal"]))
    assert Room.load(path).demo == [("press", "switch_a"), ("wait", 0.5), ("goto", "goal")]
    assert Room.load(MINI_SOCCER).demo == [("push", "ball", "net")]
    assert Room.load(SWITCH_DOOR).demo == []


# ---- demo: executor ------------------------------------------------------------------------
def test_demo_press_wait_goto(asset_dir, tmp_path):
    env = PuzzleEnv(room=write_room(tmp_path, switch_door(demo=["press switch_a", "wait 2",
                                                               "goto goal"])))
    solver = ScriptedSolver(env)
    env.reset(seed=1)
    solver.reset()
    pressed_at = waited = None
    for k in range(env.max_steps):
        _, _, term, trunc, info = env.step(solver.act())
        if pressed_at is None and env.parts["switch_a"].satisfied():
            pressed_at = k
        if waited is None and solver.step_i == 2:
            waited = k
        if term or trunc:
            break
    assert info["success"] and not info["fallen"]
    assert waited - pressed_at == pytest.approx(2 * env.hz, abs=2)
    env.close()


def test_demo_goto_waits_behind_closed_door(asset_dir, tmp_path):
    """`goto goal` with the door shut: the solver stands instead of walking into the wall."""
    env = PuzzleEnv(room=write_room(tmp_path, switch_door(demo=["goto goal"])))
    solver = ScriptedSolver(env)
    env.reset(seed=0)
    solver.reset()
    start = env.body_state.pos[:2].copy()
    for _ in range(int(3 * env.hz)):
        env.step(solver.act())
    assert np.linalg.norm(env.body_state.pos[:2] - start) < 0.1 and solver.step_i == 0
    env.close()


def test_demo_push_scores(asset_dir):
    env = PuzzleEnv(room=MINI_SOCCER)
    res = run_episode(env, 13)                 # the seed the old auto-pusher ran out of time on
    assert res["success"] and not res["fallen"]
    assert res["demo_step"] == 1
    env.close()


# ---- checkpoint snapshots ----------------------------------------------------------------------
def test_dynamic_stage_saves_checkpoint_snapshots(asset_dir, tmp_path):
    raw = switch_door(demo=["press f1", "press switch_a", "goto goal"])
    raw["parts"].append({"name": "f1", "type": "checkpoint", "order": 1, "pos": [-0.6, -0.5]})
    res = check(write_room(tmp_path, raw), seeds=1, need=1, snap_dir=tmp_path / "runs")
    assert res["ok"], res
    path = Path(res["solver"]["checkpoints"]["f1"])
    assert path == tmp_path / "runs/switch_door/checkpoints/f1.pkl"
    snap = pickle.loads(path.read_bytes())
    assert isinstance(snap, Snapshot) and snap.meta["checkpoint"] == "f1"
    assert snap.parts["f1"]["reached"]


# ---- trapdoors on the planner grid ---------------------------------------------------------
TRAP_ROOM = {
    "name": "trap_path", "size": [2.4, 1.8], "wall": {"thickness": 0.06, "height": 0.25},
    "walls": [{"from": [0.3, -0.9], "to": [0.3, -0.28]}, {"from": [0.3, 0.28], "to": [0.3, 0.9]}],
    "spawn": {"pos": [-0.75, -0.3], "yaw": 0.0, "random": [0.05, 0.05, 0.3]},
    "parts": [
        {"name": "button", "type": "button", "kind": "oneshot", "pos": [-0.6, 0.6], "size": [0.25, 0.25]},
        # opens with the door: right across the straight way from the button to the door
        {"name": "td", "type": "trapdoor", "pos": [-0.15, 0.25], "size": [0.3, 0.4], "in": ["button"]},
        {"name": "door_b", "type": "door", "pos": [0.3, 0.0], "width": 0.5, "axis": "y", "in": ["button"]},
        {"name": "goal", "type": "goal", "pos": [0.85, 0.0], "radius": 0.15},
    ],
}


def test_planner_keeps_off_open_trapdoor(asset_dir, tmp_path):
    env = PuzzleEnv(room=write_room(tmp_path, TRAP_ROOM))
    cell = env.grid.cell((-0.15, 0.25))
    assert free_mask(env.grid, env.walls, env.parts, set())[cell]            # closed: floor
    assert not free_mask(env.grid, env.walls, env.parts, {"button"})[cell]   # opens: a hole
    assert solution_order(env.grid, env.walls, env.parts, (-0.75, -0.3))["order"] == \
        ["button", "door_b", "goal"]
    for seed in range(3):                                   # the solver walks round it
        res = run_episode(env, seed)
        assert res["success"] and not res["fallen"], res
    env.close()


def test_trapdoor_over_the_only_way_fails_order(asset_dir, tmp_path):
    raw = yaml.safe_load(yaml.safe_dump(TRAP_ROOM))
    raw["parts"][1] = {"name": "td", "type": "trapdoor", "pos": [0.65, 0.0], "size": [0.6, 1.8],
                       "in": ["button"]}                    # across the whole right room
    env = PuzzleEnv(room=write_room(tmp_path, raw))
    res = solution_order(env.grid, env.walls, env.parts, (-0.75, -0.3))
    assert not res["ok"]
    env.close()


# ---- dynamic phase (slow) ------------------------------------------------------------------
@pytest.mark.slow
@pytest.mark.parametrize("room", EXAMPLES, ids=lambda p: p.stem)
def test_example_rooms_pass_check(asset_dir, room):
    res = check(room)
    assert res["ok"] and res["static"]["ok"] and res["dynamic"]["ok"], res
    assert res["solver"]["seeds"] == 20 and res["solver"]["successes"] >= 18
