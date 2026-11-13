"""R34 part 3c: the circuit graph, the `circuit` check stage, the `break` / `repair`
sub-goals and the `puzzles/circuit/` presets.

The part mechanics (the four causes, wires, relays, the destroyer, `repair_with`) are
in test_breakable.py; this file is about what `mpg check` makes of a whole room and
what the scripted solver does with it.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from mujoco_puzzle_gym import catalog
from mujoco_puzzle_gym.check import check
from mujoco_puzzle_gym.circuit import affected, break_edges, demo_breaks, lost_channels
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.logic import plan_channels
from mujoco_puzzle_gym.room import RoomError, parse_demo

from rooms import open_room
from test_doors import Source  # noqa: F401  (registers t_signal)

ROOT = Path(__file__).resolve().parents[1]
PRESETS = sorted((catalog.PRESET_ROOT / "puzzles/circuit").glob("*.yaml"))
#: Circuit presets whose `demo:` needs the beak (`head`, `grab`).
TASKS = {n: ROOT / f"examples/task_{n}.yaml"
         for n in ("peck_cable", "wrong_wire", "fuse_repair", "jam_the_turret", "sabotage_order")}


def chain_room(tmp_path, name="chain", **relay):
    """source -> wire -> relay -> not gate -> door, plus a second reader of the channel."""
    parts = [{"name": "sig", "type": "source", "mode": "const", "value": 1.0, "pos": [-1.2, 1.2]},
             {"name": "link", "type": "relay", "in": ["sig"], "pos": [0.4, 0.4],
              "breakable": True, "by": ["destroyer"], **relay},
             {"name": "cable", "type": "wire", "carries": "sig", "to": ["link"], "pos": [1.4, 0.0],
              "breakable": True, "by": ["peck"], "mount": "wall"},
             {"name": "open", "type": "gate", "op": "not", "in": ["link"]},
             {"name": "door", "type": "door", "pos": [0.0, 0.0], "width": 0.5, "in": ["open"]},
             {"name": "other", "type": "door", "pos": [0.0, 1.0], "width": 0.5, "in": ["sig"]}]
    return open_room(tmp_path, parts, spawn=(-1.0, -1.0), name=name)


# ---- the graph ----------------------------------------------------------------------
def test_breaking_a_relay_reaches_everything_behind_it(tmp_path):
    env = chain_room(tmp_path)
    assert affected(env.parts, "link") == ["door", "open"]
    assert lost_channels(env.parts, "link") == ["link", "open"]


def test_a_cut_wire_reaches_only_its_own_readers(tmp_path):
    """The channel keeps its value; the wire takes it from the parts in its `to`, so
    the other reader of `sig` is not in the set."""
    env = chain_room(tmp_path)
    assert break_edges(env.parts)["cable"] == ["link"]
    assert "other" not in affected(env.parts, "cable")
    assert affected(env.parts, "cable") == ["door", "link", "open"]


def test_the_plan_takes_a_part_the_demo_breaks_as_broken(tmp_path):
    """A room that only opens once something is cut: the plan has to know (R6a)."""
    env = chain_room(tmp_path)
    assert plan_channels(env.parts, set())["open"] == 0.0        # the relay is whole: shut
    broken = demo_breaks(type("R", (), {"demo": [("break", "link", "destroyer")]}))
    assert broken == {"link"}
    assert plan_channels(env.parts, set(), broken)["open"] == 1.0
    assert plan_channels(env.parts, set(), {"cable"})["sig"] == 0.0   # a cut wire, too


# ---- the `circuit` stage ------------------------------------------------------------
def write_room(tmp_path, raw, name="room.yaml") -> Path:
    path = tmp_path / name
    path.write_text(yaml.safe_dump(raw))
    return path


def circuit_raw(demo, **relay):
    return {"name": "c", "size": [3.0, 2.0], "wall": {"thickness": 0.06, "height": 0.25},
            "walls": [{"from": [0.5, -1.0], "to": [0.5, -0.28]},
                      {"from": [0.5, 0.28], "to": [0.5, 1.0]}],
            "spawn": {"pos": [-1.2, 0.0], "yaw": 0.0},
            "parts": [
                {"name": "sig", "preset": "logic/source_const", "pos": [-1.3, 0.9], "body": False},
                {"name": "link", "preset": "logic/relay", "pos": [-0.3, 0.6], "in": ["sig"],
                 "by": ["destroyer"], **relay},
                {"name": "press", "type": "destroyer", "pos": [-0.3, 0.6], "in": ["knob"],
                 "targets": ["link"]},
                {"name": "knob", "preset": "input/floor_momentary", "pos": [-0.9, -0.5]},
                {"name": "open", "preset": "logic/not", "in": ["link"]},
                {"name": "door", "preset": "device/sliding_door", "pos": [0.5, 0.0], "in": ["open"]},
                {"name": "goal", "type": "goal", "pos": [1.1, 0.0], "radius": 0.15}],
            "demo": demo}


def test_the_circuit_stage_reports_the_graph(asset_dir, tmp_path):
    res = check(write_room(tmp_path, circuit_raw(["break link via destroyer", "goto goal"])),
                static_only=True)
    assert res["ok"], res
    assert res["circuit"]["graph"]["link"]["affected"] == ["door", "open"]


def test_breaking_a_part_that_fails_the_episode_is_refused(asset_dir, tmp_path):
    raw = circuit_raw(["break link via destroyer", "goto goal"], on_break="fail")
    res = check(write_room(tmp_path, raw), static_only=True)
    assert not res["circuit"]["ok"]
    assert "on_break: fail" in res["circuit"]["reason"]


def test_a_part_the_answer_needs_with_no_repair_warns(asset_dir, tmp_path):
    """Nothing breaks it in the demo, so the room leans on it staying whole."""
    raw = circuit_raw(["goto goal"])
    raw["parts"][5]["in"] = ["link"]            # the door needs the relay whole
    res = check(write_room(tmp_path, raw), static_only=True)
    warn = res["circuit"].get("warnings", [])
    assert any("no way to repair" in w for w in warn), res["circuit"]


def test_a_peck_break_needs_the_head_and_grab_actions(asset_dir, tmp_path):
    raw = circuit_raw(["break link via peck", "goto goal"], by=["peck"], height=0.2)
    res = check(write_room(tmp_path, raw), static_only=True)
    assert not res["actions"]["ok"]
    assert "`grab`" in res["actions"]["reason"] and "`head`" in res["actions"]["reason"]


# ---- the sub-goals ------------------------------------------------------------------
def demo_parts(tmp_path):
    return chain_room(tmp_path, name="subgoal").parts


@pytest.mark.parametrize("entry, bad", [
    ("break link", "write `break <part> via"),
    ("break link via nibble", "is not a cause"),
    ("break link via peck", "is not broken by 'peck'"),
    ("break open via destroyer", "cannot be broken"),
    ("repair link", "has no way to be repaired"),
    ("repair open", "is not breakable"),
    ("repair", "write `repair <part>`"),
])
def test_bad_break_sub_goals_are_refused(tmp_path, entry, bad):
    with pytest.raises(RoomError) as exc:
        parse_demo([entry], demo_parts(tmp_path))
    assert bad in str(exc.value)


def test_break_and_repair_sub_goals_parse(tmp_path):
    parts = chain_room(tmp_path, name="ok_subgoal", repair="sig").parts
    assert parse_demo(["break link via destroyer", "repair link"], parts) == \
        [("break", "link", "destroyer"), ("repair", "link")]


# ---- the presets ---------------------------------------------------------------------
def test_eight_circuit_presets_exist():
    names = {p.stem for p in PRESETS}
    assert names == {"cut_the_lock", "wrong_wire", "fuse_repair", "clock_door",
                     "press_relay", "jam_the_turret", "peck_cable", "sabotage_order"}


@pytest.mark.parametrize("room", PRESETS, ids=lambda p: p.stem)
def test_circuit_presets_pass_the_static_phase(asset_dir, room):
    res = check(room, task=TASKS.get(room.stem), static_only=True)
    assert res["ok"], res
    warn = res["circuit"].get("warnings", []) + res["parts"].get("warnings", [])
    assert not warn, warn


@pytest.mark.slow
@pytest.mark.parametrize("name", ["press_relay", "peck_cable"])
def test_the_scripted_answer_breaks_the_part(asset_dir, name):
    """One episode each of a remote break (the press) and a hand break (the beak)."""
    from mujoco_puzzle_gym.solver import ScriptedSolver, run_episode

    room = catalog.PRESET_ROOT / f"puzzles/circuit/{name}.yaml"
    env = PuzzleEnv(task=TASKS.get(name), room=room)
    res = run_episode(env, 0, ScriptedSolver(env))
    broken = [p.name for p in env.parts.values() if getattr(p, "broken", False)]
    assert res["success"] and broken, (res, broken)
