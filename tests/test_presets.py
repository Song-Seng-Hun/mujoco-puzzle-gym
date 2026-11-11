"""R3 presets: part presets under presets/parts/<category>/, assembly presets under
presets/puzzles/<topic>/, `preset:` loading and overriding, `mpg parts`."""

import json
import subprocess
import sys

import mujoco
import pytest
import yaml

from mujoco_puzzle_gym import catalog
from mujoco_puzzle_gym.bodies import make_body
from mujoco_puzzle_gym.check import check, check_order
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.room import Room, RoomError

#: Values a preset needs from the room to stand alone (channels, band partners, ends).
SOLO = {
    "logic/and": {"in": ["a", "b"]}, "logic/or": {"in": ["a", "b"]},
    "logic/xor": {"in": ["a", "b"]}, "logic/latch": {"in": ["a", "b"]},
    "logic/not": {"in": ["a"]}, "logic/delay": {"in": ["a"]}, "logic/hold": {"in": ["a"]},
    "logic/pulse": {"in": ["a"]}, "logic/counter": {"up": "a"},
    "logic/transmitter": {"in": ["a"]},
    "input/laser_beam": {"to": [0.5, -0.3]},
}
EXTRA = {   # a partner part some presets need
    "logic/receiver": {"name": "tx", "preset": "logic/transmitter", "in": ["a"]},
}


def solo_room(tmp_path, preset, **over):
    entry = {"name": "p", "preset": preset, "pos": [0.5, 0.3], **SOLO.get(preset, {}), **over}
    parts = [{"name": "a", "preset": "input/floor_oneshot", "pos": [-0.8, 0.6]},
             {"name": "b", "preset": "input/floor_oneshot", "pos": [-0.8, -0.6]},
             entry, {"name": "goal", "type": "goal", "pos": [1.0, -0.6], "radius": 0.1}]
    if preset in EXTRA:
        parts.append({**EXTRA[preset], "pos": [0.0, 0.0]})
    raw = {"name": "solo", "size": [2.6, 1.8], "spawn": {"pos": [-1.0, 0.0], "yaw": 0.0},
           "parts": parts}
    path = tmp_path / "solo.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


def test_preset_folders_follow_the_spec():
    names = catalog.part_preset_names()
    assert len(names) >= 30
    assert {n.split("/")[0] for n in names} <= set(catalog.CATEGORIES)
    assert {"input/floor_oneshot", "input/floor_momentary", "input/floor_toggle",
            "logic/and", "device/oneshot_door", "trap/slippery_floor"} <= set(names)
    assert {"puzzles/coins/coin_lock", "puzzles/ball/bowling", "puzzles/ball/ball_ramp",
            "puzzles/ball/color_sort", "puzzles/ball/mini_soccer"} <= set(catalog.puzzle_preset_names())


@pytest.mark.parametrize("preset", catalog.part_preset_names())
def test_every_part_preset_builds_alone(asset_dir, tmp_path, preset):
    room = Room.load(solo_room(tmp_path, preset))
    parts = room.make_parts()
    assert parts["p"].type_name == catalog.load_part_preset(preset)["type"]
    room.build_spec(make_body(), parts).compile()


def test_override_beats_preset_and_is_resolved(tmp_path):
    room = Room.load(solo_room(tmp_path, "input/floor_toggle", size=[0.4, 0.3]))
    p = room.make_parts()["p"]
    assert p.cfg["kind"] == "toggle" and p.cfg["size"] == [0.4, 0.3]
    entry = {e["name"]: e for e in room.resolved_parts()}["p"]
    assert entry["preset"] == "input/floor_toggle" and entry["type"] == "button"
    assert entry["size"] == [0.4, 0.3] and entry["kind"] == "toggle"
    assert entry["max_height"] == 0.2                   # schema default filled in


def test_resolved_task_keeps_overrides(quick_run):
    task = yaml.safe_load((quick_run / "task.resolved.yaml").read_text())
    parts = {e["name"]: e for e in task["room_parts"]}
    assert parts["switch_a"]["type"] == "switch" and parts["switch_a"]["size"] == [0.3, 0.3]
    assert parts["door_b"]["height"] == 0.25


def test_out_of_range_override_fails_with_range(asset_dir, tmp_path):
    res = check(solo_room(tmp_path, "device/sliding_door", width=5.0), static_only=True)
    assert res["ok"] is False and res["failed"] == ["values"]
    assert "width = 5.0 m is out of range [0.1, 2]" in res["values"]["reason"]


@pytest.mark.parametrize("preset, needle", [
    ("input/nope", "no part preset 'input/nope'; input presets: ["),
    ("gadget/x", "with a category in"),
    ("../x/y", "write `<category>/<name>`"),
])
def test_bad_preset_names_fail_values(tmp_path, preset, needle):
    with pytest.raises(RoomError) as err:
        Room.load(solo_room(tmp_path, preset))
    assert err.value.stage == "values" and needle in str(err.value)


def test_preset_type_must_match(tmp_path):
    with pytest.raises(RoomError, match="preset 'input/floor_toggle' is a 'button', not a 'door'"):
        Room.load(solo_room(tmp_path, "input/floor_toggle", type="door"))


def test_m1_switch_is_oneshot_button_alias(tmp_path):
    raw = {"name": "r", "size": [2.0, 2.0], "spawn": {"pos": [-0.8, 0.0]},
           "parts": [{"name": "s", "type": "switch", "pos": [0, 0]},
                     {"name": "m", "type": "switch", "pos": [0.5, 0], "latch": False},
                     {"name": "goal", "type": "goal", "pos": [0.8, 0.8]}]}
    parts = Room.from_dict(raw).make_parts()
    assert parts["s"].kind_ == "oneshot" and parts["s"].trigger == "body"
    assert parts["m"].kind_ == "momentary" and parts["s"].role == "switch"
    oneshot = catalog.load_part_preset("input/floor_oneshot")
    assert (oneshot["kind"], oneshot["trigger"]) == ("oneshot", "body")


# ---- assembly presets -----------------------------------------------------------------------
def test_coin_lock_assembly_preset(asset_dir, tmp_path):
    raw = {"preset": "puzzles/coins/coin_lock", "name": "coins_here", "size": [2.4, 1.8],
           "walls": [{"from": [0.3, -0.9], "to": [0.3, -0.28]}, {"from": [0.3, 0.28], "to": [0.3, 0.9]}],
           "spawn": {"pos": [-0.9, -0.7], "yaw": 0.0},
           "parts": [{"name": "lock", "need": 2},                    # update a preset part
                     {"name": "goal", "type": "goal", "pos": [0.85, 0.0]}]}
    path = tmp_path / "r.yaml"
    path.write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(room=path)
    assert env.room.name == "coins_here"
    assert env.parts["lock"].target == 2 and env.parts["lock_door"].type_name == "door"
    assert check_order(env)["order"] == ["coins", "lock_door", "goal"]
    env.close()


def test_ball_preset_as_room_base(asset_dir, tmp_path):
    raw = {"preset": "puzzles/ball/bowling", "name": "my_bowling",
           "parts": [{"name": "ball", "mass": 0.08}, {"name": "pin_c", "preset": "object/bowling_pin",
                                                      "pos": [-0.27, 0.6]}]}
    path = tmp_path / "r.yaml"
    path.write_text(yaml.safe_dump(raw))
    room = Room.load(path)
    parts = room.make_parts()
    assert room.name == "my_bowling" and len(room.demo) == 2
    assert parts["ball"].cfg["mass"] == 0.08 and parts["ball"].cfg["random"] == [0.05, 0.05]
    assert parts["pin_c"].base_xy.tolist() == [-0.27, 0.6]


def test_unknown_assembly_preset(tmp_path):
    with pytest.raises(RoomError, match="no assembly preset 'puzzles/ball/nope'"):
        Room.from_dict({"preset": "puzzles/ball/nope"})


# ---- mpg parts --------------------------------------------------------------------------------
def test_mpg_parts_json_lists_parts_presets_and_schemas():
    out = subprocess.run([sys.executable, "-m", "mujoco_puzzle_gym.cli", "parts", "--json"],
                         capture_output=True, text=True)
    res = json.loads(out.stdout)
    assert out.returncode == 0 and res["ok"]
    assert {"switch", "button", "door", "gate", "counter", "ball"} <= set(res["parts"])
    width = res["parts"]["door"]["schema"]["width"]
    assert (width["unit"], width["min"], width["max"], width["default"]) == ("m", 0.1, 2.0, 0.5)
    assert res["presets"]["input/floor_oneshot"]["type"] == "button"
    assert "puzzles/coins/coin_lock" in res["assemblies"]


def test_mpg_parts_markdown_prints_tables():
    out = subprocess.run([sys.executable, "-m", "mujoco_puzzle_gym.cli", "parts", "--markdown"],
                         capture_output=True, text=True)
    assert out.returncode == 0 and out.stdout == catalog.markdown()
    assert "### `door`" in out.stdout and "| `input/floor_oneshot` | `button` |" in out.stdout
