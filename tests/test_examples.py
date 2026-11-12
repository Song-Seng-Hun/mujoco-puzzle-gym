"""R24: example rooms in examples/rooms/ (six or more, one per kind so far), and the
assembly presets that are whole rooms. Each passes `mpg check` (slow); the
example rooms' 20-seed check is in test_check2.py."""

from pathlib import Path

import pytest
import yaml

from mujoco_puzzle_gym import catalog
from mujoco_puzzle_gym.check import check

ROOT = Path(__file__).resolve().parents[1]
ROOMS = sorted((ROOT / "examples/rooms").glob("*.yaml"))
#: R24 kinds built so far (cubes: E3).
KINDS = {"logic_lock": "buttons and logic", "doors": "doors and trapdoors",
         "four_flags": "four checkpoints", "coin_room": "coins", "mini_soccer": "ball",
         "two_rooms_map": "letter map", "stairs_map": "letter map",
         "lift_spring": "platforms, springs and setters", "trap_run": "traps", "herd": "npc",
         "warp_hop": "warps"}
ROOM_PRESETS = [n for n in catalog.puzzle_preset_names()
                if "map" in (r := yaml.safe_load((catalog.PRESET_ROOT / f"{n}.yaml").read_text()))
                or "size" in r]


def test_six_or_more_example_rooms():
    assert len(ROOMS) >= 6
    assert set(KINDS) <= {p.stem for p in ROOMS}


def test_four_flags_has_four_ordered_flags():
    raw = yaml.safe_load((ROOT / "examples/rooms/four_flags.yaml").read_text())
    orders = [p["order"] for p in raw["parts"] if p.get("preset") == "input/checkpoint_flag"]
    assert orders == [1, 2, 3, 4]


@pytest.mark.parametrize("room", ["logic_lock", "doors", "four_flags"])
def test_new_example_rooms_use_presets(room):
    raw = yaml.safe_load((ROOT / f"examples/rooms/{room}.yaml").read_text())
    assert sum("preset" in p for p in raw["parts"]) >= 3


@pytest.mark.slow
@pytest.mark.parametrize("name", ROOM_PRESETS)
def test_room_presets_pass_check(asset_dir, tmp_path, name):
    res = check(catalog.PRESET_ROOT / f"{name}.yaml", snap_dir=tmp_path)
    assert res["ok"], res
    assert res["solver"]["successes"] >= 18


@pytest.mark.slow
def test_four_flags_check_writes_flag_snapshots(asset_dir, tmp_path):
    res = check(ROOT / "examples/rooms/four_flags.yaml", snap_dir=tmp_path)
    assert res["ok"], res
    assert sorted(res["solver"]["checkpoints"]) == ["flag_1", "flag_2", "flag_3", "flag_4"]
    assert all(Path(p).is_file() for p in res["solver"]["checkpoints"].values())
