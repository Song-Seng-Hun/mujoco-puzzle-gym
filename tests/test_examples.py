"""R24: example rooms in examples/rooms/ (six or more, one per kind so far), and the
assembly presets that are whole rooms. Each passes `mpg check` (slow); the
example rooms' 20-seed check is in test_check2.py. Slow: the scripted solution of
the laundry room (R28) and the wall button room (R19) renders to an MP4, and
`mpg demo <room>` does the same from the CLI."""

import json
from pathlib import Path

import pytest
import yaml

from mujoco_puzzle_gym import catalog
from mujoco_puzzle_gym.check import check
from mujoco_puzzle_gym.cli import main
from mujoco_puzzle_gym.evaluate import render_solution as _render_solution

ROOT = Path(__file__).resolve().parents[1]
ROOMS = sorted((ROOT / "examples/rooms").glob("*.yaml"))
#: R24 kinds built so far.
KINDS = {"logic_lock": "buttons and logic", "doors": "doors and trapdoors",
         "four_flags": "four checkpoints", "coin_room": "coins", "mini_soccer": "ball",
         "two_rooms_map": "letter map", "stairs_map": "letter map",
         "lift_spring": "platforms, springs and setters", "trap_run": "traps", "herd": "npc",
         "warp_hop": "warps", "cube_plate": "cube and grab",
         "turret": "cannon turret and lever", "cube_socket": "key and socket",
         "wall_button": "beak wall button", "laundry": "laundry and basket",
         "roller_rink": "addon station", "roller_route": "roller mechanics", "tow_yard": "tow rope",
         "power_cut": "circuit, breaking and repair",
         "shuffle_switches": "randomized wiring",
         "mixed_stock": "randomized parts and counts",
         "room_roulette": "randomized rooms",
         "order_cue": "order reasoning, cue on", "order_blind": "order reasoning, cue off"}
#: Example rooms whose task adds actions beyond `move` (R19).
TASKS = {r: ROOT / f"examples/task_{r}.yaml"
         for r in ("cube_plate", "cube_socket", "laundry", "power_cut")}
#: Room presets whose `demo:` needs task actions beyond `move` (R30).
PRESET_TASKS = {f"puzzles/roller/{n}": ROOT / f"examples/task_{n}.yaml" for n in ("glide_under", "roll_delivery")}
PRESET_TASKS["puzzles/tow/overload"] = ROOT / "examples/task_overload.yaml"
#: R47 wide rooms: their task is what says which body they were sized for, so checking
#: them without it would check them with the wrong one.
PRESET_TASKS.update({f"puzzles/wide/{n}": ROOT / f"examples/task_wide_{n}.yaml"
                     for n in ("switch_door", "pit_detour")})
#: M4 R61: the tabletop room is for a body with no legs; its task is what says so.
PRESET_TASKS["puzzles/tabletop/reach_target"] = ROOT / "examples/task_reach_target.yaml"
PRESET_TASKS["puzzles/tabletop/pick_and_place"] = ROOT / "examples/task_pick_and_place.yaml"
#: M5 R75: the arm delegates its weight to an object; needs `head` and `grab`.
PRESET_TASKS["puzzles/tabletop/weigh_it_down"] = ROOT / "examples/task_weigh_it_down.yaml"
#: R34 circuit presets whose `demo:` pecks a cover or carries a fuse (`head`, `grab`).
PRESET_TASKS.update({f"puzzles/circuit/{n}": ROOT / f"examples/task_{n}.yaml"
                     for n in ("peck_cable", "wrong_wire", "fuse_repair", "jam_the_turret",
                               "sabotage_order")})
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
    res = check(catalog.PRESET_ROOT / f"{name}.yaml", task=PRESET_TASKS.get(name), snap_dir=tmp_path)
    assert res["ok"], res
    assert res["solver"]["successes"] >= 18


@pytest.mark.slow
def test_four_flags_check_writes_flag_snapshots(asset_dir, tmp_path):
    res = check(ROOT / "examples/rooms/four_flags.yaml", snap_dir=tmp_path)
    assert res["ok"], res
    assert sorted(res["solver"]["checkpoints"]) == ["flag_1", "flag_2", "flag_3", "flag_4"]
    assert all(Path(p).is_file() for p in res["solver"]["checkpoints"].values())


def render_solution(room: str, out: Path, seed: int = 0) -> dict:
    """The scripted solution of an example room, rendered to `out` (MP4)."""
    return _render_solution(ROOT / f"examples/rooms/{room}.yaml", out=out, seed=seed,
                            task=TASKS.get(room))


@pytest.mark.slow
@pytest.mark.parametrize("room", ["laundry", "wall_button"])
def test_demo_video_of_the_scripted_solution(asset_dir, tmp_path, room):
    """R28 (laundry) and R19 (wall button): a demo video of the solved room."""
    out = tmp_path / f"{room}.mp4"
    info = render_solution(room, out)
    assert info["success"] and not info["fallen"]
    assert out.stat().st_size > 10_000


@pytest.mark.slow
def test_mpg_demo_renders_a_room_solution(asset_dir, tmp_path, capsys):
    """R24: `mpg demo <room>` writes the scripted solution of any room, with no run folder."""
    out = tmp_path / "wall_button.mp4"
    assert main(["demo", str(ROOT / "examples/rooms/wall_button.yaml"),
                 "--out", str(out), "--json"]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["ok"] and result["success"] and not result["fallen"]
    assert result["room"] == "wall_button" and result["seed"] == 0
    assert Path(result["mp4"]) == out.resolve() and out.stat().st_size > 10_000


def test_mpg_demo_rejects_a_missing_room_and_a_stray_task(tmp_path, capsys):
    """The argument errors cost no simulation, so they are checked in the fast set."""
    assert main(["demo", str(tmp_path / "nope.yaml"), "--json"]) == 1
    assert "no such room file" in json.loads(capsys.readouterr().out)["error"]
    assert main(["demo", "--task", str(ROOT / "examples/task_ppo.yaml"), "--json"]) == 1
    assert "--task goes with a room file" in json.loads(capsys.readouterr().out)["error"]
