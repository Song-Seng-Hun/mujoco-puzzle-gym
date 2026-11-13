"""R24: example rooms in examples/rooms/ (six or more, one per kind so far), and the
assembly presets that are whole rooms. Each passes `mpg check` (slow); the
example rooms' 20-seed check is in test_check2.py. Slow: the scripted solution of
the laundry room (R28) and the wall button room (R19) renders to an MP4."""

from pathlib import Path

import pytest
import yaml

from mujoco_puzzle_gym import catalog
from mujoco_puzzle_gym.check import check
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.evaluate import write_video
from mujoco_puzzle_gym.solver import ScriptedSolver

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
         "roller_rink": "addon station", "roller_route": "roller mechanics", "tow_yard": "tow rope"}
#: Example rooms whose task adds actions beyond `move` (R19).
TASKS = {r: ROOT / f"examples/task_{r}.yaml" for r in ("cube_plate", "cube_socket", "laundry")}
#: Room presets whose `demo:` needs task actions beyond `move` (R30).
PRESET_TASKS = {f"puzzles/roller/{n}": ROOT / f"examples/task_{n}.yaml" for n in ("glide_under", "roll_delivery")}
PRESET_TASKS["puzzles/tow/overload"] = ROOT / "examples/task_overload.yaml"
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
    env = PuzzleEnv(task=TASKS.get(room), room=ROOT / f"examples/rooms/{room}.yaml",
                    render_mode="rgb_array")
    solver = ScriptedSolver(env)
    env.reset(seed=seed)
    solver.reset()
    frames = [env.render()]
    info: dict = {}
    while True:
        _, _, term, trunc, info = env.step(solver.act())
        frames.append(env.render())
        if term or trunc:
            break
    write_video(frames, out, fps=env.hz)
    env.close()
    return info


@pytest.mark.slow
@pytest.mark.parametrize("room", ["laundry", "wall_button"])
def test_demo_video_of_the_scripted_solution(asset_dir, tmp_path, room):
    """R28 (laundry) and R19 (wall button): a demo video of the solved room."""
    out = tmp_path / f"{room}.mp4"
    info = render_solution(room, out)
    assert info["success"] and not info["fallen"]
    assert out.stat().st_size > 10_000
