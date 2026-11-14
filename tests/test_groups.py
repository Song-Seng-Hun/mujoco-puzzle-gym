"""R35 part 4b-2: the structure layer's groups -- a part drawn in several shapes
(`type`) or numbers (`count`) -- and the storage shelf the members that are not
drawn wait on.

Every candidate and every copy is built when the room loads, because MuJoCo cannot
add a body while running; the draw only moves them on and off the shelf. This file
is about what the room expands into, what a draw keeps, and that a part off stage
is invisible to the plan, the observations and the physics.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym.check import check
from mujoco_puzzle_gym.env import SHELF_X, PuzzleEnv
from mujoco_puzzle_gym.room import RoomError

ROOT = Path(__file__).resolve().parents[1]
ROOM = {
    "name": "grp", "size": [3.0, 2.0], "wall": {"thickness": 0.06, "height": 0.25},
    "walls": [{"from": [0.5, -1.0], "to": [0.5, -0.28]},
              {"from": [0.5, 0.28], "to": [0.5, 1.0]}],
    "spawn": {"pos": [-1.2, 0.0], "yaw": 0.0},
    "parts": [{"name": "trigger", "preset": "input/floor_oneshot", "pos": [-0.7, 0.55]},
              {"name": "door", "preset": "device/sliding_door", "pos": [0.5, 0.0],
               "in": ["trigger"]},
              {"name": "goal", "type": "goal", "pos": [1.1, 0.0], "radius": 0.15}],
}
TYPES = {"type": {"candidates": ["input/floor_oneshot", "input/floor_toggle"]}}
SPOTS = [[-0.3, -0.5], [0.15, -0.55], [-0.75, -0.45]]


def room_file(tmp_path, *, parts=None, name="grp.yaml", add=(), **top):
    raw = yaml.safe_load(yaml.safe_dump(ROOM))
    for pname, block in (parts or {}).items():
        next(c for c in raw["parts"] if c["name"] == pname).update(block)
    raw["parts"] += [dict(c) for c in add]
    raw.update(top)
    path = tmp_path / name
    path.write_text(yaml.safe_dump(raw))
    return path


def env_of(tmp_path, *, task=None, **kw) -> PuzzleEnv:
    path = room_file(tmp_path, **kw)
    return PuzzleEnv(task={"room": str(path), **(task or {})}, room=path)


def crates(count=(0, 2), spots=None):
    return {"name": "crate", "preset": "beak/cube_weight", "pos": [-0.3, -0.5],
            "randomize": {"count": list(count),
                          "pos": {"candidates": [list(s) for s in (spots or SPOTS)]}}}


# ---- what the room expands into -----------------------------------------------------
def test_a_type_group_becomes_one_part_per_candidate(tmp_path):
    env = env_of(tmp_path, parts={"trigger": {"randomize": TYPES}})
    assert [n for n in env.parts if n.startswith("trigger")] == ["trigger#0", "trigger#1"]
    group = env.room.groups[0]
    assert (group.kind, group.members, group.readers) == \
        ("type", ["trigger#0", "trigger#1"], ["door"])
    assert env.parts["trigger#1"].type_name == "button"       # built from its own preset


def test_a_count_group_becomes_one_copy_per_spot(tmp_path):
    """`hi` copies, and at least that many spots: the extra spots give the draw places
    to choose from."""
    env = env_of(tmp_path, add=[crates((0, 2))])
    assert [n for n in env.parts if n.startswith("crate")] == ["crate#1", "crate#2"]
    assert [list(np.round(env.parts[n].base_xy, 3)) for n in ("crate#1", "crate#2")] \
        == SPOTS[:2]                                           # the nominal room: one per spot


# ---- what a draw keeps --------------------------------------------------------------
def test_the_drawn_candidate_is_the_one_in_the_room(tmp_path):
    env = env_of(tmp_path, parts={"trigger": {"randomize": TYPES}})
    seen = set()
    for s in range(20):
        env.reset(seed=s)
        drawn = env.layout["used"]["trigger"]
        seen.add(drawn[0])
        assert len(drawn) == 1
        assert env.parts["door"].inputs == drawn           # the reader follows the draw
        assert not env.parts[drawn[0]].parked
        other = next(n for n in ("trigger#0", "trigger#1") if n != drawn[0])
        assert env.parts[other].parked and env.parts[other].xy[0] >= SHELF_X
        assert env.plan["ok"] and env.plan["order"][0] == drawn[0]
    assert seen == {"trigger#0", "trigger#1"}


def test_a_count_draw_fills_that_many_different_spots(tmp_path):
    env = env_of(tmp_path, add=[crates()])
    seen = set()
    for s in range(30):
        env.reset(seed=s)
        keep = env.layout["used"]["crate"]
        seen.add(len(keep))
        assert 0 <= len(keep) <= 2
        places = [tuple(np.round(env.layout["parts"][n], 3)) for n in keep]
        assert len(set(places)) == len(places)             # never two copies on one spot
        assert all(not env.parts[n].parked for n in keep)
        assert all(env.parts[n].parked for n in ("crate#1", "crate#2") if n not in keep)
    assert seen == {0, 1, 2}


def test_the_nominal_room_is_the_first_candidate_and_the_fewest_copies(tmp_path):
    env = env_of(tmp_path, parts={"trigger": {"randomize": TYPES}}, add=[crates((1, 3))])
    env.reset(seed=0, options={"randomize": "nominal"})
    assert not env.parts["trigger#0"].parked and env.parts["trigger#1"].parked
    assert env.parts["door"].inputs == ["trigger#0"]
    assert [n for n in ("crate#1", "crate#2", "crate#3")
            if not env.parts[n].parked] == ["crate#1"]


def test_the_structure_strength_can_pin_the_groups(tmp_path):
    env = env_of(tmp_path, parts={"trigger": {"randomize": TYPES}}, add=[crates((1, 3))],
                 task={"randomize": {"structure": 0.0}})
    for s in range(5):
        env.reset(seed=s)
        assert env.layout["used"] == {}
        assert env.parts["door"].inputs == ["trigger#0"] and env.parts["trigger#1"].parked


def test_wide_draws_the_evaluation_only_candidate(tmp_path):
    env = env_of(tmp_path, parts={"trigger": {"randomize": TYPES}},
                 randomize={"holdout": {"trigger": ["input/floor_toggle"]}})
    for s in range(10):
        env.reset(seed=s)
        assert env.layout["used"]["trigger"] == ["trigger#0"]      # training never sees it
    env.reset(seed=0, options={"randomize": "wide"})
    assert env.layout["used"]["trigger"] == ["trigger#1"]
    assert env.parts["door"].inputs == ["trigger#1"]


# ---- a part off stage is not there --------------------------------------------------
def test_a_shelved_part_blocks_nothing_and_writes_nothing(tmp_path):
    """The plan, the observations and the collisions all leave it out."""
    from rooms import stand_at, tick

    env = env_of(tmp_path, parts={"trigger": {"randomize": TYPES}},
                 task={"obs": ["part_pos", "part_state", "part_used", "part_group"]})
    env.reset(seed=0)
    off = next(n for n in ("trigger#0", "trigger#1") if env.parts[n].parked)
    assert env.parts[off].xy[0] >= SHELF_X
    gids = env.parts[off].own_geoms(env.model)
    assert gids and all(env.model.geom_contype[g] == 0 for g in gids)
    assert off not in env.plan["order"]
    stand_at(env, *env.parts[off].xy)                 # standing on it changes nothing
    tick(env)
    assert env.signals.get(off) == 0.0


def test_part_used_and_part_group_report_the_draw(tmp_path):
    env = env_of(tmp_path, parts={"trigger": {"randomize": TYPES}}, add=[crates()],
                 task={"obs": ["part_used", "part_group"]})
    names = list(env.parts)
    for s in range(8):
        obs, _ = env.reset(seed=s)
        used, group = obs[:len(names)], obs[len(names):]
        assert list(used) == [0.0 if env.parts[n].parked else 1.0 for n in names]
        assert group.shape == (4,)                    # 2 candidates + 2 copies
        assert group[:2].sum() == 1.0
        assert group[2:].sum() == len(env.layout["used"]["crate"])


# ---- what a room may declare --------------------------------------------------------
@pytest.mark.parametrize("block, bad", [
    ({"type": {"candidates": ["input/floor_oneshot"]}}, "at least two"),
    ({"type": {"candidates": "input/floor_oneshot"}}, "with at least two"),
    ({"type": {"candidates": ["input/floor_oneshot", 7]}}, "a preset or type name"),
    ({"type": {"candidates": ["input/floor_oneshot", "input/floor_toggle"]},
      "count": [1, 2]}, "not both"),
    ({"count": [1, 2]}, "a spot each"),
    ({"count": [2, 1], "pos": {"candidates": SPOTS}}, "0 <= lo <= hi"),
    ({"count": [0, 4], "pos": {"candidates": SPOTS}}, "at least 4 spots"),
])
def test_bad_group_declarations_are_refused(tmp_path, block, bad):
    with pytest.raises(RoomError) as exc:
        env_of(tmp_path, parts={"trigger": {"randomize": block}})
    assert bad in str(exc.value)


def test_a_counted_part_whose_channel_is_read_is_refused(tmp_path):
    """Nothing can read "the group": which copy would it be?"""
    with pytest.raises(RoomError) as exc:
        env_of(tmp_path, parts={"trigger": {"randomize": {
            "count": [1, 2], "pos": {"candidates": SPOTS}}}})
    assert "read its channel" in str(exc.value)


def test_a_part_that_cannot_be_shelved_is_refused(tmp_path):
    """A gate closes a gap in a wall: drawing it away would leave a hole."""
    with pytest.raises(RoomError) as exc:
        env_of(tmp_path, parts={"door": {"randomize": {
            "count": [1, 2], "pos": {"candidates": [[0.5, 0.0], [0.5, 0.1]]}}}})
    assert "cannot be put on the storage shelf" in str(exc.value)
    assert "gate closes a gap in a wall" in str(exc.value)


@pytest.mark.parametrize("hold, bad", [
    ({"trigger": ["input/nope"]}, "`holdout` names one of the candidates"),
    ({"trigger": ["input/floor_oneshot", "input/floor_toggle"]}, "cannot take every candidate"),
    ({"trigger": []}, "`holdout` is a list of candidate names"),
])
def test_bad_group_holdouts_are_refused(tmp_path, hold, bad):
    with pytest.raises(RoomError) as exc:
        env_of(tmp_path, parts={"trigger": {"randomize": TYPES}}, randomize={"holdout": hold})
    assert bad in str(exc.value)


def test_a_count_group_has_no_holdout(tmp_path):
    with pytest.raises(RoomError) as exc:
        env_of(tmp_path, add=[crates()], randomize={"holdout": {"crate": ["beak/cube_weight"]}})
    assert "no evaluation-only candidates" in str(exc.value)


# ---- `mpg check` --------------------------------------------------------------------
def test_a_group_room_passes_every_static_stage(asset_dir, tmp_path):
    path = room_file(tmp_path, parts={"trigger": {"randomize": TYPES}}, add=[crates()])
    res = check(path, static_only=True)
    assert res["ok"], {k: res[k] for k in res["failed"]}
    assert res["resample"]["ratio"] == 0.0


def test_the_example_room_passes_the_static_phase(asset_dir):
    res = check(ROOT / "examples/rooms/mixed_stock.yaml",
                task=ROOT / "examples/task_mixed_stock.yaml", static_only=True)
    assert res["ok"], {k: res[k] for k in res["failed"]}
    assert not res["resample"].get("warnings")


@pytest.mark.slow
def test_the_scripted_answer_solves_both_candidates(asset_dir):
    """One episode per candidate: the answer presses whichever plate is in the room."""
    from mujoco_puzzle_gym.solver import ScriptedSolver, run_episode

    env = PuzzleEnv(task=ROOT / "examples/task_mixed_stock.yaml")
    seen = set()
    for seed in range(6):
        res = run_episode(env, seed, ScriptedSolver(env))
        drawn = env.layout["used"]["trigger"][0]
        assert res["success"], (seed, drawn, res)
        seen.add(drawn)
    assert seen == {"trigger#0", "trigger#1"}
