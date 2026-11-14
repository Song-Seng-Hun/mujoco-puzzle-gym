"""R21 + R35 part 4c-3: where a checkpoint segment starts when the room is drawn.

A later segment starts from its flag's snapshot, which `mpg check` saved in one
episode's room. That is fine while the room is fixed -- and the physics and body
layers are drawn again over it, so a run does not train on one frozen set of part
values. When the room draws its layout, its structure or its room, those few
snapshot layouts are what a policy would memorize, so the segment starts from a
freshly drawn episode with the room's own declared start on it
(`checkpoint_states:`). A flag with a declaration but no snapshot takes the
declaration too.
"""

from __future__ import annotations

import numpy as np
import pytest
import yaml
from flag_room import FLAGS, ROOM, STAND, save_snapshots, task_dict

from mujoco_puzzle_gym.curriculum import CurriculumError, begin_segment
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.room import Room, RoomError

#: The declared starts: the body stands on the flag, and the flags before it are reached.
STATES = {"f1": {"body": {"at": "f1", "yaw": 0.0}, "parts": {"f1": {"state": {"reached": True}}}},
          "f2": {"body": {"at": "f2", "yaw": 0.0},
                 "parts": {"f1": {"state": {"reached": True}},
                           "f2": {"state": {"reached": True}}}}}


def room_file(tmp_path, *, states=None, fan=False, drawn=False, name="flags.yaml") -> object:
    raw = yaml.safe_load(yaml.safe_dump(ROOM))
    if fan:          # a part value to draw (the physics layer)
        raw["parts"].insert(0, {"name": "fan", "preset": "trap/crosswind", "pos": [-1.0, 0.9],
                                "randomize": {"power": {"range": 0.4}}})
    if drawn:        # a place to draw (the layout layer)
        raw["parts"][-1]["randomize"] = {"pos": {"range": [0.0, 0.3]}}
    if states is not None:
        raw["checkpoint_states"] = states
    path = tmp_path / name
    path.write_text(yaml.safe_dump(raw))
    return path


def env_of(tmp_path, *, snaps=True, task=None, **kw) -> PuzzleEnv:
    room = room_file(tmp_path, **kw)
    env = PuzzleEnv(task=task_dict(room, tmp_path / "snaps", **(task or {})))
    if snaps:
        save_snapshots(env, tmp_path / "snaps")
    return env


# ---- what a room may declare --------------------------------------------------------
def test_a_declared_start_is_read_back(tmp_path):
    room = Room.load(room_file(tmp_path, states=STATES))
    assert sorted(room.checkpoint_states) == ["f1", "f2"]
    assert room.checkpoint_states["f1"]["body"] == {"at": "f1", "yaw": 0.0}
    assert room.checkpoint_states["f2"]["parts"]["f1"] == {"state": {"reached": True}, "values": {}}


@pytest.mark.parametrize("states, bad", [
    ({"nope": {"body": {"pos": [0.0, 0.0]}}}, "is not a checkpoint flag"),
    ({"f1": {"nonsense": 1}}, "takes `body:` and `parts:`"),
    ({"f1": {"body": {"pos": [0.0, 0.0], "at": "f1"}}}, "write {pos: [x, y], yaw: 0}"),
    ({"f1": {"body": {}}}, "write {pos: [x, y], yaw: 0}"),
    ({"f1": {"body": {"at": "nope"}}}, "`at` names a part of this room"),
    ({"f1": {"body": {"pos": ["x", 0.0]}}}, "`pos` must be [x, y]"),
    ({"f1": {"parts": {"nope": {"state": {}}}}}, "is not a part of this room"),
    ({"f1": {"parts": {"f1": {"state": {"nonsense": 1}}}}}, "are not state of a"),
    ({"f1": {"parts": {"f1": {"values": {"order": 3}}}}}, "cannot be set at an episode start"),
    ({"f1": {"parts": {"f1": {}}}}, "takes `state:` and `values:`"),
])
def test_bad_declarations_are_refused(tmp_path, states, bad):
    with pytest.raises(RoomError) as exc:
        Room.load(room_file(tmp_path, states=states))
    assert bad in str(exc.value)


def test_a_drawn_value_can_be_declared(tmp_path):
    """A run-time value belongs in `values:` (a rope's `length`, a fan's power)."""
    states = {"f1": {"body": {"at": "f1"}, "parts": {"fan": {"values": {"power": 1.5}}}}}
    room = Room.load(room_file(tmp_path, states=states, fan=True))
    assert room.checkpoint_states["f1"]["parts"]["fan"]["values"] == {"power": 1.5}


# ---- a fixed room still starts from its snapshot ------------------------------------
def test_a_fixed_room_starts_from_the_snapshot(tmp_path):
    env = env_of(tmp_path)
    begin_segment(env, 1, False)
    assert env.parts["f1"].reached and not env.parts["f2"].reached
    assert np.allclose(env.body_state.pos[:2], env.parts["f1"].xy, atol=0.05)
    assert env.checkpoint_snap is not None
    env.close()


def test_the_physics_layer_is_drawn_again_over_the_snapshot(tmp_path):
    """The snapshot holds the `mpg check` run's values; a segment episode draws its own,
    so a run does not train on one frozen set."""
    env = env_of(tmp_path, fan=True)
    powers, places = set(), set()
    for s in range(6):
        env.reset(seed=s, options={"segment": 1})
        powers.add(round(float(env.parts["fan"].live["power"]), 4))
        places.add(tuple(np.round(env.parts["goal"].xy, 4)))
        assert env.checkpoint_snap.values["fan"]["power"] == env.parts["fan"].live["power"]
    assert len(powers) > 1                       # the values differ per episode
    assert len(places) == 1                      # the snapshot's layout does not
    env.close()


def test_the_physics_strength_can_pin_the_values(tmp_path):
    env = env_of(tmp_path, fan=True, task={"randomize": {"physics": 0.0}})
    base = env.parts["fan"].room_value("power")
    for s in range(3):
        env.reset(seed=s, options={"segment": 1})
        assert env.parts["fan"].live["power"] == base
    env.close()


# ---- a drawn room starts from the declaration ---------------------------------------
def test_a_drawn_room_starts_from_the_declaration_on_this_episode_s_layout(tmp_path):
    env = env_of(tmp_path, states=STATES, drawn=True)
    places = set()
    for s in range(6):
        env.reset(seed=s, options={"segment": 2})
        places.add(tuple(np.round(env.parts["goal"].xy, 4)))
        assert env.parts["f1"].reached and env.parts["f2"].reached
        assert np.allclose(env.body_state.pos[:2], env.parts["f2"].xy, atol=0.05)
        assert env.segment["index"] == 2
    assert len(places) > 1                       # a fresh layout every episode
    env.close()


def test_a_drawn_room_without_a_declaration_is_refused(tmp_path):
    env = env_of(tmp_path, drawn=True)
    with pytest.raises(CurriculumError) as exc:
        begin_segment(env, 1, False)
    assert "draws its layout or its structure" in str(exc.value)
    assert "`checkpoint_states:`" in str(exc.value)
    env.close()


def test_pinning_the_layout_layer_brings_the_snapshot_back(tmp_path):
    env = env_of(tmp_path, drawn=True, task={"randomize": {"layout": 0.0}})
    begin_segment(env, 1, False)
    assert env.parts["f1"].reached
    assert env.checkpoint_snap.meta["checkpoint"] == "f1"
    env.close()


def test_a_declared_start_works_without_any_snapshot(tmp_path):
    """A room with no `mpg check` run yet: the declaration is the start."""
    env = env_of(tmp_path, states=STATES, snaps=False)
    begin_segment(env, 1, False)
    assert env.parts["f1"].reached and np.allclose(env.body_state.pos[:2],
                                                   env.parts["f1"].xy, atol=0.05)
    env.close()


def test_without_a_snapshot_or_a_declaration_the_missing_flags_are_named(tmp_path):
    env = env_of(tmp_path, snaps=False)
    with pytest.raises(CurriculumError) as exc:
        begin_segment(env, 1, False)
    assert "no snapshots for flags ['f1', 'f2']" in str(exc.value)
    assert "`checkpoint_states:`" in str(exc.value)
    env.close()


def test_a_declared_segment_runs_on_to_its_end(tmp_path):
    """From the declared start on f2 a forward walk reaches the goal, as from a snapshot."""
    from flag_room import FORWARD

    env = env_of(tmp_path, states=STATES, drawn=True)
    env.reset(seed=1, options={"segment": 2})
    for _ in range(40):
        _, _, term, trunc, info = env.step(FORWARD)
        if term or trunc:
            break
    assert info["segment_success"] and env.segment["end"] == "goal"
    env.close()


def test_the_declared_start_is_the_state_a_respawn_goes_back_to(tmp_path):
    env = env_of(tmp_path, states=STATES, drawn=True,
                 task={"env": {"hz": 10, "episode_seconds": 6.0, "respawn": "checkpoint"}})
    env.reset(seed=2, options={"segment": 1})
    snap = env.checkpoint_snap
    assert snap is not None and snap.meta["checkpoint"] == "f1"
    assert snap.parts["f1"]["reached"] and not snap.parts["f2"]["reached"]
    env.close()


def test_every_flag_of_the_helper_room_can_be_declared(tmp_path):
    env = env_of(tmp_path, states=STATES, drawn=True)
    for i, flag in enumerate(FLAGS, start=1):
        env.reset(seed=3, options={"segment": i})
        assert env.parts[flag].reached
        assert np.allclose(env.body_state.pos[:2], env.parts[flag].xy, atol=0.05)
    env.step(STAND)
    env.close()
