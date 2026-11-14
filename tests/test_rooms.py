"""R35 part 4c-1: the `rooms` layer -- which experiment room an episode plays in.

Every candidate room file is built when the room loads, as a zone of one model (the
machinery `stages:` uses), because MuJoCo cannot add a body while running. A draw
picks one zone and starts at its own spawn; the other zones rest, so the plan, the
observations and the signals leave them out and the observation keeps one size
whichever room is drawn.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym.check import check, check_resample
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.room import Room, RoomError

ROOT = Path(__file__).resolve().parents[1]
BASE = {"size": [3.0, 2.0], "wall": {"thickness": 0.06, "height": 0.25},
        "walls": [{"from": [0.5, -1.0], "to": [0.5, -0.28]},
                  {"from": [0.5, 0.28], "to": [0.5, 1.0]}],
        "spawn": {"pos": [-1.2, 0.0], "yaw": 0.0}}


def room_raw(prefix: str, name: str, **top) -> dict:
    """A complete little room: plate -> door -> goal, its parts named `<prefix>_...`."""
    raw = yaml.safe_load(yaml.safe_dump(BASE))
    raw["name"] = name
    raw["parts"] = [
        {"name": f"{prefix}_plate", "preset": "input/floor_oneshot", "pos": [-0.7, 0.55]},
        {"name": f"{prefix}_door", "preset": "device/sliding_door", "pos": [0.5, 0.0],
         "in": [f"{prefix}_plate"]},
        {"name": f"{prefix}_goal", "type": "goal", "pos": [1.1, 0.0], "radius": 0.15}]
    raw.update(top)
    return raw


def rooms_file(tmp_path, *, candidates=("b",), holdout=(), block=None, host=None,
               cand_top=None) -> Path:
    """The declaring room plus one file per candidate, written next to it."""
    for c in candidates:
        (tmp_path / f"{c}.yaml").write_text(
            yaml.safe_dump(room_raw(c, c, **(cand_top or {}))))
    rooms = {"candidates": [f"{c}.yaml" for c in candidates]} if block is None else block
    if holdout and block is None:
        rooms["holdout"] = list(holdout)
    raw = room_raw("a", "host", randomize={"rooms": rooms}, **(host or {}))
    path = tmp_path / "host.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


def env_of(tmp_path, *, task=None, **kw) -> PuzzleEnv:
    path = rooms_file(tmp_path, **kw)
    return PuzzleEnv(task={"room": str(path), **(task or {})}, room=path)


def zone_of(env, name: str) -> int:
    return int(env.parts[name].zone)


# ---- what the room expands into -----------------------------------------------------
def test_each_candidate_room_becomes_a_zone(tmp_path):
    env = env_of(tmp_path, candidates=("b", "c"))
    assert env.room.zone_names == ["host", "b", "c"]
    assert [zone_of(env, n) for n in ("a_plate", "b_plate", "c_plate")] == [0, 1, 2]
    zones = env.room.zone_extents()
    assert len(zones) == 3 and zones[1].x0 > zones[0].x1    # side by side, in one model
    assert env.room.zone_rooms()[2][1].name == "c"


def test_the_zones_keep_their_own_spawns(tmp_path):
    env = env_of(tmp_path, candidates=("b",))
    (dx, sub) = env.room.zone_rooms()[1]
    assert env.room.zone_spawns()[1] == (float(sub.spawn_pos[0]) + dx,
                                         float(sub.spawn_pos[1]), sub.spawn_yaw)


# ---- what a draw keeps --------------------------------------------------------------
def test_an_episode_plays_in_the_drawn_room_alone(tmp_path):
    env = env_of(tmp_path, candidates=("b", "c"))
    seen = set()
    for s in range(20):
        env.reset(seed=s)
        zone = env.layout["zone"]
        seen.add(zone)
        assert env.zone == zone
        ext = env.room.zone_extents()[zone]
        assert ext.x0 <= env.layout["spawn"][0] <= ext.x1    # starts in the drawn room
        assert env.plan["ok"] and all(zone_of(env, n) == zone for n in env.plan["order"])
        for name, part in env.parts.items():
            assert part.parked == (zone_of(env, name) != zone)
    assert seen == {0, 1, 2}


def test_the_nominal_room_is_the_declaring_one(tmp_path):
    env = env_of(tmp_path, candidates=("b",))
    env.reset(seed=3, options={"randomize": "nominal"})
    assert env.layout["zone"] == 0 and not env.parts["a_plate"].parked
    assert env.parts["b_plate"].parked


def test_the_rooms_strength_can_pin_the_room(tmp_path):
    env = env_of(tmp_path, candidates=("b", "c"), task={"randomize": {"rooms": 0.0}})
    for s in range(5):
        env.reset(seed=s)
        assert env.layout["zone"] == 0 and not env.parts["a_plate"].parked


def test_wide_draws_the_evaluation_only_room(tmp_path):
    env = env_of(tmp_path, candidates=("b", "c"), holdout=("c",))
    for s in range(10):
        env.reset(seed=s)
        assert env.layout["zone"] in (0, 1)                  # training never sees it
    for s in range(3):
        env.reset(seed=s, options={"randomize": "wide"})
        assert env.layout["zone"] == 2 and not env.parts["c_plate"].parked


def test_the_observation_keeps_one_size(tmp_path):
    """Why every candidate is built in: the policy's input cannot change size."""
    env = env_of(tmp_path, candidates=("b", "c"), holdout=("c",),
                 task={"obs": ["part_pos", "part_state", "part_used"]})
    sizes = {env.observation_space.shape}
    for mode in ("nominal", "train", "wide"):
        for s in range(4):
            obs, _ = env.reset(seed=s, options={"randomize": mode})
            sizes.add(obs.shape)
    assert len(sizes) == 1


# ---- a resting zone is not there ----------------------------------------------------
def test_a_resting_room_writes_nothing_and_is_invisible(tmp_path):
    from rooms import tick

    env = env_of(tmp_path, candidates=("b",),
                 task={"obs": ["part_pos", "part_state", "part_used"]})
    names = list(env.parts)
    obs, _ = env.reset(seed=0, options={"randomize": "nominal"})
    tick(env)
    rest = [n for n in names if env.parts[n].parked]
    assert rest == ["b_plate", "b_door", "b_goal"]
    assert env.signals.get("b_plate") == 0.0            # its channel reads 0
    assert all(n not in env.plan["order"] for n in rest)
    pos, state, used = obs[:2 * len(names)], obs[2 * len(names):3 * len(names)], obs[3 * len(names):]
    assert list(used) == [0.0 if env.parts[n].parked else 1.0 for n in names]
    for i, n in enumerate(names):                        # zeros where the room is resting
        if env.parts[n].parked:
            assert np.allclose(pos[2 * i:2 * i + 2], 0.0) and state[i] == 0.0


def test_a_snapshot_keeps_the_room_it_was_recorded_in(tmp_path):
    env = env_of(tmp_path, candidates=("b",))
    seeds = {}
    for s in range(10):
        env.reset(seed=s)
        seeds[env.layout["zone"]] = env.save_snapshot()
        if len(seeds) == 2:
            break
    assert sorted(seeds) == [0, 1]
    for zone, snap in seeds.items():
        assert snap.meta["zone"] == zone
        env.reset(seed=99)
        env.restore_snapshot(snap)
        assert env.zone == zone


def test_a_group_inside_a_room_keeps_its_shelf(tmp_path):
    """The two structure draws meet: a resting room parks everything, and inside the
    drawn room the shelf still holds the candidates that were not drawn."""
    raw = room_raw("a", "host", randomize={"rooms": {"candidates": ["b.yaml"]}})
    raw["parts"][0]["randomize"] = {"type": {"candidates": ["input/floor_oneshot",
                                                            "input/floor_toggle"]}}
    (tmp_path / "b.yaml").write_text(yaml.safe_dump(room_raw("b", "b")))
    (tmp_path / "host.yaml").write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(task={"room": str(tmp_path / "host.yaml")}, room=tmp_path / "host.yaml")
    members = ["a_plate#0", "a_plate#1"]
    seen = set()
    for s in range(12):
        env.reset(seed=s)
        zone = env.layout["zone"]
        seen.add(zone)
        live = [n for n in members if not env.parts[n].parked]
        assert live == ([] if zone else env.layout["used"]["a_plate"])
        assert env.plan["ok"]
    assert seen == {0, 1}


# ---- what a room may declare --------------------------------------------------------
@pytest.mark.parametrize("block, bad", [
    ({}, "`candidates` is a list"),
    ({"candidates": []}, "`candidates` is a list"),
    ({"candidates": "b.yaml"}, "`candidates` is a list"),
    ({"candidates": ["b.yaml", "b.yaml"]}, "`candidates` is a list"),
    ({"candidates": [f"b{i}.yaml" for i in range(8)]}, "at most 7 candidates"),
    ({"candidates": ["b.yaml"], "holdout": ["nope"]}, "`holdout` names one of the candidate"),
    ({"candidates": ["b.yaml"], "holdout": "b"}, "`holdout` is a list"),
    ({"candidates": ["b.yaml"], "extra": 1}, "write {candidates:"),
])
def test_bad_rooms_declarations_are_refused(tmp_path, block, bad):
    with pytest.raises(RoomError) as exc:
        env_of(tmp_path, candidates=("b",), block=block)
    assert bad in str(exc.value)


def test_a_missing_candidate_file_is_refused(tmp_path):
    with pytest.raises(RoomError) as exc:
        env_of(tmp_path, candidates=("b",), block={"candidates": ["nope.yaml"]})
    assert "cannot read room file" in str(exc.value)


def test_two_rooms_cannot_share_a_part_name(tmp_path):
    """The channels are shared over the whole model, so each zone needs its own names."""
    (tmp_path / "b.yaml").write_text(yaml.safe_dump(room_raw("a", "b")))
    path = tmp_path / "host.yaml"
    path.write_text(yaml.safe_dump(room_raw("a", "host",
                                            randomize={"rooms": {"candidates": ["b.yaml"]}})))
    with pytest.raises(RoomError) as exc:
        Room.load(path)
    assert "is already used by another zone" in str(exc.value)


def test_rooms_and_stages_cannot_both_be_used(tmp_path):
    with pytest.raises(RoomError) as exc:
        env_of(tmp_path, candidates=("b",), host={"stages": ["b.yaml"]})
    assert "both build the model's zones" in str(exc.value)


def test_a_candidate_cannot_draw_rooms_itself(tmp_path):
    with pytest.raises(RoomError) as exc:
        env_of(tmp_path, candidates=("b",),
               cand_top={"randomize": {"rooms": {"candidates": ["b.yaml"]}}})
    assert "cannot draw rooms itself" in str(exc.value)


def test_a_map_room_cannot_draw_rooms_yet(tmp_path):
    raw = room_raw("a", "host", randomize={"rooms": {"candidates": ["b.yaml"]}})
    raw.pop("size"), raw.pop("walls")
    raw["map"] = "#####\n#...#\n#####\n"
    (tmp_path / "b.yaml").write_text(yaml.safe_dump(room_raw("b", "b")))
    (tmp_path / "host.yaml").write_text(yaml.safe_dump(raw))
    with pytest.raises(RoomError) as exc:
        Room.load(tmp_path / "host.yaml")
    assert "cannot draw rooms yet" in str(exc.value)


def test_a_map_candidate_is_refused(tmp_path):
    raw = room_raw("b", "b")
    raw.pop("size"), raw.pop("walls")
    raw["map"] = "#####\n#...#\n#####\n"
    raw["spawn"] = {"pos": [0.0, 0.0]}
    (tmp_path / "b.yaml").write_text(yaml.safe_dump(raw))
    (tmp_path / "host.yaml").write_text(yaml.safe_dump(
        room_raw("a", "host", randomize={"rooms": {"candidates": ["b.yaml"]}})))
    with pytest.raises(RoomError) as exc:
        Room.load(tmp_path / "host.yaml")
    assert "cannot be a zone yet" in str(exc.value)


def test_a_next_stage_warp_is_refused_with_rooms(tmp_path):
    """`to: next_stage` belongs to `stages:`; here the next zone is another room."""
    pad = {"name": "a_pad", "type": "warp", "pos": [0.0, -0.8], "size": [0.2, 0.2],
           "to": "next_stage"}
    with pytest.raises(RoomError) as exc:
        env_of(tmp_path, candidates=("b",), host={"parts": [
            *room_raw("a", "host")["parts"], pad]})
    assert "a `to: next_stage` warp is for `stages:`" in str(exc.value)


def test_an_absolute_spawn_sampler_is_refused_with_rooms(tmp_path):
    """`region` and `candidates` are places in one room; a drawn room has its own."""
    with pytest.raises(RoomError) as exc:
        env_of(tmp_path, candidates=("b",),
               host={"spawn": {"pos": [-1.2, 0.0],
                               "randomize": {"pos": {"region": [[-1.3, -1.0, -0.3, 0.3]]}}}})
    assert "the spawn place must be drawn with `range`" in str(exc.value)


# ---- `mpg check` --------------------------------------------------------------------
def test_the_resample_stage_counts_the_rooms(tmp_path):
    env = env_of(tmp_path, candidates=("b", "c"))
    res = check_resample(env, seeds=30)
    assert res["ok"] and sorted(res["rooms"]) == ["b", "c", "host"]
    assert sum(res["rooms"].values()) == 30


def test_a_room_the_training_never_draws_is_warned_about(tmp_path):
    """With few seeds a room may be missed; a room meant to stay unseen is a holdout."""
    env = env_of(tmp_path, candidates=("b", "c"))
    res = check_resample(env, seeds=1)
    assert any("never drew" in w for w in res.get("warnings", []))
    env = env_of(tmp_path, candidates=("b", "c"), holdout=("c",))
    res = check_resample(env, seeds=30)
    assert "c" not in res["rooms"] and not res.get("warnings")


def test_a_rooms_room_passes_every_static_stage(asset_dir, tmp_path):
    path = rooms_file(tmp_path, candidates=("b", "c"), holdout=("c",))
    res = check(path, static_only=True)
    assert res["ok"], {k: res[k] for k in res["failed"]}
    assert res["resample"]["ratio"] == 0.0


def test_the_community_floor_limit_counts_the_largest_room(asset_dir):
    """The zones are alternatives, so three 3 x 2 m rooms are not an 11 m floor."""
    res = check(ROOT / "examples/rooms/room_roulette.yaml", static_only=True, community=True)
    assert res["community"] == {"ok": True}, res["community"]


def test_the_example_room_passes_the_static_phase(asset_dir):
    res = check(ROOT / "examples/rooms/room_roulette.yaml",
                task=ROOT / "examples/task_room_roulette.yaml", static_only=True)
    assert res["ok"], {k: res[k] for k in res["failed"]}
    assert res["resample"]["rooms"].keys() == {"room_roulette", "roulette_b"}


@pytest.mark.slow
def test_the_scripted_answer_solves_whichever_room_is_drawn(asset_dir):
    from mujoco_puzzle_gym.solver import ScriptedSolver, run_episode

    env = PuzzleEnv(task=ROOT / "examples/task_room_roulette.yaml")
    seen = set()
    for seed in range(6):
        res = run_episode(env, seed, ScriptedSolver(env))
        seen.add(env.layout["zone"])
        assert res["success"], (seed, env.layout["zone"], res)
    assert seen == {0, 1}
