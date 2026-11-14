"""R35 part 4b-1: the structure layer's `shuffle` — which part feeds a reader and
which plate stands where — the colour cue, and the `part_links` / `part_used`
observations.

The samplers themselves are in test_randomize.py; this file is about a wiring that
changes every episode: the bus wired again, the plan following the draw, the cue
that lets a policy see it, and what `mpg check` makes of a hidden cue.
"""

from __future__ import annotations

import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym.check import check
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.parts.base import CUE_OFF_RGBA
from mujoco_puzzle_gym.randomize import CUE_RGBA
from mujoco_puzzle_gym.room import RoomError

ROOM = {
    "name": "shuf", "size": [3.0, 2.0], "wall": {"thickness": 0.06, "height": 0.25},
    "walls": [{"from": [0.5, -1.0], "to": [0.5, -0.28]},
              {"from": [0.5, 0.28], "to": [0.5, 1.0]}],
    "spawn": {"pos": [-1.2, 0.0], "yaw": 0.0},
    "randomize": {"shuffle": [{"links": "door", "from": ["plate_a", "plate_b", "plate_c"]},
                              {"places": ["plate_a", "plate_b", "plate_c"]}]},
    "parts": [{"name": "plate_a", "preset": "input/floor_oneshot", "pos": [-1.0, 0.6]},
              {"name": "plate_b", "preset": "input/floor_oneshot", "pos": [-0.5, 0.6]},
              {"name": "plate_c", "preset": "input/floor_oneshot", "pos": [-0.5, -0.6]},
              {"name": "door", "preset": "device/sliding_door", "pos": [0.5, 0.0],
               "in": ["plate_a"]},
              {"name": "goal", "type": "goal", "pos": [1.1, 0.0], "radius": 0.15}],
}
PLATES = ("plate_a", "plate_b", "plate_c")


def room_file(tmp_path, *, shuffle=None, parts=None, name="shuf.yaml", **top):
    raw = yaml.safe_load(yaml.safe_dump(ROOM))
    if shuffle is not None:
        raw["randomize"] = {"shuffle": shuffle}
    for pname, block in (parts or {}).items():
        next(c for c in raw["parts"] if c["name"] == pname).update(block)
    raw.update(top)
    path = tmp_path / name
    path.write_text(yaml.safe_dump(raw))
    return path


def env_of(tmp_path, *, task=None, **kw) -> PuzzleEnv:
    path = room_file(tmp_path, **kw)
    return PuzzleEnv(task={"room": str(path), **(task or {})}, room=path)


def cue_of(env, name) -> tuple:
    g = env.model.geom(env.parts[name].body_name + "/cue").id
    return tuple(round(float(v), 3) for v in env.model.geom_rgba[g])


# ---- which part feeds the reader ----------------------------------------------------
def test_the_drawn_source_is_the_one_the_reader_reads(tmp_path):
    env = env_of(tmp_path)
    seen = set()
    for s in range(30):
        env.reset(seed=s)
        drawn = env.layout["links"]["door"]
        seen.add(drawn)
        assert env.parts["door"].inputs == [drawn]
        assert [p.name for p in env.parts["door"].sources] == [drawn]
        assert env.plan["ok"] and env.plan["order"][0] == drawn
    assert seen == set(PLATES)


def test_pressing_the_drawn_plate_opens_the_door(tmp_path):
    """The bus is wired again every reset, so the drawn plate -- and only it -- works."""
    from rooms import stand_at, tick

    env = env_of(tmp_path)
    for s in range(6):
        env.reset(seed=s)
        drawn = env.layout["links"]["door"]
        for name in PLATES:
            env.reset(seed=s)
            stand_at(env, *env.parts[name].xy)
            for _ in range(5):
                tick(env)
            assert env.signals.get(name) > 0.5                 # the plate itself is pressed
            assert env.parts["door"].inputs_on() == (name == drawn), (s, name, drawn)


def test_the_same_seed_gives_the_same_wiring_and_places(tmp_path):
    env = env_of(tmp_path)
    runs = []
    for _ in range(2):
        out = []
        for s in range(4):
            env.reset(seed=s)
            out.append((env.layout["links"]["door"],
                        tuple(tuple(np.round(env.layout["parts"][n], 3)) for n in PLATES)))
        runs.append(out)
    assert runs[0] == runs[1]
    assert len({r[0] for r in runs[0]}) > 1


def test_places_swap_the_same_set_of_places(tmp_path):
    env = env_of(tmp_path)
    nominal = {(-1.0, 0.6), (-0.5, 0.6), (-0.5, -0.6)}
    seen = set()
    for s in range(30):
        env.reset(seed=s)
        got = {tuple(np.round(env.layout["parts"][n], 3)) for n in PLATES}
        assert got == nominal                      # a permutation, never a new place
        seen.add(tuple(tuple(np.round(env.layout["parts"][n], 3)) for n in PLATES))
    assert len(seen) > 1


def test_the_structure_strength_can_pin_the_wiring(tmp_path):
    env = env_of(tmp_path, task={"randomize": {"structure": 0.0}})
    for s in range(5):
        env.reset(seed=s)
        assert env.layout["links"] == {} and env.parts["door"].inputs == ["plate_a"]
        assert np.allclose(env.layout["parts"]["plate_a"], [-1.0, 0.6])


# ---- the cue ------------------------------------------------------------------------
def test_the_cue_lights_the_reader_and_the_drawn_source(tmp_path):
    env = env_of(tmp_path)
    for s in range(6):
        env.reset(seed=s)
        drawn = env.layout["links"]["door"]
        assert cue_of(env, "door") == tuple(round(v, 3) for v in CUE_RGBA[0])
        for name in PLATES:
            want = CUE_RGBA[0] if name == drawn else CUE_OFF_RGBA
            assert cue_of(env, name) == tuple(round(v, 3) for v in want), (s, name)


def test_a_hidden_cue_paints_nothing_and_shows_nothing(tmp_path):
    env = env_of(tmp_path, shuffle=[{"links": "door", "from": list(PLATES), "hidden": True}],
                 task={"obs": ["part_links"]})
    obs, _ = env.reset(seed=0)
    assert np.all(obs == 0.0)
    assert all(cue_of(env, n) == tuple(round(v, 3) for v in CUE_OFF_RGBA)
               for n in (*PLATES, "door"))


def test_the_cue_mats_are_the_same_size_whatever_is_drawn(tmp_path):
    """Only the colour may carry the cue; a different shape would leak it."""
    env = env_of(tmp_path)
    sizes = {tuple(env.model.geom_size[env.model.geom(env.parts[n].body_name + "/cue").id])
             for n in PLATES}
    assert len(sizes) == 1


# ---- the observations ---------------------------------------------------------------
def test_part_links_is_a_one_hot_over_the_candidates(tmp_path):
    env = env_of(tmp_path, task={"obs": ["part_links"]})
    for s in range(10):
        obs, _ = env.reset(seed=s)
        assert obs.shape == (3,) and obs.sum() == pytest.approx(1.0)
        assert PLATES[int(np.argmax(obs))] == env.layout["links"]["door"]


def test_part_links_covers_the_evaluation_only_sources_too(tmp_path):
    env = env_of(tmp_path, shuffle=[{"links": "door", "from": ["plate_a", "plate_b"],
                                     "holdout": ["plate_c"]}],
                 task={"obs": ["part_links"]})
    assert env.reset(seed=0)[0].shape == (3,)           # length fixed over the modes
    obs, _ = env.reset(seed=0, options={"randomize": "wide"})
    assert list(obs) == [0.0, 0.0, 1.0] and env.layout["links"]["door"] == "plate_c"


def test_part_used_is_one_for_every_part_in_play(tmp_path):
    env = env_of(tmp_path, task={"obs": ["part_used"]})
    obs, _ = env.reset(seed=0)
    assert obs.shape == (len(ROOM["parts"]),) and np.all(obs == 1.0)


# ---- what a room may declare --------------------------------------------------------
@pytest.mark.parametrize("shuffle, bad", [
    ([{"places": ["plate_a"]}], "at least two different part names"),
    ([{"places": ["plate_a", "nope"]}], "not a part"),
    ([{"places": ["plate_a", "door"]}], "swaps parts of one type"),
    ([{"places": ["plate_a", "plate_b"], "min_gap": 0.1}], "takes no other key"),
    ([{"links": "door", "from": ["plate_a", "plate_b"], "places": ["plate_a", "plate_b"]}],
     "write {places: [...]} or"),
    ([{"links": "nope", "from": ["plate_a", "plate_b"]}], "names the part that reads"),
    ([{"links": "door", "from": ["plate_a"]}], "at least two different part names"),
    ([{"links": "door", "from": ["plate_a", "nope"]}], "is not a part of this room"),
    ([{"links": "door", "from": ["plate_b", "plate_c"]}], "must read exactly one of the"),
    ([{"links": "door", "from": ["plate_a", "plate_b"], "nonsense": 1}], "got extra"),
    ([{"links": "door", "from": ["plate_a", "plate_b"], "holdout": "plate_c"}],
     "`holdout` is a list"),
    ("nope", "is a list of"),
])
def test_bad_shuffle_blocks_are_refused(tmp_path, shuffle, bad):
    with pytest.raises(RoomError) as exc:
        env_of(tmp_path, shuffle=shuffle)
    assert bad in str(exc.value)


def test_a_part_cannot_both_swap_places_and_draw_one(tmp_path):
    with pytest.raises(RoomError) as exc:
        env_of(tmp_path, parts={"plate_a": {"random": [0.1, 0.1]}})
    assert "swaps places or draws one" in str(exc.value)


def test_the_room_block_takes_only_shuffle(tmp_path):
    with pytest.raises(RoomError) as exc:
        env_of(tmp_path, randomize={"shuffle": [{"places": list(PLATES)}], "pos": {"range": 1}})
    assert "takes only `shuffle:`" in str(exc.value)


def test_a_snapshot_keeps_the_wiring_it_was_recorded_with(tmp_path):
    """A checkpoint restored in another episode must be read with its own wiring
    (R35), or its part states do not match the room."""
    env = env_of(tmp_path)
    env.reset(seed=0)
    snap = env.save_snapshot()
    assert snap.meta["links"] == {"door": env.layout["links"]["door"]}
    env.reset(seed=1)
    env._reset_sim(None, {"spawn": env.layout["spawn"], "parts": env.layout["parts"],
                          "links": dict(snap.meta["links"])})
    assert env.parts["door"].inputs == [snap.meta["links"]["door"]]


# ---- `mpg check` --------------------------------------------------------------------
def test_a_drawn_wiring_the_signal_checks_refuse_is_thrown_away(tmp_path):
    """Drawing `g2` as `g1`'s source closes a loop through two stateless gates, so that
    draw cannot be used and the room is drawn again."""
    raw = {"name": "loop", "size": [3.0, 2.0], "wall": {"thickness": 0.06, "height": 0.25},
           "spawn": {"pos": [-1.2, 0.0], "yaw": 0.0},
           "randomize": {"shuffle": [{"links": "g1", "from": ["src", "g2"]}]},
           "parts": [{"name": "src", "preset": "logic/source_const", "pos": [-1.3, 0.9],
                      "body": False},
                     {"name": "g1", "preset": "logic/not", "in": ["src"]},
                     {"name": "g2", "preset": "logic/not", "in": ["g1"]},
                     {"name": "door", "preset": "device/sliding_door", "pos": [0.5, 0.0],
                      "in": ["g1"]},
                     {"name": "goal", "type": "goal", "pos": [1.1, 0.0], "radius": 0.15}]}
    path = tmp_path / "loop.yaml"
    path.write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(room=path)
    for s in range(20):
        layout = env.sample_layout(np.random.default_rng(s))
        assert layout["links"]["g1"] == "src"       # the loop draw never survives
    res = check(path, static_only=True)
    assert res["ok"] and 0.0 < res["resample"]["ratio"] < 1.0


def test_check_warns_about_a_hidden_cue(tmp_path):
    path = room_file(tmp_path, shuffle=[{"links": "door", "from": list(PLATES), "hidden": True}])
    res = check(path, static_only=True)
    assert res["ok"], res["failed"]
    assert any("cue is hidden" in w for w in res["resample"]["warnings"])


def test_the_example_room_passes_the_static_phase(asset_dir):
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    res = check(root / "examples/rooms/shuffle_switches.yaml",
                task=root / "examples/task_shuffle_switches.yaml", static_only=True)
    assert res["ok"], {k: res[k] for k in res["failed"]}
    assert not res["resample"].get("warnings")


@pytest.mark.slow
def test_the_scripted_answer_presses_the_plate_that_is_wired(asset_dir):
    from pathlib import Path

    from mujoco_puzzle_gym.solver import ScriptedSolver, run_episode

    root = Path(__file__).resolve().parents[1]
    env = PuzzleEnv(task=root / "examples/task_shuffle_switches.yaml")
    for seed in (0, 1, 2):
        res = run_episode(env, seed, ScriptedSolver(env))
        drawn = env.layout["links"]["door"]
        assert res["success"], (seed, drawn, res)
        assert env.parts[drawn].state() > 0.0
