"""R35 part 4a: the `range`, `region` and `candidates` samplers, the task's per-layer
strengths, the redraw loop, the `resample` check stage and the `randomized` obs item.

The room says what may change, the task how much of it to use; this file is about
what comes out of a draw and what `mpg check` makes of it.
"""

from __future__ import annotations

import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym import randomize
from mujoco_puzzle_gym.check import RESAMPLE_WARN, check
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.room import Room, RoomError
from mujoco_puzzle_gym.task import TaskError

#: A room with a divider, a switch for the door and a goal behind it.
BASE = {
    "name": "rnd", "size": [3.0, 2.0], "wall": {"thickness": 0.06, "height": 0.25},
    "walls": [{"from": [0.5, -1.0], "to": [0.5, -0.28]},
              {"from": [0.5, 0.28], "to": [0.5, 1.0]}],
    "spawn": {"pos": [-1.2, 0.0], "yaw": 0.0},
    "parts": [{"name": "switch_a", "preset": "input/floor_momentary", "pos": [-0.6, 0.5]},
              {"name": "door", "preset": "device/sliding_door", "pos": [0.5, 0.0],
               "in": ["switch_a"]},
              {"name": "fan", "preset": "trap/crosswind", "pos": [-1.0, -0.65],
               "power": 1.0},
              {"name": "goal", "type": "goal", "pos": [1.1, 0.0], "radius": 0.15}],
}


def room_file(tmp_path, *, parts=None, spawn=None, name="rnd.yaml", **top):
    """BASE with per-part `randomize:` blocks (`parts`: name -> block) written out."""
    raw = yaml.safe_load(yaml.safe_dump(BASE))
    for pname, block in (parts or {}).items():
        cfg = next(c for c in raw["parts"] if c["name"] == pname)
        cfg.update(block)
    if spawn is not None:
        raw["spawn"].update(spawn)
    raw.update(top)
    path = tmp_path / name
    path.write_text(yaml.safe_dump(raw))
    return path


def env_of(tmp_path, *, task=None, **kw) -> PuzzleEnv:
    path = room_file(tmp_path, **kw)
    return PuzzleEnv(task={"room": str(path), **(task or {})}, room=path)


def draws(env, seeds=range(30), mode="train"):
    return [env.sample_layout(np.random.default_rng(s), mode) for s in seeds]


# ---- the samplers -------------------------------------------------------------------
def test_range_stays_inside_its_half_range(tmp_path):
    env = env_of(tmp_path, parts={"switch_a": {"randomize": {"pos": {"range": [0.2, 0.1]}}}})
    xy = np.array([l["parts"]["switch_a"] for l in draws(env, range(200))])
    off = xy - np.array([-0.6, 0.5])
    assert np.all(np.abs(off) <= np.array([0.2, 0.1]) + 1e-9)
    assert np.all(np.abs(off).max(axis=0) > np.array([0.15, 0.07]))        # the whole range is used


def test_region_keeps_min_gap_from_walls_and_parts(tmp_path):
    env = env_of(tmp_path, parts={"switch_a": {
        "randomize": {"pos": {"region": [[-1.2, 0.45, -0.9, 0.9]], "min_gap": 0.25}}}})
    for layout in draws(env, range(60)):
        xy = layout["parts"]["switch_a"]
        assert -1.2 <= xy[0] <= 0.45 and -0.9 <= xy[1] <= 0.9
        assert not layout["why"], layout["why"]          # a gap break is redrawn
        assert float(np.hypot(*(xy - np.array(layout["spawn"][:2])))) >= 0.25
        assert randomize._seg_distance(xy, Room.load(env.room.path).walls()[0]) >= 0.25


def test_candidates_follow_their_weights(tmp_path):
    env = env_of(tmp_path, parts={"goal": {"randomize": {"pos": {
        "candidates": [[1.1, 0.0], [1.1, 0.5], [1.1, -0.5]], "weights": [2, 1, 1]}}}})
    rng = np.random.default_rng(0)
    seen = [tuple(np.round(env.randomizer.draw(env.room, env.parts, rng)["parts"]["goal"], 2))
            for _ in range(10_000)]
    share = {p: seen.count(p) / len(seen) for p in set(seen)}
    assert set(share) == {(1.1, 0.0), (1.1, 0.5), (1.1, -0.5)}
    assert share[(1.1, 0.0)] == pytest.approx(0.5, abs=0.03)
    assert share[(1.1, 0.5)] == pytest.approx(0.25, abs=0.03)


def test_the_same_seed_gives_the_same_room(tmp_path):
    env = env_of(tmp_path, parts={
        "switch_a": {"randomize": {"pos": {"region": [[-1.2, 0.0, -0.9, 0.9]]}}},
        "goal": {"randomize": {"pos": {"range": [0.1, 0.3]}}}},
        spawn={"randomize": {"pos": {"range": [0.1, 0.2]}, "yaw": {"range": 1.0}}})
    a, b = draws(env, range(5)), draws(env, range(5))
    for x, y in zip(a, b):
        assert x["spawn"] == y["spawn"]
        assert all(np.allclose(x["parts"][n], y["parts"][n]) for n in x["parts"])
    assert len({l["spawn"] for l in a}) == 5            # and different seeds differ


def test_m1_random_is_the_range_shorthand(tmp_path):
    """A room written the M1 way keeps drawing exactly as before (no `randomize:`)."""
    env = env_of(tmp_path, parts={"goal": {"random": [0.1, 0.4]}})
    assert not env.randomizer.entries
    xy = np.array([l["parts"]["goal"] for l in draws(env, range(50))])
    assert np.all(np.abs(xy - np.array([1.1, 0.0])) <= np.array([0.1, 0.4]) + 1e-9)
    assert np.ptp(xy[:, 1]) > 0.3


def test_a_value_is_drawn_from_the_room_value_not_the_last_draw(tmp_path):
    env = env_of(tmp_path, parts={"fan": {"randomize": {"power": {"range": 0.3}}}})
    vals = [l["values"]["fan"]["power"] for l in draws(env, range(200))]
    nominal = env.parts["fan"].room_value("power")
    assert np.all(np.abs(np.array(vals) - nominal) <= 0.3 + 1e-9)
    env.reset(seed=1)
    assert env.parts["fan"].live["power"] == pytest.approx(env.layout["values"]["fan"]["power"])


def test_a_whole_number_value_stays_whole(tmp_path):
    """`range` on an `int` value rounds (a cannon cannot fire 2.4 shells)."""
    path = room_file(tmp_path, name="ints.yaml")
    raw = yaml.safe_load(path.read_text())
    raw["parts"].append({"name": "gun", "preset": "trap/cannon", "pos": [-1.3, 0.8],
                         "yaw": 0.0, "in": ["switch_a"], "burst": 3,
                         "randomize": {"burst": {"range": 2}}})
    path.write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(room=path)
    seen = {l["values"]["gun"]["burst"] for l in draws(env, range(60))}
    assert seen <= {1, 2, 3, 4, 5} and len(seen) > 2
    assert all(isinstance(v, int) for v in seen)


def test_min_gap_ignores_the_part_s_own_place(tmp_path):
    """The gap is to the walls and the other parts; a region around the part's own
    nominal place must not reject every draw."""
    env = env_of(tmp_path, parts={"switch_a": {"randomize": {"pos": {
        "region": [[-0.75, -0.45, 0.35, 0.65]], "min_gap": 0.3}}}})
    for layout in draws(env, range(10)):
        assert layout["draws"] == 1 and not layout["why"]


# ---- the task's strengths -----------------------------------------------------------
def test_a_strength_scales_the_room_range(tmp_path):
    env = env_of(tmp_path, parts={"switch_a": {"randomize": {"pos": {"range": [0.4, 0.4]}}}},
                 task={"randomize": {"layout": 0.25}})
    off = np.array([l["parts"]["switch_a"] for l in draws(env, range(100))]) - [-0.6, 0.5]
    assert np.all(np.abs(off) <= 0.1 + 1e-9) and np.abs(off).max() > 0.05


def test_zero_pins_a_layer_to_the_nominal_value(tmp_path):
    env = env_of(tmp_path, parts={"switch_a": {"randomize": {"pos": {"range": [0.4, 0.4]}}},
                                  "fan": {"randomize": {"power": {"range": 0.3}}}},
                 task={"randomize": {"layout": 0.0, "physics": 0.0}})
    assert not env.randomizer.active()
    for layout in draws(env, range(5)):
        assert np.allclose(layout["parts"]["switch_a"], [-0.6, 0.5])
        assert not layout["values"]


@pytest.mark.parametrize("raw, bad", [
    ({"layout": -1.0}, "must be a number >= 0"),
    ({"nonsense": 1.0}, "unknown layer"),
    ({"look": True}, "not available yet"),
    ({"rooms": 1.0}, "not available yet"),
    ([1.0], "must be a mapping"),
])
def test_bad_task_strengths_are_refused(tmp_path, raw, bad):
    with pytest.raises(TaskError) as exc:
        env_of(tmp_path, task={"randomize": raw})
    assert bad in str(exc.value)


# ---- the draw modes -----------------------------------------------------------------
def test_nominal_draws_nothing(tmp_path):
    env = env_of(tmp_path, parts={"switch_a": {"randomize": {"pos": {"range": [0.4, 0.4]}}}},
                 spawn={"random": [0.1, 0.1, 0.5]})
    layout = env.sample_layout(np.random.default_rng(0), "nominal")
    assert np.allclose(layout["parts"]["switch_a"], [-0.6, 0.5])
    assert layout["spawn"] == (-1.2, 0.0, 0.0)


def test_wide_uses_the_evaluation_only_values(tmp_path):
    env = env_of(tmp_path, parts={
        "goal": {"randomize": {"pos": {"candidates": [[1.1, 0.0], [1.1, 0.4]],
                                       "holdout": [[0.9, -0.4]]}}},
        "fan": {"randomize": {"power": {"range": 0.2, "holdout_scale": 2.0}}}})
    nominal = env.parts["fan"].room_value("power")
    for layout in draws(env, range(20), "wide"):
        assert np.allclose(layout["parts"]["goal"], [0.9, -0.4])
        off = abs(layout["values"]["fan"]["power"] - nominal)
        assert 0.2 - 1e-9 <= off <= 0.4 + 1e-9        # the band outside the training range


def test_wide_leaves_a_sampler_without_a_holdout_alone(tmp_path):
    env = env_of(tmp_path, parts={
        "switch_a": {"randomize": {"pos": {"region": [[-1.2, 0.0, -0.9, 0.9]]}}},
        "goal": {"randomize": {"pos": {"candidates": [[1.1, 0.0]], "holdout": [[1.1, 0.4]]}}}})
    assert env.randomizer.wide_missing() == ["part 'switch_a' randomize.pos"]
    layout = env.sample_layout(np.random.default_rng(0), "wide")
    assert np.allclose(layout["parts"]["switch_a"], [-0.6, 0.5])
    assert np.allclose(layout["parts"]["goal"], [1.1, 0.4])


def test_wide_is_refused_when_the_room_has_no_evaluation_only_value(tmp_path):
    env = env_of(tmp_path, parts={"switch_a": {"randomize": {
        "pos": {"region": [[-1.2, 0.0, -0.9, 0.9]]}}}})
    with pytest.raises(randomize.RandomizeError) as exc:
        env.sample_layout(np.random.default_rng(0), "wide")
    assert "needs an evaluation-only value" in str(exc.value)


# ---- what a room may declare --------------------------------------------------------
@pytest.mark.parametrize("parts, spawn, bad", [
    ({"switch_a": {"random": [0.1, 0.1], "randomize": {"pos": {"range": [0.1, 0.1]}}}}, None,
     "not both"),
    ({"switch_a": {"randomize": {"pos": {"range": [0.1, 0.1, 0.1]}}}}, None,
     "`range` for a position is [dx, dy]"),
    ({"switch_a": {"randomize": {"pos": {"range": [-0.1, 0.1]}}}}, None, ">= 0"),
    ({"switch_a": {"randomize": {"pos": {"region": [[0.5, -0.5, 0.0, 1.0]]}}}}, None,
     "needs x1 > x0"),
    ({"switch_a": {"randomize": {"pos": {"region": [[-1.0, 0.0, -1.0, 1.0]],
                                         "min_gap": -0.1}}}}, None, "`min_gap` must be"),
    ({"switch_a": {"randomize": {"pos": {"candidates": [[0.0, 0.0]], "weights": [1, 2]}}}},
     None, "`weights` must be 1 numbers"),
    ({"switch_a": {"randomize": {"pos": {"range": [0.1, 0.1], "region": [[0, 1, 0, 1]]}}}},
     None, "exactly one of"),
    ({"switch_a": {"randomize": {"pos": {}}}}, None, "exactly one of"),
    ({"switch_a": {"randomize": {"size": {"range": 0.1}}}}, None,
     "only `pos` and values marked run-time"),
    ({"fan": {"randomize": {"power": {"region": [[0, 1, 0, 1]]}}}}, None,
     "draws a position, not a value"),
    ({"fan": {"randomize": {"power": {"candidates": [0.5, 0.9], "pick": 2}}}}, None,
     "`pick` must be 1"),
    ({"fan": {"randomize": {"power": {"candidates": [99.0]}}}}, None, "power"),
    ({"switch_a": {"randomize": {"pos": {"range": [0.1, 0.1], "holdout_scale": 0.5}}}}, None,
     "`holdout_scale` must be"),
    ({"switch_a": {"randomize": []}}, None, "must be a mapping"),
    (None, {"randomize": {"nonsense": {"range": 1.0}}}, "takes pos and yaw"),
    (None, {"random": [0.1, 0.1, 0.1], "randomize": {"pos": {"range": [0.1, 0.1]}}},
     "not both"),
])
def test_bad_randomize_blocks_are_refused(tmp_path, parts, spawn, bad):
    with pytest.raises(RoomError) as exc:
        env_of(tmp_path, parts=parts, spawn=spawn)
    assert bad in str(exc.value)


def test_a_bound_value_cannot_be_randomized(tmp_path):
    with pytest.raises(RoomError) as exc:
        env_of(tmp_path, parts={"fan": {
            "bind": {"power": {"from": "switch_a", "map": [0, 1, 0.5, 1.0]}},
            "randomize": {"power": {"range": 0.2}}}})
    assert "driven by a `bind`" in str(exc.value)


def test_a_value_cannot_be_both_a_difficulty_step_and_randomized(tmp_path):
    with pytest.raises(RoomError) as exc:
        env_of(tmp_path, parts={"fan": {"randomize": {"power": {"range": 0.2}}}},
               task={"difficulty": [{"param": "fan.power", "from": 0.5, "to": 1.0, "steps": 2}]})
    assert "both a difficulty step and randomized" in str(exc.value)


def test_max_resample_must_be_a_count(tmp_path):
    with pytest.raises(RoomError) as exc:
        env_of(tmp_path, parts={"switch_a": {"randomize": {"pos": {"range": [0.1, 0.1]}}}},
               max_resample=0)
    assert "`max_resample` must be" in str(exc.value)


# ---- the redraw loop and the `resample` stage ---------------------------------------
def unsolvable(tmp_path, **top):
    """The switch is drawn behind the shut door, so no draw has a solution order."""
    return room_file(tmp_path, parts={"switch_a": {"randomize": {"pos": {
        "candidates": [[1.2, 0.6]]}}}}, **top)


def test_an_unsolvable_draw_is_thrown_away(tmp_path):
    env = PuzzleEnv(room=unsolvable(tmp_path, max_resample=3))
    layout = env.sample_layout(np.random.default_rng(0))
    assert layout["draws"] == 3 and layout["why"]


def test_the_resample_stage_refuses_a_room_that_runs_out_of_draws(tmp_path):
    res = check(unsolvable(tmp_path, max_resample=2), static_only=True)
    assert not res["ok"] and res["failed"] == ["resample"]
    assert "found no solvable room within max_resample 2" in res["resample"]["reason"]
    assert res["static"]["failed"] == ["resample"]      # the solver stage is then skipped


def test_the_resample_stage_warns_when_many_draws_are_thrown_away(tmp_path):
    """One of the two candidates is behind the shut door, so about half the draws go."""
    path = room_file(tmp_path, parts={"switch_a": {"randomize": {"pos": {
        "candidates": [[-0.6, 0.5], [1.2, 0.6]]}}}})
    res = check(path, static_only=True)
    assert res["ok"], res["failed"]
    assert res["resample"]["ratio"] > RESAMPLE_WARN
    assert any("thrown away" in w for w in res["resample"]["warnings"])


def test_the_resample_stage_is_skipped_without_randomization(tmp_path):
    res = check(room_file(tmp_path), static_only=True)
    assert res["ok"] and res["resample"] == {"ok": True,
                                             "skipped": "the room declares no `randomize:`"}


def test_a_randomized_room_still_passes_every_static_stage(tmp_path):
    path = room_file(tmp_path, parts={
        "switch_a": {"randomize": {"pos": {"region": [[-1.2, 0.0, -0.9, 0.9]], "min_gap": 0.2}}},
        "fan": {"randomize": {"power": {"range": 0.2}}},
        "goal": {"randomize": {"pos": {"candidates": [[1.1, 0.0], [1.1, 0.5]],
                                       "holdout": [[0.9, -0.5]]}}}})
    res = check(path, static_only=True)
    assert res["ok"], {k: res[k] for k in res["failed"]}
    assert res["resample"]["ok"] and res["resample"]["seeds"] == 100


# ---- the `randomized` observation ---------------------------------------------------
def test_the_randomized_item_reports_the_drawn_values(tmp_path):
    env = env_of(tmp_path, parts={"fan": {"randomize": {"power": {"range": 0.2}}}},
                 task={"obs": ["part_pos", "randomized"]})
    nominal = env.parts["fan"].room_value("power")
    seen = []
    for s in range(5):
        obs, _ = env.reset(seed=s)
        drawn = env.layout["values"]["fan"]["power"]
        assert obs[-1] == pytest.approx((drawn - nominal) / 0.2, abs=1e-6)
        seen.append(obs[-1])
    assert -1.0 <= min(seen) and max(seen) <= 1.0 and len(set(np.round(seen, 6))) == 5
    obs, _ = env.reset(seed=0, options={"randomize": "nominal"})
    assert obs[-1] == 0.0


def test_the_randomized_item_keeps_its_length(tmp_path):
    env = env_of(tmp_path, parts={"fan": {"randomize": {"power": {"range": 0.2}}},
                                  "goal": {"randomize": {"pos": {"range": [0.1, 0.1]}}}},
                 task={"obs": ["randomized"]})
    assert env.observation_space.shape == (1,)          # positions are not in it
    assert all(env.reset(seed=s)[0].shape == (1,) for s in range(3))


def test_a_room_without_randomized_values_gives_an_empty_item(tmp_path):
    env = env_of(tmp_path, task={"obs": ["part_pos", "randomized"]})
    assert env.randomizer.observation(env.layout, env.parts).shape == (0,)
