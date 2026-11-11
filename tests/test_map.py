"""R37: letter maps (shape, heights, low walls, ramps, cliffs) and their checks."""

from __future__ import annotations

import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym.check import check, check_order
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.room import Room, RoomError
from mujoco_puzzle_gym.roommap import RoomMap, merge_cells

# 'ㄱ'-shaped floor: a top bar and a right leg; the rest is cliff (no walls along it,
# except a short wall at the bar's left end).
GIEUK = """
..........
..........
###.......
       ...
       ...
       ...
"""


def write(tmp_path, raw, name="m"):
    raw = {"name": name, **raw}
    path = tmp_path / f"{name}.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


def env_of(tmp_path, raw, name="m"):
    env = PuzzleEnv(room=write(tmp_path, raw, name))
    env.reset(seed=0, options={"layout": env.sample_layout(None)})
    return env


def test_map_coordinates_and_merging():
    m = RoomMap(GIEUK, cell=0.1)
    assert (m.nrows, m.ncols) == (6, 10)
    assert m.centre(0, 0) == pytest.approx((-0.45, 0.25))       # top left: -x, +y
    assert m.cell_of((-0.45, 0.25)) == (0, 0)
    assert m.cell_of((0.44, -0.24)) == (5, 9)
    assert m.cell_of((0.6, 0.0)) is None
    assert m.kind_at((0.0, -0.2)) == ("void",)
    tiles = m.floor_tiles()
    area = sum((t.rect.x1 - t.rect.x0) * (t.rect.y1 - t.rect.y0) for t in tiles)
    assert area == pytest.approx(0.01 * (10 + 10 + 7 + 9))
    assert len(tiles) <= 3                                      # same-height cells merged
    mask = np.array([[1, 1, 0], [1, 1, 0], [0, 1, 1]], dtype=bool)
    rects = merge_cells(mask)
    cover = np.zeros_like(mask)
    for r0, r1, c0, c1 in rects:
        assert not cover[r0:r1 + 1, c0:c1 + 1].any()
        cover[r0:r1 + 1, c0:c1 + 1] = True
    assert (cover == mask).all()


def test_gieuk_room_builds_and_cliff_fall_within_2s(tmp_path):
    raw = {"map": GIEUK, "spawn": {"pos": [0.35, -0.15], "yaw": 3.14},
           "parts": [{"name": "goal", "type": "goal", "pos": [-0.35, 0.2], "radius": 0.08}]}
    env = env_of(tmp_path, raw, "gieuk")
    assert env.floor is not None and not env.room.outer_walls()
    assert not env.body_state.fallen
    assert check_order(env)["ok"]
    # push the body off the leg toward the cliff (-x) and see the fall within 2 s
    q, v = env.body._root_qpos, env.body._root_qvel
    env.data.qvel[v:v + 3] = [-0.8, 0.0, 0.0]
    for i in range(20):                                         # 20 env steps of 0.1 s
        _, _, terminated, _, info = env.step(np.array([-1.0, 0.0, 0.0]))
        if terminated:
            break
    assert terminated and info["fallen"], env.body_state.pos
    assert (i + 1) / env.hz <= 2.0
    assert env.floor_height(env.body_state.pos[:2])[0] is None


def test_on_fall_reset_goes_back_to_start(tmp_path):
    raw = {"map": GIEUK, "on_fall": "reset", "spawn": {"pos": [0.35, -0.15], "yaw": 3.14},
           "parts": [{"name": "goal", "type": "goal", "pos": [-0.35, 0.2], "radius": 0.08}]}
    env = env_of(tmp_path, raw, "reset")
    env.data.qvel[env.body._root_qvel] = -0.8
    respawned = False
    for _ in range(20):
        _, _, terminated, _, info = env.step(np.array([-1.0, 0.0, 0.0]))
        assert not terminated
        respawned |= info["respawned"]
        if respawned:
            break
    assert respawned
    assert np.hypot(*(env.body_state.pos[:2] - [0.35, -0.15])) < 0.05


def test_map_with_size_or_walls_is_rejected(tmp_path):
    base = {"map": GIEUK, "spawn": {"pos": [0, 0.2]},
            "parts": [{"name": "goal", "type": "goal", "pos": [-0.35, 0.2]}]}
    for extra in ({"size": [1, 1]}, {"walls": [{"from": [0, 0], "to": [0.2, 0]}]}):
        with pytest.raises(RoomError, match="map"):
            Room.load(write(tmp_path, {**base, **extra}, "bad"))
    with pytest.raises(RoomError, match="unknown symbol"):
        Room.load(write(tmp_path, {**base, "map": "..x.."}, "sym"))


STEP_ROOM = """
############
#.....3333 #
#.....3333 #
#.....3333 #
#.....3333 #
############
"""


def step_raw(map_text, legend=None):
    raw = {"map": map_text, "step_height": 0.01, "spawn": {"pos": [-0.35, 0.0], "yaw": 0.0},
           "parts": [{"name": "goal", "type": "goal", "pos": [0.25, 0.0], "radius": 0.08}]}
    if legend:
        raw["legend"] = legend
    return raw


def test_step_above_max_step_fails_order_with_cells(tmp_path):
    env = env_of(tmp_path, step_raw(STEP_ROOM), "step")
    res = check_order(env)
    assert not res["ok"]
    assert "cell (row 2, col 7)" in res["reason"] and "max_step" in res["reason"], res
    # a ramp up to the dais makes it reachable
    ramped = STEP_ROOM.replace("#.....3333 #\n#.....3333 #\n#.....3333 #\n#.....3333 #",
                               "#..////3333#\n#..////3333#\n#..////3333#\n#..////3333#")
    env = env_of(tmp_path, step_raw(ramped, {"/": {"type": "ramp", "dir": "+x", "from": 0.0,
                                                   "to": 0.03}}), "ramp")
    assert env.room.map.ramps[0].slope_deg < 5.0
    assert check_order(env)["ok"]


def test_steep_ramp_blocks(tmp_path):
    steep = STEP_ROOM.replace(".....3333 ", "....//3333")
    env = env_of(tmp_path, step_raw(steep, {"/": {"type": "ramp", "dir": "+x", "from": 0.0,
                                                  "to": 0.03}}), "steep")
    assert env.room.map.ramps[0].slope_deg > 5.0
    res = check_order(env)
    assert not res["ok"] and "max_slope" in res["reason"], res


def test_low_wall_height_from_legend(tmp_path):
    raw = {"map": "#####\n#.=.#\n#...#\n#####", "legend": {"=": {"type": "wall", "height": 0.06}},
           "spawn": {"pos": [-0.1, -0.05]},
           "parts": [{"name": "goal", "type": "goal", "pos": [0.1, -0.05], "radius": 0.05}]}
    env = env_of(tmp_path, raw, "low")
    tops = sorted({round(float(env.model.geom_pos[i][2] + env.model.geom_size[i][2]), 4)
                   for i in range(env.model.ngeom) if env.model.geom(i).name.startswith("wall/")})
    assert tops[0] == pytest.approx(0.06, abs=0.001)
    assert tops[-1] == pytest.approx(0.25, abs=0.001)


def test_spawn_and_parts_on_walls_or_cliffs_fail_with_cells(tmp_path):
    raw = {"map": GIEUK, "spawn": {"pos": [0.0, -0.15], "yaw": 0.0},
           "parts": [{"name": "b", "type": "button", "pos": [-0.35, 0.05], "size": [0.1, 0.1]},
                     {"name": "goal", "type": "goal", "pos": [-0.35, 0.2], "radius": 0.08,
                      "random": [0.2, 0.0]}]}
    env = env_of(tmp_path, raw, "place")
    res = check_order(env)
    assert not res["ok"]
    text = " | ".join(res["placement"])
    assert "spawn" in text and "no floor" in text and "cell (row 5, col 6)" in text
    assert "part 'b'" in text and "a wall" in text
    assert "part 'goal'" in text and "outside the map" in text


def test_raised_spawn_stands_on_its_cell(tmp_path):
    raw = {"map": "5555\n5555\n5555", "spawn": {"pos": [0.0, 0.0]},
           "parts": [{"name": "goal", "type": "goal", "pos": [0.1, 0.0], "radius": 0.05}]}
    env = env_of(tmp_path, raw, "raised")
    assert env.body_state.pos[2] == pytest.approx(0.125 + 0.05, abs=1e-6)
    for _ in range(10):
        env.advance(np.zeros(3))
    assert not env.body_state.fallen


@pytest.mark.slow
@pytest.mark.parametrize("room", ["two_rooms_map", "stairs_map"])
def test_example_map_rooms_pass_check(repo_root, room):
    res = check(repo_root / f"examples/rooms/{room}.yaml")
    assert res["ok"], res
    assert res["solver"]["successes"] >= 18
