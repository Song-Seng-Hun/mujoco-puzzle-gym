"""R10: tile floors and the floor_tile part (friction, tilt, height, fall rule)."""

from __future__ import annotations

import math

import mujoco
import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.floor import Rect, cover
from mujoco_puzzle_gym.room import RoomError

from rooms import open_room, stand_at
from test_doors import Source, run  # noqa: F401  (registers t_signal)


def tile_env(tmp_path, name="tile", spawn=(-1.0, -1.0), **tile):
    return open_room(tmp_path, [
        {"name": "s", "type": "t_signal", "pos": [1.0, -1.0]},
        {"name": "ft", "type": "floor_tile", "pos": [0.3, 0.3], "size": [0.4, 0.4], "in": ["s"], **tile},
    ], spawn=spawn, name=name)


# ---- gapless cover ---------------------------------------------------------------------
def test_cover_tiles_the_area_without_gaps():
    area = Rect(-1.5, 1.5, -1.0, 1.0)
    holes = [Rect(-0.2, 0.2, -0.2, 0.2), Rect(0.5, 0.9, -1.2, -0.6), Rect(-1.5, -1.1, 0.7, 1.0)]
    tiles = cover(area, holes)
    covered = sum((t.x1 - t.x0) * (t.y1 - t.y0) for t in tiles)
    hole_area = (0.4 * 0.4) + (0.4 * 0.4) + (0.4 * 0.3)
    assert covered == pytest.approx(3.0 * 2.0 - hole_area)
    for i, a in enumerate(tiles):
        assert not any(a.overlaps(h) for h in holes)
        assert not any(a.overlaps(b) for b in tiles[i + 1:])
    rng = np.random.default_rng(0)
    for xy in rng.uniform([-1.5, -1.0], [1.5, 1.0], size=(500, 2)):
        n = sum(t.contains(xy, 1e-9) for t in tiles) + sum(h.contains(xy, 1e-9) for h in holes)
        assert n >= 1, xy


def test_plain_rooms_keep_the_plane(tmp_path):
    env = open_room(tmp_path, [], name="plain")
    assert env.floor is None
    assert env.model.geom("floor").pos[2] == 0.0


def test_room_floor_friction(tmp_path):
    raw = {"name": "slick", "size": [2, 2], "spawn": {"pos": [-0.8, -0.8]},
           "floor": {"friction": [0.2, 0.005, 0.0001]},
           "parts": [{"name": "goal", "type": "goal", "pos": [0.8, 0.8]}]}
    path = tmp_path / "slick.yaml"
    path.write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(room=path)
    g = env.model.geom("floor")
    assert g.friction[0] == pytest.approx(0.2) and g.priority[0] == 1
    raw["parts"].append({"name": "ft", "type": "floor_tile", "pos": [0, 0]})
    path.write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(room=path)
    tiles = [env.model.geom(i) for i in range(env.model.ngeom)
             if env.model.geom(i).name.startswith("floor/tile/")]
    assert tiles and all(t.friction[0] == pytest.approx(0.2) and t.priority[0] == 1 for t in tiles)


# ---- runtime values -----------------------------------------------------------------------
@pytest.mark.parametrize("key, off, on, speed_key, speed", [
    ("friction", [1.0, 0.005, 0.0001], [0.2, 0.005, 0.0001], "friction_rate", 0.8),
    ("tilt", 0.0, 8.0, "tilt_speed", 8.0),
    ("height", 0.0, 0.04, "height_speed", 0.04),
])
def test_signal_moves_value_at_set_speed(tmp_path, key, off, on, speed_key, speed):
    env = tile_env(tmp_path, name=key, **{key: off, "on_" + key: on, speed_key: speed})
    ft = env.parts["ft"]

    def value():
        v = ft.values(env.data)[key]
        return float(np.asarray(v).ravel()[0])

    off0, on0 = float(np.ravel(off)[0]), float(np.ravel(on)[0])
    run(env, np.zeros(3), 0.5)
    assert value() == pytest.approx(off0, abs=0.02 * abs(on0 - off0))
    run(env, np.zeros(3), 0.5, hold_on="s")           # half way at the set speed
    assert value() == pytest.approx((off0 + on0) / 2, abs=0.08 * abs(on0 - off0))
    run(env, np.zeros(3), 0.6, hold_on="s")           # ramp done (1 s) plus 0.1 s settling
    assert value() == pytest.approx(on0, abs=0.02 * abs(on0 - off0))
    if key == "friction":
        assert env.model.geom_friction[ft._gid][0] == pytest.approx(on0)
    run(env, np.zeros(3), 1.2)                         # back to the off value
    assert value() == pytest.approx(off0, abs=0.02 * abs(on0 - off0))


def test_static_tile_has_no_joints(tmp_path):
    env = open_room(tmp_path, [{"name": "ft", "type": "floor_tile", "pos": [0.3, 0.3], "tilt": 5.0}],
                    name="static")
    assert env.model.body("part/ft").jntnum[0] == 0
    top, _ = env.floor_height((0.3, 0.4))                 # +y side raised (axis x)
    assert top == pytest.approx(0.1 * math.tan(math.radians(5.0)))


def test_out_of_range_tilt_is_rejected(tmp_path):
    for k, v in (("tilt", 20.0), ("on_tilt", -14.0), ("height", 0.3)):
        with pytest.raises(RoomError, match=r"out of range"):
            open_room(tmp_path, [{"name": "ft", "type": "floor_tile", "pos": [0, 0], k: v}], name=k)


@pytest.mark.parametrize("friction, slides", [(0.05, True), (1.0, False)])
def test_cube_on_tilted_tile(tmp_path, friction, slides):
    # tan(10 deg) = 0.18: slides at friction 0.05, stays at 1.0 (tile friction wins: priority 1)
    env = open_room(tmp_path, [
        {"name": "ft", "type": "floor_tile", "pos": [0.0, 0.0], "size": [0.8, 0.8], "tilt": 10.0,
         "friction": [friction, 0.005, 0.0001]},
        {"name": "box", "type": "t_box", "pos": [0.0, 0.0], "half": 0.03},
    ], spawn=(-1.0, -1.0), name=f"slope{friction}")
    box = env.parts["box"]
    a = math.radians(10.0)
    box.put(env.data, [0.0, 0.0, 0.03 / math.cos(a) + 0.001])
    env.data.qpos[box._q + 3:box._q + 7] = [math.cos(a / 2), math.sin(a / 2), 0, 0]
    mujoco.mj_forward(env.model, env.data)
    start = box.pos(env.data)
    run(env, np.zeros(3), 1.0)
    moved = np.linalg.norm(box.pos(env.data)[:2] - start[:2])
    assert (moved > 0.1) if slides else (moved < 0.01), moved


# ---- fall rule (R10) ------------------------------------------------------------------------
@pytest.mark.parametrize("height", [-0.06, 0.06])
def test_standing_on_low_or_high_tile_is_not_a_fall(tmp_path, height):
    env = open_room(tmp_path, [{"name": "ft", "type": "floor_tile", "pos": [0.0, 0.0],
                                "size": [0.6, 0.6], "height": height}], spawn=(0.0, 0.0),
                    name=f"h{height}")
    assert env.body_state.pos[2] == pytest.approx(0.125 + height, abs=1e-6)
    assert not run(env, np.zeros(3), 2.0)
    assert env.floor_height(env.body_state.pos[:2])[0] == pytest.approx(height)
    # the same trunk height over the room floor would be a fall on a low tile only
    assert env.body.state(env.data).fallen == (height < 0)


def test_tilted_tile_under_the_body_is_not_a_fall(tmp_path):
    env = open_room(tmp_path, [{"name": "ft", "type": "floor_tile", "pos": [0.0, 0.0],
                                "size": [0.6, 0.6], "tilt": 8.0}], spawn=(0.0, 0.2), name="tilt")
    assert not run(env, np.zeros(3), 2.0)


@pytest.mark.slow
def test_walk_on_tile_floor(tmp_path):
    """M1-R3 walk test (vx = 0.3, 5 s, 0.5-1.1 m) on a tile floor, across tile seams."""
    tiles = [{"name": f"t{i}", "type": "floor_tile", "pos": [-0.6 + 0.3 * i, 0.0], "size": [0.3, 0.6]}
             for i in range(6)]
    env = open_room(tmp_path, tiles, spawn=(-0.75, 0.0), name="walk")
    assert env.floor is not None
    start = env.body_state.pos.copy()
    fell = run(env, np.array([0.3, 0.0, 0.0]), 5.0)
    assert not fell
    forward = env.body_state.pos[0] - start[0]
    assert 0.5 < forward < 1.1, forward
    assert abs(env.body_state.pos[1] - start[1]) < 0.3
