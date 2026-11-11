"""R27: the gaze sensor (geometry, not recognition): fov, distance, facing,
occlusion, dwell -> `<part>.seen`, and the `markers_seen` obs item.

The body only turns (plan 7b): poses are set by the layout / warps, the head is
not tilted.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym import gaze
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.obs import obs_kind
from mujoco_puzzle_gym.room import Room, RoomError

STAND = np.array([-1.0, 0.0, 0.0], dtype=np.float32)
SLOT = 5


def _room(tmp_path, walls=(), gaze_cfg=None, face="back"):
    raw = {"name": "gaze", "size": [3.6, 3.0], "wall": {"thickness": 0.06, "height": 0.5},
           "walls": [{"from": list(a), "to": list(b)} for a, b in walls],
           "spawn": {"pos": [0.0, 0.0], "yaw": 0.0},
           "parts": [
               {"name": "door_a", "type": "door", "pos": [0.8, 0.0], "height": 0.3,
                "marker": {"id": 7, "size": 0.12, "face": face}},
               {"name": "door_b", "type": "door", "pos": [-1.2, 1.2], "axis": "x",
                "in": ["door_a.seen"]},
               {"name": "goal", "type": "goal", "pos": [-1.4, -1.2]}]}
    if gaze_cfg:
        raw["gaze"] = gaze_cfg
    path = tmp_path / "gaze.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


def _env(path):
    return PuzzleEnv(task={"obs": ["markers_seen"], "env": {"hz": 10}}, room=path)


def _look(env, x=0.0, y=0.0, yaw=0.0):
    """Reset with the body at (x, y, yaw); markers_seen slots as rows (id order)."""
    layout = env.sample_layout(None)
    layout["spawn"] = (x, y, yaw)
    obs, _ = env.reset(options={"layout": layout})
    return obs.reshape(-1, SLOT)


def _slot7(env, rows):
    ids = sorted({p.marker.id for p in env.parts.values() if p.marker is not None})
    return rows[ids.index(7)]


def test_slots_are_fixed_and_in_id_order(tmp_path, asset_dir):
    env = _env(_room(tmp_path))
    ids = sorted(p.marker.id for p in env.parts.values() if p.marker is not None)
    assert ids == [0, 7]                                    # door_b auto 0, door_a 7
    assert env.observation_space.shape == (len(ids) * SLOT,)
    assert obs_kind({"obs": ["gravity", "markers_seen"]}) == "sensors_only"


def test_seen_when_facing_it(tmp_path, asset_dir):
    env = _env(_room(tmp_path))
    row = _slot7(env, _look(env))
    assert row[0] == 1.0
    assert row[1] == pytest.approx(0.78, abs=0.01) and abs(row[2]) < 0.01   # door face at x = 0.78
    assert row[3:] == pytest.approx([-1.0, 0.0], abs=1e-6)                   # face points back at us


@pytest.mark.parametrize("yaw,seen", [(math.radians(15), 1.0), (math.radians(-15), 1.0),
                                      (math.radians(45), 0.0), (math.radians(-90), 0.0),
                                      (math.pi, 0.0)])
def test_field_of_view(tmp_path, asset_dir, yaw, seen):
    env = _env(_room(tmp_path))                             # fov 60: half 30 degrees
    assert _slot7(env, _look(env, yaw=yaw))[0] == seen


def test_out_of_range(tmp_path, asset_dir):
    env = _env(_room(tmp_path, gaze_cfg={"max_dist": 0.6}))
    assert _slot7(env, _look(env))[0] == 0.0
    assert _slot7(env, _look(env, x=0.3))[0] == 1.0         # 0.48 m away


def test_back_face_is_not_seen(tmp_path, asset_dir):
    env = _env(_room(tmp_path))
    assert _slot7(env, _look(env, x=1.6, yaw=math.pi))[0] == 0.0   # marker on the far side
    env = _env(_room(tmp_path, face="front"))
    assert _slot7(env, _look(env, x=1.6, yaw=math.pi))[0] == 1.0
    assert _slot7(env, _look(env))[0] == 0.0


def test_tilt_limit(tmp_path, asset_dir):
    # seen from 45 degrees off the face normal, not with max_tilt 30
    pose = dict(x=0.8 - 0.55, y=0.55, yaw=-math.pi / 4)
    env = _env(_room(tmp_path))
    assert _slot7(env, _look(env, **pose))[0] == 1.0
    env = _env(_room(tmp_path, gaze_cfg={"max_tilt": 30}))
    assert _slot7(env, _look(env, **pose))[0] == 0.0


def test_wall_hides_it(tmp_path, asset_dir):
    env = _env(_room(tmp_path, walls=[((0.4, -0.3), (0.4, 0.3))]))
    assert _slot7(env, _look(env))[0] == 0.0
    assert _slot7(env, _look(env, y=-1.0, yaw=math.atan2(1.0, 0.78)))[0] == 1.0   # past the wall end


def test_dwell_sets_the_seen_channel(tmp_path, asset_dir):
    env = _env(_room(tmp_path, gaze_cfg={"dwell": 0.5}))
    assert "_gaze" in env.parts and env.parts["_gaze"].outputs == ["door_a.seen"]
    _look(env)
    for _ in range(3):                                      # 0.3 s < dwell
        env.step(STAND)
    assert env.signals.get("door_a.seen") == 0.0
    env.warp("body", (0.0, 0.0, math.pi))                   # look away: the timer restarts
    env.step(STAND)
    assert env.signals.get("door_a.seen") == 0.0
    env.warp("body", (0.0, 0.0, 0.0))
    for _ in range(3):                                      # 0.3 s again: still short
        env.step(STAND)
    assert env.signals.get("door_a.seen") == 0.0
    for _ in range(3):                                      # 0.6 s without a break
        env.step(STAND)
    assert env.signals.get("door_a.seen") == 1.0
    for _ in range(20):                                     # door_b reads the channel and opens
        env.step(STAND)
    assert env.parts["door_b"].state() > 0.5


def test_no_gaze_part_without_readers(tmp_path):
    raw = yaml.safe_load(_room(tmp_path).read_text())
    raw["parts"][1].pop("in")
    parts = Room.from_dict(raw).make_parts()
    assert "_gaze" not in parts


@pytest.mark.parametrize("change,needle,stage", [
    ({"gaze": {"fov": 0}}, "gaze.fov", "values"),
    ({"gaze": {"speed": 1}}, "`gaze:` takes", "values"),
    ({"seen_of": "goal"}, "has no marker", "channels"),
])
def test_bad_gaze_settings(tmp_path, change, needle, stage):
    raw = yaml.safe_load(_room(tmp_path).read_text())
    if "seen_of" in change:
        raw["parts"][1]["in"] = [change["seen_of"] + ".seen"]
    else:
        raw.update(change)
    with pytest.raises(RoomError) as exc:
        Room.from_dict(raw)
    assert needle in str(exc.value) and exc.value.stage == stage


def test_config_defaults():
    assert gaze.parse_config(None) == gaze.GazeConfig()
    assert gaze.parse_config({"dwell": 0}).dwell == 0.0
