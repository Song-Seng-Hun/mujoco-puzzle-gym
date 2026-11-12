"""R13: setter (change a run-time value on a signal edge) and `bind` (value follows a channel)."""

from __future__ import annotations

import pytest

from mujoco_puzzle_gym.check import check
from mujoco_puzzle_gym.room import RoomError

from rooms import body_steps, open_room, tick
from test_doors import Source  # noqa: F401  (registers t_signal)

SIG = {"name": "s", "type": "t_signal", "pos": [1.0, -1.0]}
SPRING = {"name": "sp", "preset": "device/spring_pad", "pos": [0.5, 0.5], "velocity": [0.0, 0.0, 1.0]}


def setter_env(tmp_path, name="st", **setter):
    st = {"name": "set", "type": "setter", "in": ["s"], "target": "sp.velocity", "value": [0.5, 0.0, 2.0],
          **setter}
    return open_room(tmp_path, [SIG, SPRING, st], spawn=(-1.0, -1.0), name=name)


def test_setter_changes_spring_strength(tmp_path):
    env = setter_env(tmp_path)
    sp = env.parts["sp"]
    tick(env)
    assert sp.live["velocity"] == [0.0, 0.0, 1.0]
    env.parts["s"].value = 1.0
    tick(env)
    assert sp.live["velocity"] == [0.5, 0.0, 2.0]
    env.parts["s"].value = 0.0                     # no revert: stays
    tick(env)
    assert sp.live["velocity"] == [0.5, 0.0, 2.0]
    env.reset(seed=0)                              # reset: back to the room value
    assert sp.live["velocity"] == [0.0, 0.0, 1.0]


def test_setter_revert(tmp_path):
    env = setter_env(tmp_path, name="rv", revert=True)
    sp = env.parts["sp"]
    env.parts["s"].value = 1.0
    tick(env)
    assert sp.live["velocity"] == [0.5, 0.0, 2.0]
    env.parts["s"].value = 0.0
    tick(env)
    assert sp.live["velocity"] == [0.0, 0.0, 1.0]


def test_setter_launches_with_the_new_strength(tmp_path):
    env = open_room(tmp_path, [SIG, {**SPRING, "trigger": "object", "in": ["s"]},
                               {"name": "set", "type": "setter", "in": ["s"], "target": "sp.velocity",
                                "value": [0.0, 0.0, 2.0]},
                               {"name": "box", "type": "t_box", "pos": [0.5, 0.5], "half": 0.03}],
                    spawn=(-1.0, -1.0), name="launch")
    env.parts["s"].value = 1.0
    peak = []
    body_steps(env, 0.6, every=lambda e: peak.append(e.parts["box"].pos(e.data)[2]))
    assert max(peak) - 0.03 == pytest.approx(2.0 ** 2 / (2 * 9.81), rel=0.1)   # 2 m/s, not 1


def test_size_values_are_rejected_at_build(tmp_path):
    door = {"name": "d", "type": "door", "pos": [0.0, 0.8], "in": ["s"]}
    with pytest.raises(RoomError, match="cannot change while running: 'width'"):
        open_room(tmp_path, [SIG, door, {"name": "set", "type": "setter", "in": ["s"],
                                         "target": "d.width", "value": 1.0}], name="dw")
    with pytest.raises(RoomError, match="out of range"):
        setter_env(tmp_path, name="oor", value=[0.0, 0.0, 9.0])
    with pytest.raises(RoomError, match="not a part"):
        setter_env(tmp_path, name="np", target="nope.velocity")
    with pytest.raises(RoomError, match="not a run-time value"):
        open_room(tmp_path, [SIG, {**door, "bind": {"width": {"from": "s", "map": [0, 1, 0.2, 0.5]}}}],
                  name="bw")


def test_door_size_setter_fails_mpg_check(tmp_path):
    room = tmp_path / "chk.yaml"
    room.write_text("""
name: chk
size: [2.0, 2.0]
spawn: {pos: [-0.6, -0.6], yaw: 0.0}
parts:
  - {name: b, preset: input/floor_oneshot, pos: [0.5, -0.5]}
  - {name: d, preset: device/sliding_door, pos: [0.0, 0.6], in: [b]}
  - {name: set, preset: device/setter, in: [b], target: d.width, value: 0.8}
  - {name: goal, type: goal, pos: [0.6, 0.6], radius: 0.15}
""")
    res = check(room, static_only=True)
    assert not res["values"]["ok"] and "width" in res["values"]["reason"]


def test_bind_follows_the_channel_and_clips(tmp_path):
    belt = {"name": "c", "preset": "device/conveyor", "pos": [0.3, 0.3],
            "bind": {"speed": {"from": "s", "map": [0.0, 1.0, 0.0, 0.4]}}}
    env = open_room(tmp_path, [SIG, belt], spawn=(-1.0, -1.0), name="bind")
    c = env.parts["c"]
    for value, speed in ((0.0, 0.0), (0.5, 0.2), (1.0, 0.4), (2.0, 0.5), (-3.0, -0.5)):
        env.parts["s"].value = value
        tick(env)                                   # source writes, then the bind reads it
        assert c.live["speed"] == pytest.approx(speed), value


def test_bind_and_setter_on_one_value_is_rejected(tmp_path):
    belt = {"name": "c", "preset": "device/conveyor", "pos": [0.3, 0.3],
            "bind": {"speed": {"from": "s", "map": [0.0, 1.0, 0.0, 0.4]}}}
    with pytest.raises(RoomError, match="bound"):
        open_room(tmp_path, [SIG, belt, {"name": "set", "type": "setter", "in": ["s"],
                                         "target": "c.speed", "value": 0.3}], name="both")
    with pytest.raises(RoomError, match="only number values"):
        open_room(tmp_path, [SIG, {**SPRING, "bind": {"velocity": {"from": "s", "map": [0, 1, 0, 1]}}}],
                  name="vec")


def test_setter_on_tile_tilt_builds_its_joint(tmp_path):
    tile = {"name": "t", "type": "floor_tile", "pos": [0.3, 0.3], "size": [0.4, 0.4], "tilt_speed": 20.0}
    env = open_room(tmp_path, [SIG, tile, {"name": "set", "type": "setter", "in": ["s"],
                                           "target": "t.tilt", "value": 8.0}], name="tilt")
    t = env.parts["t"]
    assert "tilt" in t._j
    env.parts["s"].value = 1.0
    body_steps(env, 1.0)
    assert t.values(env.data)["tilt"] == pytest.approx(8.0, abs=0.3)


def test_bind_on_on_tilt_builds_its_joint(tmp_path):
    # review fix: a bind on `on_tilt` needs the tilt joint even when the room sets no on_tilt
    tile = {"name": "t", "type": "floor_tile", "pos": [0.3, 0.3], "size": [0.4, 0.4], "in": ["s"],
            "bind": {"on_tilt": {"from": "s", "map": [0.0, 1.0, 0.0, 8.0]}}}
    env = open_room(tmp_path, [SIG, tile], name="ontilt")
    assert "tilt" in env.parts["t"]._j


def test_wind_power_by_setter_and_bind(tmp_path):
    """R13 on a trap: a setter changes the wind's power on an edge; a bind makes it follow a channel."""
    g = {"name": "g", "type": "t_signal", "pos": [1.0, 1.0]}
    wind = {"name": "w", "preset": "trap/crosswind", "pos": [-0.8, -0.8], "in": ["s"], "power": 1.0}
    env = open_room(tmp_path, [SIG, g, wind, {"name": "set", "type": "setter", "in": ["g"],
                                              "target": "w.power", "value": 6.0}],
                    spawn=(-1.0, -1.0), name="wset")
    w = env.parts["w"]
    tick(env)
    assert w.live["power"] == 1.0
    env.parts["g"].value = 1.0
    tick(env)
    assert w.live["power"] == 6.0
    env = open_room(tmp_path, [SIG, g, {**wind, "bind": {"power": {"from": "g", "map": [0.0, 1.0, 0.0, 8.0]}}}],
                    spawn=(-1.0, -1.0), name="wbind")
    w = env.parts["w"]
    for value, power in ((0.0, 0.0), (0.25, 2.0), (1.0, 8.0), (5.0, 20.0)):
        env.parts["g"].value = value
        tick(env)
        assert w.live["power"] == pytest.approx(power), value
