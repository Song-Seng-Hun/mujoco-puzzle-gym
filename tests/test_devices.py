"""R12: lift pillar, turntable and conveyor (signal 0 vs 1), and R6 snapshots of the
R11-R15 devices."""

from __future__ import annotations

import numpy as np
import pytest

from rooms import body_steps, open_room
from test_doors import Source  # noqa: F401  (registers t_signal)

SIG = {"name": "s", "type": "t_signal", "pos": [1.0, -1.0]}


def env_with(tmp_path, part, name, extra=()):
    return open_room(tmp_path, [SIG, {**part, "in": ["s"]}, *extra], spawn=(-1.0, -1.0), name=name)


def test_pillar_rises_with_the_signal(tmp_path):
    env = env_with(tmp_path, {"name": "p", "preset": "device/lift_pillar", "pos": [0.0, 0.5],
                              "height": 0.25, "speed": 0.25}, "pillar")
    p = env.parts["p"]
    body_steps(env, 1.0)
    assert p.top() == pytest.approx(0.0, abs=0.002)
    env.parts["s"].value = 1.0
    body_steps(env, 0.5)
    assert p.top() == pytest.approx(0.125, abs=0.01)          # 0.25 m/s for 0.5 s
    body_steps(env, 1.0)
    assert p.top() == pytest.approx(0.25, abs=0.003)
    env.parts["s"].value = 0.0
    body_steps(env, 1.5)
    assert p.top() == pytest.approx(0.0, abs=0.003)          # follow: back down


def test_pillar_blocks_planning_once_raised(tmp_path):
    from mujoco_puzzle_gym.logic import free_mask
    env = env_with(tmp_path, {"name": "p", "preset": "device/lift_pillar", "pos": [0.0, 0.5]}, "pplan")
    cell = env.grid.cell((0.0, 0.5))
    assert free_mask(env.grid, env.walls, env.parts, set())[cell]
    assert not free_mask(env.grid, env.walls, env.parts, {"s"})[cell]


def test_turntable_spins_only_with_the_signal(tmp_path):
    box = {"name": "box", "type": "t_box", "pos": [1.0, 1.0], "half": 0.03}
    env = env_with(tmp_path, {"name": "t", "preset": "device/turntable", "pos": [0.0, 0.5],
                              "speed": 90.0}, "turn", extra=[box])
    t = env.parts["t"]
    env.parts["box"].put(env.data, [0.15, 0.5, 0.008 + 0.031])  # load near the rim
    body_steps(env, 1.0)
    assert abs(t.rate(env.data)) < 0.5 and abs(t.angle(env.data)) < 0.01
    env.parts["s"].value = 1.0
    body_steps(env, 0.5)
    a0 = t.angle(env.data)
    body_steps(env, 1.0)
    assert np.degrees(t.angle(env.data) - a0) == pytest.approx(90.0, rel=0.05)
    p = env.parts["box"].pos(env.data)
    assert np.hypot(p[0], p[1] - 0.5) == pytest.approx(0.15, abs=0.02)     # carried round
    assert np.arctan2(p[1] - 0.5, p[0]) > 0.5
    env.parts["s"].value = 0.0
    body_steps(env, 1.0)
    assert abs(t.rate(env.data)) < 1.0


def test_conveyor_carries_a_cube_at_belt_speed(tmp_path):
    box = {"name": "box", "type": "t_box", "pos": [1.0, 1.0], "half": 0.03}
    env = env_with(tmp_path, {"name": "c", "preset": "device/conveyor", "pos": [0.0, 0.5],
                              "size": [1.2, 0.3], "speed": 0.15}, "belt", extra=[box])
    b = env.parts["box"]
    b.put(env.data, [-0.4, 0.5, 0.03])
    body_steps(env, 1.0)
    assert b.pos(env.data)[0] == pytest.approx(-0.4, abs=0.002)        # signal 0: still
    env.parts["s"].value = 1.0
    x0 = b.pos(env.data)[0]
    body_steps(env, 4.0)
    v = (b.pos(env.data)[0] - x0) / 4.0
    assert v == pytest.approx(0.15, rel=0.05)
    body_steps(env, 3.0)                                               # off the end: stops
    x1 = b.pos(env.data)[0]
    body_steps(env, 1.0)
    assert x1 > 0.6 and b.pos(env.data)[0] == pytest.approx(x1, abs=0.01)


def test_conveyor_moves_the_body(tmp_path):
    env = env_with(tmp_path, {"name": "c", "preset": "device/conveyor", "pos": [-1.0, -1.0],
                              "axis": "y", "speed": 0.1}, "bbelt")
    y0 = env.read_body().pos[1]
    body_steps(env, 0.5)
    assert env.read_body().pos[1] == pytest.approx(y0, abs=0.01)
    env.parts["s"].value = 1.0
    body_steps(env, 1.0)
    assert env.read_body().pos[1] - y0 == pytest.approx(0.1, abs=0.02)


@pytest.mark.slow
@pytest.mark.parametrize("speed", [0.1, 0.2])
def test_body_on_belt_record(tmp_path, speed):
    """Record only (R12): does the walking body stay up 3 s standing on a running belt?"""
    env = env_with(tmp_path, {"name": "c", "preset": "device/conveyor", "pos": [-0.6, -1.0],
                              "size": [1.6, 0.4], "speed": speed}, "rbelt")
    env.parts["s"].value = 1.0
    start = env.read_body().pos.copy()
    fell = body_steps(env, 3.0)
    moved = env.read_body().pos - start
    print(f"\nR12 record: body on belt {speed} m/s 3 s: fell={fell}, moved={moved.round(3).tolist()}")


# ---- R6: snapshots restore the new devices ---------------------------------------------
def test_snapshot_restores_devices(tmp_path):
    parts = [
        {"name": "pl", "type": "platform", "pos": [-0.5, 0.8], "path": [[0, 0, 0.03], [1.0, 0, 0.03]],
         "speed": 0.2, "in": ["s"], "run": "on_signal"},
        {"name": "p", "preset": "device/lift_pillar", "pos": [0.8, -0.3], "in": ["s"]},
        {"name": "t", "preset": "device/turntable", "pos": [-0.5, -0.2], "in": ["s"]},
        {"name": "c", "preset": "device/conveyor", "pos": [0.3, 0.3], "in": ["s"]},
        {"name": "sp", "preset": "device/spring_pad", "pos": [1.0, 0.5], "in": ["s"]},
        {"name": "set", "type": "setter", "in": ["s"], "target": "sp.velocity", "value": [0.0, 0.0, 2.0],
         "revert": True},
        {"name": "box", "type": "t_box", "pos": [0.1, 0.3], "half": 0.03},
    ]
    env = open_room(tmp_path, [SIG, *parts], spawn=(-1.0, -1.0), name="snap")
    env.parts["s"].value = 1.0
    body_steps(env, 1.0)
    snap = env.save_snapshot()
    keep = (env.data.qpos.copy(), env.parts["pl"].s, env.parts["sp"].live["velocity"])
    assert keep[2] == [0.0, 0.0, 2.0]
    rec = []
    body_steps(env, 1.0, every=lambda e: rec.append(e.data.qpos.copy()))
    env.parts["s"].value = 0.0                    # setter reverts, devices stop
    body_steps(env, 1.0)
    assert env.parts["sp"].live["velocity"] == [0.0, 0.0, 1.5]
    env.restore_snapshot(snap)
    assert np.allclose(env.data.qpos, keep[0])
    assert env.parts["pl"].s == keep[1] and env.parts["set"].applied
    assert env.parts["sp"].live["velocity"] == [0.0, 0.0, 2.0]
    env.parts["s"].value = 1.0
    again = []
    body_steps(env, 1.0, every=lambda e: again.append(e.data.qpos.copy()))
    devices = [env.parts[n] for n in ("pl", "p", "t")]
    idx = [qa for d in devices for qa in ([j[1] for j in d._j] if hasattr(d, "_j") else [d._qadr])]
    box = env.parts["box"]._q
    assert np.abs(again[-1][idx] - rec[-1][idx]).max() < 0.005
    assert np.abs(again[-1][box:box + 3] - rec[-1][box:box + 3]).max() < 0.005
    env.reset(seed=0)
    assert env.parts["sp"].live["velocity"] == [0.0, 0.0, 1.5] and not env.parts["set"].applied


def test_snapshot_without_values_restores(tmp_path):
    # review fix: snapshots pickled before R13 have no `values`; restore falls back to room values
    belt = {"name": "c", "preset": "device/conveyor", "pos": [0.3, 0.3], "in": ["s"]}
    env = open_room(tmp_path, [SIG, belt], spawn=(-1.0, -1.0), name="oldsnap")
    snap = env.save_snapshot()
    object.__delattr__(snap, "values")
    env.parts["c"].live["speed"] = 0.4
    env.restore_snapshot(snap)
    assert env.parts["c"].live["speed"] == env.parts["c"]._live0["speed"]
