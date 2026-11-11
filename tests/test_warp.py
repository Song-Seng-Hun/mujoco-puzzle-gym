"""R14: warp pads (place, reset, next_stage, routes, cooldown) and `stages:` rooms."""

from __future__ import annotations

import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym.check import check_order
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.room import Room, RoomError
from mujoco_puzzle_gym.solver import run_episode

from rooms import open_room, stand_at, tick
from test_doors import Source, run  # noqa: F401  (registers t_signal)

STAND = np.zeros(3)


def body_warp_env(tmp_path, **warp):
    w = {"name": "w", "type": "warp", "pos": [-0.5, -1.0], "to": {"pos": [0.5, 0.6], "yaw": 1.2}}
    return open_room(tmp_path, [{**w, **warp}], spawn=(-1.0, -1.0), name="bw")


def test_body_warp_place_and_no_fall(tmp_path):
    env = body_warp_env(tmp_path)
    stand_at(env, -0.5, -1.0)
    env.advance(STAND)                           # the env runs the request after the body step
    pos = env.body_state.pos
    assert np.hypot(pos[0] - 0.5, pos[1] - 0.6) < 0.01, pos
    assert env.body_state.yaw == pytest.approx(1.2, abs=0.02)
    assert env.parts["w"].uses == 1
    assert not run(env, STAND, 3.0)              # R14: no fall for 3 s after the warp


def test_warp_needs_to_or_routes(tmp_path):
    with pytest.raises(RoomError, match="exactly one of"):
        body_warp_env(tmp_path, to=None)
    with pytest.raises(RoomError, match="destination"):
        body_warp_env(tmp_path, to={"where": 1})
    with pytest.raises(RoomError, match="no stage follows"):
        body_warp_env(tmp_path, to="next_stage")


def test_object_warp(tmp_path):
    env = open_room(tmp_path, [
        {"name": "w", "type": "warp", "pos": [0.0, 0.0], "trigger": "object", "to": {"pos": [0.8, -0.6]}},
        {"name": "box", "type": "t_box", "pos": [-0.8, 0.8], "half": 0.03},
    ], spawn=(-1.0, -1.0), name="ow")
    box = env.parts["box"]
    box.put(env.data, [0.0, 0.0, 0.03])
    tick(env)
    env._run_requests()
    p = box.pos(env.data)
    assert np.hypot(p[0] - 0.8, p[1] + 0.6) < 0.01, p
    assert p[2] == pytest.approx(0.03, abs=0.005)
    # the body does not trigger an object-only pad
    stand_at(env, 0.0, 0.0)
    assert not env.world.requests


def test_cooldown_stops_ping_pong(tmp_path):
    env = open_room(tmp_path, [
        {"name": "a", "type": "warp", "pos": [-0.5, -1.0], "to": {"pos": [0.5, 0.5]}, "cooldown": 1.0},
        {"name": "b", "type": "warp", "pos": [0.5, 0.5], "to": {"pos": [-0.5, -1.0]}, "cooldown": 1.0},
    ], spawn=(-1.0, -1.0), name="pp")
    stand_at(env, -0.5, -1.0)
    env.advance(STAND)
    assert np.hypot(*(env.body_state.pos[:2] - [0.5, 0.5])) < 0.01
    run(env, STAND, 2.0)                         # still standing on b: no new entry, no warp
    assert env.parts["b"].uses == 0
    assert np.hypot(*(env.body_state.pos[:2] - [0.5, 0.5])) < 0.05


def test_routes_pick_the_first_true(tmp_path):
    env = open_room(tmp_path, [
        {"name": "s", "type": "t_signal", "pos": [1.0, -1.0]},
        {"name": "w", "type": "warp", "pos": [-0.5, -1.0], "routes": [
            {"when": "s", "to": {"pos": [0.5, 0.5]}}, {"else": {"pos": [-0.5, 0.5]}}]},
    ], spawn=(-1.0, -1.0), name="rt")
    tick(env)
    assert env.signals.get("w.route") == 1.0
    env.parts["s"].value = 1.0
    tick(env)
    assert env.signals.get("w.route") == 0.0
    stand_at(env, -0.5, -1.0)
    env._run_requests()
    assert np.hypot(*(env.body_state.pos[:2] - [0.5, 0.5])) < 0.01


def test_route_without_else_can_be_off(tmp_path):
    env = open_room(tmp_path, [
        {"name": "s", "type": "t_signal", "pos": [1.0, -1.0]},
        {"name": "w", "type": "warp", "pos": [-0.5, -1.0], "routes": [{"when": "s", "to": "reset"}]},
    ], spawn=(-1.0, -1.0), name="rt2")
    tick(env)
    assert env.signals.get("w.route") == -1.0
    stand_at(env, -0.5, -1.0)
    assert not env.world.requests


def test_reset_warp_goes_to_start_and_keeps_persist_counters(tmp_path):
    env = open_room(tmp_path, [
        {"name": "z", "type": "t_signal", "pos": [1.0, -1.0]},
        {"name": "c", "type": "counter", "up": "z", "persist": True},
        {"name": "k", "type": "counter", "up": "z"},
        {"name": "w", "type": "warp", "pos": [0.5, 0.5], "to": "reset"},
    ], spawn=(-1.0, -1.0), name="rs")
    for _ in range(2):
        env.parts["z"].value = 1.0
        tick(env)
        env.parts["z"].value = 0.0
        tick(env)
    assert env.signals.get("c.count") == 2 and env.signals.get("k.count") == 2
    stand_at(env, 0.5, 0.5)
    env._run_requests()
    assert np.hypot(*(env.body_state.pos[:2] - [-1.0, -1.0])) < 0.01
    assert env.signals.get("c.count") == 2       # persist: kept
    assert env.signals.get("k.count") == 0       # back to the start snapshot


# ---- stages ---------------------------------------------------------------------------------
def stage_rooms(tmp_path):
    first = {"name": "first", "size": [1.6, 1.2], "spawn": {"pos": [-0.5, 0.0], "yaw": 0.0},
             "stages": ["second.yaml"],
             "parts": [{"name": "w", "type": "warp", "pos": [0.4, 0.0], "to": "next_stage"}]}
    second = {"name": "second", "size": [1.6, 1.2], "spawn": {"pos": [-0.5, 0.0], "yaw": 0.0},
              "walls": [{"from": [0.0, -0.6], "to": [0.0, -0.2]}, {"from": [0.0, 0.2], "to": [0.0, 0.6]}],
              "parts": [{"name": "b2", "type": "button", "kind": "oneshot", "pos": [-0.5, 0.4],
                         "size": [0.2, 0.2]},
                        {"name": "d2", "type": "door", "pos": [0.0, 0.0], "width": 0.4, "in": ["b2"]},
                        {"name": "goal", "type": "goal", "pos": [0.5, 0.0], "radius": 0.12}]}
    (tmp_path / "second.yaml").write_text(yaml.safe_dump(second))
    path = tmp_path / "first.yaml"
    path.write_text(yaml.safe_dump(first))
    return path


def test_stages_are_zones_of_one_model(tmp_path):
    room = Room.load(stage_rooms(tmp_path))
    (dx, sub), = room.stages
    assert dx == pytest.approx(0.86 + 1.0 + 0.86)
    parts = room.make_parts()
    assert parts["goal"].base_xy[0] == pytest.approx(dx + 0.5)
    assert parts["w"].next_stage_pose == pytest.approx((dx - 0.5, 0.0, 0.0))
    grid = room.grid()
    assert grid.x0 <= -0.86 and grid.x0 + grid.nx * grid.res >= dx + 0.86


def test_next_stage_warp(tmp_path):
    env = PuzzleEnv(room=stage_rooms(tmp_path))
    env.reset(seed=0, options={"layout": env.sample_layout(None)})
    dx = env.room.stages[0][0]
    stand_at(env, 0.4, 0.0)
    _, _, _, _, info = env.step(np.array([-1.0, 0.0, 0.0]))
    assert info["stage_changed"] and info["warped"] == ["w"] and info["zone"] == 1
    assert np.hypot(*(env.body_state.pos[:2] - [dx - 0.5, 0.0])) < 0.01
    _, _, _, _, info = env.step(np.array([-1.0, 0.0, 0.0]))
    assert not info["stage_changed"]


def test_order_goes_through_the_warp(tmp_path):
    env = PuzzleEnv(room=stage_rooms(tmp_path))
    res = check_order(env)
    assert res["ok"], res
    assert res["order"] == ["w", "b2", "d2", "goal"]


@pytest.mark.slow
def test_solver_crosses_stages(tmp_path):
    env = PuzzleEnv(room=stage_rooms(tmp_path))
    res = run_episode(env, 0)
    assert res["success"], res
