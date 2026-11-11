"""R4: switch, sliding door and goal; part states readable every step."""

import dataclasses

import numpy as np
import pytest

from mujoco_puzzle_gym.env import PuzzleEnv

ROOM = "examples/switch_door.yaml"
DOOR_X = 0.3          # door plane in the example room


@pytest.fixture(scope="module")
def env(asset_dir, repo_root):
    e = PuzzleEnv(room=repo_root / ROOM)
    yield e
    e.close()


def layout(env, spawn, **parts):
    lay = env.sample_layout(None)
    lay["spawn"] = spawn
    for name, xy in parts.items():
        lay["parts"][name] = np.asarray(xy, float)
    return lay


def walk_east(env, seconds):
    """Walk along y = 0 toward +x, steering only by heading (no teleport)."""
    xs, door_contact = [], False
    door_geom = env.model.geom("part/door_b/panel").id
    for _ in range(int(seconds * env.hz)):
        s = env.body_state
        aim = np.arctan2(0.0 - s.pos[1], 1.0) - s.yaw
        env.step(np.array([1.0, 0.0, np.clip(2.5 * aim, -1, 1)], dtype=np.float32))
        xs.append(env.body_state.pos[0])
        c = env.data.contact
        door_contact |= bool(np.any((c.geom1 == door_geom) | (c.geom2 == door_geom)))
    return np.array(xs), door_contact


def test_switch_opens_door_within_2s(env):
    _, info = env.reset(options={"layout": layout(env, (-0.25, 0.5, 0.0))})
    assert info["parts"]["door_b"] == 0.0
    states = []
    for _ in range(int(2.0 * env.hz)):
        env.advance(np.zeros(3))
        states.append(env.parts["door_b"].state())
    assert env.parts["switch_a"].satisfied()
    assert states[-1] >= 0.9, states
    assert env.parts["door_b"].is_open()


def test_switch_latches_after_leaving(env):
    env.reset(options={"layout": layout(env, (-0.25, 0.5, 0.0))})
    env.advance(np.zeros(3))
    away = dataclasses.replace(env.body_state, pos=np.array([-1.0, -0.5, 0.1]))
    env.parts["switch_a"].update(env.model, env.data, away)
    assert env.parts["switch_a"].satisfied()


def test_closed_door_blocks_straight_walk(env):
    env.reset(options={"layout": layout(env, (-0.2, 0.0, 0.0))})
    xs, contact = walk_east(env, 12.0)
    assert not env.parts["switch_a"].satisfied()
    assert contact, "the body never touched the door"
    assert xs.max() < DOOR_X, xs.max()


def test_open_door_lets_the_same_walk_through(env):
    # Switch moved under the spawn point: it turns on at the first step.
    env.reset(options={"layout": layout(env, (-0.2, 0.0, 0.0), switch_a=(-0.2, 0.0))})
    xs, _ = walk_east(env, 12.0)
    assert env.parts["door_b"].is_open()
    assert xs.max() > DOOR_X + 0.2, xs.max()


def test_goal_reached_ends_episode(env):
    env.reset(options={"layout": layout(env, (0.85, 0.0, 0.0))})
    _, _, terminated, _, info = env.step(np.zeros(3, dtype=np.float32))
    assert info["success"] and terminated
    assert info["parts"]["goal"] == 1.0


def test_part_states_in_every_step_info(env):
    env.reset(seed=0)
    for _ in range(3):
        *_, info = env.step(env.action_space.sample())
        assert set(info["parts"]) == {"switch_a", "door_b", "goal"}
        assert all(0.0 <= v <= 1.0 for v in info["parts"].values())
