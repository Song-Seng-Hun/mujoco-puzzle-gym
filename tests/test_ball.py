"""R38 ball and goal net: bounce ratios, scoring channels, colour filter,
scripted push, static checks, observation and reward terms, pins, presets.
The 20-seed `mpg check` of examples/rooms/mini_soccer.yaml is slow."""

from pathlib import Path

import mujoco
import numpy as np
import pytest
from rooms import open_room, tick

from mujoco_puzzle_gym.bodies import flat_floor_spec
from mujoco_puzzle_gym.check import check, check_order, check_parts
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.obs import BALLS_K
from mujoco_puzzle_gym.parts.object.ball import Ball
from mujoco_puzzle_gym.push import Pusher
from mujoco_puzzle_gym.registry import OBS, REWARDS
from mujoco_puzzle_gym.room import Room

ROOT = Path(__file__).resolve().parents[1]
PRESETS = ROOT / "src/mujoco_puzzle_gym/presets/puzzles/ball"
MINI_SOCCER = ROOT / "examples/rooms/mini_soccer.yaml"


@pytest.fixture
def make(asset_dir, tmp_path):
    envs = []

    def _make(parts, **kw):
        env = open_room(tmp_path, parts, **kw)
        envs.append(env)
        return env

    yield _make
    for e in envs:
        e.close()


# ---- bounce (elasticity through solref) --------------------------------------------------
def bounce_ratio(elasticity: float, mass: float = 0.05, drop: float = 0.3) -> float:
    """Height of the first bounce over the drop height (both above the resting centre)."""
    spec = flat_floor_spec()
    spec.option.timestep = 0.005
    ball = Ball("b", {"pos": [0.0, 0.0], "elasticity": elasticity, "mass": mass})
    ball.build(spec)
    model = spec.compile()
    data = mujoco.MjData(model)
    ball.bind(model, data)
    ball.put(data, [0.0, 0.0, drop])
    z = []
    for _ in range(500):
        mujoco.mj_step(model, data)
        z.append(data.qpos[ball._q + 2])
    z = np.array(z)
    up = np.nonzero(np.diff(z) > 1e-5)[0]
    if not len(up):
        return 0.0
    return float((z[up[0]:].max() - ball.radius) / (drop - ball.radius))


def test_bounce_follows_elasticity():
    es = [0.0, 0.3, 0.5, 0.7, 0.9]
    ratios = [bounce_ratio(e) for e in es]
    assert ratios == sorted(ratios) and ratios[0] < 0.02, ratios
    for e, r in zip(es, ratios):
        assert abs(r - e * e) < 0.03, (e, r)          # ideal restitution: height ratio e^2
    assert abs(bounce_ratio(0.7, mass=0.5) - ratios[3]) < 0.01   # mass does not matter


# ---- goal net channels ------------------------------------------------------------------
NET = {"name": "net", "type": "goal_net", "pos": [0.0, 0.6], "width": 0.4, "depth": 0.2,
       "facing": -90}


def test_net_scores_and_counts(make):
    env = make([{"name": "ball", "type": "ball", "pos": [0.0, 0.0]}, NET])
    ball, scored = env.parts["ball"], []
    for y in (0.3, 0.55, 0.6, 0.62):                 # rolls in through the mouth (y 0.5)
        ball.put(env.data, [0.0, y, ball.radius])
        tick(env)
        scored.append(env.signals.get("net.scored"))
    assert scored == [0, 1, 0, 0]
    assert env.signals.get("net.count") == 1 and env.signals.get("net") == 1
    ball.put(env.data, [0.0, 0.3, ball.radius])      # out again
    tick(env)
    assert env.signals.get("net.count") == 0 and env.signals.get("net") == 0


def test_net_accepts_colours(make):
    env = make([{"name": "red", "type": "ball", "color": "red", "pos": [-0.5, 0.0]},
                {"name": "blue", "type": "ball", "color": "blue", "pos": [0.5, 0.0]},
                {**NET, "accepts": ["red"]}])
    env.parts["blue"].put(env.data, [0.0, 0.6, 0.05])
    tick(env)
    assert env.signals.get("net.count") == 0 and env.signals.get("net.scored") == 0
    env.parts["red"].put(env.data, [0.1, 0.6, 0.05])
    tick(env)
    assert env.signals.get("net.count") == 1 and env.signals.get("net.scored") == 1


def test_net_keeps_a_ball_rolled_in(make):
    """Dead net walls: a ball rolled in hard does not bounce back out."""
    env = make([{"name": "ball", "type": "ball", "pos": [0.0, 0.0], "elasticity": 0.9}, NET])
    env.parts["ball"].put(env.data, [0.0, 0.35, 0.05], vel=[0.0, 1.0, 0.0])
    for _ in range(30):
        env.advance(np.array([0.0, 0.0, 0.0]))
    assert env.signals.get("net.count") == 1


# ---- scripted push -------------------------------------------------------------------------
def test_scripted_push_scores_once(make):
    """The scripted push gets the ball in without a fall; `.scored` is high for
    exactly one body step and `.count` ends at 1."""
    env = make([{"name": "ball", "type": "ball", "pos": [0.0, 0.0]}, NET], spawn=(-0.6, -0.6))
    pusher = Pusher(env)
    pulses = 0
    for _ in range(int(30 * env.hz)):
        cmd = env.body.command_from_action(pusher.act(env.parts["net"]))
        for _ in range(env.body_steps):
            env.body.step(env.data, cmd)
            env.signals.step(env.model, env.data, env.body.state(env.data))
            pulses += int(env.signals.get("net.scored"))
        env.body_state = env.body.state(env.data)
        assert not env.body_state.fallen
        if env.parts["net"].satisfied():
            break
    assert pulses == 1 and env.signals.get("net.count") == 1


def test_default_ball_pushed_without_falling(make):
    """OQ10 regression: steering into the default ball for 6 s keeps the body up
    and moves the ball about as fast as the body walks."""
    env = make([{"name": "ball", "type": "ball", "pos": [-0.8, 0.0]}], spawn=(-1.05, 0.0))
    ball = env.parts["ball"]
    for _ in range(int(6 * env.hz)):
        d = ball.pos(env.data)[:2] - env.body_state.pos[:2]
        err = (np.arctan2(d[1], d[0]) - env.body_state.yaw + np.pi) % (2 * np.pi) - np.pi
        env.advance(env.body.command_from_action([1.0, 0.0, np.clip(2.5 * err, -1, 1)]))
        assert not env.body_state.fallen
    assert ball.pos(env.data)[0] + 0.8 > 0.8               # > 0.13 m/s


# ---- static checks ----------------------------------------------------------------------
def test_ball_too_big_fails_static_check(make):
    env = make([{"name": "ball", "type": "ball", "pos": [0.0, 0.0], "radius": 0.15},
                {**NET, "width": 0.25}])
    res = check_parts(env)
    assert not res["ok"] and "too big: ball" in res["reason"] and "width 0.25" in res["reason"]


def test_ball_in_wall_fails_static_check(asset_dir, tmp_path):
    room = Room.load(MINI_SOCCER).raw
    room["parts"][0]["pos"] = [0.5, -0.6]                # inside the divider wall
    path = tmp_path / "bad.yaml"
    import yaml
    path.write_text(yaml.safe_dump(room))
    res = check(path, seeds=1, need=1)
    assert res["failed"][0] == "parts" and "inside a wall" in res["parts"]["reason"]
    assert res["solver"].get("skipped")


def test_mini_soccer_static_and_order(asset_dir):
    env = PuzzleEnv(room=MINI_SOCCER)
    assert check_parts(env)["ok"]
    assert check_order(env)["order"] == ["net", "door", "goal"]
    env.close()


# ---- observation and reward -------------------------------------------------------------
def test_balls_obs_and_progress_reward(make):
    env = make([{"name": "ball", "type": "ball", "pos": [0.0, 0.0]}, NET], spawn=(0.0, -0.5))
    obs = OBS["balls"](env)
    assert obs.shape == (BALLS_K * 5,)
    assert np.allclose(obs[:2], [0.0, 0.5], atol=0.02)   # on the left: the body faces +x
    assert obs[4] == 1 and not obs[5:].any()
    env.targets, env.stage = [env.parts["net"]], 0     # the open room's plan goes straight to its goal
    env.parts["ball"].put(env.data, [0.0, 0.0, 0.05], vel=[0.0, 0.5, 0.0])
    mujoco.mj_forward(env.model, env.data)
    assert REWARDS["ball_progress"](env) == pytest.approx(0.5 / env.hz, rel=0.05)
    env.parts["ball"].put(env.data, [0.0, 0.0, 0.05], vel=[0.0, -0.5, 0.0])
    assert REWARDS["ball_progress"](env) < 0


# ---- pins -------------------------------------------------------------------------------
def test_pin_down_when_tipped(make):
    env = make([{"name": "pin", "type": "pin", "pos": [0.0, 0.0]}])
    tick(env)
    assert env.signals.get("pin.down") == 0
    pin = env.parts["pin"]
    env.data.qpos[pin._q:pin._q + 7] = [0, 0, 0.02, np.cos(np.pi / 4), np.sin(np.pi / 4), 0, 0]
    tick(env)
    assert env.signals.get("pin.down") == 1 and pin.satisfied()


# ---- presets ----------------------------------------------------------------------------
@pytest.mark.parametrize("name", ["mini_soccer", "color_sort", "bowling"])
def test_presets_pass_static_checks(asset_dir, name):
    env = PuzzleEnv(room=PRESETS / f"{name}.yaml")
    assert check_parts(env)["ok"] and check_order(env)["ok"]
    env.close()


@pytest.mark.slow
def test_mini_soccer_check():
    res = check(MINI_SOCCER)
    assert res["ok"], res
    assert res["solver"]["successes"] >= 18
