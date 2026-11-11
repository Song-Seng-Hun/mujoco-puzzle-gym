"""R3: the default body walks on a flat floor through the BodyAdapter boundary."""

import mujoco
import numpy as np
import pytest

from mujoco_puzzle_gym.bodies import flat_floor_spec, make_body

SECONDS = 5.0


@pytest.fixture(scope="module")
def sim(asset_dir):
    body = make_body()
    model = body.spawn(flat_floor_spec()).compile()
    return body, model, mujoco.MjData(model)


def walk(sim, cmd, seconds=SECONDS):
    body, _, data = sim
    body.reset(data)
    start = body.state(data)
    fell = False
    for _ in range(int(round(seconds / body.control_dt))):
        body.step(data, cmd)
        fell |= body.state(data).fallen
    return start, body.state(data), fell


def test_timing_and_limits(sim):
    body, model, _ = sim
    assert model.opt.timestep == pytest.approx(0.005)
    assert body.control_dt == pytest.approx(0.02)
    assert body.cmd_limits == ((-0.4, 0.4), (-0.3, 0.3), (-1.0, 1.0))
    np.testing.assert_allclose(body.clip_cmd([1.0, -1.0, 2.0]), [0.4, -0.3, 1.0])


def test_observation_is_61d(sim):
    body, _, data = sim
    body.reset(data)
    assert body.observation(data, np.zeros(3)).shape == (61,)


def test_forward_walk(sim):
    start, end, fell = walk(sim, (0.3, 0.0, 0.0))
    assert not fell
    forward = end.pos[0] - start.pos[0]
    # spec R3: 0.5-1.1 m. velstand tracks about half of vx: measured 0.73 m here
    # (~0.16 m/s); upstream's own sim eval reports 0.4 m/s -> 0.17 m/s
    # (docs/velstand_policy.md @0daab16).
    assert 0.5 < forward < 1.1, forward
    assert abs(end.pos[1] - start.pos[1]) < 0.3


def test_zero_command_stands_still(sim):
    start, end, fell = walk(sim, (0.0, 0.0, 0.0))
    assert not fell
    assert np.linalg.norm(end.pos[:2] - start.pos[:2]) < 0.1
