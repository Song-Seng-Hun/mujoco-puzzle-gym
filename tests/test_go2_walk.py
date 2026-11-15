"""R43: the Go2 walks on a flat floor, and left is still left.

The bounds come from measurements taken 2026-11-15 (spec R43 holds the table); they are
deliberately two-sided, since how well the policy follows a command is part of what is
being measured. The `vy` and `wz` sign checks are the only defence against a left/right
joint mix-up: swapping them keeps every dimension intact and still looks like walking
(R42).
"""

import mujoco
import pytest

from mujoco_puzzle_gym.bodies import flat_floor_spec, make_body

SECONDS = 5.0


@pytest.fixture(scope="module")
def sim(go2_asset_dir):
    body = make_body("go2")
    model = body.spawn(flat_floor_spec()).compile()
    return body, model, mujoco.MjData(model)


def walk(sim, cmd, seconds=SECONDS):
    """Run `cmd` for `seconds` from a fresh reset; return (dx, dy, dyaw, fell, end)."""
    body, _, data = sim
    body.reset(data)
    start = body.state(data)
    fell = False
    for _ in range(int(round(seconds / body.control_dt))):
        body.step(data, cmd)
        fell |= body.state(data).fallen
    end = body.state(data)
    return (float(end.pos[0] - start.pos[0]), float(end.pos[1] - start.pos[1]),
            float(end.yaw - start.yaw), fell, end)


def test_standing_still_stays_put(sim):
    dx, dy, dyaw, fell, end = walk(sim, (0.0, 0.0, 0.0))
    assert not fell
    assert (dx ** 2 + dy ** 2) ** 0.5 < 0.25          # measured 0.11-0.14 m
    assert end.pos[2] > 0.25


def test_forward(sim):
    dx, dy, dyaw, fell, end = walk(sim, (0.5, 0.0, 0.0))
    assert not fell
    assert 2.0 < dx < 3.0                             # measured 2.27-2.49 m
    assert abs(dy) < 0.5
    assert end.pos[2] > 0.25


def test_backward(sim):
    dx, _, _, fell, _ = walk(sim, (-0.5, 0.0, 0.0))
    assert not fell
    assert dx < -0.8                                  # measured -1.37 m


def test_faster_forward_goes_further(sim):
    slow = walk(sim, (0.5, 0.0, 0.0))[0]
    fast = walk(sim, (1.0, 0.0, 0.0))[0]
    assert fast > slow + 1.0                          # measured 2.3 vs 5.5 m


@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_strafe_goes_the_commanded_way(sim, sign):
    """`vy` > 0 is left (+y). A left/right joint swap flips this (R42)."""
    dx, dy, _, fell, _ = walk(sim, (0.0, 0.5 * sign, 0.0))
    assert not fell
    assert sign * dy > 1.5                            # measured 2.14-2.18 m
    assert abs(dx) < 1.0


@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_turn_goes_the_commanded_way(sim, sign):
    """`wz` > 0 is counter-clockwise. A left/right joint swap flips this (R42)."""
    dx, dy, dyaw, fell, end = walk(sim, (0.0, 0.0, 0.5 * sign))
    assert not fell
    assert 0.9 < sign * dyaw < 1.8                    # measured 1.28-1.39 rad
    assert (dx ** 2 + dy ** 2) ** 0.5 < 1.0           # turns in place
    assert end.pos[2] > 0.25


def test_step_needs_reset_first(sim):
    body, model, _ = sim
    other = mujoco.MjData(model)
    with pytest.raises(RuntimeError, match="reset"):
        body.step(other, (0.0, 0.0, 0.0))
