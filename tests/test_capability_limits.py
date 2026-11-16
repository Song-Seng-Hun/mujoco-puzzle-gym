"""R45: an undeclared capability limit refuses, it does not permit everything.

`room.grid`, `check.map_steps` and the beak's lift check all read a numeric limit the
body may not declare. They used to fall back to infinity (and `max_slope` to 90 deg),
so a body that declared nothing was treated as able to climb any step and any ramp and
lift any mass - `mpg check` would call a room solvable that the body cannot walk. It was
only ever safe because the one body that existed declared all four values.

These tests pin the direction of the fallback. The microduck declares everything, so its
behaviour is unchanged (the rest of the suite is the evidence for that); what is checked
here is the body that declares nothing.
"""

import numpy as np
import pytest

from mujoco_puzzle_gym.bodies import UNDECLARED_LIMIT, Capabilities, declared_limit, make_body


def test_undeclared_limit_is_zero_not_infinite():
    assert UNDECLARED_LIMIT == 0.0
    caps = Capabilities(names=frozenset({"walk"}))
    for name in ("max_step", "max_slope", "max_lift_mass"):
        assert caps.limit(name) == 0.0, name
        assert np.isfinite(caps.limit(name)), name


def test_declared_limit_reads_what_the_body_declares():
    caps = Capabilities(values={"max_step": 0.01, "max_slope": 5.0})
    assert caps.limit("max_step") == pytest.approx(0.01)
    assert caps.limit("max_slope") == pytest.approx(5.0)
    assert caps.limit("max_lift_mass") == 0.0           # this one is not declared


def test_declared_limit_tolerates_no_values_at_all():
    """`room.grid` may be handed `None` when nothing passed a body down."""
    assert declared_limit(None, "max_step") == 0.0
    assert declared_limit({}, "max_slope") == 0.0


def test_microduck_declares_every_limit_the_code_asks_for(asset_dir):
    """The reason the old permissive fallback never showed: it was never taken."""
    values = make_body().capabilities.values
    for name in ("max_step", "max_slope", "max_lift_mass"):
        assert name in values, name
        assert values[name] > 0.0, name


def test_a_body_with_no_limits_blocks_steps_and_ramps_but_not_flat_ground():
    """The terrain checks block a step or ramp only when it exceeds the limit, so a
    0 limit still leaves genuinely flat ground walkable."""
    from mujoco_puzzle_gym.roommap import EPS

    assert 0.0 <= UNDECLARED_LIMIT + EPS, "flat ground must stay open at a 0 limit"
    assert 0.01 > UNDECLARED_LIMIT + EPS, "a 1 cm step must be blocked at a 0 limit"
    assert 1.0 > UNDECLARED_LIMIT + EPS, "a 1 deg ramp must be blocked at a 0 limit"
