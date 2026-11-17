"""R91 budget: a pool that only goes down, what each `spend` mode costs, the two things it
adds that an expression cannot (time across a break, one pool for several loads), empty for
good, refill, and the snapshot.

Why the part exists at all is in its module docstring: `count(x) < n` is already a use
budget, and `timer(x) < s` is already a time limit - but only an unbroken one.
"""

import pytest

from mujoco_puzzle_gym.parts.base import Part
from mujoco_puzzle_gym.parts.logic import Budget
from mujoco_puzzle_gym.registry import register_part
from mujoco_puzzle_gym.schema import SchemaError
from mujoco_puzzle_gym.signals import wire

STEP = 0.02           # [s] one body step, the default body's control_dt


class Src(Part):
    stage, emits, has_body = "input", True, False
    value = 0.0

    def update(self, model, data, body):
        self.write(self.value)


register_part("t_bsrc", Src, replace=True)


class Clock:
    """The least `data` a budget needs: a clock. It measures the step length itself rather
    than assuming one, so a body with another control rate spends at the same rate per
    second."""

    def __init__(self):
        self.time = 0.0

    def tick(self, dt=STEP):
        self.time += dt


def rig(**cfg):
    parts = {n: Src(n, {"pos": [0, 0]}) for n in ("a", "b", "fill")}
    parts["pool"] = Budget("pool", {"pos": [0, 0], **cfg})
    bus = wire(parts)
    clock = Clock()
    for p in parts.values():
        p.reset(None, clock)
    return parts, bus, clock


def run(parts, bus, clock, seconds, **on):
    """Hold these loads for `seconds` and return what is left."""
    for n, v in on.items():
        parts[n].value = float(v)
    for _ in range(int(round(seconds / STEP))):
        clock.tick()
        bus.step(None, clock, None)
    return parts["pool"].left


def test_a_time_budget_drains_only_while_a_load_is_on():
    parts, bus, clock = rig(amount=5.0, rate=1.0, loads=["a"])
    assert run(parts, bus, clock, 1.0, a=0) == pytest.approx(5.0)
    assert run(parts, bus, clock, 2.0, a=1) == pytest.approx(3.0, abs=0.05)
    assert run(parts, bus, clock, 1.0, a=0) == pytest.approx(3.0, abs=0.05)


def test_a_budget_keeps_draining_across_a_break_unlike_timer():
    """The reason this part exists. `timer(load)` is "seconds on without a break" and goes
    back to 0 when the load drops, so a room built on it is solved by switching off and on.
    A pool does not come back."""
    parts, bus, clock = rig(amount=4.0, rate=1.0, loads=["a"])
    run(parts, bus, clock, 1.5, a=1)
    run(parts, bus, clock, 1.0, a=0)
    left = run(parts, bus, clock, 1.5, a=1)
    assert left == pytest.approx(1.0, abs=0.05)
    assert bus.get("pool") == 1


def test_two_loads_share_one_pool():
    """The other reason: expressions refuse arithmetic between channels, so two timers
    cannot be added. `time` charges per load, so two of them drain it twice as fast."""
    one, bus1, c1 = rig(amount=10.0, rate=1.0, loads=["a", "b"])
    assert run(one, bus1, c1, 2.0, a=1, b=0) == pytest.approx(8.0, abs=0.05)
    two, bus2, c2 = rig(amount=10.0, rate=1.0, loads=["a", "b"])
    assert run(two, bus2, c2, 2.0, a=1, b=1) == pytest.approx(6.0, abs=0.05)


def test_a_flow_budget_charges_one_tap_for_any_number_of_loads():
    parts, bus, clock = rig(amount=10.0, rate=1.0, spend="flow", loads=["a", "b"])
    assert run(parts, bus, clock, 2.0, a=1, b=1) == pytest.approx(8.0, abs=0.05)


def test_an_empty_budget_stays_empty():
    parts, bus, clock = rig(amount=1.0, rate=1.0, loads=["a"])
    assert run(parts, bus, clock, 2.0, a=1) == 0.0
    assert bus.get("pool") == 0
    assert run(parts, bus, clock, 2.0, a=0) == 0.0
    assert bus.get("pool") == 0


def test_a_refill_channel_puts_the_amount_back_on_its_turn_on():
    parts, bus, clock = rig(amount=3.0, rate=1.0, loads=["a"], refill="fill")
    run(parts, bus, clock, 3.5, a=1)
    assert bus.get("pool") == 0
    run(parts, bus, clock, 0.1, a=0, fill=1)
    assert parts["pool"].left == pytest.approx(3.0)
    run(parts, bus, clock, 0.5, fill=1)            # held, not a second turn-on
    assert parts["pool"].left == pytest.approx(3.0)


def test_a_budget_survives_a_checkpoint():
    parts, bus, clock = rig(amount=5.0, rate=1.0, loads=["a"])
    run(parts, bus, clock, 2.0, a=1)
    saved = parts["pool"].save_state()
    run(parts, bus, clock, 2.0, a=1)
    assert parts["pool"].left == pytest.approx(1.0, abs=0.05)
    parts["pool"].load_state(None, clock, saved)
    assert parts["pool"].left == pytest.approx(3.0, abs=0.05)
    assert bus.get("pool.left") == pytest.approx(3.0, abs=0.05)


def test_the_refill_cannot_also_be_a_load():
    with pytest.raises(SchemaError, match="both a load and the refill"):
        Budget("pool", {"pos": [0, 0], "loads": ["fill"], "refill": "fill"})
