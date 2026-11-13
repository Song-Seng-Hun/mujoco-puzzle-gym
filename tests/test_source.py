"""R33 signal sources and the optional body of a logic part (`body: true`).

Time is counted in body steps of `bus.dt` = 0.02 s, as in test_logic.py.
"""

import numpy as np
import pytest

from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.parts.logic import Counter, Gate, Source
from mujoco_puzzle_gym.schema import SchemaError
from mujoco_puzzle_gym.signals import wire

DT = 0.02


def rig(**cfgs):
    """Sources by name; returns (parts, bus) with every part reset."""
    parts = {n: Source(n, cfg) for n, cfg in cfgs.items()}
    bus = wire(parts)
    for p in parts.values():
        p.reset(None, None)
    return parts, bus


def trace(bus, name, steps, enable=None):
    """`steps` body steps; returns the channel values written each step.
    `enable` is a {channel: value} the test writes before each step."""
    out = []
    for _ in range(steps):
        if enable:
            for ch, v in enable.items():
                bus.set(ch, v)
        bus.step(None, None, None)
        out.append(bus.get(name))
    return out


# ---- modes --------------------------------------------------------------------------
def test_const_writes_its_value_every_step():
    _, bus = rig(s={"pos": [0, 0], "mode": "const", "value": 0.7})
    assert trace(bus, "s", 5) == [pytest.approx(0.7)] * 5


def runs(values):
    """[(value, how many steps in a row), ...]."""
    out = []
    for v in values:
        if out and out[-1][0] == v:
            out[-1][1] += 1
        else:
            out.append([v, 1])
    return [(v, n) for v, n in out]


def test_clock_period_and_duty():
    """`period` 0.2 s, `duty` 0.5: runs of five steps on and five off (+-1 step)."""
    _, bus = rig(s={"pos": [0, 0], "mode": "clock", "period": 0.2, "duty": 0.5})
    got = trace(bus, "s", 21)
    rs = runs(got)
    assert {v for v, _ in rs} == {0.0, 1.0}
    assert [v for v, _ in rs][0] == 1.0, got                  # it starts on
    assert all(abs(n - 5) <= 1 for _, n in rs[1:-1]), rs      # the whole runs


def test_clock_duty_quarter():
    """`duty` 0.25 of a 0.4 s period: five steps on, fifteen off (+-1)."""
    _, bus = rig(s={"pos": [0, 0], "mode": "clock", "period": 0.4, "duty": 0.25})
    rs = runs(trace(bus, "s", 41))
    on = [n for v, n in rs[1:-1] if v == 1.0]
    off = [n for v, n in rs[1:-1] if v == 0.0]
    assert on and off and all(abs(n - 5) <= 1 for n in on), rs
    assert all(abs(n - 15) <= 1 for n in off), rs


def test_clock_phase_shifts_the_cycle():
    """With half a period of `phase` the source starts in the off half."""
    _, bus = rig(s={"pos": [0, 0], "mode": "clock", "period": 0.2, "duty": 0.5,
                    "phase": 0.1})
    got = trace(bus, "s", 10)
    rs = runs(got)
    assert rs[0][0] == 0.0 and abs(rs[0][1] - 5) <= 1, got


def test_clock_duty_zero_and_one():
    _, bus = rig(off={"pos": [0, 0], "mode": "clock", "period": 0.1, "duty": 0.0},
                 on={"pos": [1, 0], "mode": "clock", "period": 0.1, "duty": 1.0})
    assert trace(bus, "off", 10) == [0.0] * 10
    assert trace(bus, "on", 10) == [1.0] * 10


def test_pattern_runs_in_order_and_repeats():
    """Each value is held for `step` (two body steps here) and the list repeats;
    the first and last run may be a step short (+-1 step)."""
    _, bus = rig(s={"pos": [0, 0], "mode": "pattern", "pattern": [1.0, 0.0, 0.5],
                    "step": 0.04})
    rs = runs(trace(bus, "s", 13))
    assert [v for v, _ in rs][:7] == [1.0, 0.0, 0.5, 1.0, 0.0, 0.5, 1.0]
    assert all(abs(n - 2) <= 1 for _, n in rs[1:-1]), rs


def test_random_holds_its_value_and_stays_in_range():
    _, bus = rig(s={"pos": [0, 0], "mode": "random", "hold": 0.06, "low": -2.0, "high": 3.0})
    got = trace(bus, "s", 13)
    assert all(-2.0 <= v <= 3.0 for v in got)
    rs = runs(got)
    assert len(rs) >= 4, got                               # it is redrawn, not constant
    assert all(abs(n - 3) <= 1 for _, n in rs[1:-1]), rs   # each value held for `hold`


def test_random_is_the_same_sequence_for_the_same_room():
    cfg = {"pos": [0, 0], "mode": "random", "hold": 0.02}
    first = trace(rig(s=dict(cfg))[1], "s", 8)
    again = trace(rig(s=dict(cfg))[1], "s", 8)
    assert first == again
    other = trace(rig(s={**cfg, "seed": 7})[1], "s", 8)
    assert other != first                      # `seed` changes the sequence


def test_two_random_sources_differ():
    _, bus = rig(a={"pos": [0, 0], "mode": "random", "hold": 0.02},
                 b={"pos": [1, 0], "mode": "random", "hold": 0.02})
    assert trace(bus, "a", 8) != trace(bus, "b", 8)


# ---- enable -------------------------------------------------------------------------
def test_enable_off_writes_off_value_and_holds_the_phase():
    """While `enable` is off the source writes `off_value` and its clock does not run,
    so the phase continues where it stopped."""
    parts, bus = rig(pwr={"pos": [1, 0], "mode": "const", "value": 1.0},
                     s={"pos": [0, 0], "mode": "clock", "period": 0.2, "duty": 0.5,
                        "enable": "pwr", "off_value": -1.0})

    def power(v):
        parts["pwr"].set_value(None, None, "value", float(v))

    on = trace(bus, "s", 3)
    power(0)
    off = trace(bus, "s", 10)
    power(1)
    back = trace(bus, "s", 3)
    assert on == [1.0, 1.0, 1.0]
    assert off == [-1.0] * 10
    # The on half is t in [0, 0.1): three steps were used, so one is left and the
    # source goes off on the next one - the ten dark steps did not consume the phase.
    assert back == [1.0, 0.0, 0.0]
    assert parts["s"].t == pytest.approx(6 * DT)


# ---- snapshots ----------------------------------------------------------------------
@pytest.mark.parametrize("cfg", [
    {"mode": "clock", "period": 0.2, "duty": 0.5},
    {"mode": "pattern", "pattern": [1.0, 0.0, 0.5], "step": 0.04},
    {"mode": "random", "hold": 0.04},
])
def test_restore_continues_the_phase(cfg):
    """A snapshot taken mid-cycle and restored gives exactly the run that was not cut."""
    parts, bus = rig(s={"pos": [0, 0], **cfg})
    trace(bus, "s", 7)
    st = parts["s"].save_state()
    straight = trace(bus, "s", 10)

    parts2, bus2 = rig(s={"pos": [0, 0], **cfg})
    trace(bus2, "s", 3)                        # a different history before the restore
    parts2["s"].load_state(None, None, st)
    assert trace(bus2, "s", 10) == straight


# ---- bad settings -------------------------------------------------------------------
@pytest.mark.parametrize("cfg, bad", [
    ({"mode": "pattern"}, "non-empty `pattern`"),
    ({"mode": "clock", "pattern": [1.0]}, "only means something in mode `pattern`"),
    ({"mode": "random", "low": 2.0, "high": 1.0}, "low <= high"),
])
def test_bad_settings_are_refused(cfg, bad):
    with pytest.raises(SchemaError) as exc:
        Source("s", {"pos": [0, 0], **cfg})
    assert bad in str(exc.value)


# ---- the body of a logic part (`body: true`) ------------------------------------------
def test_source_has_a_body_by_default_and_blocks_walking():
    s = Source("s", {"pos": [0.5, 0.0]})
    assert s.has_body
    segs = s.blockers(False)
    assert len(segs) == 1 and segs[0].thickness == pytest.approx(0.08)


def test_bodyless_source_has_no_body_and_no_blocker():
    s = Source("s", {"pos": [0.5, 0.0], "body": False})
    assert not s.has_body
    assert s.build(None) is None and s.blockers(False) == []


@pytest.mark.parametrize("cls, cfg", [
    (Gate, {"op": "and", "in": ["a"]}),
    (Counter, {"up": "a"}),
])
def test_gates_and_counters_have_no_body_unless_asked(cls, cfg):
    plain = cls("p", {"pos": [0, 0], **cfg})
    assert not plain.has_body and plain.blockers(False) == []
    bodied = cls("p", {"pos": [0, 0], "body": True, **cfg})
    assert bodied.has_body and len(bodied.blockers(False)) == 1


def test_logic_bodies_are_built_with_a_box_and_a_lamp(tmp_path):
    """In a room the bodied parts get their geoms, the bodyless ones none."""
    room = tmp_path / "room.yaml"
    room.write_text("""
name: lamps
size: [3.0, 2.0]
spawn: {pos: [-1.0, 0.0], yaw: 0.0}
parts:
- {name: clk, type: source, mode: clock, period: 0.2, duty: 0.5, pos: [0.3, 0.6]}
- {name: ghost, type: source, mode: const, body: false, pos: [0.3, -0.6]}
- {name: g, type: gate, op: not, body: true, in: [clk], pos: [0.8, 0.6]}
- {name: goal, type: goal, pos: [1.2, 0.0], radius: 0.1}
demo: [goto goal]
""")
    env = PuzzleEnv(room=str(room))
    try:
        names = [env.model.geom(i).name for i in range(env.model.ngeom)]
        assert "part/clk/box" in names and "part/clk/lamp" in names
        assert "part/g/box" in names and "part/g/lamp" in names
        assert not any(n.startswith("part/ghost/") for n in names)

        env.reset(seed=0)
        lamp = env.model.geom("part/clk/lamp").id
        seen = set()
        for _ in range(40):
            env.step(env.scripted_action([0.0, 0.0, 0.0]))
            seen.add(bool(env.model.geom_rgba[lamp][0] > 0.9))
        assert seen == {True, False}, "the lamp follows the clock"
    finally:
        env.close()


def test_a_clock_source_opens_and_closes_a_door(tmp_path):
    """R33 in a room: the door follows the source with no switch in the room, and the
    planner takes the clock as eventually on (`plan_values`)."""
    from mujoco_puzzle_gym.logic import plan_channels

    room = tmp_path / "clockdoor.yaml"
    room.write_text("""
name: clock_door
size: [4.0, 2.0]
spawn: {pos: [-1.5, 0.0], yaw: 0.0}
parts:
- {name: clk, type: source, mode: clock, period: 1.0, duty: 0.5, pos: [-1.0, 0.7]}
- {name: door, type: door, pos: [0.0, 0.0], width: 0.6, in: [clk]}
- {name: goal, type: goal, pos: [1.5, 0.0], radius: 0.1}
demo: [goto goal]
""")
    env = PuzzleEnv(room=str(room))
    try:
        env.reset(seed=0)
        states = []
        for _ in range(40):
            env.step(env.scripted_action([0.0, 0.0, 0.0]))
            states.append(env.parts["door"].state())
        assert max(states) > 0.9 and min(states) < 0.1, states
        vals = plan_channels(env.parts, set())
        assert vals["clk"] == pytest.approx(1.0)
    finally:
        env.close()


def test_source_state_is_clipped_to_the_observation_range():
    s = Source("s", {"pos": [0, 0], "mode": "const", "value": 7.0})
    parts = {"s": s}
    wire(parts)
    s.reset(None, None)
    assert s.value == pytest.approx(7.0) and s.state() == pytest.approx(1.0)
    assert np.all(np.isfinite([s.state()]))
