"""R99-R101 light: a path with a direction, mirrors that bend it, panels that dim it.

Every number here was measured when the parts were built; the two R104 tests are the
probe measurements that decided what the parts had to be (a beam `laser` cannot ask
about, and a door that already works as a shutter).
"""

import numpy as np
import pytest
from rooms import open_room, tick

import mujoco


@pytest.fixture
def make(asset_dir, tmp_path):
    envs = []

    def _make(parts, **kw):
        env = open_room(tmp_path, parts, **kw)
        envs.append(env)
        mujoco.mj_forward(env.model, env.data)
        tick(env)
        return env

    yield _make
    for e in envs:
        e.close()


def lamp(**kw):
    cfg = {"name": "lamp", "type": "emitter", "pos": [-0.6, 0.0], "yaw": 0.0,
           "power": 1.0, "bounces": 3}
    cfg.update(kw)
    return cfg


def cell(name="eye", pos=(0.6, 0.0), min_power=0.1, **kw):
    return {"name": name, "type": "photocell", "pos": list(pos), "min_power": min_power, **kw}


def panel(kind, name, pos, yaw, **kw):
    return {"name": name, "type": kind, "pos": list(pos), "yaw": yaw, "width": 0.2, **kw}


def power(env, name="eye"):
    return round(float(env.signals.get(name + ".power")), 3)


# ---- the path ---------------------------------------------------------------------
def test_light_reaches_a_photocell_in_a_straight_line(make):
    env = make([lamp(bounces=0), cell()])
    assert env.signals.get("eye") == 1
    assert power(env) == 1.0
    assert env.signals.get("lamp") == 1


def test_a_box_in_the_way_leaves_the_cell_dark(make):
    env = make([lamp(bounces=0), cell(),
                {"name": "crate", "type": "t_box", "pos": [0.0, 0.0], "half": 0.05}])
    assert env.signals.get("eye") == 0
    assert power(env) == 0.0
    assert env.signals.get("lamp") == 0


def test_a_cell_under_min_power_reads_zero_but_still_reports_it(make):
    env = make([lamp(bounces=1), cell(min_power=0.6),
                panel("filter", "tint", (0.0, 0.0), 0.0, transmission=0.5)])
    assert power(env) == 0.5
    assert env.signals.get("eye") == 0


# ---- mirrors ---------------------------------------------------------------------
def test_a_mirror_bends_the_beam_onto_a_cell(make):
    """A beam going +x off a mirror facing -45 deg leaves towards +y; 0.95 of the power
    arrives with the default 5 % loss."""
    env = make([lamp(), panel("mirror", "tilt", (0.6, 0.0), -45.0), cell(pos=(0.6, 0.6))])
    assert env.signals.get("eye") == 1
    assert power(env) == 0.95


@pytest.mark.parametrize("yaw", [0.0, 45.0])
def test_a_mirror_aimed_wrong_leaves_the_cell_dark(make, yaw):
    env = make([lamp(), panel("mirror", "tilt", (0.6, 0.0), yaw), cell(pos=(0.6, 0.6))],
               name=f"aim{yaw}")
    assert env.signals.get("eye") == 0


def test_a_mirror_loses_the_share_it_is_given(make):
    env = make([lamp(), panel("mirror", "tilt", (0.6, 0.0), -45.0, loss=0.5),
                cell(pos=(0.6, 0.6))])
    assert power(env) == 0.5


def test_bounces_limits_how_often_one_path_turns(make):
    """Two mirrors need two turns. With `bounces: 1` the path stops at the first."""
    parts = [panel("mirror", "m1", (0.6, 0.0), -45.0), panel("mirror", "m2", (0.6, 0.6), 45.0),
             cell(pos=(0.0, 0.6))]
    one = make([lamp(bounces=1), *parts], name="b1")
    assert one.signals.get("eye") == 0
    two = make([lamp(bounces=2), *parts], name="b2")
    assert two.signals.get("eye") == 1
    assert power(two) == round(0.95 * 0.95, 3)


def test_going_straight_through_a_filter_costs_no_bounce(make):
    """`bounces` counts turns. The first version spent one on every panel, so with
    `bounces: 1` a beam through a single filter died at the filter and the cell behind it
    read 0.000 at every setting - which made a room's control vacuous."""
    env = make([lamp(bounces=1), panel("filter", "tint", (0.0, 0.0), 0.0, transmission=0.5),
                panel("mirror", "tilt", (0.6, 0.0), -45.0), cell(pos=(0.6, 0.6))])
    assert env.signals.get("eye") == 1
    assert power(env) == round(0.5 * 0.95, 3)


def test_a_setter_can_re_aim_a_mirror_while_the_room_runs(make):
    """What makes the mirror a puzzle rather than a fixed pipe: `yaw` is a run-time value,
    so a room can hand it to the body through a switch."""
    env = make([lamp(), panel("mirror", "tilt", (0.6, 0.0), 0.0), cell(pos=(0.6, 0.6)),
                {"name": "sw", "type": "source", "pos": [0.0, -0.8], "value": 1.0,
                 "body": False, "mode": "const"},
                {"name": "aim", "type": "setter", "pos": [0.0, -1.0], "in": ["sw"],
                 "target": "tilt.yaw", "value": -45.0}])
    for _ in range(3):
        mujoco.mj_forward(env.model, env.data)
        tick(env)
    assert env.signals.get("eye") == 1


# ---- the panels that dim ---------------------------------------------------------
def test_filters_multiply_along_the_path(make):
    one = make([lamp(bounces=1), panel("filter", "t1", (0.0, 0.0), 0.0, transmission=0.5),
                cell()], name="f1")
    assert power(one) == 0.5
    two = make([lamp(bounces=2), panel("filter", "t1", (-0.2, 0.0), 0.0, transmission=0.5),
                panel("filter", "t2", (0.2, 0.0), 0.0, transmission=0.5), cell()], name="f2")
    assert power(two) == 0.25


def test_a_splitter_divides_one_beam_between_two_cells(make):
    """`split` 0.6 carries on, (1 - 0.6) * (1 - 0.1) reflects. Measured 0.6 and 0.36 -
    and this is the arithmetic a gate expression refuses to do between channels."""
    env = make([lamp(), panel("splitter", "half", (0.0, 0.0), -45.0, split=0.6, loss=0.1),
                cell("eye_a", (0.6, 0.0)), cell("eye_b", (0.0, 0.6))])
    assert power(env, "eye_a") == 0.6
    assert power(env, "eye_b") == 0.36


def test_a_splitter_cannot_light_two_demanding_cells_at_once(make):
    """The trade-off the part exists for: both cells want 0.6, one beam has 1.0."""
    env = make([lamp(), panel("splitter", "half", (0.0, 0.0), -45.0, split=0.6, loss=0.1),
                cell("eye_a", (0.6, 0.0), min_power=0.55),
                cell("eye_b", (0.0, 0.6), min_power=0.55)])
    assert env.signals.get("eye_a") == 1
    assert env.signals.get("eye_b") == 0


# ---- R104: what the probe rooms measured before any of this was written ----------
def test_a_closed_door_is_already_a_shutter(make):
    """So a `filter` is not an on-off part. A door across the path stops the light, and
    the same door open lets it through - no new part needed for that."""
    shut = make([lamp(bounces=0), cell(),
                 {"name": "gate", "preset": "device/sliding_door", "pos": [0.0, 0.0],
                  "width": 0.4}], name="shut")
    assert shut.signals.get("eye") == 0

    open_ = make([lamp(bounces=0), cell(),
                  {"name": "gate", "preset": "device/sliding_door", "pos": [0.0, 0.0],
                   "width": 0.4, "in": ["sw"]},
                  {"name": "sw", "type": "source", "pos": [0.0, -0.8], "value": 1.0,
                   "body": False, "mode": "const"}], name="open")
    for _ in range(int(3.0 * open_.hz)):
        open_.advance(np.zeros(3))
    assert open_.signals.get("eye") == 1


def test_a_laser_cannot_tell_a_blocked_beam_from_a_quiet_one(make):
    """Why `photocell` exists. `laser` reads 0 both when nothing cut its beam and when a
    wall hides the beam entirely, so no room could ask whether light arrived."""
    quiet = make([{"name": "trip", "preset": "input/laser_beam", "pos": [-0.6, 0.0],
                   "to": [0.6, 0.0], "height": 0.06, "trigger": "object"}], name="quiet")
    assert quiet.signals.get("trip") == 0

    hidden = make([{"name": "trip", "preset": "input/laser_beam", "pos": [-0.6, 0.0],
                    "to": [0.6, 0.0], "height": 0.06, "trigger": "object"},
                   {"name": "crate", "type": "t_box", "pos": [0.3, 0.0], "half": 0.05},
                   {"name": "wall", "type": "mirror", "pos": [0.0, 0.0], "yaw": 0.0,
                    "width": 0.3}], name="hidden")
    assert hidden.signals.get("trip") == 0        # the same 0, with a box right there
