"""R98 magnet: pulls the objects it accepts, holds them, and leaves the rest.

Five of these are the part's own behaviour; the last one is the measurement that
sent the part into the box (R104): a constant force cannot hold anything still.
"""

import numpy as np
import pytest
from rooms import open_room

HOLD = 0.06


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


def run(env, seconds):
    for _ in range(int(round(seconds * env.hz))):
        env.advance(np.zeros(env.action_space.shape[0]))


def at(env, name):
    return env.data.xpos[env.parts[name]._bid].copy()


def far(env, name, coil="coil"):
    return float(np.linalg.norm(at(env, name)[:2] - env.parts[coil].xy))


def coil(**kw):
    cfg = {"name": "coil", "type": "magnet", "pos": [0.0, 0.0], "radius": 0.4,
           "power": 1.0, "hold": HOLD, "damp": 0.4}
    cfg.update(kw)
    return cfg


def box(name, x, y, mass=0.05):
    return {"name": name, "type": "t_box", "pos": [x, y], "half": 0.03, "mass": mass}


def test_a_magnet_pulls_an_accepted_object(make):
    env = make([box("b", 0.3, 0.0), coil(accept=["b"])])
    assert far(env, "b") == pytest.approx(0.3, abs=0.01)
    run(env, 2.0)
    assert far(env, "b") <= HOLD
    assert env.signals.get("coil") == 1


def test_a_magnet_leaves_what_it_does_not_accept(make):
    """The selectivity a wind cannot express: its `trigger` names kinds, not objects."""
    env = make([box("b", 0.3, 0.0), box("decoy", 0.0, 0.3), coil(accept=["b"])])
    start = at(env, "decoy")
    run(env, 2.0)
    assert float(np.linalg.norm(at(env, "decoy") - start)) < 0.001
    assert far(env, "b") <= HOLD


def test_a_magnet_pulls_from_any_side(make):
    """The direction follows the object: one magnet collects from several sides."""
    env = make([box("east", 0.3, 0.0), box("north", 0.0, 0.3), coil()])
    run(env, 2.0)
    assert far(env, "east") <= HOLD
    assert far(env, "north") <= HOLD


def test_a_magnet_that_goes_off_does_not_deliver_the_object(make):
    """Switching off stops the pull; the object coasts and friction stops it short of
    the post. Measured 2026-11-17: released at 0.472 m it coasted 0.102 m and rested at
    0.370 m; released at 0.359 m it coasted 0.283 m. So "off" is not "frozen", it is
    "no longer arriving"."""
    env = make([box("b", 0.5, 0.0), coil(radius=0.6, enable="sw"),
                {"name": "sw", "type": "source", "pos": [0.0, -0.8], "value": 1.0,
                 "body": False}])
    run(env, 0.15)
    moving = far(env, "b")
    assert moving < 0.5
    env.parts["sw"].set_value(env.model, env.data, "value", 0.0)
    run(env, 2.0)
    rest = far(env, "b")
    assert HOLD < rest < moving            # it stopped on the way, it never arrived
    assert env.signals.get("coil") == 0


def test_a_magnet_reads_one_only_inside_hold(make):
    env = make([box("b", 0.35, 0.0), coil(power=0.0)])      # in the field, never pulled
    run(env, 0.5)
    assert far(env, "b") == pytest.approx(0.35, abs=0.01)
    assert env.signals.get("coil") == 0


def test_a_magnet_keeps_its_hold_across_a_checkpoint(make):
    env = make([box("b", 0.3, 0.0), coil(accept=["b"])])
    run(env, 2.0)
    saved = env.parts["coil"].save_state()
    assert saved["held"] is True
    env.parts["coil"].held = False
    env.parts["coil"].load_state(env.model, env.data, saved)
    assert env.parts["coil"].held is True


def test_a_magnet_without_damping_throws_the_object_past_the_pad(make):
    """Why `damp` exists. A constant pull over the radius gives the object as much energy
    as friction takes back over the same distance, so it crosses the pad and leaves the
    field. Measured on a 0.03 kg box pulled 0.39 m at 1.0 N: it rested 0.269 m away with
    no damping and 0.004 m away with the preset 0.4 N s/m."""
    loose = make([box("b", 0.39, 0.06, mass=0.03), coil(radius=0.45, damp=0.0)],
                 name="loose")
    run(loose, 4.0)
    assert far(loose, "b") > 0.1
    assert loose.signals.get("coil") == 0

    damped = make([box("b", 0.39, 0.06, mass=0.03), coil(radius=0.45, damp=0.4)],
                  name="damped")
    run(damped, 4.0)
    assert far(damped, "b") < 0.02
    assert damped.signals.get("coil") == 1


def test_a_wind_cannot_hold_an_object_in_place(make):
    """R104: the measurement that justified the part. A wind's force is one constant
    direction, so the box it moves never stops - it crosses the zone and keeps going.
    Measured before `magnet` was written: 0.5 N on a 0.05 kg cube, x 1.1 -> -1.07 m."""
    env = make([box("b", 0.3, 0.0),
                {"name": "gale", "type": "wind", "pos": [0.0, 0.0], "size": [0.9, 0.3],
                 "yaw": 180.0, "power": 0.5, "trigger": "object", "height": 0.3}])
    run(env, 2.0)
    x = at(env, "b")[0]
    assert x < -0.4                        # blown clean past the centre, never held at it
    assert abs(x) > 0.45                   # and out of the zone, so nothing stopped it there
