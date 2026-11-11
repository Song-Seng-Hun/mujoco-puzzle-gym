"""R5 sensors: each turns on in one case and stays off in another.

Zone (direction, rearm, named objects), laser, pressure plate (a light and a
heavy test box; the cube itself comes with E3), receptacle (require x4,
wrong x4, close_on_ok, socket plate, held objects). The target sensor needs
cannon shells (E2). Also the R8b lap counter run with the walking body (slow).
"""

import numpy as np
import pytest
from rooms import open_room, stand_at, tick

from mujoco_puzzle_gym.parts.input.pressure import THICK


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


def settle(env, seconds=0.4):
    for _ in range(int(round(seconds * env.hz))):
        env.advance(np.zeros(3))


# ---- zone --------------------------------------------------------------------------------
def test_zone_on_and_off(make):
    env = make([{"name": "z", "type": "zone", "pos": [0, 0], "size": [0.4, 0.4]}])
    stand_at(env, 0.0, 0.1)
    assert env.signals.get("z") == 1
    stand_at(env, 0.5, 0.0)
    assert env.signals.get("z") == 0


def test_zone_direction(make):
    env = make([{"name": "z", "type": "zone", "pos": [0, 0], "size": [0.4, 0.4],
                 "direction": [1, 0]}])
    for x in (-0.4, 0.0):                               # enters moving +x
        stand_at(env, x, 0.0)
    assert env.signals.get("z") == 1
    for x in (0.4, 0.0):                                # enters moving -x
        stand_at(env, x, 0.0)
    assert env.signals.get("z") == 0


@pytest.mark.parametrize("rearm, at_edge", [("leave", 1), ("always", 0)])
def test_zone_rearm(make, rearm, at_edge):
    env = make([{"name": "z", "type": "zone", "pos": [0, 0], "size": [0.4, 0.4], "rearm": rearm,
                 "leave_margin": 0.1}])
    stand_at(env, 0.0, 0.0)
    stand_at(env, 0.25, 0.0)                            # just outside, inside the margin
    assert env.signals.get("z") == at_edge
    stand_at(env, 0.4, 0.0)
    assert env.signals.get("z") == 0


def test_zone_named_objects(make):
    env = make([{"name": "z", "type": "zone", "pos": [0, 0], "size": [0.4, 0.4],
                 "objects": ["box_a"]},
                {"name": "box_a", "type": "t_box", "pos": [1.0, 0.0]},
                {"name": "box_b", "type": "t_box", "pos": [1.0, 0.4]}])
    stand_at(env, 0.0, 0.0)                             # the body does not count
    assert env.signals.get("z") == 0
    env.parts["box_b"].put(env.data, [0.0, 0.1, 0.03])
    tick(env)
    assert env.signals.get("z") == 0
    env.parts["box_a"].put(env.data, [0.0, -0.1, 0.03])
    tick(env)
    assert env.signals.get("z") == 1 and env.parts["z"].role == "sensor"


# ---- laser -------------------------------------------------------------------------------
@pytest.mark.parametrize("trigger, body_on, box_on", [("any", 1, 1), ("body", 1, 0)])
def test_laser(make, trigger, body_on, box_on):
    env = make([{"name": "beam", "type": "laser", "pos": [0.0, -0.5], "to": [0.0, 0.5],
                 "trigger": trigger},
                {"name": "box", "type": "t_box", "pos": [1.0, 0.0]}])
    stand_at(env, -0.8, 0.0)
    assert env.signals.get("beam") == 0
    stand_at(env, 0.0, 0.0)
    assert env.signals.get("beam") == body_on and env.parts["beam"].hit == "body"
    stand_at(env, -0.8, 0.0)
    env.parts["box"].put(env.data, [0.0, 0.2, 0.03])
    tick(env)
    assert env.signals.get("beam") == box_on and env.parts["beam"].hit == "object"


# ---- pressure plate ---------------------------------------------------------------------
@pytest.mark.parametrize("mass, on", [(0.05, 0), (0.2, 1)])
def test_pressure_plate_weighs(make, mass, on):
    env = make([{"name": "plate", "type": "pressure", "pos": [0, 0], "size": [0.2, 0.2],
                 "min_mass": 0.1},
                {"name": "box", "type": "t_box", "pos": [1.0, 0.0], "mass": mass}])
    env.parts["box"].put(env.data, [0.0, 0.0, THICK + 0.031])
    settle(env)
    plate = env.parts["plate"]
    assert plate.mass == pytest.approx(mass, rel=0.1)
    assert env.signals.get("plate") == on


def test_pressure_plate_ignores_body_by_default(make):
    env = make([{"name": "plate", "type": "pressure", "pos": [0, 0], "size": [0.3, 0.3],
                 "min_mass": 0.1}])
    env.body.teleport(env.data, (0.0, 0.0, 0.0), z=THICK)
    settle(env)
    assert env.parts["plate"].mass == 0.0 and env.signals.get("plate") == 0


# ---- receptacle -------------------------------------------------------------------------
SPOTS = {"a": (-0.07, 0.0), "b": (0.07, 0.0), "c": (0.0, 0.08)}


def rec_env(make, **rec):
    boxes = [{"name": n, "type": "t_box", "pos": [1.0, 0.4 * i], "mass": 0.1}
             for i, n in enumerate("abc")]
    return make([{"name": "rec", "type": "receptacle", "pos": [0, 0], "size": [0.3, 0.3], **rec},
                 *boxes])


def put_in(env, name):
    env.parts[name].put(env.data, [*SPOTS[name], 0.032])
    tick(env)


def take_out(env, name):
    env.parts[name].put(env.data, [1.0, -0.5, 0.03])
    tick(env)


def ch(env, suffix):
    return env.signals.get("rec." + suffix)


@pytest.mark.parametrize("rec, steps", [
    ({"accept": ["a", "b"]}, [("a", 0), ("b", 1)]),                       # all
    ({"require": {"count": 2}}, [("a", 0), ("c", 1)]),
    ({"require": "mass", "mass": 0.15}, [("b", 0), ("a", 1)]),
    ({"require": {"order": ["a", "b"]}}, [("a", 0), ("b", 1)]),
    ({"accept": ["box"], "require": "count", "count": 3}, [("a", 0), ("b", 0), ("c", 1)]),  # kind
])
def test_receptacle_require(make, rec, steps):
    env = rec_env(make, **rec)
    for name, ok in steps:
        put_in(env, name)
        assert ch(env, "ok") == ok, name
    assert ch(env, "count") == len(steps) and ch(env, "wrong") == 0


def test_receptacle_order_wrong_turn(make):
    env = rec_env(make, require={"order": ["a", "b"]})
    put_in(env, "b")
    put_in(env, "a")
    assert ch(env, "ok") == 0 and ch(env, "wrong") == 1


def test_receptacle_wrong_ignore(make):
    env = rec_env(make, accept=["a"])
    put_in(env, "c")
    assert ch(env, "wrong") == 1 and ch(env, "ok") == 0
    put_in(env, "a")
    assert ch(env, "ok") == 1 and ch(env, "count") == 1
    take_out(env, "c")
    assert ch(env, "wrong") == 0


def test_receptacle_wrong_eject(make):
    env = rec_env(make, accept=["a"], wrong="eject")
    put_in(env, "c")
    settle(env, 1.0)
    pos = env.parts["c"].pos(env.data)
    assert np.abs(pos[:2]).max() > 0.15                 # out over the wall
    assert ch(env, "wrong") == 0


def test_receptacle_wrong_lock(make):
    env = rec_env(make, accept=["a"], wrong="lock")
    rec = env.parts["rec"]
    put_in(env, "c")
    assert rec.locked and env.data.mocap_pos[rec._mocap][2] > 0
    take_out(env, "c")
    assert ch(env, "wrong") == 1                        # stays locked


def test_receptacle_wrong_fail_ends_episode(make):
    env = rec_env(make, accept=["a"], wrong="fail")
    env.parts["c"].put(env.data, [*SPOTS["c"], 0.032])
    _, _, terminated, _, info = env.step(np.array([-1.0, 0.0, 0.0]))
    assert terminated and info["failed"] == ["rec"] and not info["success"]


def test_receptacle_close_on_ok(make):
    env = rec_env(make, accept=["a"], close_on_ok=True)
    rec = env.parts["rec"]
    assert env.data.mocap_pos[rec._mocap][2] < 0
    put_in(env, "a")
    assert ch(env, "ok") == 1 and env.data.mocap_pos[rec._mocap][2] > 0


@pytest.mark.parametrize("xy, yaw_deg, ok", [
    ((0.01, 0.0), 0, 1), ((0.01, 0.0), 90, 1), ((0.01, 0.0), 45, 0), ((0.06, 0.0), 0, 0),
])
def test_socket_plate(make, xy, yaw_deg, ok):
    env = rec_env(make, shape="plate", accept=["a"], tolerance=0.03, angle_tolerance=10)
    env.parts["a"].put(env.data, [*xy, 0.032], yaw=np.deg2rad(yaw_deg))
    tick(env)
    assert ch(env, "ok") == ok


def test_receptacle_held_object_not_counted(make):
    env = rec_env(make, accept=["a"])
    env.world.held.add(env.parts["a"]._bid)
    put_in(env, "a")
    assert ch(env, "ok") == 0
    env.world.held.clear()
    tick(env)
    assert ch(env, "ok") == 1


# ---- R8b: lap counter with the walking body ------------------------------------------------
LAP = [(0.5, -0.5), (0.5, 0.5), (-0.5, 0.5), (-0.5, -0.5)]


def drive(env, waypoints, laps):
    """Pursue the waypoints in order for `laps` rounds (solver-style steering)."""
    targets = waypoints * laps
    i = 0
    for _ in range(int(60 * laps * env.hz)):
        s = env.body_state
        d = np.asarray(targets[i]) - s.pos[:2]
        if np.hypot(*d) < 0.15:
            i += 1
            if i == len(targets):
                return True
            continue
        err = (np.arctan2(d[1], d[0]) - s.yaw + np.pi) % (2 * np.pi) - np.pi
        env.step(np.array([1.0 if abs(err) < 0.4 else 0.0, 0.0, np.clip(2.5 * err, -1, 1)]))
        assert not env.body_state.fallen
    return False


@pytest.mark.slow
@pytest.mark.parametrize("ccw, want", [(True, 3), (False, 0)])
def test_lap_counter_three_laps(make, ccw, want):
    # Zone A on the bottom edge counts +x passes, zone B on the top edge -x passes;
    # a sequence lock A -> B feeds the counter: only counter-clockwise laps count.
    parts = [
        {"name": "a", "type": "zone", "pos": [0.0, -0.5], "size": [0.3, 0.6], "direction": [1, 0],
         "rearm": "leave"},
        {"name": "b", "type": "zone", "pos": [0.0, 0.5], "size": [0.3, 0.6], "direction": [-1, 0],
         "rearm": "leave"},
        {"name": "lap", "type": "gate", "sequence": ["a", "b"]},
        {"name": "laps", "type": "counter", "up": "lap", "target": 3},
    ]
    task = {"env": {"episode_seconds": 600.0}}
    yaw = 0.0 if ccw else np.pi / 2
    env = make(parts, spawn=(-0.5, -0.5), task=task)
    env.body.teleport(env.data, (-0.5, -0.5, yaw))
    env.body_state = env.body.state(env.data)
    route = LAP if ccw else [LAP[2], LAP[1], LAP[0], LAP[3]]
    assert drive(env, route, 3)
    assert env.signals.get("laps.count") == want
    assert env.signals.get("laps.reached") == float(want >= 3)
