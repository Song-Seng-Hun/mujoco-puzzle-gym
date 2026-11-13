"""R18 beak grab (weld) and the `beak` obs item, R6 snapshot while holding, R14 warp
and reset while holding. The cube's own tests (R17) are in test_cube.py.

The cube is put with a handle at the beak tip (or near it) instead of walking
up to it; the scripted `grab` sub-goal is tested with the example room (slow).
"""

import mujoco
import numpy as np
import pytest
from rooms import body_steps, open_room, stand_at


@pytest.fixture
def make(asset_dir, tmp_path):
    envs = []

    def _make(parts, **kw):
        env = open_room(tmp_path, parts, spawn=kw.pop("spawn", (0.0, 0.0)), **kw)
        envs.append(env)
        return env

    yield _make
    for e in envs:
        e.close()


def tip(env):
    return env.data.site_xpos[env.world.beak_site].copy()


def to_beak(env, name, face="-x", offset=(0.0, 0.0, 0.0)):
    """Put cube `name` with handle `face` at the beak tip (+ offset)."""
    c = env.parts[name]
    c.put(env.data, tip(env) - c._local[c.handles.index(face)] + np.asarray(offset))
    mujoco.mj_forward(env.model, env.data)


def grab(env, on=True, seconds=0.04):
    env.world.grab = 1.0 if on else -1.0
    body_steps(env, seconds)


CUBE = {"name": "c", "type": "cube", "pos": [0.8, 0.8]}


# ---- R18 grab --------------------------------------------------------------------------
def test_carried_cube_stays_at_the_beak(make):
    env = make([CUBE])
    to_beak(env, "c", offset=(0.01, 0.0, 0.01))
    grab(env)
    assert env.signals.get("beak.holding") == 1 and env.signals.get("c.held") == 1
    c = env.parts["c"]
    rid = int(env.model.body_rootid[c._bid])
    assert env.world.held == {rid}
    d0 = np.linalg.norm(c.pos(env.data) - tip(env))
    start = env.body_state.pos[:2].copy()
    dev = []
    body_steps(env, 4.0, cmd=np.array([0.3, 0.0, 0.0]),
               every=lambda e: dev.append(abs(np.linalg.norm(c.pos(e.data) - tip(e)) - d0)))
    assert np.linalg.norm(env.body_state.pos[:2] - start) > 0.3
    assert max(dev) < 0.02
    grab(env, on=False, seconds=1.0)                     # let go: it falls to the floor
    assert env.signals.get("beak.holding") == 0 and env.world.held == set()
    assert c.pos(env.data)[2] == pytest.approx(c.size / 2, abs=0.005)


def test_no_grab_outside_the_radius(make):
    env = make([{**CUBE, "grab_radius": 0.03}])
    to_beak(env, "c", offset=(0.0, 0.0, 0.05))
    grab(env)
    assert env.signals.get("beak.holding") == 0
    to_beak(env, "c", offset=(0.0, 0.0, 0.02))
    grab(env)
    assert env.signals.get("beak.holding") == 1


def test_too_heavy_is_not_lifted(make):
    env = make([{**CUBE, "mass": 0.08}])                   # body max_lift_mass 0.05 kg
    to_beak(env, "c")
    env.world.grab = 1.0
    body_steps(env, 0.02)
    assert env.signals.get("beak.holding") == 0 and env.signals.get("beak.too_heavy") == 1


def test_hold_force_limit_drops_and_needs_a_new_grab(make):
    env = make([{**CUBE, "max_hold_force": 0.1}])          # below the cube's weight (0.29 N)
    to_beak(env, "c")
    seen = []
    env.world.grab = 1.0
    body_steps(env, 0.1, every=lambda e: seen.append(e.signals.get("c.dropped")))
    assert 1.0 in seen and env.signals.get("beak.holding") == 0
    to_beak(env, "c")
    body_steps(env, 0.04)                                  # still closed: no grab again
    assert env.signals.get("beak.holding") == 0


def test_snapshot_while_holding_restores_the_hold(make):
    env = make([CUBE])
    to_beak(env, "c")
    grab(env)
    snap = env.save_snapshot()
    grab(env, on=False, seconds=1.0)
    assert env.signals.get("beak.holding") == 0
    env.restore_snapshot(snap)
    eq = env.parts["_beak"]._eq[0]
    assert env.data.eq_active[eq] == 1                     # mjSTATE_INTEGRATION keeps eq_active (R6)
    assert env.signals.get("beak.holding") == 1 and env.world.held
    env.world.grab = 1.0
    c = env.parts["c"]
    d0 = np.linalg.norm(c.pos(env.data) - tip(env))
    body_steps(env, 2.0, cmd=np.array([0.3, 0.0, 0.0]))
    assert abs(np.linalg.norm(c.pos(env.data) - tip(env)) - d0) < 0.02


def test_beak_obs_item(make):
    from mujoco_puzzle_gym.obs import beak

    env = make([CUBE])
    assert beak(env)[:2].tolist() == [0.0, 0.0]
    to_beak(env, "c", offset=(0.0, 0.0, 0.03))
    o = beak(env)
    assert o[4] == pytest.approx(0.03, abs=1e-6)              # nearest handle 3 cm over the tip
    to_beak(env, "c")
    grab(env)
    assert beak(env)[:2].tolist() == [1.0, 1.0]                # holding a cube (kind 1)


def test_warp_carries_the_held_cube(make):
    """R14: a body warp takes the held cube along at the same pose to the head."""
    env = make([CUBE])
    to_beak(env, "c")
    grab(env)
    c, head = env.parts["c"], env.parts["_beak"]._head
    def rel(e):
        return e.data.xmat[head].reshape(3, 3).T @ (c.pos(e.data) - e.data.xpos[head])
    r0 = rel(env)
    env.warp("body", (0.6, -0.5, 1.2))
    assert np.allclose(rel(env), r0, atol=1e-6)
    assert np.hypot(*(c.pos(env.data)[:2] - [0.6, -0.5])) < 0.15
    body_steps(env, 1.0)
    assert env.signals.get("beak.holding") == 1 and np.all(np.isfinite(env.data.qpos))
    assert np.linalg.norm(rel(env) - r0) < 0.02 and not env.body_state.fallen


def test_reset_clears_the_hold(make):
    env = make([CUBE])
    to_beak(env, "c")
    grab(env)
    eq = env.parts["_beak"]._eq[0]
    assert env.data.eq_active[eq] == 1
    env.reset(seed=1)
    assert env.data.eq_active[eq] == 0 and env.world.held == set()
    assert env.parts["_beak"].held is None and env.signals.get("beak.holding") == 0


def test_walk_off_is_not_blocked_by_a_zone_sensor_box(make):
    """The walk-off ray ignores zone sensor boxes (group 1, no contacts): standing in a
    drop zone after letting go is not 'a wall ahead' (R31 overload's dump)."""
    from mujoco_puzzle_gym.solver import ScriptedSolver

    env = make([{"name": "dump", "preset": "input/object_zone", "pos": [0.0, 0.0], "size": [0.4, 0.4]}])
    stand_at(env, 0.1, 0.0)                              # the box's far face 0.1 m ahead
    assert not ScriptedSolver(env).carrier._blocked_ahead()
