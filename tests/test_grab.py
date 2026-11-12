"""R18 beak grab (weld) and the `beak` obs item, R17 cube with pressure plate and socket,
R6 snapshot while holding.

The cube is put with a handle at the beak tip (or near it) instead of walking
up to it; the scripted `grab` sub-goal is tested with the example room (slow).
"""

import mujoco
import numpy as np
import pytest
from rooms import body_steps, open_room

from mujoco_puzzle_gym.room import RoomError


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


# ---- R17 cube -----------------------------------------------------------------------------
def test_cube_mass_and_handles(make):
    env = make([{**CUBE, "mass": 0.04, "handles": ["+x", "+z"]}])
    m, c = env.model, env.parts["c"]
    assert m.body_subtreemass[c._bid] == pytest.approx(0.04)
    names = {m.geom(i).name for i in range(m.ngeom) if m.geom(i).name.startswith("part/c/handle")}
    assert names == {"part/c/handle+x", "part/c/handle+z"}
    pts = c.handle_points(env.data) - c.pos(env.data)
    assert pts[0][0] > 0.02 and pts[1][2] > 0.02          # +x knob sticks out along x, +z up


def test_cube_rejects_unknown_face(tmp_path, asset_dir):
    with pytest.raises(RoomError, match="handles"):
        open_room(tmp_path, [{**CUBE, "handles": ["+q"]}])


@pytest.mark.parametrize("mass, on", [(0.02, 0), (0.04, 1)])
def test_pressure_plate_with_cube(make, mass, on):
    env = make([{"name": "plate", "type": "pressure", "pos": [0.5, 0.0], "size": [0.2, 0.2],
                 "min_mass": 0.03}, {**CUBE, "mass": mass}])
    env.parts["c"].put(env.data, [0.5, 0.0, 0.05])
    body_steps(env, 0.6)
    assert env.signals.get("plate") == on


def test_socket_takes_the_cube_once_let_go(make):
    env = make([{"name": "s", "type": "receptacle", "shape": "plate", "pos": [0.6, 0.0],
                 "size": [0.12, 0.12], "tolerance": 0.04, "accept": ["c"]}, CUBE])
    to_beak(env, "c")
    grab(env)
    assert env.signals.get("beak.holding") == 1
    c = env.parts["c"]
    hold = c.pos(env.data)
    env.parts["s"].place(env.model, hold[:2])            # the socket right under the held cube
    body_steps(env, 0.3)
    assert env.signals.get("s.ok") == 0                    # held: not in (R5)
    grab(env, on=False, seconds=0.8)
    assert env.signals.get("s.ok") == 1


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
