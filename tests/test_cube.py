"""R17 cube: mass and handles, the pressure plate and the socket count it (not
while held); R31 `rest_on`: a cube starts resting on another cube's top. Grab
helpers come from test_grab.py."""

import numpy as np
import pytest
from rooms import body_steps, open_room
from test_grab import CUBE, grab, make, to_beak  # noqa: F401  (make: fixture)

from mujoco_puzzle_gym.room import RoomError


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


# ---- R31 overload: a cube resting on a cube (`rest_on`) -------------------------------------
CART = {"name": "cart", "type": "cube", "pos": [0.8, 0.8], "size": 0.18, "mass": 0.06, "friction": 0.4,
        "hitch": True, "random": [0.1, 0.1]}
LOAD = {"name": "c", "type": "cube", "pos": [0.87, 0.75], "size": 0.04, "mass": 0.05, "friction": 0.8,
        "rest_on": "cart"}


def test_rest_on_starts_on_the_base_top_and_moves_with_it(make):
    env = make([CART, LOAD])
    cart, c = env.parts["cart"], env.parts["c"]
    bases = []
    for seed in range(3):
        env.reset(seed=seed)
        base = env.data.xpos[cart._bid].copy()
        bases.append(base[:2])
        assert c.pos(env.data)[:2] == pytest.approx(base[:2] + [0.07, -0.05], abs=1e-6)
        assert c.pos(env.data)[2] == pytest.approx(0.18 + 0.02, abs=2e-3)
    assert np.ptp(np.array(bases), axis=0).max() > 0.01                 # the cart's random range moved it
    body_steps(env, 0.5)
    assert c.pos(env.data)[2] == pytest.approx(0.2, abs=5e-3)          # it rests there


def test_rest_on_cube_gets_the_hitch_contact_priority(make):
    from mujoco_puzzle_gym.parts.object.cube import REST_PRIORITY

    env = make([CART, LOAD])
    m = env.model
    assert m.geom("part/c/box").priority == REST_PRIORITY
    assert m.geom("part/cart/box").priority == 0           # not resting on anything (no rope here: no hitch priority)


@pytest.mark.parametrize("change, why", [
    ({"random": [0.05, 0.0]}, "random"),
    ({"rest_on": "goal"}, "must name another cube"),
    ({"rest_on": "c"}, "must name another cube"),
    ({"pos": [1.0, 0.8]}, "outside its top face"),
])
def test_rest_on_rejects(tmp_path, asset_dir, change, why):
    with pytest.raises(RoomError, match=why):
        open_room(tmp_path, [CART, {**LOAD, **change}])


def test_rest_on_base_must_not_rest_on_another(tmp_path, asset_dir):
    top = {**LOAD, "name": "top", "rest_on": "c"}
    with pytest.raises(RoomError, match="must name another cube that is not stacked"):
        open_room(tmp_path, [CART, LOAD, top])
