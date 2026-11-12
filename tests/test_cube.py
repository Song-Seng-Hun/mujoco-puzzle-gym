"""R17 cube: mass and handles, the pressure plate and the socket count it (not
while held). Grab helpers come from test_grab.py."""

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
