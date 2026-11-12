"""R4 buttons: kind (momentary, oneshot, toggle) x mount (floor, wall) x enable
on/off output sequences, pressed by moving the body on and off the plate (wall:
standing so the beak tip is at the plate); what `trigger` counts. R19: the
scripted solver lowers the head to press a low wall plate (slow)."""

import pytest
from rooms import open_room, stand_at, tick

from mujoco_puzzle_gym.env import PuzzleEnv

from mujoco_puzzle_gym.expr import compile_expr

ON_XY, OFF_XY = (0.0, 0.0), (-0.8, -0.8)
WANT = {
    "momentary": [1, 0, 1, 0],
    "oneshot": [1, 1, 1, 1],
    "toggle": [1, 1, 0, 0],
}


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


@pytest.mark.parametrize("power", [1, 0])
@pytest.mark.parametrize("kind", sorted(WANT))
def test_kind_and_enable(make, kind, power):
    env = make([{"name": "power", "type": "gate", "expr": str(power)},
                {"name": "btn", "type": "button", "kind": kind, "pos": [0, 0], "enable": "power"}])
    stand_at(env, *OFF_XY)          # the power gate (logic stage) writes after inputs ran once
    seen = []
    for xy in (ON_XY, OFF_XY) * 2:                      # press, release, press, release
        stand_at(env, *xy)
        seen.append(env.signals.get("btn"))
    assert seen == (WANT[kind] if power else [0, 0, 0, 0])
    assert env.parts["btn"].state() == seen[-1]


def test_power_on_while_held_is_not_a_press(make):
    env = make([{"name": "power", "type": "gate", "expr": "0"},
                {"name": "btn", "type": "button", "kind": "toggle", "pos": [0, 0],
                 "enable": "power"}])
    stand_at(env, *ON_XY)                               # pressed while off
    env.parts["power"].expr = compile_expr("1")
    tick(env)
    tick(env)                                           # power on, still standing there
    assert env.signals.get("power") == 1 and env.signals.get("btn") == 0


@pytest.mark.parametrize("trigger, body_presses, box_presses", [
    ("any", True, True), ("body", True, False), ("object", False, True), ("npc", False, False),
])
def test_trigger(make, trigger, body_presses, box_presses):
    env = make([{"name": "btn", "type": "button", "pos": [0, 0], "trigger": trigger},
                {"name": "box", "type": "t_box", "pos": [0.8, 0.8]}])
    stand_at(env, *ON_XY)
    assert env.signals.get("btn") == float(body_presses)
    stand_at(env, *OFF_XY)
    env.parts["box"].put(env.data, [0.05, 0.0, 0.03])
    tick(env)
    assert env.signals.get("btn") == float(box_presses)
    assert env.parts["btn"].role == ("switch" if body_presses else "sensor")


def test_object_carried_high_or_held_does_not_press(make):
    env = make([{"name": "btn", "type": "button", "pos": [0, 0], "trigger": "object"},
                {"name": "box", "type": "t_box", "pos": [0.8, 0.8]}])
    box = env.parts["box"]
    box.put(env.data, [0.0, 0.0, 0.3])                  # above max_height (0.2 m)
    tick(env)
    assert env.signals.get("btn") == 0
    box.put(env.data, [0.0, 0.0, 0.03])
    env.world.held.add(box._bid)                        # in the beak (R18)
    tick(env)
    assert env.signals.get("btn") == 0
    env.world.held.clear()
    tick(env)
    assert env.signals.get("btn") == 1


def wall_spots(env):
    """(on, off) body poses for a wall plate at the origin facing -x: on puts the
    beak tip on the plate face."""
    stand_at(env, *OFF_XY)
    off = env.data.site_xpos[env.world.beak_site] - env.body_state.pos
    return (-off[0] - 0.01, -off[1]), OFF_XY


def wall_button(tip_z, **kw):
    return {"name": "btn", "type": "button", "mount": "wall", "pos": [0, 0], "facing": 180,
            "height": tip_z, "size": [0.1, 0.08], **kw}


@pytest.mark.parametrize("power", [1, 0])
@pytest.mark.parametrize("kind", sorted(WANT))
def test_wall_kind_and_enable(make, kind, power):
    env = make([{"name": "power", "type": "gate", "expr": str(power)},
                wall_button(0.21, kind=kind, enable="power")])
    on, off = wall_spots(env)
    seen = []
    for xy in (on, off) * 2:
        stand_at(env, *xy)
        seen.append(env.signals.get("btn"))
    assert seen == (WANT[kind] if power else [0, 0, 0, 0])


def test_wall_plate_too_high_or_beside_is_not_pressed(make):
    env = make([wall_button(0.35)])
    on, _ = wall_spots(env)
    stand_at(env, *on)
    assert env.signals.get("btn") == 0                  # tip 14 cm under the plate centre
    env2 = make([wall_button(0.21)], name="beside")
    stand_at(env2, on[0], on[1] + 0.12)
    assert env2.signals.get("btn") == 0


def test_wall_plate_not_pressed_from_behind_a_thin_wall(make):
    """R4: behind the face only plate thickness + 1 cm counts (the plate faces -x)."""
    env = make([wall_button(0.21)])
    on, _ = wall_spots(env)                             # tip 1 cm in front of the wall face
    stand_at(env, on[0] + 0.015, on[1])                 # tip 0.5 cm into the wall: pressed
    assert env.signals.get("btn") == 1
    stand_at(env, on[0] + 0.04, on[1])                  # tip 3 cm behind: across a thin wall
    assert env.signals.get("btn") == 0


@pytest.mark.slow
def test_solver_lowers_the_head_to_press_a_low_wall_plate(tmp_path, asset_dir):
    """R19: plate centre 0.15 m up (the level tip is at 0.21 m), `head` in the actions."""
    import yaml

    from mujoco_puzzle_gym.solver import ScriptedSolver

    room = {"name": "wall_press", "size": [2.6, 1.8], "wall": {"thickness": 0.06, "height": 0.25},
            "walls": [{"from": [0.3, -0.9], "to": [0.3, -0.2]}, {"from": [0.3, 0.2], "to": [0.3, 0.9]}],
            "spawn": {"pos": [-0.6, 0.0], "yaw": 3.14},
            "parts": [wall_button(0.15, size=[0.12, 0.06], pos=[-1.27, 0.0], facing=0, kind="oneshot"),
                      {"name": "door", "type": "door", "pos": [0.3, 0.0], "width": 0.4, "axis": "y",
                       "in": ["btn"]},
                      {"name": "goal", "type": "goal", "pos": [0.9, 0.0], "radius": 0.12}]}
    (tmp_path / "wall_press.yaml").write_text(yaml.safe_dump(room))
    env = PuzzleEnv(task={"actions": ["move", "head"]}, room=tmp_path / "wall_press.yaml")
    assert env.action_space.shape == (4,)
    solver = ScriptedSolver(env)
    env.reset(seed=0)
    solver.reset()
    pressed_head, term, trunc = None, False, False
    while not (term or trunc):
        a = solver.act()
        _, _, term, trunc, info = env.step(a)
        if pressed_head is None and env.signals.get("btn"):
            pressed_head = a[3]
    assert info["success"]
    assert pressed_head is not None and pressed_head > -0.5   # head lowered when pressed
