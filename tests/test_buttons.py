"""R4 floor buttons: kind (momentary, oneshot, toggle) x enable on/off output
sequences, pressed by moving the body on and off the plate; what `trigger`
counts. Wall buttons (beak tip) are tested with the grab milestone (E3)."""

import pytest
from rooms import open_room, stand_at, tick

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
    env.world.held.add(box._bid)                        # in the beak (E3)
    tick(env)
    assert env.signals.get("btn") == 0
    env.world.held.clear()
    tick(env)
    assert env.signals.get("btn") == 1


def test_wall_mount_not_yet():
    from mujoco_puzzle_gym.parts.input import Button
    with pytest.raises(ValueError, match="mount"):
        Button("b", {"pos": [0, 0], "mount": "wall"})
