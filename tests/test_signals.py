"""R1, R2: signal bus — channel values, evaluation order input -> logic -> device each
body step, stateless loops rejected by `mpg check`, loops through a stateful part
allowed, M1 `links:` read as channels.

Buttons, NOT and delay gates arrive in later steps (R4, R7); the smallest test
doubles below stand in for them.
"""

import json

import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym import cli
from mujoco_puzzle_gym.check import check_body, check_order
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.parts.base import Part
from mujoco_puzzle_gym.registry import register_part
from mujoco_puzzle_gym.room import Room, RoomError
from mujoco_puzzle_gym.signals import SignalError, wire

ROOM = "examples/switch_door.yaml"


# ---- test doubles -----------------------------------------------------------------
class NoBody(Part):
    """A part without a MuJoCo body (logic parts get theirs in step 3)."""

    def build(self, spec):
        return None

    def bind(self, model, data):
        pass

    def place(self, model, xy):
        self.xy = np.asarray(xy, dtype=float)


class Button(NoBody):
    stage, emits = "input", True
    pressed = False

    def update(self, model, data, body):
        if self.enabled():
            self.write(float(self.pressed))


class Not(NoBody):
    stage, emits = "logic", True

    def update(self, model, data, body):
        self.write(0.0 if self.inputs_on() else 1.0)


class Delay(NoBody):
    """One-step delay: outputs the input it read on the previous update."""

    stage, emits, stateful = "logic", True, True
    mem = 0.0

    def update(self, model, data, body):
        self.write(self.mem)
        self.mem = self.read(self.inputs[0])


class Probe(NoBody):
    """Device that records what it reads."""

    stage = "device"

    def update(self, model, data, body):
        self.seen = self.read(self.inputs[0])


for _name, _cls in (("t_button", Button), ("t_not", Not), ("t_delay", Delay), ("t_probe", Probe)):
    register_part(_name, _cls, replace=True)


def make(**cfgs) -> dict[str, Part]:
    """Parts from {name: (class, cfg)} in the given (room) order."""
    return {n: cls(n, {"pos": [0.0, 0.0], **cfg}) for n, (cls, cfg) in cfgs.items()}


def room_with(repo_root, tmp_path, parts):
    raw = yaml.safe_load((repo_root / ROOM).read_text())
    raw["parts"] += parts
    path = tmp_path / "room.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


# ---- bus ----------------------------------------------------------------------------
def test_order_input_logic_device_in_one_step():
    # Room order is device first and the NOT chain reversed; the bus still
    # carries a press through both NOTs to the device within one step.
    parts = make(probe=(Probe, {"in": ["n2"]}), n2=(Not, {"in": ["n1"]}),
                 n1=(Not, {"in": ["btn"]}), btn=(Button, {}))
    bus = wire(parts)
    assert [p.name for p in bus.order] == ["btn", "n1", "n2", "probe"]
    parts["btn"].pressed = True
    bus.step(None, None, None)
    assert bus.values == {"btn": 1.0, "n1": 0.0, "n2": 1.0}
    assert parts["probe"].seen == 1.0
    parts["btn"].pressed = False
    bus.step(None, None, None)
    assert parts["probe"].seen == 0.0


def test_enable_freezes_output():
    parts = make(power=(Button, {}), btn=(Button, {"enable": "power"}))
    bus = wire(parts)
    parts["btn"].pressed = True
    bus.step(None, None, None)
    assert bus.get("btn") == 0.0                 # power off: press ignored
    parts["power"].pressed = True
    bus.step(None, None, None)
    assert bus.get("btn") == 1.0
    parts["power"].pressed, parts["btn"].pressed = False, False
    bus.step(None, None, None)
    assert bus.get("btn") == 1.0                 # power off again: output held


def test_values_are_floats_and_on_above_half():
    parts = make(btn=(Button, {}))
    bus = wire(parts)
    bus.set("btn", 3)                            # a counter's value rides as is
    assert bus.get("btn") == 3.0 and isinstance(bus.get("btn"), float) and bus.on("btn")
    bus.set("btn", 0.5)
    assert not bus.on("btn")


def test_stateless_loop_rejected():
    parts = make(n1=(Not, {"in": ["n2"]}), n2=(Not, {"in": ["n1"]}))
    with pytest.raises(SignalError, match="n1 -> n2 -> n1"):
        wire(parts)


def test_loop_through_delay_allowed_and_runs():
    parts = make(n1=(Not, {"in": ["d"]}), d=(Delay, {"in": ["n1"]}))
    bus = wire(parts)
    seq = []
    for _ in range(6):
        bus.step(None, None, None)
        seq.append(bus.get("n1"))
    assert seq == [1.0, 1.0, 0.0, 0.0, 1.0, 1.0]   # NOT through a one-step delay oscillates


@pytest.mark.parametrize("cfgs, needle", [
    ({"p": (Probe, {"in": ["nope"]})}, "reads channel 'nope' that no part writes"),
    ({"a": (Button, {"out": ["x"]}), "b": (Button, {"out": ["x"]})}, "written by both 'a' and 'b'"),
])
def test_bad_channels_rejected(cfgs, needle):
    with pytest.raises(SignalError, match=needle):
        wire(make(**cfgs))


# ---- rooms and mpg check --------------------------------------------------------------
def test_m1_links_become_channels(repo_root):
    parts = Room.load(repo_root / ROOM).make_parts()
    assert parts["switch_a"].outputs == ["switch_a"]
    assert parts["door_b"].inputs == ["switch_a"]
    assert [s.name for s in parts["door_b"].sources] == ["switch_a"]
    assert [p.name for p in parts.bus.order] == ["switch_a", "goal", "door_b"]


def test_in_channel_equals_link(repo_root, tmp_path):
    raw = yaml.safe_load((repo_root / ROOM).read_text())
    raw["links"] = []
    raw["parts"][1]["in"] = ["switch_a"]
    path = tmp_path / "room.yaml"
    path.write_text(yaml.safe_dump(raw))
    parts = Room.load(path).make_parts()
    assert parts["door_b"].inputs == ["switch_a"] and parts["door_b"].sources == [parts["switch_a"]]


def test_switch_to_door_same_body_step(asset_dir, repo_root, tmp_path):
    # Door listed before the switch: in room order it would see the switch one step late.
    raw = yaml.safe_load((repo_root / ROOM).read_text())
    raw["parts"] = [raw["parts"][1], raw["parts"][0], raw["parts"][2]]
    raw["parts"][1]["latch"] = False             # momentary: a floor button
    path = tmp_path / "room.yaml"
    path.write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(room=path)
    lay = env.sample_layout(None)
    lay["spawn"] = (-0.25, 0.5, 0.0)             # standing on the switch
    env.reset(options={"layout": lay})
    door = env.parts["door_b"]
    assert env.signals.get("switch_a") == 0.0 and env.data.ctrl[door._act] == 0.0
    env.signals.step(env.model, env.data, env.body.state(env.data))   # one body step
    assert env.signals.get("switch_a") == 1.0
    assert env.data.ctrl[door._act] == pytest.approx(door.travel)
    env.close()


def test_stateless_loop_fails_mpg_check(repo_root, tmp_path, capsys):
    path = room_with(repo_root, tmp_path, [
        {"name": "n1", "type": "t_not", "pos": [0, 0], "in": ["n2"]},
        {"name": "n2", "type": "t_not", "pos": [0, 0], "in": ["n1"]},
    ])
    code = cli.main(["check", str(path), "--json"])
    res = json.loads(capsys.readouterr().out)
    assert code != 0 and res["ok"] is False
    assert "stateless" in res["error"] and "n1 -> n2 -> n1" in res["error"]


def test_delay_loop_passes_static_check(asset_dir, repo_root, tmp_path):
    path = room_with(repo_root, tmp_path, [
        {"name": "n1", "type": "t_not", "pos": [0, 0], "in": ["d"]},
        {"name": "d", "type": "t_delay", "pos": [0, 0], "in": ["n1"]},
    ])
    env = PuzzleEnv(room=path)
    assert check_body(env)["ok"] and check_order(env)["ok"]
    env.reset(seed=0)
    seen = []
    for _ in range(4):
        env.advance(np.zeros(3))                 # 5 body steps per env step
        seen.append(env.signals.get("n1"))
    assert set(seen) == {0.0, 1.0}
    env.close()


def test_room_error_names_loop(repo_root, tmp_path):
    path = room_with(repo_root, tmp_path, [
        {"name": "n1", "type": "t_not", "pos": [0, 0], "in": ["n1"]},
    ])
    with pytest.raises(RoomError, match="n1 -> n1"):
        Room.load(path)
