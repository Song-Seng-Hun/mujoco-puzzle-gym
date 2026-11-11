"""R9: sliding door, hinged door and trapdoor."""

from __future__ import annotations

import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym.check import check_order
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.room import Room, RoomError

from mujoco_puzzle_gym.parts.base import Part
from mujoco_puzzle_gym.registry import register_part

from rooms import open_room

WALK = np.array([0.3, 0.0, 0.0])


class Source(Part):
    """Test-only input: writes `value` (set by the test) on its channel every step."""

    role = "switch"
    stage = "input"
    emits = True
    has_body = False

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.value = 0.0

    def reset(self, model, data):
        self.value = 0.0

    def update(self, model, data, body):
        self.write(self.value)

    def satisfied(self):
        return self.value > 0.5

    def target_mask(self, grid):
        return grid.disk(self.xy, 0.1)


register_part("t_signal", Source, replace=True)


def wall_room(tmp_path, door: dict, name="doors"):
    """A wall across x = 0 with a 0.5 m gap at y = 0 closed by `door`; a far switch drives it."""
    raw = {
        "name": name, "size": [2.0, 1.6], "spawn": {"pos": [-0.35, 0.0], "yaw": 0.0},
        "walls": [{"from": [0.0, -0.8], "to": [0.0, -0.25]}, {"from": [0.0, 0.25], "to": [0.0, 0.8]}],
        "parts": [
            {"name": "b", "type": "t_signal", "pos": [-0.8, 0.6]},
            {"name": "d", "pos": [0.0, 0.0], "in": ["b"], **door},
            {"name": "goal", "type": "goal", "pos": [0.7, 0.0], "radius": 0.1},
        ],
    }
    path = tmp_path / f"{name}.yaml"
    path.write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(room=path)
    env.reset(seed=0, options={"layout": env.sample_layout(None)})
    return env


def hold(env, name: str, value: float):
    """Set a test source (`t_signal`) or turn a button on."""
    part = env.parts[name]
    if isinstance(part, Source):
        part.value = value
    else:
        part._set(env.model, bool(value))


def run(env, cmd, seconds, hold_on: str | None = None):
    """Body steps with `cmd` for `seconds`; `hold_on` keeps that source at 1, else 0."""
    fell = False
    src = [p for p in env.parts.values() if isinstance(p, Source)]
    for p in src:
        p.value = 1.0 if p.name == hold_on else 0.0
    for _ in range(int(round(seconds / env.body.control_dt))):
        env.body.step(env.data, cmd, on_substep=env._on_substep)
        env.signals.step(env.model, env.data, env.read_body())
        fell |= env.read_body().fallen
    env.body_state = env.read_body()
    return fell


SLIDING = {"type": "door", "width": 0.5, "axis": "y"}
SWINGING = {"type": "swing_door", "width": 0.5, "axis": "y", "hinge": -1, "open_dir": 1}


@pytest.mark.parametrize("door", [SLIDING, SWINGING], ids=["sliding", "swing"])
def test_open_door_lets_the_body_through(tmp_path, door):
    env = wall_room(tmp_path, door)
    run(env, np.zeros(3), 1.5, hold_on="b")
    assert env.parts["d"].state() > 0.95
    fell = run(env, WALK, 6.0, hold_on="b")
    assert not fell
    assert env.body_state.pos[0] > 0.25, env.body_state.pos


@pytest.mark.parametrize("door", [SLIDING, SWINGING], ids=["sliding", "swing"])
def test_closed_door_blocks(tmp_path, door):
    env = wall_room(tmp_path, door)
    run(env, WALK, 6.0)
    assert env.parts["d"].state() < 0.05
    assert env.body_state.pos[0] < 0.0, env.body_state.pos


@pytest.mark.parametrize("door", [SLIDING, SWINGING], ids=["sliding", "swing"])
@pytest.mark.parametrize("mode", ["follow", "oneshot"])
def test_mode_follow_and_oneshot(tmp_path, door, mode):
    env = wall_room(tmp_path, {**door, "mode": mode})
    run(env, np.zeros(3), 1.5, hold_on="b")
    assert env.parts["d"].state() > 0.95
    run(env, np.zeros(3), 1.5)            # nothing on the momentary button: signal 0
    assert env.signals.get("b") == 0.0
    if mode == "oneshot":
        assert env.parts["d"].state() > 0.95
    else:
        assert env.parts["d"].state() < 0.05


def test_sliding_door_speed(tmp_path):
    env = wall_room(tmp_path, {**SLIDING, "speed": 0.26})       # travel 0.52 m in 2 s
    run(env, np.zeros(3), 1.0, hold_on="b")
    assert env.parts["d"].state() == pytest.approx(0.5, abs=0.06)
    run(env, np.zeros(3), 1.5, hold_on="b")
    assert env.parts["d"].state() > 0.95


def test_swing_door_speed(tmp_path):
    env = wall_room(tmp_path, {**SWINGING, "speed": 45.0})      # 90 deg in 2 s
    run(env, np.zeros(3), 1.0, hold_on="b")
    assert env.parts["d"].state() == pytest.approx(0.5, abs=0.06)


def test_swing_door_opens_to_the_chosen_side(tmp_path):
    for side in (1, -1):
        env = wall_room(tmp_path, {**SWINGING, "open_dir": side}, name=f"side{side}")
        run(env, np.zeros(3), 1.5, hold_on="b")
        free_end = env.data.geom_xpos[env.model.geom("part/d/panel").id]
        assert np.sign(free_end[0]) == side


def test_order_passes_through_doors(tmp_path):
    for door in (SLIDING, SWINGING):
        env = wall_room(tmp_path, door, name=door["type"])
        res = check_order(env)
        assert res["ok"], res
        assert res["order"] == ["b", "d", "goal"]


# ---- trapdoor ------------------------------------------------------------------------
def trap_env(tmp_path, **kw):
    return open_room(tmp_path, [
        {"name": "b", "type": "button", "kind": "oneshot", "pos": [1.0, -1.0]},
        {"name": "td", "type": "trapdoor", "pos": [0.0, 0.0], "size": [0.4, 0.4], "in": ["b"], **kw},
    ], spawn=(0.0, 0.0), name="trap")


def test_trapdoor_makes_a_tile_floor(tmp_path):
    env = trap_env(tmp_path)
    assert env.floor is not None
    assert env.model.geom("floor").pos[2] < -0.1          # catch plane
    assert env.floor_height((0.0, 0.0)) == (0.0, 0.0)
    assert env.floor_height((-1.0, 1.0)) == (0.0, 0.0)


def test_closed_trapdoor_holds_the_body(tmp_path):
    env = trap_env(tmp_path)
    assert not run(env, np.zeros(3), 2.0)
    assert env.body_state.pos[2] > 0.1


@pytest.mark.parametrize("side", ["-x", "+x", "-y", "+y"])
def test_open_trapdoor_drops_the_body_within_2s(tmp_path, side):
    env = trap_env(tmp_path, hinge_side=side)
    run(env, np.zeros(3), 0.5)
    hold(env, "b", 1.0)
    t, fell = 0.0, False
    while t < 2.0 and not fell:
        fell = run(env, np.zeros(3), 0.02)
        t += 0.02
    assert fell, env.body_state.pos
    assert env.floor_height((0.0, 0.0))[0] is None
    # the env ends the episode as a fall
    _, _, terminated, _, info = env.step(np.array([-1.0, 0.0, 0.0]))
    assert terminated and info["fallen"]


def test_trapdoor_cannot_move(tmp_path):
    with pytest.raises(RoomError, match="random"):
        trap_env(tmp_path, random=[0.1, 0.0])


def test_overlapping_floor_parts_are_rejected(tmp_path):
    raw = {"name": "ov", "size": [2, 2], "spawn": {"pos": [-0.8, -0.8]}, "parts": [
        {"name": "a", "type": "trapdoor", "pos": [0, 0], "size": [0.4, 0.4]},
        {"name": "c", "type": "floor_tile", "pos": [0.3, 0], "size": [0.4, 0.4]},
        {"name": "goal", "type": "goal", "pos": [0.8, 0.8]}]}
    with pytest.raises(RoomError, match="overlap"):
        Room.from_dict(raw)
