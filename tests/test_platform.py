"""R11: moving platform and elevator (slide joints + position actuators)."""

from __future__ import annotations

import numpy as np
import pytest

from mujoco_puzzle_gym.room import RoomError

from rooms import body_steps, open_room
from test_doors import Source  # noqa: F401  (registers t_signal)

DT = 0.02


def platform_env(tmp_path, name="pl", extra=(), **plat):
    p = {"name": "pl", "type": "platform", "pos": [-0.5, 0.5],
         "path": [[0.0, 0.0, 0.03], [1.0, 0.0, 0.03]], "speed": 0.2, **plat}
    return open_room(tmp_path, [p, *extra], spawn=(-1.0, -1.0), name=name)


def track(env, seconds, part="pl"):
    """Slab top centre after every body step."""
    out = []
    body_steps(env, seconds, every=lambda e: out.append(e.parts[part].top(e.data)))
    return np.array(out)


@pytest.mark.parametrize("speed", [0.1, 0.3])
def test_speed_along_the_path(tmp_path, speed):
    env = platform_env(tmp_path, speed=speed)
    tops = track(env, 3.0)
    mid = tops[25:125]                                  # 0.5 s .. 2.5 s, still on the first leg
    v = np.linalg.norm(mid[-1] - mid[0]) / (len(mid) - 1) / DT
    assert v == pytest.approx(speed, rel=0.05)
    assert np.abs(tops[:, 1] - 0.5).max() < 0.005       # stays on the path
    assert np.abs(tops[:, 2] - 0.03).max() < 0.005


def test_modes(tmp_path):
    # pingpong turns back at the end; once stays there; loop comes back to the start
    for mode, at_6s in (("pingpong", -0.5 + 0.2), ("once", 0.5), ("loop", -0.5 + 0.2)):
        env = platform_env(tmp_path, name=mode, mode=mode, speed=0.5,
                           path=[[0.0, 0.0, 0.03], [1.0, 0.0, 0.03]])
        tops = track(env, 3.6)                          # 1.8 m of travel
        assert tops[-1, 0] == pytest.approx(at_6s, abs=0.03), mode


def test_while_signal_and_on_signal(tmp_path):
    env = platform_env(tmp_path, name="ws", run="while_signal",
                       extra=[{"name": "s", "type": "t_signal", "pos": [1.0, -1.0]}], **{"in": ["s"]})
    x0 = env.parts["pl"].top(env.data)[0]
    body_steps(env, 1.0)
    assert env.parts["pl"].top(env.data)[0] == pytest.approx(x0, abs=0.002)   # off: still
    env.parts["s"].value = 1.0
    body_steps(env, 1.0)
    assert env.parts["pl"].top(env.data)[0] - x0 == pytest.approx(0.2, abs=0.02)

    lift = open_room(tmp_path, [
        {"name": "s", "type": "t_signal", "pos": [1.0, -1.0]},
        {"name": "lift", "preset": "device/elevator", "pos": [0.0, 0.5], "in": ["s"], "speed": 0.2}],
        spawn=(-1.0, -1.0), name="lift")
    top = lambda: lift.parts["lift"].top(lift.data)[2]            # noqa: E731
    body_steps(lift, 0.5)
    assert top() == pytest.approx(0.02, abs=0.003)
    lift.parts["s"].value = 1.0                                   # rising edge: one trip up
    body_steps(lift, 2.0)
    assert top() == pytest.approx(0.3, abs=0.005)
    lift.parts["s"].value = 0.0
    body_steps(lift, 1.0)
    assert top() == pytest.approx(0.3, abs=0.005)                 # stays up
    lift.parts["s"].value = 1.0                                   # next edge: back down
    body_steps(lift, 2.0)
    assert top() == pytest.approx(0.02, abs=0.005)


def test_bad_values_are_rejected(tmp_path):
    with pytest.raises(RoomError, match="path"):
        platform_env(tmp_path, name="b1", path=[[0, 0, 0.03]])
    with pytest.raises(RoomError, match="needs `in`"):
        platform_env(tmp_path, name="b2", run="on_signal")
    with pytest.raises(RoomError, match="speed"):
        platform_env(tmp_path, name="b3", speed=5.0)


def test_cube_rides_along(tmp_path):
    # R11: a cube on the slab moves with it (relative drift < 5 cm), 0.3 m/s for 3 s.
    env = platform_env(tmp_path, name="ride", speed=0.3,
                       extra=[{"name": "box", "type": "t_box", "pos": [1.0, 1.0], "half": 0.03}])
    box = env.parts["box"]
    box.put(env.data, [-0.5, 0.5, 0.03 + 0.03 + 0.001])
    body_steps(env, 0.3)
    rel0 = box.pos(env.data) - env.parts["pl"].top(env.data)
    body_steps(env, 3.0)
    rel = box.pos(env.data) - env.parts["pl"].top(env.data)
    assert env.parts["pl"].top(env.data)[0] > 0.3          # it did travel ~0.9 m
    print(f"\nR11 cube drift on the slab: {np.linalg.norm((rel - rel0)[:2]) * 100:.2f} cm")
    assert np.linalg.norm((rel - rel0)[:2]) < 0.05, rel - rel0


@pytest.mark.slow
@pytest.mark.parametrize("speed", [0.1, 0.3])
def test_body_on_moving_platform_record(tmp_path, speed):
    """Record only (R11): does the walking body stay up 3 s on a moving slab?"""
    env = platform_env(tmp_path, name="bodyride", speed=speed, size=[0.5, 0.5],
                       path=[[0.0, 0.0, 0.03], [1.5, 0.0, 0.03]])
    env.body.teleport(env.data, (-0.5, 0.5, 0.0), z=0.03)
    env.body_state = env.read_body()
    start = env.body_state.pos.copy()
    fell = body_steps(env, 3.0)
    moved = env.body_state.pos - start
    print(f"\nR11 record: body on platform {speed} m/s 3 s: fell={fell}, moved={moved.round(3).tolist()}, "
          f"slab moved {env.parts['pl'].top(env.data)[0] + 0.5:.3f} m")


def test_platform_on_a_raised_cell_keeps_its_height(tmp_path):
    # review fix: place() on reset must not drop the raised-cell height the room added (R37)
    import yaml
    from mujoco_puzzle_gym.env import PuzzleEnv
    raw = {"name": "raised", "cell": 0.1, "step_height": 0.01,
           "map": "##########\n#........#\n#..3333..#\n#..3333..#\n#........#\n##########\n",
           "spawn": {"pos": [-0.3, -0.1], "yaw": 0.0},
           "parts": [{"name": "pl", "type": "platform", "pos": [0.0, 0.05],
                      "path": [[-0.05, 0.0, 0.03], [0.05, 0.0, 0.03]], "speed": 0.1},
                     {"name": "goal", "type": "goal", "pos": [0.3, -0.1], "radius": 0.05}]}
    path = tmp_path / "raised.yaml"
    path.write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(room=path)
    for seed in (0, 1):
        env.reset(seed=seed, options={"layout": env.sample_layout(None)})
        assert env.parts["pl"].top(env.data)[2] == pytest.approx(0.03 + 0.03, abs=0.002)


def test_swept_cells_follow_the_restored_position(tmp_path):
    # review fix: planning reads the part's position, not the shift of the last place()
    env = platform_env(tmp_path)
    pl = env.parts["pl"]
    before = pl._swept(env.grid)
    pl.xy = pl.base_xy + np.array([0.0, 0.3])              # as a snapshot restore sets it
    after = pl._swept(env.grid)
    assert not np.array_equal(before, after)
    dy = env.grid.Y[after].mean() - env.grid.Y[before].mean()
    assert dy == pytest.approx(0.3, abs=env.grid.res)
