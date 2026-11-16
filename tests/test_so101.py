"""M4 R61: a fixed-base arm is a `BodyAdapter`, and the boundary did not have to change.

The claim these tests hold up is the one that reverses M3's judgement: the puzzle side asks
a body where it is on the plane, which way it faces, how fast it is going and whether it
has fallen, and plans on a 2D grid - so an arm can answer all four about its **gripper**
and reuse the whole boundary. What the arm genuinely does differently is here too: it
cannot walk (and the gate says so by name), it cannot fall, its base is bolted down, and
its heading is not a channel it can command.
"""

import math

import mujoco
import numpy as np
import pytest

from mujoco_puzzle_gym.bodies import BODIES, flat_floor_spec, make_body
from mujoco_puzzle_gym.bodies.so101 import HOME, PAN_LIMIT, REACH_MAX, So101Adapter

ROOM = {
    "name": "table_touch",
    "size": [1.0, 0.8],
    "wall": {"thickness": 0.02, "height": 0.1},
    "spawn": {"pos": [-0.3, 0.0], "yaw": 0.0},
    "parts": [{"name": "goal", "type": "goal", "pos": [0.0, 0.1], "radius": 0.06}],
    "demo": ["goto goal"],
}


@pytest.fixture(scope="module")
def arm(so101_asset_dir):
    return make_body("so101")


@pytest.fixture(scope="module")
def scene(so101_asset_dir):
    body = make_body("so101")
    model = body.spawn(flat_floor_spec(), (0.0, 0.0, 0.0)).compile()
    data = mujoco.MjData(model)
    body.reset(data)
    return body, model, data


def test_the_arm_is_a_registered_body(so101_asset_dir):
    assert "so101" in BODIES
    assert isinstance(make_body("so101"), So101Adapter)


def test_it_has_no_free_joint_and_cannot_fall(scene):
    """A bolted base is the structural difference; `fallen` and the gravity vector follow
    from it, and the fall reward and termination never fire."""
    body, model, data = scene
    assert model.nq == model.nv == 6          # five arm joints and the jaw, no root
    state = body.state(data)
    assert state.fallen is False
    assert np.allclose(state.gravity, [0.0, 0.0, -1.0])
    assert body.fall_height == 0.0


def test_the_home_pose_puts_the_gripper_where_it_was_measured(scene):
    body, _, data = scene
    body.reset(data)
    pos = body.state(data).pos
    assert np.allclose(pos, [0.215, 0.0, 0.122], atol=0.005), pos.tolist()
    assert np.allclose(data.qpos[body._qpos], HOME)


def test_reset_cannot_move_a_bolted_base(scene):
    """`reset` takes a pose because walking bodies need one; this body ignores it, and the
    test states that rather than leaving a caller to assume otherwise."""
    body, _, data = scene
    body.reset(data, (0.0, 0.0, 0.0))
    home = body.state(data).pos.copy()
    body.reset(data, (5.0, -3.0, 1.2))
    assert np.allclose(body.state(data).pos, home)


def test_it_declares_what_it_can_do_and_not_walking(arm):
    caps = arm.capabilities
    assert "walk" not in caps.names
    assert {"reach", "gripper"} <= caps.names
    # The gate's question is "can it pick an object up with its own end effector", which
    # this arm can; the name is anatomy from the first body (M4 R60 renames it).
    assert "beak_grab" in caps.names
    assert caps.limit("reach_max") == pytest.approx(REACH_MAX)


def test_a_forward_command_moves_the_gripper_outward(scene):
    """Within about a centimetre over a 10 cm move. The first implementation integrated
    joint targets from the measured angles and accumulated the position actuators' lag
    instead: a 0.1 m/s command for 2 s travelled 0.243 m with 0.036 m of sideways drift.
    Driving a moving aim point has no such integral error."""
    body, _, data = scene
    body.reset(data)
    start = body.state(data).pos.copy()
    for _ in range(int(1.0 / body.control_dt)):
        body.step(data, (0.1, 0.0, 0.0))
    moved = (body.state(data).pos - start)[:2]
    assert np.linalg.norm(moved - np.array([0.1, 0.0])) < 0.02, moved.tolist()


def test_a_lateral_command_sweeps_an_arc_around_the_base(scene):
    """Not a straight line, and that is the geometry rather than an error: the command is
    read in the body's frame, this body's frame always faces radially outward, so a
    lateral command is a tangential one and the path curves around the base. Measured: 2 s
    at 0.1 m/s keeps the radius within 2 cm and sweeps 0.2 m of arc, so the straight-line
    displacement is the chord, 0.18 m, not 0.2 m."""
    body, _, data = scene
    body.reset(data)
    here = body.state(data).pos
    r0, a0 = float(np.hypot(here[0], here[1])), math.atan2(here[1], here[0])
    for _ in range(int(2.0 / body.control_dt)):
        body.step(data, (0.0, 0.1, 0.0))
    there = body.state(data).pos
    r1, a1 = float(np.hypot(there[0], there[1])), math.atan2(there[1], there[0])
    assert abs(r1 - r0) < 0.02, (r0, r1)                      # stayed on its circle
    arc = abs(a1 - a0) * (r0 + r1) / 2
    assert 0.15 < arc < 0.25, (arc, a1 - a0)                  # swept about the commanded 0.2 m
    assert a1 > a0                                            # and in the commanded direction


def test_a_zero_command_leaves_the_gripper_where_it_is(scene):
    body, _, data = scene
    body.reset(data)
    start = body.state(data).pos.copy()
    for _ in range(int(0.5 / body.control_dt)):
        body.step(data, (0.0, 0.0, 0.0))
    assert np.linalg.norm((body.state(data).pos - start)[:2]) < 0.02


def test_the_working_height_is_held(scene):
    body, _, data = scene
    body.reset(data)
    for _ in range(100):
        body.step(data, (0.05, 0.05, 0.0))
    assert 0.08 <= body.state(data).pos[2] <= 0.16


def test_the_aim_stays_inside_the_measured_envelope(scene):
    """Commanded outward for long enough to leave the envelope, the gripper stops at it
    instead of the solve blowing up where the Jacobian loses rank."""
    body, _, data = scene
    body.reset(data)
    for _ in range(400):
        body.step(data, (0.15, 0.0, 0.0))
    pos = body.state(data).pos
    assert np.hypot(pos[0], pos[1]) <= REACH_MAX + 0.02
    assert np.isfinite(data.qpos).all()


def test_its_heading_is_not_a_channel_it_can_command(scene):
    """Measured: the shoulder pan is the only joint that turns the jaw across the table,
    and it is also the joint that places the jaw, so the gripper always faces radially
    outward. This is why the body declares itself holonomic and the shared steering drives
    it by direction instead of by heading."""
    body, _, data = scene
    body.reset(data)
    before = body.state(data).yaw
    for _ in range(50):                       # a full second of turn command
        body.step(data, (0.0, 0.0, 1.0))
    assert abs(body.state(data).yaw - before) < 0.2
    assert body.holonomic is True


def test_the_other_bodies_are_not_holonomic():
    assert all(not make_body(n).holonomic for n in BODIES if n != "so101")


# ---- the gate, both ways -------------------------------------------------------------

def test_a_walking_room_is_refused_by_name(so101_asset_dir, asset_dir, repo_root):
    from mujoco_puzzle_gym.check import check

    result = check(repo_root / "examples/switch_door.yaml", static_only=True, body="so101")
    assert result["ok"] is False
    assert result["body"]["missing"] == {"switch_a": ["walk"]}
    assert "walk" in result["body"]["reason"]


def test_a_tabletop_room_passes_every_static_stage(so101_asset_dir, tmp_path):
    import yaml

    from mujoco_puzzle_gym.check import check

    path = tmp_path / "table.yaml"
    path.write_text(yaml.safe_dump(ROOM, sort_keys=False))
    result = check(path, static_only=True, body="so101")
    assert result["ok"], result
    assert result["body_name"] == "so101"


@pytest.mark.slow
def test_the_arm_solves_its_room(so101_asset_dir, tmp_path):
    """The whole point, measured: the scripted solver drives the gripper to a target with
    the shared grid planner and steering, on a body with no legs."""
    import yaml

    from mujoco_puzzle_gym.check import check

    path = tmp_path / "table.yaml"
    path.write_text(yaml.safe_dump(ROOM, sort_keys=False))
    result = check(path, body="so101", seeds=20, need=18, snap_dir=tmp_path)
    assert result["ok"], result
    assert result["solver"]["successes"] >= 18


# ---- the question the reach room could not answer -----------------------------------

@pytest.mark.slow
def test_the_arm_picks_an_object_up_and_puts_it_somewhere(so101_asset_dir, repo_root, tmp_path):
    """The weld grab (R18) was written for a beak on the front of a walking trunk, and
    nothing had measured it on a jaw that *is* the body. Measured: 20/20 seeds, mean 2.9 s.

    It was 0/20 until the shared approach learned this body is holonomic: the carrier
    closes in at `SLOW = -0.5`, which on a symmetric command mapping is half speed
    backwards, so the arm drove away from the block it was told to pick up.
    """
    from mujoco_puzzle_gym import catalog
    from mujoco_puzzle_gym.check import check

    result = check(catalog.PRESET_ROOT / "puzzles/tabletop/pick_and_place.yaml",
                   task=repo_root / "examples/task_pick_and_place.yaml",
                   seeds=20, need=18, snap_dir=tmp_path)
    assert result["ok"], result
    assert result["solver"]["successes"] >= 18


def test_the_jaw_is_the_grab_site(scene):
    """And it is the same site `state` reports as the body's position: for this body the
    end effector is the body."""
    body, model, data = scene
    name = body.grab_site()
    assert name and model.site(name).id == body._site
