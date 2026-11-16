"""M4: a humanoid driven by a frozen velocity-commanded policy, on the 29-joint model.

What is worth holding up here is the part that was not obvious. The policy is a legs-only
one — upstream's observation is `obs[9:21] = d.qpos[7:]` with twelve actions, which only
types if their robot has exactly twelve joints — so this body feeds it the twelve leg
joints of a 29-joint model and holds the waist and both arms still. The policy never saw
arms. Whether it walks anyway is therefore a measurement, and these tests are where it is
kept honest.
"""

import warnings

import mujoco
import numpy as np
import pytest

from mujoco_puzzle_gym.bodies import BODIES, flat_floor_spec, make_body
from mujoco_puzzle_gym.bodies.g1 import ACT_WIDTH, GAIT_PERIOD, LEGS, OBS_WIDTH, G1Adapter

WIDE = "puzzles/wide/switch_door.yaml"


@pytest.fixture(scope="module")
def scene(g1_asset_dir):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        body = make_body("g1")
        model = body.spawn(flat_floor_spec(), (0.0, 0.0, 0.0)).compile()
    data = mujoco.MjData(model)
    body.reset(data)
    return body, model, data


def test_the_humanoid_is_a_registered_body(g1_asset_dir):
    assert "g1" in BODIES
    assert isinstance(make_body("g1"), G1Adapter)


def test_the_policy_contract_is_what_the_pinned_config_says(scene):
    """Shapes, not faith: the observation this adapter assembles and the action the
    checkpoint returns both have to match the config's `num_obs` and `num_actions`, or the
    adapter raises at load (`_check_contract`)."""
    body, _, data = scene
    obs = body.observation(data, (0.3, 0.0, 0.0))
    assert obs.shape == (OBS_WIDTH,)
    import torch

    with torch.no_grad():
        action = body.policy(torch.from_numpy(obs).unsqueeze(0))
    assert tuple(action.shape) == (1, ACT_WIDTH)


def test_it_drives_twelve_joints_of_twenty_nine(scene):
    body, model, data = scene
    assert len(LEGS) == ACT_WIDTH
    assert model.nu == 29 + 0                      # the whole body is actuated
    assert len(body._leg_ctrl) == ACT_WIDTH
    assert len(body._top_ctrl) == 29 - ACT_WIDTH   # waist and both arms, held still


def test_it_only_writes_its_own_actuators(g1_asset_dir, repo_root):
    """The bug this test exists for: `_top_ctrl` was "every actuator except the legs",
    which across a whole scene is also the room's. This body held a door's actuator at its
    own standing pose every step, so a door wired to a pressed button stayed shut and the
    solver stood in front of it for the rest of the episode."""
    from mujoco_puzzle_gym import catalog
    from mujoco_puzzle_gym.env import PuzzleEnv

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        env = PuzzleEnv(task={"room": str(catalog.PRESET_ROOT / WIDE), "body": "g1"})
    try:
        body = env.body
        own = {mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_ACTUATOR, int(i))
               for i in [*body._leg_ctrl, *body._top_ctrl]}
        assert own and all(n.startswith("g1/") for n in own), sorted(own)
        assert len(own) < env.model.nu              # the room has actuators of its own
    finally:
        env.close()


@pytest.mark.parametrize("action,forward", [(-1.0, 0.0), (0.0, 0.3), (1.0, 0.6)])
def test_the_slowest_action_is_a_stop_not_a_reverse(action, forward, scene):
    """The repository convention. With the default linear mapping onto an asymmetric range
    it was full reverse, and the solver's "wall ahead" value then drove the humanoid
    backwards in a straight line for thirteen seconds until it fell off the room."""
    body, _, _ = scene
    assert body.command_from_action([action, 0.0, 0.0])[0] == pytest.approx(forward)


def test_the_gait_clock_runs_on_simulated_time(scene):
    """The policy's last two observations are a 0.8 s phase, which the quadruped's policy
    was trained without. The clock has to advance with the physics, not with calls."""
    body, _, data = scene
    body.reset(data)
    assert body.clock == 0.0
    for _ in range(10):
        body.step(data, (0.0, 0.0, 0.0))
    assert body.clock == pytest.approx(10 * body.control_dt, abs=1e-9)
    phase = (body.clock % GAIT_PERIOD) / GAIT_PERIOD
    obs = body.observation(data, (0.0, 0.0, 0.0))
    assert obs[45] == pytest.approx(np.sin(2 * np.pi * phase), abs=1e-6)
    assert obs[46] == pytest.approx(np.cos(2 * np.pi * phase), abs=1e-6)


@pytest.mark.slow
@pytest.mark.parametrize("cmd,least", [((0.3, 0.0, 0.0), 1.0), ((0.6, 0.0, 0.0), 2.0),
                                       ((0.0, 0.3, 0.0), 0.9)])
def test_it_walks_with_arms_the_policy_never_saw(cmd, least, scene):
    """Measured over 6 s: 0.223 m/s commanded 0.3, 0.455 m/s at the ceiling 0.6, and
    0.211 m/s sideways at 0.3 - and it stays up through all of them."""
    body, _, data = scene
    body.reset(data)
    start = body.state(data).pos.copy()
    for _ in range(int(6.0 / body.control_dt)):
        body.step(data, cmd)
    state = body.state(data)
    assert np.linalg.norm((state.pos - start)[:2]) > least
    assert not state.fallen


@pytest.mark.slow
def test_it_turns_on_the_spot(scene):
    """0.44 rad/s at wz 0.5 while moving less than 0.15 m, which is why its arc at the
    middle of the command range is 0.34 m and the shared steering needs no standing turn
    for it (`stand_to_turn_above` stays infinite, unlike the quadruped's)."""
    body, _, data = scene
    body.reset(data)
    start = body.state(data)
    for _ in range(int(6.0 / body.control_dt)):
        body.step(data, (0.0, 0.0, 0.5))
    end = body.state(data)
    assert abs(end.yaw - start.yaw) > 1.5
    assert np.linalg.norm((end.pos - start.pos)[:2]) < 0.3
    assert not end.fallen


@pytest.mark.slow
def test_a_push_it_cannot_take_reads_as_fallen(scene):
    """`fall_height` 0.45 m is measured, not chosen: a 1.0 m/s side kick is survived with
    the pelvis dipping to 0.72 m, and 2.0 m/s puts it down at 0.06-0.12 m."""
    body, _, data = scene
    for kick, down in ((1.0, False), (2.0, True)):
        body.reset(data)
        for _ in range(25):
            body.step(data, (0.3, 0.0, 0.0))
        body.kick(data, (0.0, kick, 0.0))
        for _ in range(int(4.0 / body.control_dt)):
            body.step(data, (0.3, 0.0, 0.0))
        assert body.state(data).fallen is down, (kick, body.state(data).pos[2])


@pytest.mark.slow
@pytest.mark.parametrize("room", ["switch_door", "pit_detour"])
def test_it_solves_the_large_body_rooms_unchanged(room, g1_asset_dir, asset_dir, tmp_path):
    """The finding worth keeping: the humanoid needs no room of its own and no longer
    episode. Both rooms sized for the quadruped (0.41 m footprint, this body 0.32 m) go
    20/20 with the default 40 s, mean 18.3 s and 19.4 s, no falls."""
    from mujoco_puzzle_gym import catalog
    from mujoco_puzzle_gym.check import check

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        result = check(catalog.PRESET_ROOT / f"puzzles/wide/{room}.yaml", body="g1",
                       seeds=20, need=18, snap_dir=tmp_path)
    assert result["ok"], result
    assert result["solver"]["successes"] >= 18
