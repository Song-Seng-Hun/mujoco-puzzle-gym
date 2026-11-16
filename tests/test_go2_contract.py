"""R41, R42, R44, R53: the Go2 adapter's contract with the pinned policy and model.

These tests read the pinned `deploy.yaml` and the compiled model and compare them with
what `bodies/go2.py` assumes. They are the reason a change in either one cannot drift
silently; the behavioural checks (does it walk, are left and right the right way round)
live in `test_go2_walk.py`.
"""

import json

import mujoco
import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym.bodies import BODIES, flat_floor_spec, make_body
from mujoco_puzzle_gym.bodies.go2 import GROUPS, LEGS, PREFIX, Go2Adapter

GAINS = {"hip": (20.0, 1.0, 23.5, 0.01),        # kp, kv, force limit, armature
         "thigh": (20.0, 1.0, 23.5, 0.01),
         "calf": (40.0, 2.0, 45.0, 0.02)}
JOINT_NAMES = [f"{leg}_{g}_joint" for leg in LEGS for g in GROUPS]


@pytest.fixture(scope="module")
def body(go2_asset_dir):
    return make_body("go2")


@pytest.fixture(scope="module")
def model(body):
    return body.spawn(flat_floor_spec()).compile()


@pytest.fixture(scope="module")
def deploy(body):
    return yaml.safe_load(body.deploy_path.read_text())


# ---- R41: the policy contract ---------------------------------------------

def test_make_body_knows_go2(go2_asset_dir):
    assert "go2" in BODIES
    assert isinstance(make_body("go2"), Go2Adapter)
    with pytest.raises(ValueError, match="unknown body 'spot'"):
        make_body("spot")


def test_onnx_signature(body):
    inp, out = body.session.get_inputs()[0], body.session.get_outputs()[0]
    assert list(inp.shape)[-1] == 45, inp.shape
    assert list(out.shape)[-1] == 12, out.shape


def test_observation_terms_add_up_to_45(body, deploy):
    widths = {"base_ang_vel": 3, "projected_gravity": 3, "velocity_commands": 3,
              "joint_pos_rel": 12, "joint_vel_rel": 12, "last_action": 12}
    assert list(deploy["observations"]) == list(widths)
    assert sum(widths.values()) == 45
    for name, term in deploy["observations"].items():
        assert int(term.get("history_length", 1)) == 1, name
        assert np.allclose(np.asarray(term["scale"], float), 1.0), name


def test_adapter_reads_the_numbers_from_deploy_yaml(body, deploy):
    """R41: nothing in the table is written twice; the adapter mirrors the pinned file."""
    act = deploy["actions"]["JointPositionAction"]
    assert body.step_dt == float(deploy["step_dt"]) == 0.02
    assert body.decimation == 10 and body.control_dt == 0.02
    assert np.allclose(body.action_scale, act["scale"]) and np.allclose(body.action_scale, 0.5)
    assert np.allclose(body.action_offset, act["offset"])
    assert np.allclose(body.default_q, deploy["default_joint_pos"])
    assert np.allclose(body.action_offset, body.default_q)
    assert np.allclose(body.stiffness, deploy["stiffness"])
    assert np.allclose(body.damping, deploy["damping"])


def test_observation_vector_is_45_and_ordered(body, model):
    data = mujoco.MjData(model)
    body.reset(data)
    obs = body.observation(data, (0.3, -0.2, 0.1))
    assert obs.shape == (45,) and obs.dtype == np.float32
    # velocity_commands sits third, after two 3-wide terms.
    assert np.allclose(obs[6:9], [0.3, -0.2, 0.1])
    # at the reset pose joint_pos_rel is zero and last_action is zero
    assert np.allclose(obs[9:21], 0.0, atol=1e-6)
    assert np.allclose(obs[33:45], 0.0)
    # projected gravity points down in the body frame when standing level
    assert obs[5] == pytest.approx(-1.0, abs=1e-3)


def test_a_tampered_deploy_yaml_fails_at_load(body, deploy, tmp_path, monkeypatch):
    """The adapter must refuse a pinned config it was not written against (R41)."""
    bad = dict(deploy)
    bad["observations"] = dict(deploy["observations"])
    bad["observations"]["joint_pos_rel"] = dict(deploy["observations"]["joint_pos_rel"],
                                                scale=[1.0] * 13)
    root = tmp_path / "cache"
    for rel in ("robot/go2.xml", "policies/policy.onnx", "policies/policy.onnx.data"):
        dst = root / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes((body.robot_xml.parent.parent / rel).read_bytes())
    (root / "policies/deploy.yaml").write_text(yaml.safe_dump(bad))
    with pytest.raises(ValueError, match="joint_pos_rel"):
        Go2Adapter(asset_dir=root)


# ---- R42: joint order, and the map we must not apply -----------------------

def test_model_joint_order_is_fl_fr_rl_rr(model):
    names = [model.joint(i).name for i in range(model.njnt)]
    legs = [n for n in names if n.endswith("_joint")]
    assert legs == [PREFIX + n for n in JOINT_NAMES]


def test_default_pose_hip_signs_match_the_model_order(body, deploy):
    """Upstream's INIT_STATE is R_hip +0.1 / L_hip -0.1, so the policy's per-leg hip
    signs are L, R, L, R. That is what pins the policy order to the model order."""
    hips = np.asarray(deploy["default_joint_pos"], float)[0::3]
    expected = [-0.1 if leg[1] == "L" else 0.1 for leg in LEGS]
    assert np.allclose(hips, expected), (hips, expected, LEGS)


def test_joint_ids_map_is_present_but_unused(body, deploy, model):
    """R42: `joint_ids_map` maps to the real robot's SDK order, not to MuJoCo.

    It is not the identity, so if the adapter applied it the joint indices would come
    out permuted. Asserting the indices are in plain model order proves it does not.
    """
    jmap = deploy["joint_ids_map"]
    assert jmap != list(range(12)), "map is the identity; this test no longer proves anything"
    names = [f"{PREFIX}{n}" for n in JOINT_NAMES]
    plain = np.array([model.jnt_qposadr[model.joint(n).id] for n in names])
    assert np.array_equal(body._qpos_idx, plain)
    assert not hasattr(body, "joint_ids_map")


# ---- R44: command range and action mapping --------------------------------

def test_trained_range_comes_from_the_pinned_config(body, deploy):
    r = deploy["commands"]["base_velocity"]["ranges"]
    assert body.trained_limits == ((r["lin_vel_x"][0], r["lin_vel_x"][1]),
                                   (r["lin_vel_y"][0], r["lin_vel_y"][1]),
                                   (r["ang_vel_z"][0], r["ang_vel_z"][1]))
    assert body.trained_limits == ((-1.0, 2.0), (-1.0, 1.0), (-1.0, 1.0))


def test_puzzle_range_is_narrower_and_inside_the_trained_one(body):
    assert body.cmd_limits == ((-0.5, 1.0), (-0.5, 0.5), (-0.5, 0.5))
    for (lo, hi), (tlo, thi) in zip(body.cmd_limits, body.trained_limits):
        assert tlo <= lo < hi <= thi


def test_full_command_range_opens_the_trained_band(go2_asset_dir):
    full = make_body("go2", full_command_range=True)
    assert full.cmd_limits == full.trained_limits


def test_action_minus_one_is_stop_not_reverse(body):
    """The trained lin_vel_x starts below zero, so the default linear mapping would make
    the bottom of the action range back up. This body overrides it (R44)."""
    assert np.allclose(body.command_from_action([-1, 0, 0]), [0.0, 0.0, 0.0])
    assert np.allclose(body.command_from_action([1, 1, 1]), [1.0, 0.5, 0.5])
    assert np.allclose(body.command_from_action([0, 0, 0]), [0.5, 0.0, 0.0])
    assert np.allclose(body.command_from_action([0, -1, -1]), [0.5, -0.5, -0.5])


def test_clip_cmd_uses_the_puzzle_band(body):
    assert np.allclose(body.clip_cmd([5.0, 5.0, 5.0]), [1.0, 0.5, 0.5])
    assert np.allclose(body.clip_cmd([-5.0, -5.0, -5.0]), [-0.5, -0.5, -0.5])


# ---- R53: the in-memory model edits ---------------------------------------

def test_actuators_are_position_with_the_pinned_gains(model):
    for leg in LEGS:
        for g in GROUPS:
            a = model.actuator(f"{PREFIX}{leg}_{g}")
            kp, kv, force, _ = GAINS[g]
            assert a.gaintype == mujoco.mjtGain.mjGAIN_FIXED, (leg, g)
            assert a.biastype == mujoco.mjtBias.mjBIAS_AFFINE, (leg, g)
            assert a.gainprm[0] == pytest.approx(kp), (leg, g)
            assert a.biasprm[1] == pytest.approx(-kp), (leg, g)
            assert a.biasprm[2] == pytest.approx(-kv), (leg, g)
            assert np.allclose(a.forcerange, [-force, force]), (leg, g)


def test_joint_damping_and_frictionloss_are_zero_and_calf_armature_is_doubled(model):
    for leg in LEGS:
        for g in GROUPS:
            j = model.joint(f"{PREFIX}{leg}_{g}_joint")
            dof = int(model.jnt_dofadr[j.id])
            assert model.dof_damping[dof] == 0.0, (leg, g)
            assert model.dof_frictionloss[dof] == 0.0, (leg, g)
            assert model.dof_armature[dof] == pytest.approx(GAINS[g][3]), (leg, g)


def test_menagerie_contact_settings_are_kept(model):
    """Stripping these the way upstream's MJCF does collapses a turn in place (R53)."""
    assert model.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC
    assert model.opt.impratio == pytest.approx(100.0)
    assert model.opt.timestep == pytest.approx(0.002)
    for leg in LEGS:
        g = model.geom(f"{PREFIX}{leg}")
        assert g.condim == 6, leg
        assert np.allclose(g.friction, [0.8, 0.02, 0.01]), leg
        assert np.allclose(g.solimp[:3], [0.015, 1.0, 0.022]), leg


def test_reset_puts_the_trunk_at_the_training_spawn_height(body, model):
    data = mujoco.MjData(model)
    body.reset(data, (1.0, -2.0, 0.5))
    st = body.state(data)
    assert st.pos[0] == pytest.approx(1.0) and st.pos[1] == pytest.approx(-2.0)
    assert st.yaw == pytest.approx(0.5)
    assert st.pos[2] == pytest.approx(0.32)
    assert not st.fallen
    assert np.allclose(data.qpos[body._qpos_idx], body.default_q)
    assert np.allclose(data.ctrl[body._ctrl_idx], body.default_q)


# ---- R45 groundwork: what this body cannot do -----------------------------

def test_go2_declares_only_walking_and_no_optional_hardware(body, model):
    assert sorted(body.capabilities.names) == ["walk"]
    assert body.grab_site() is None
    assert body.tow_site() is None
    assert body.view_camera(model) is None
    assert body.addons == frozenset()
    with pytest.raises(NotImplementedError):
        body.add_rangefinders(flat_floor_spec(), [0.0], "r")


def test_capability_values_are_the_measured_ones(body):
    """Measured 2026-11-16 (see the comment on `Go2Adapter.capabilities` for the ratios).

    Pinned here so a change to any of them has to be a deliberate edit to this test as
    well; `mpg check` plans and refuses rooms from these numbers.
    """
    values = body.capabilities.values
    assert values["max_step"] == pytest.approx(0.03)
    assert values["max_slope"] == pytest.approx(6.0)
    assert values["max_lift_mass"] == 0.0          # no beak, nothing to carry with
    for key in ("max_step", "max_slope", "max_lift_mass"):
        assert np.isfinite(values[key]), key


def test_footprint_radius_covers_the_whole_body(body, model):
    """Too small a radius plans a route through a gap the body does not fit (R47)."""
    data = mujoco.MjData(model)
    body.reset(data)
    mujoco.mj_forward(model, data)
    root = body.root_body_id(model)
    cx, cy = float(data.qpos[body._root_qpos]), float(data.qpos[body._root_qpos + 1])
    reach = 0.0
    for g in range(model.ngeom):
        if model.body_rootid[model.geom_bodyid[g]] != model.body_rootid[root]:
            continue
        p = data.geom_xpos[g]
        # `geom_rbound` is the radius of a sphere around the geom, so this is an upper
        # bound on its horizontal reach: good enough to catch a radius that is too small.
        reach = max(reach, float(np.hypot(p[0] - cx, p[1] - cy)) + float(model.geom_rbound[g]))
    assert body.footprint_radius == pytest.approx(0.41)
    assert body.footprint_radius <= reach, "declared radius must not exceed the bound"
    assert reach - body.footprint_radius < 0.25, f"declared radius far below {reach:.3f}"


def test_history_state_round_trips(body, model):
    data = mujoco.MjData(model)
    body.reset(data)
    body.step(data, (0.3, 0.0, 0.0))
    saved = json.loads(json.dumps({"last_action": body.last_action.tolist()}))
    before = body.last_action.copy()
    body.step(data, (0.3, 0.0, 0.0))
    assert not np.allclose(body.last_action, before)
    body.load_history(data, {"last_action": np.asarray(saved["last_action"], np.float32)})
    assert np.allclose(body.last_action, before)
