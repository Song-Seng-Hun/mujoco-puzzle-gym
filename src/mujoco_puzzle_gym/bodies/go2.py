"""Unitree Go2 walking body (M3, R41-R44, R53).

The second body. Assets are pinned in `upstream.go2.lock.json` (R40): the MJCF and
meshes come from MuJoCo Menagerie and the walking policy from Hugging Face, both
BSD-3-Clause. **The policy is a community checkpoint, not an official Unitree
release** - Unitree publishes training code (unitree_rl_gym, unitree_rl_mjlab), not a
frozen policy. It was trained on the flat-terrain velocity task of unitree_rl_mjlab,
without `gait_phase`, with action scale 0.5.

Everything the policy contract needs is read from the pinned `deploy.yaml` rather than
copied here (R41): the observation terms and their order, the action scale and offset,
the PD gains and the command ranges. This module only carries the *expectation* that
the observation is 45 wide and the action 12, and fails at load time when the pinned
file disagrees.

Joint order (R42): the policy's order is the model's order, `FL, FR, RL, RR` with
hip/thigh/calf in each - no remapping. `deploy.yaml` also holds a `joint_ids_map`;
that is the mapping to the *real robot's* SDK order (its own copy lives under
`deploy/robots/go2/` in the upstream repo, next to the C++ deployment code) and
applying it in simulation would swap left and right. It is deliberately unused here.

Model edits (R53). Menagerie's physics and contact settings are kept as shipped and
only the PD contract is changed. Upstream's MJCF strips the contact settings because
it runs on MuJoCo-Warp; on plain MuJoCo they are what keeps the robot standing -
measured 2026-11-15: removing `cone="elliptic" impratio="100"` and the foot contact
class collapses a turn in place (trunk down to 0.058 m, both directions), while
keeping them turns upright at 0.367 m.

| What                         | Menagerie            | here                       |
|------------------------------|----------------------|----------------------------|
| actuators                    | `motor`, torque      | `position`, kp/kv built in |
| joint damping, frictionloss  | 2.0, 0.2             | 0, 0 (kv is the damping)   |
| calf armature                | 0.01                 | 0.02                       |
| spawn height                 | keyframe 0.27        | 0.32                       |
| cone, impratio, foot contact | elliptic, 100, …     | unchanged                  |

The kp/kv come from `deploy.yaml`; the force limits and armature from upstream's
`go2_constants.py` (`BuiltinPositionActuatorCfg`: hip and thigh effort 23.5, calf 45;
armature 0.01, 0.01, 0.02), the spawn height from its `INIT_STATE.pos`.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

import mujoco
import numpy as np

from .. import assets
from .base import BodyAdapter, BodyState, Capabilities, Pose, SubstepFn, rotate_inverse, yaw_from_quat

LOCK = Path(__file__).resolve().parents[1] / "upstream.go2.lock.json"
ROBOT_XML = "robot/go2.xml"
POLICY_ONNX = "policies/policy.onnx"
DEPLOY_YAML = "policies/deploy.yaml"

PREFIX = "go2/"
TRUNK_BODY = "base"
LEGS = ("FL", "FR", "RL", "RR")          # model order == policy order (R42)
GROUPS = ("hip", "thigh", "calf")
TIMESTEP = 0.002                         # 10 physics steps per 0.02 s policy step
SPAWN_HEIGHT = 0.32                      # go2_constants.py INIT_STATE.pos
FALL_HEIGHT = 0.15                       # trunk stands at ~0.36 m; see `fall_height`
FALL_GRAVITY_Z = -0.5                    # body-frame gravity z above this = tipped over
OBS_WIDTH = 45
ACT_WIDTH = 12
#: Per joint group, from go2_constants.py: (force limit [Nm], armature).
EFFORT_ARMATURE = {"hip": (23.5, 0.01), "thigh": (23.5, 0.01), "calf": (45.0, 0.02)}
#: Observation terms we know how to build, in the order `deploy.yaml` must list them.
OBS_TERMS = ("base_ang_vel", "projected_gravity", "velocity_commands",
             "joint_pos_rel", "joint_vel_rel", "last_action")


def _group(joint_name: str) -> str:
    """`FL_hip_joint` -> `hip` (the name may carry an attach prefix)."""
    return joint_name.rsplit("_", 1)[0].rsplit("_", 1)[-1]


class Go2Adapter(BodyAdapter):
    """Unitree Go2 on the pinned community velocity policy."""

    control_dt = 0.02
    timestep = TIMESTEP
    fall_height = FALL_HEIGHT
    #: Measured on the compiled model at the reset pose: the furthest any geom reaches
    #: horizontally from the trunk centre is 0.406 m (a visual calf mesh) and 0.397 m
    #: counting collision geoms only (the rear thigh's collision box). Rounded up to
    #: cover both, since callers use it to inflate walls for path planning and a value
    #: that is too small plans a route through a gap the body does not fit.
    footprint_radius = 0.41
    #: Stand and turn whenever the heading is off by more than the solver's own "facing"
    #: band (R47). This body turns at 0.5 rad/s and walks at up to 1.0 m/s, so correcting
    #: on the move sweeps 1.0 m at the middle of its command range and 2.0 m at the top -
    #: wider than half a 5 x 4 m room. Measured on `puzzles/wide/switch_door`: correcting
    #: on the move 5/20 seeds (it circled the 0.6 m button, passing within 0.07 m of it
    #: twice a lap, for the whole 40 s episode), standing to turn 20/20.
    stand_to_turn_above = 0.0
    #: Go2 walks and nothing else: no beak (`grab_site`), no addons, no tow hitch, no eye
    #: camera (`go2.xml` has no camera, only an imu site) and no rangefinder mount. All of
    #: those stay at the `BodyAdapter` defaults, which is what `mpg check` gates on (R45).
    #: `max_step` [m] and `max_slope` [deg] are measurements (2026-11-16, this policy at
    #: vx 0.5 on MuJoCo's default floor friction, 10 seeds per point, the seed jittering
    #: the start pose by +/-0.02 m and +/-0.03 rad).
    #:
    #: `max_step` 0.03: up 10/10 at 0.01, 0.02 and 0.03 m, 3/10 at 0.04, 0/10 at 0.05
    #: and 0.08-0.10 (1/10 at 0.06 is sample noise). Nothing ever fell at any height -
    #: past the limit the body simply stops at the riser - so this is a limit on
    #: climbing, not on staying upright. Stepping *down*, it takes 0.09 m with no fall
    #: (10/10 through 0.09, then 6/10 at both 0.10 and 0.12, 0/10 from 0.15); those are
    #: real falls. Only the up limit is declared, as nothing consumes a down limit.
    #:
    #: `max_slope` 6.0: up 10/10 at 4, 5 and 6 deg, 9/10 at 7, 0/10 at 8 and above. Here
    #: too the body never fell, on any ramp up to 20 deg; it stalls. Distance climbed
    #: along the ramp in 10 s falls off gradually (2.78 m at 4 deg, 2.02 at 6, 1.44 at 8,
    #: 0.38 at 10), so this number says "still makes useful progress", not "does not tip
    #: over", and a second criterion (>= 1.5 m climbed) puts the boundary in the same
    #: place. 8-9 deg is where forward progress collapses.
    #:
    #: `max_lift_mass` 0.0 is not a placeholder: without `beak_grab` this body has
    #: nothing to carry with. An undeclared limit would also read as 0
    #: (`bodies.declared_limit`); these are written out so a reader sees they were
    #: measured rather than skipped.
    #:
    #: `walk_speed` 1.0 [m/s]: a steady run at the top command this body is given
    #: (vx 1.0) covers 1.019 m/s on flat ground, measured over the last 3 s of a 6 s
    #: run (2026-11-16). Parts compare an NPC's speed against it (R29); 4.6x the
    #: microduck's declared 0.16.
    capabilities = Capabilities(
        names=frozenset({"walk"}),
        values={"max_lift_mass": 0.0, "max_step": 0.03, "max_slope": 6.0,
                "walk_speed": 1.0},
    )

    def __init__(self, asset_dir: Path | None = None, full_command_range: bool = False):
        import onnxruntime as ort
        import yaml

        root = Path(asset_dir) if asset_dir is not None else assets.cache_dir(assets.load_lock(LOCK))
        self.robot_xml = root / ROBOT_XML
        self.policy_path = root / POLICY_ONNX
        self.deploy_path = root / DEPLOY_YAML
        for p in (self.robot_xml, self.policy_path, self.deploy_path):
            if not p.exists():
                raise FileNotFoundError(
                    f"{p} is missing; run `mpg fetch-assets --body go2` first")

        cfg = yaml.safe_load(self.deploy_path.read_text())
        self._check_contract(cfg)
        self.step_dt = float(cfg["step_dt"])
        self.decimation = int(round(self.step_dt / TIMESTEP))
        self.control_dt = self.step_dt
        self.default_q = np.asarray(cfg["default_joint_pos"], dtype=np.float64)
        act = cfg["actions"]["JointPositionAction"]
        self.action_scale = np.asarray(act["scale"], dtype=np.float64)
        self.action_offset = np.asarray(act["offset"], dtype=np.float64)
        self.stiffness = np.asarray(cfg["stiffness"], dtype=np.float64)
        self.damping = np.asarray(cfg["damping"], dtype=np.float64)
        ranges = cfg["commands"]["base_velocity"]["ranges"]
        self.trained_limits = tuple(
            (float(ranges[k][0]), float(ranges[k][1]))
            for k in ("lin_vel_x", "lin_vel_y", "ang_vel_z"))
        # Puzzle default: the narrower band upstream itself uses outside training
        # (unitree_rl_mjlab `unitree_go2_flat_env_cfg(play=True)`). At 2 m/s a 10 Hz
        # high-level step covers 0.2 m and the body overshoots a room in one step.
        self.cmd_limits = (self.trained_limits if full_command_range
                           else ((-0.5, 1.0), (-0.5, 0.5), (-0.5, 0.5)))

        self.session = ort.InferenceSession(str(self.policy_path),
                                            providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        shape = self.session.get_inputs()[0].shape
        if list(shape)[-1] != OBS_WIDTH:
            raise ValueError(f"{self.policy_path} takes {shape}, expected [..., {OBS_WIDTH}]")

        self.last_action = np.zeros(ACT_WIDTH, dtype=np.float32)
        self._data: mujoco.MjData | None = None

    # ---- contract ----------------------------------------------------------

    def _check_contract(self, cfg: dict[str, Any]) -> None:
        """Fail loudly when the pinned deploy config is not the one this code expects."""
        terms = list(cfg["observations"])
        if tuple(terms) != OBS_TERMS:
            raise ValueError(f"{self.deploy_path}: observation terms {terms} "
                             f"are not the expected {list(OBS_TERMS)}")
        widths = {"base_ang_vel": 3, "projected_gravity": 3, "velocity_commands": 3,
                  "joint_pos_rel": ACT_WIDTH, "joint_vel_rel": ACT_WIDTH,
                  "last_action": ACT_WIDTH}
        total = 0
        for name in terms:
            term = cfg["observations"][name]
            if int(term.get("history_length", 1)) != 1:
                raise ValueError(f"{self.deploy_path}: {name} stacks history, unsupported")
            scale = np.asarray(term["scale"], dtype=np.float64)
            if scale.size != widths[name] or not np.allclose(scale, 1.0):
                raise ValueError(f"{self.deploy_path}: {name} scale {term['scale']} is not "
                                 f"{widths[name]} ones")
            total += widths[name]
        if total != OBS_WIDTH:
            raise ValueError(f"{self.deploy_path}: observations add up to {total}, "
                             f"expected {OBS_WIDTH}")
        if len(cfg["default_joint_pos"]) != ACT_WIDTH:
            raise ValueError(f"{self.deploy_path}: default_joint_pos is not {ACT_WIDTH} long")

    # ---- scene -------------------------------------------------------------

    def robot_spec(self) -> mujoco.MjSpec:
        """The pinned MJCF with the R53 edits applied in memory (never on disk)."""
        spec = mujoco.MjSpec.from_file(str(self.robot_xml))
        for j in spec.joints:
            if not j.name.endswith("_joint"):
                continue
            j.damping = [0.0, 0.0, 0.0]          # a free/ball joint takes three
            j.frictionloss = 0.0
            j.armature = EFFORT_ARMATURE[_group(j.name)][1]
        kp_kv = self._gains_by_group()
        for a in spec.actuators:
            kp, kv = kp_kv[_group(a.target)]
            effort = EFFORT_ARMATURE[_group(a.target)][0]
            a.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            a.gainprm[0] = kp
            a.biasprm[0], a.biasprm[1], a.biasprm[2] = 0.0, -kp, -kv
            a.forcerange = [-effort, effort]
            a.ctrlrange = [-1e6, 1e6]            # ctrl is a target angle, not a torque
        return spec

    def _gains_by_group(self) -> dict[str, tuple[float, float]]:
        """kp/kv per joint group, read off `deploy.yaml`'s per-joint arrays (R41).

        The arrays are in policy order, so group `i % 3` repeats per leg; this checks
        that every leg really carries the same gains instead of assuming it.
        """
        out: dict[str, tuple[float, float]] = {}
        for i, g in enumerate(GROUPS):
            kp = self.stiffness[i::len(GROUPS)]
            kv = self.damping[i::len(GROUPS)]
            if not (np.all(kp == kp[0]) and np.all(kv == kv[0])):
                raise ValueError(f"{self.deploy_path}: {g} gains differ between legs: "
                                 f"{kp.tolist()} / {kv.tolist()}")
            out[g] = (float(kp[0]), float(kv[0]))
        return out

    def spawn(self, spec: mujoco.MjSpec, pose: Pose = (0.0, 0.0, 0.0)) -> mujoco.MjSpec:
        x, y, yaw = pose
        frame = spec.worldbody.add_frame(
            pos=[x, y, 0.0], quat=[np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])
        robot = self.robot_spec()
        # Scene-level options do not come along with `attach`, and Menagerie's are the
        # ones the body needs to stay up in a turn (R53). Set them on the parent *before*
        # attaching: afterwards `attach` has already reported a conflict and kept the
        # parent's defaults, so the user would see a warning for a value we then fix.
        spec.option.timestep = TIMESTEP
        spec.option.cone = robot.option.cone
        spec.option.impratio = robot.option.impratio
        spec.attach(robot, frame=frame, prefix=PREFIX)
        return spec

    # ---- runtime -----------------------------------------------------------

    def _bind(self, data: mujoco.MjData) -> None:
        model = data.model
        self._data = data
        body = model.body(PREFIX + TRUNK_BODY)
        self._root_qpos = int(model.jnt_qposadr[model.body_jntadr[body.id]])
        self._root_qvel = int(model.jnt_dofadr[model.body_jntadr[body.id]])
        names = [f"{PREFIX}{leg}_{g}_joint" for leg in LEGS for g in GROUPS]
        self._qpos_idx = np.array([model.jnt_qposadr[model.joint(n).id] for n in names])
        self._qvel_idx = np.array([model.jnt_dofadr[model.joint(n).id] for n in names])
        self._ctrl_idx = np.array(
            [model.actuator(f"{PREFIX}{leg}_{g}").id for leg in LEGS for g in GROUPS])

    def reset(self, data: mujoco.MjData, pose: Pose = (0.0, 0.0, 0.0)) -> None:
        if self._data is not data:
            self._bind(data)
        x, y, yaw = pose
        q, v = self._root_qpos, self._root_qvel
        data.qpos[q:q + 3] = [x, y, SPAWN_HEIGHT]
        data.qpos[q + 3:q + 7] = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
        data.qvel[v:v + 6] = 0.0
        data.qpos[self._qpos_idx] = self.default_q
        data.qvel[self._qvel_idx] = 0.0
        data.ctrl[self._ctrl_idx] = self.default_q
        self.last_action[:] = 0.0

    def observation(self, data: mujoco.MjData, cmd: Sequence[float]) -> np.ndarray:
        """The 45-wide vector in `deploy.yaml`'s order (R41)."""
        q, v = self._root_qpos, self._root_qvel
        quat = data.qpos[q + 3:q + 7]
        obs = np.concatenate([
            data.qvel[v + 3:v + 6],                       # free joint: already body frame
            rotate_inverse(quat, np.array([0.0, 0.0, -1.0])),
            np.asarray(cmd, dtype=np.float64),
            data.qpos[self._qpos_idx] - self.default_q,
            data.qvel[self._qvel_idx],
            self.last_action.astype(np.float64),
        ])
        return obs.astype(np.float32)

    def step(self, data: mujoco.MjData, cmd: Sequence[float],
             on_substep: SubstepFn | None = None) -> None:
        if self._data is not data:
            raise RuntimeError("call reset(data) before step(data)")
        obs = self.observation(data, self.clip_cmd(cmd))
        action = self.session.run([self.output_name], {self.input_name: obs[None, :]})[0][0]
        self.last_action = action.astype(np.float32)
        data.ctrl[self._ctrl_idx] = (
            self.action_offset + self.action_scale * self.last_action.astype(np.float64))
        model = data.model
        for _ in range(self.decimation):
            mujoco.mj_step(model, data)
            if on_substep is not None:
                on_substep(model, data)

    def state(self, data: mujoco.MjData, floor_z: float = 0.0) -> BodyState:
        q, v = self._root_qpos, self._root_qvel
        quat = data.qpos[q + 3:q + 7]
        yaw = yaw_from_quat(quat)
        c, s = np.cos(yaw), np.sin(yaw)
        vw = data.qvel[v:v + 3]
        gravity = rotate_inverse(quat, np.array([0.0, 0.0, -1.0]))
        pos = data.qpos[q:q + 3].copy()
        return BodyState(
            pos=pos,
            yaw=yaw,
            lin_vel=np.array([c * vw[0] + s * vw[1], -s * vw[0] + c * vw[1], vw[2]]),
            ang_vel=data.qvel[v + 3:v + 6].copy(),
            gravity=gravity,
            fallen=bool(gravity[2] > FALL_GRAVITY_Z or pos[2] - floor_z < FALL_HEIGHT),
        )

    def teleport(self, data: mujoco.MjData, pose: Pose, z: float = 0.0) -> None:
        self.reset(data, pose)
        if z:
            data.qpos[self._root_qpos + 2] += z
            mujoco.mj_forward(data.model, data)

    def root_body_id(self, model: mujoco.MjModel) -> int:
        return model.body(PREFIX + TRUNK_BODY).id

    def command_from_action(self, action: Sequence[float]) -> np.ndarray:
        """Keep the repo convention where the trained range is asymmetric.

        `action[0]` = -1 must be the slowest the body goes, not full reverse: the
        trained `lin_vel_x` starts at -1.0 m/s, so the default linear mapping would turn
        the bottom of the action range into backing up. Forward speed therefore maps onto
        [0, vx_max] and the lateral and turn axes stay linear over their symmetric range.
        """
        a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        (_, vx_hi), (vy_lo, vy_hi), (wz_lo, wz_hi) = self.cmd_limits
        return np.array([
            (a[0] + 1.0) / 2.0 * vx_hi,
            vy_lo + (a[1] + 1.0) / 2.0 * (vy_hi - vy_lo),
            wz_lo + (a[2] + 1.0) / 2.0 * (wz_hi - wz_lo),
        ])

    def history_state(self) -> dict[str, Any]:
        return {"last_action": self.last_action.copy()}

    def load_history(self, data: mujoco.MjData, state: dict[str, Any]) -> None:
        if self._data is not data:
            self._bind(data)
        self.last_action = np.asarray(state["last_action"], dtype=np.float32).copy()
