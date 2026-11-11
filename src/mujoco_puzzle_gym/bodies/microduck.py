"""Microduck walking body (R3, unofficial).

Runs Pollen Robotics' `velstand.onnx` walk policy at 50 Hz on CPU MuJoCo
(timestep 5 ms, 4 physics steps per policy step) with the BAM M6 XL330 motor
model, behind the `BodyAdapter` boundary.

Attribution: the observation layout, default pose, action scaling, BAM actuator
setup and control loop are re-implemented from `scripts/infer_policy.py` in
pollen-robotics/microduck_rl (Apache-2.0) at commit
0daab160d3fb1c2cbaba16a4a19052f36709dd9e, cross-checked against
`src/mjlab_microduck/tasks/microduck_velocity_env_cfg.py` /
`microduck_velstand_env_cfg.py` and `robot/microduck_constants.py` at the same
commit. Robot MJCF (Apache-2.0) and meshes (CC BY-NC-SA 4.0, Pollen Robotics)
are fetched by `mpg fetch-assets`, never vendored.

Where infer_policy.py's defaults differ from how velstand was trained
(task `Mjlab-VelStand-Rough-Backlash-MicroDuck`), this follows training:
- robot `robot_allcollisions_backlash.xml` (infer_policy default: scene.xml with
  the non-backlash groundcontact robot; it accepts other scenes via --scene);
- encoder through backlash: joint obs and the firmware PD read servo + backlash
  (tasks/backlash.py, actuator/friction_dr_bam.py BacklashEncoderBamActuator);
- actuator command delay of ACTION_LAG physics steps (training: 3..6;
  infer_policy: --delay, off by default).
Without the delay and backlash the policy stands still for some commands it
follows otherwise (e.g. wz=-1.0, vx=-0.4). BAM motor parameters come from the
pinned better-actuator-models 1.0.3, not the git build used for training.
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import mujoco
import numpy as np

from .. import assets
from .base import BodyAdapter, BodyState, Pose, rotate_inverse, yaw_from_quat

# Files in the asset cache (keys of upstream.lock.json).
ROBOT_XML = "robot/robot_allcollisions_backlash.xml"
POLICY_ONNX = "policies/velstand.onnx"

# Names in the compiled scene carry PREFIX (MjSpec.attach namespace).
PREFIX = "body/"
TRUNK_BODY = "trunk_base"
FREEJOINT = "trunk_base_freejoint"
GYRO_SENSOR = "imu_ang_vel"

# Actuator order of the MJCF == policy action order (infer_policy.py DEFAULT_POSE).
JOINTS = (
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
)
# STAND2 / HOME_FRAME pose (infer_policy.py DEFAULT_POSE, microduck_constants.HOME_FRAME).
DEFAULT_POSE = np.array([
    0.0, -0.0873, -0.4579, -0.0049, 0.4530,
    0.3491, 0.3491, 0.0, 0.0,
    0.0, 0.0873, 0.4579, 0.0049, -0.4530,
], dtype=np.float32)
ACTION_SCALE = 1.0          # velocity_env_cfg: joint_pos_action.scale = 1.0; infer_policy default
SPAWN_HEIGHT = 0.125        # infer_policy.py initial trunk z (non-roller)
TIMESTEP = 0.005
DECIMATION = 4              # 50 Hz policy
OBS_DIM = 61                # 3 + 3 + 14 + 14 + 14 + 13
CMD_DIM = 13                # twist(3) + head_pose(4) + body_pose(6); only twist is used here

# Actuator command delay in physics steps. Training samples 3..6 per env
# (_BAM_ACTUATOR_KWARGS delay_min_lag / delay_max_lag); infer_policy.py's --delay.
ACTION_LAG = 4

# BAM M6 defaults, as in infer_policy.py (mirrors _BAM_ACTUATOR_KWARGS).
BAM_MOTOR_NAME = "xl330"
BAM_MODEL = "m6"
BAM_KP_FW = 200.0
BAM_VIN = 7.4               # nominal 2S LiPo; training samples (6.5, 8.2)
BAM_VIN_DROP_GAIN = 0.1     # V/Nm, V = vin - gain * sum|tau|; training samples (0.0, 0.2)
BAM_VIN_MIN = 6.0
BAM_STIFF_SOLREF_FRICTION = (-5.0e4, -2.0e2)
BAM_STIFF_SOLIMP_FRICTION = (0.99, 0.9999, 0.001, 0.5, 2.0)

# Fall test: trunk tilt beyond 45 deg or trunk origin below 7 cm (stands at ~12 cm).
FALL_GRAVITY_Z = -np.cos(np.deg2rad(45.0))
FALL_HEIGHT = 0.07


class MicroduckAdapter(BodyAdapter):
    cmd_limits = ((-0.4, 0.4), (-0.3, 0.3), (-1.0, 1.0))   # velstand twist ranges
    control_dt = TIMESTEP * DECIMATION
    timestep = TIMESTEP

    def __init__(self, asset_dir: Path | None = None, vin: float = BAM_VIN,
                 vin_drop_gain: float | None = BAM_VIN_DROP_GAIN,
                 action_lag: int = ACTION_LAG, robot_xml: Path | None = None):
        import onnxruntime as ort
        from bam.model import load_model

        root = Path(asset_dir) if asset_dir is not None else assets.cache_dir()
        self.robot_xml = Path(robot_xml) if robot_xml is not None else root / ROBOT_XML
        self.policy_path = root / POLICY_ONNX
        for p in (self.robot_xml, self.policy_path):
            if not p.exists():
                raise FileNotFoundError(f"{p} missing; run `mpg fetch-assets` first")

        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 1
        opts.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(self.policy_path), opts,
                                            providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

        self.bam_model = load_model(motor_name=BAM_MOTOR_NAME, model=BAM_MODEL)
        self.bam_model.actuator.kp = BAM_KP_FW
        self.bam_model.actuator.vin = vin
        self.bam_model.actuator.max_current = None   # training runs without the limiter
        self.vin = vin
        self.vin_drop_gain = vin_drop_gain
        self.action_lag = action_lag

        self._data: mujoco.MjData | None = None
        self._ctrl = None
        self.last_action = np.zeros(len(JOINTS), dtype=np.float32)
        self._prev_torque = np.zeros(len(JOINTS))

    # ---- scene -------------------------------------------------------------
    def robot_spec(self) -> mujoco.MjSpec:
        """The robot MJCF with BAM's actuator edits (bam.mjlab.BamActuator.edit_spec)."""
        spec = mujoco.MjSpec.from_file(str(self.robot_xml))
        mesh_dir = self.robot_xml.parent / (spec.meshdir or "")
        for mesh in spec.meshes:
            if mesh.file:
                mesh.file = str((mesh_dir / mesh.file).resolve())
        spec.meshdir = ""
        kt = self.bam_model.kt.value
        force_limit = self.bam_model.actuator.vin * kt / self.bam_model.R.value
        joints = {j.name: j for j in spec.joints}
        for act in spec.actuators:
            if act.target.startswith("passive_"):
                continue
            act.set_to_motor()
            act.forcelimited = True
            act.forcerange = (-force_limit, force_limit)
            act.ctrllimited = False
            act.gear = [1.0, 0, 0, 0, 0, 0]
            joint = joints[act.target]
            joint.damping = np.zeros((3, 1))
            joint.frictionloss = 0.0
            joint.solref_friction = BAM_STIFF_SOLREF_FRICTION
            joint.solimp_friction = BAM_STIFF_SOLIMP_FRICTION
        return spec

    def spawn(self, spec: mujoco.MjSpec, pose: Pose = (0.0, 0.0, 0.0)) -> mujoco.MjSpec:
        x, y, yaw = pose
        frame = spec.worldbody.add_frame(
            pos=[x, y, 0.0], quat=[np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])
        spec.attach(self.robot_spec(), frame=frame, prefix=PREFIX)
        spec.option.timestep = TIMESTEP
        return spec

    # ---- runtime -----------------------------------------------------------
    def _bind(self, data: mujoco.MjData) -> None:
        from bam.mujoco import MujocoController

        model = data.model
        self._data = data
        P = PREFIX
        jids = [model.joint(P + j).id for j in JOINTS]
        self._qpos_idx = model.jnt_qposadr[jids]
        self._qvel_idx = model.jnt_dofadr[jids]
        # Actuators in policy order (one per servo joint).
        by_joint = {int(model.actuator_trnid[i, 0]): i for i in range(model.nu)}
        if any(j not in by_joint for j in jids):
            raise RuntimeError("robot MJCF lacks an actuator for a policy joint")
        self._act_names = [model.actuator(by_joint[j]).name for j in jids]
        # Backlash models: an unactuated passive_<joint>_backlash hinge follows each
        # servo; the encoder (obs and firmware PD) reads servo + backlash.
        bl = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{P}passive_{j}_backlash") for j in JOINTS]
        self._has_backlash = all(b >= 0 for b in bl)
        self._bl_qpos = model.jnt_qposadr[bl] if self._has_backlash else None
        self._bl_qvel = model.jnt_dofadr[bl] if self._has_backlash else None
        fj = model.joint(P + FREEJOINT)
        self._root_qpos = int(fj.qposadr[0])
        self._root_qvel = int(fj.dofadr[0])
        self._trunk = model.body(P + TRUNK_BODY).id
        gyro = model.sensor(P + GYRO_SENSOR)
        self._gyro = slice(int(gyro.adr[0]), int(gyro.adr[0]) + 3)
        # vin drop is applied here with the training formula (gain * sum|tau|);
        # the BAM controller's own resistance-based drop stays off.
        self._ctrl = MujocoController(self.bam_model, self._act_names, model, data,
                                      vin_drop_resistance=None, vin_min=None)

    def reset(self, data: mujoco.MjData, pose: Pose = (0.0, 0.0, 0.0)) -> None:
        if self._data is not data:
            self._bind(data)
        x, y, yaw = pose
        q, v = self._root_qpos, self._root_qvel
        data.qpos[q:q + 3] = [x, y, SPAWN_HEIGHT]
        data.qpos[q + 3:q + 7] = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
        data.qvel[v:v + 6] = 0.0
        data.qpos[self._qpos_idx] = DEFAULT_POSE
        data.qvel[self._qvel_idx] = 0.0
        if self._has_backlash:
            data.qpos[self._bl_qpos] = 0.0
            data.qvel[self._bl_qvel] = 0.0
        self._ctrl.reset()
        self._ctrl.q_target[:] = DEFAULT_POSE
        self._prev_torque[:] = 0.0
        self.last_action[:] = 0.0
        self._targets = [DEFAULT_POSE.astype(np.float64)] * (self.action_lag + 1)
        mujoco.mj_forward(data.model, data)

    def encoder_pos(self, data: mujoco.MjData) -> np.ndarray:
        q = data.qpos[self._qpos_idx]
        return q + data.qpos[self._bl_qpos] if self._has_backlash else q

    def encoder_vel(self, data: mujoco.MjData) -> np.ndarray:
        v = data.qvel[self._qvel_idx]
        return v + data.qvel[self._bl_qvel] if self._has_backlash else v

    def observation(self, data: mujoco.MjData, cmd: np.ndarray) -> np.ndarray:
        quat = data.xquat[self._trunk]
        command = np.zeros(CMD_DIM, dtype=np.float32)
        command[:3] = cmd
        return np.concatenate([
            data.sensordata[self._gyro],                                 # base_ang_vel (IMU gyro)
            rotate_inverse(quat, np.array([0.0, 0.0, -1.0])),            # projected_gravity
            self.encoder_pos(data) - DEFAULT_POSE,                       # joint_pos rel. default
            self.encoder_vel(data),                                      # joint_vel
            self.last_action,                                            # last action
            command,                                                     # command (13)
        ]).astype(np.float32)

    def step(self, data: mujoco.MjData, cmd: Sequence[float]) -> None:
        if self._data is not data:
            raise RuntimeError("call reset(data) before step(data)")
        obs = self.observation(data, self.clip_cmd(cmd))
        action = self.session.run([self.output_name], {self.input_name: obs[None, :]})[0][0]
        self.last_action = action.astype(np.float32)
        target = DEFAULT_POSE + self.last_action.astype(np.float64) * ACTION_SCALE
        model, act = data.model, self.bam_model.actuator
        for _ in range(DECIMATION):
            self._targets = self._targets[1:] + [target]
            q_target = self._targets[0]
            if self._has_backlash:
                # BAM's PD error is (q_target - q_servo); closing it on the encoder
                # (q_servo + q_backlash) is the same as shifting the target.
                q_target = q_target - data.qpos[self._bl_qpos]
            self._ctrl.q_target[:] = q_target
            if self.vin_drop_gain:
                act.vin = max(BAM_VIN_MIN, self.vin - self.vin_drop_gain * float(np.abs(self._prev_torque).sum()))
            self._ctrl.update()
            act.vin = self.vin
            self._prev_torque = data.ctrl[self._ctrl.act_indexes].copy()
            mujoco.mj_step(model, data)

    def state(self, data: mujoco.MjData) -> BodyState:
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
            fallen=bool(gravity[2] > FALL_GRAVITY_Z or pos[2] < FALL_HEIGHT),
        )
