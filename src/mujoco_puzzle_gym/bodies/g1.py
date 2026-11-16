"""A humanoid as a `BodyAdapter` (M4): legs driven by a frozen velocity-commanded policy.

Same shape as the quadruped: a pinned MJCF, a pinned policy that takes `(vx, vy, wz)`, and
a planar command in, `BodyState` out. Three things differ and each one is in the code
below rather than in a comment hoping to be read:

- **The policy drives 12 joints, the model has 29.** Upstream's observation is
  `obs[9:21] = d.qpos[7:]` with `num_actions = 12`, which only types if their robot has
  exactly twelve joints - it is a legs-only model. This body feeds the policy the twelve
  leg joints of the 29-joint model and holds the waist and both arms at the model's own
  standing pose. That is a departure from what the policy was trained on (it never saw
  arms), so whether it walks is a measurement, not an assumption: see
  `docs/bodies.md` section 10.
- **The observation carries a gait clock.** 47 terms, of which the last two are the sine
  and cosine of a 0.8 s phase. The quadruped's policy was trained without one; this
  adapter runs the clock off simulated time, as upstream does.
- **The checkpoint is TorchScript, not ONNX.** `torch.jit.load`, not onnxruntime.

Every number comes from the pinned `policies/g1.yaml`, read at load time; none of them is
written twice. The observation *order* is upstream's deployment script at the pinned
commit (`deploy/deploy_mujoco/deploy_mujoco.py`), transcribed in `observation` with the
slice arithmetic kept visible so it can be checked against that file.

The policy is **not an official Unitree frozen release**: it is the checkpoint that
repository ships with its deployment example, BSD-3-Clause (R51).
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Sequence

import mujoco
import numpy as np
import yaml

from .base import BodyAdapter, BodyState, Capabilities, Pose, SubstepFn

LOCK = Path(__file__).resolve().parents[1] / "upstream.g1.lock.json"
PREFIX = "g1/"
ROBOT_XML = "humanoid/g1.xml"
POLICY = "policies/motion.pt"
CONFIG = "policies/g1.yaml"
PELVIS = "pelvis"
#: The twelve joints the policy drives, in the order its `default_angles` are written.
LEGS = tuple(f"{side}_{part}_joint" for side in ("left", "right")
             for part in ("hip_pitch", "hip_roll", "hip_yaw", "knee",
                          "ankle_pitch", "ankle_roll"))
#: Gait clock the policy was trained with [s] (upstream's `period`).
GAIT_PERIOD = 0.8
OBS_WIDTH = 47
ACT_WIDTH = 12
#: Pelvis height [m] at the model's standing keyframe; the spawn uses it with the policy's
#: own default leg angles, which bend the knees a little more than the keyframe's zeros.
STAND_Z = 0.79
#: Pelvis height below which this body counts as fallen [m]. Measured by knocking it over
#: sideways while walking: a 1.0 m/s kick is survived, with the pelvis dipping to 0.722 m
#: and recovering to 0.766 m, while 2.0 m/s puts it down at 0.058-0.118 m with body-frame
#: gravity turned to +0.18 (tipped). 0.45 separates the two with room on both sides.
FALL_HEIGHT = 0.45


def _gravity_in_body(quat: np.ndarray) -> np.ndarray:
    """Upstream's `get_gravity_orientation`: world -z expressed in the body frame."""
    w, x, y, z = (float(v) for v in quat)
    return np.array([2 * (-z * x + w * y),
                     -2 * (z * y + w * x),
                     1 - 2 * (w * w + z * z)])


class G1Adapter(BodyAdapter):
    """Unitree G1, legs driven by the pinned TorchScript walking policy."""

    control_dt = 0.02
    timestep = 0.002
    fall_height = FALL_HEIGHT
    #: Measured at the standing pose: the furthest any geom of this body reaches
    #: horizontally from the pelvis centre, its own size included, is 0.316 m. Rounded up,
    #: as the quadruped's is, because callers inflate walls by it and a value too small
    #: plans a route through a gap the body does not fit.
    footprint_radius = 0.32
    #: Puzzle range, deliberately narrower than anything the config documents: the config
    #: records only `cmd_init` 0.5 m/s and no trained range, so the honest move is to stay
    #: near what it was initialised with and widen it when a measurement supports it.
    cmd_limits = ((-0.3, 0.6), (-0.3, 0.3), (-0.5, 0.5))
    #: It walks and nothing else here yet. Two hands and a reach height are what this body
    #: is *for* (M4 R60), but declaring a capability no part can use yet would be
    #: decoration; `max_step` and `max_slope` stay undeclared, which refuses rather than
    #: permits (R2b).
    capabilities = Capabilities(
        names=frozenset({"walk"}),
        values={"max_lift_mass": 0.0, "max_step": 0.0, "max_slope": 0.0,
                # Measured at the command ceiling (vx 0.6 for 6 s): 0.455 m/s, so the
                # command is tracked at about three quarters. Turning in place at wz 0.5
                # measured 0.44 rad/s, which puts the arc at the middle of the command
                # range near 0.34 m - small enough that the shared steering drives this
                # body as it does the first one, and `stand_to_turn_above` stays infinite.
                "walk_speed": 0.45},
    )

    def __init__(self, asset_dir: Path | None = None):
        from .. import assets

        base = Path(asset_dir) if asset_dir else Path(assets.fetch(LOCK)["cache_dir"])
        self.robot_xml = base / ROBOT_XML
        self.policy_path = base / POLICY
        cfg = yaml.safe_load((base / CONFIG).read_text())
        self._check_contract(cfg)
        self.kps = np.asarray(cfg["kps"], dtype=float)
        self.kds = np.asarray(cfg["kds"], dtype=float)
        self.default_q = np.asarray(cfg["default_angles"], dtype=float)
        self.ang_vel_scale = float(cfg["ang_vel_scale"])
        self.dof_pos_scale = float(cfg["dof_pos_scale"])
        self.dof_vel_scale = float(cfg["dof_vel_scale"])
        self.action_scale = float(cfg["action_scale"])
        self.cmd_scale = np.asarray(cfg["cmd_scale"], dtype=float)
        self._policy = None
        self._data: mujoco.MjData | None = None
        self.last_action = np.zeros(ACT_WIDTH, dtype=np.float32)
        self.clock = 0.0                       # simulated seconds, for the gait phase

    def _check_contract(self, cfg: dict[str, Any]) -> None:
        """Fail at load if the pinned config stopped matching what this code assembles."""
        want = {"num_obs": OBS_WIDTH, "num_actions": ACT_WIDTH}
        for key, value in want.items():
            if int(cfg[key]) != value:
                raise ValueError(f"{CONFIG}: {key} is {cfg[key]}, this adapter builds {value}")
        for key in ("kps", "kds", "default_angles"):
            if len(cfg[key]) != ACT_WIDTH:
                raise ValueError(f"{CONFIG}: {key} has {len(cfg[key])} entries, expected {ACT_WIDTH}")
        step = float(cfg["simulation_dt"]) * int(cfg["control_decimation"])
        if abs(step - self.control_dt) > 1e-9:
            raise ValueError(f"{CONFIG}: simulation_dt x control_decimation is {step}, "
                             f"this adapter steps {self.control_dt}")

    @property
    def policy(self):
        if self._policy is None:
            import torch

            self._policy = torch.jit.load(str(self.policy_path))
            self._policy.eval()
        return self._policy

    # ---- scene -------------------------------------------------------------

    def robot_spec(self) -> mujoco.MjSpec:
        """The pinned MJCF with the policy's own gains on the leg actuators.

        A position servo with gain kp and damping kd applies `kp*(target - q) - kd*dq`,
        which is the control law upstream writes by hand as a torque, so putting the
        config's numbers on the model's actuators is the same controller rather than an
        approximation of it. The seventeen upper-body actuators keep the model's gains:
        the policy does not drive them and they only have to hold still.
        """
        spec = mujoco.MjSpec.from_file(str(self.robot_xml))
        gains = dict(zip(LEGS, zip(self.kps, self.kds)))
        for a in spec.actuators:
            if a.target not in gains:
                continue
            kp, kv = gains[a.target]
            a.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            a.gainprm[0] = kp
            a.biasprm[0], a.biasprm[1], a.biasprm[2] = 0.0, -kp, -kv
            a.ctrlrange = [-1e6, 1e6]            # ctrl is a target angle, not a torque
        return spec

    def spawn(self, spec: mujoco.MjSpec, pose: Pose = (0.0, 0.0, 0.0)) -> mujoco.MjSpec:
        x, y, yaw = pose
        frame = spec.worldbody.add_frame(
            pos=[x, y, 0.0], quat=[np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])
        robot = self.robot_spec()
        # Scene options do not come along with `attach`, and this model ships its own:
        # an implicit integrator and the contact settings its walking was tuned against.
        # Set them on the parent before attaching, or `attach` warns, keeps the parent's
        # defaults, and every scene with this body prints a conflict.
        spec.option.timestep = self.timestep
        spec.option.integrator = robot.option.integrator
        spec.option.cone = robot.option.cone
        spec.option.impratio = robot.option.impratio
        spec.attach(robot, frame=frame, prefix=PREFIX)
        return spec

    # ---- runtime -----------------------------------------------------------

    def _bind(self, data: mujoco.MjData) -> None:
        model = data.model
        self._data = data
        pelvis = model.body(PREFIX + PELVIS)
        self._root_qpos = int(model.jnt_qposadr[model.body_jntadr[pelvis.id]])
        self._root_qvel = int(model.jnt_dofadr[model.body_jntadr[pelvis.id]])
        self._leg_qpos = np.array([model.jnt_qposadr[model.joint(PREFIX + n).id] for n in LEGS])
        self._leg_qvel = np.array([model.jnt_dofadr[model.joint(PREFIX + n).id] for n in LEGS])
        self._leg_ctrl = np.array([model.actuator(PREFIX + n).id for n in LEGS])
        # The upper body is every actuator **of this body** that the policy does not
        # drive, held at the model's own standing pose so the arms neither swing nor fall.
        # The prefix test is the whole point: written as "every actuator except the legs"
        # it reached across the entire scene and overwrote the room's own actuators every
        # step - a door wired to a pressed button stayed shut for the whole episode,
        # because this body was writing its standing pose into the door's control signal.
        legs = set(self._leg_ctrl.tolist())
        self._top_ctrl = np.array([i for i in range(model.nu)
                                   if i not in legs
                                   and mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                                         i).startswith(PREFIX)])
        self._top_qpos = np.array([model.jnt_qposadr[model.actuator_trnid[i, 0]]
                                   for i in self._top_ctrl])
        key = model.key(PREFIX + "stand") if model.nkey else None
        self._top_hold = (np.array(key.qpos)[self._top_qpos] if key is not None
                          else np.zeros(len(self._top_ctrl)))

    def reset(self, data: mujoco.MjData, pose: Pose = (0.0, 0.0, 0.0)) -> None:
        if self._data is not data:
            self._bind(data)
        x, y, yaw = pose
        q, v = self._root_qpos, self._root_qvel
        data.qpos[q:q + 3] = [x, y, STAND_Z]
        data.qpos[q + 3:q + 7] = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
        data.qvel[v:v + 6] = 0.0
        data.qpos[self._leg_qpos] = self.default_q
        data.qvel[self._leg_qvel] = 0.0
        data.ctrl[self._leg_ctrl] = self.default_q
        data.qpos[self._top_qpos] = self._top_hold
        data.ctrl[self._top_ctrl] = self._top_hold
        self.last_action[:] = 0.0
        self.clock = 0.0
        mujoco.mj_forward(data.model, data)

    def observation(self, data: mujoco.MjData, cmd: Sequence[float]) -> np.ndarray:
        """The 47-wide vector in upstream's order (`deploy_mujoco.py` at the pinned commit):
        angular velocity, projected gravity, the scaled command, leg positions relative to
        the default pose, leg velocities, the previous action, and the gait phase."""
        q, v = self._root_qpos, self._root_qvel
        obs = np.zeros(OBS_WIDTH, dtype=np.float32)
        obs[0:3] = data.qvel[v + 3:v + 6] * self.ang_vel_scale
        obs[3:6] = _gravity_in_body(data.qpos[q + 3:q + 7])
        obs[6:9] = np.asarray(cmd, dtype=float) * self.cmd_scale
        obs[9:21] = (data.qpos[self._leg_qpos] - self.default_q) * self.dof_pos_scale
        obs[21:33] = data.qvel[self._leg_qvel] * self.dof_vel_scale
        obs[33:45] = self.last_action
        phase = (self.clock % GAIT_PERIOD) / GAIT_PERIOD
        obs[45:47] = [math.sin(2 * math.pi * phase), math.cos(2 * math.pi * phase)]
        return obs

    def step(self, data: mujoco.MjData, cmd: Sequence[float],
             on_substep: SubstepFn | None = None) -> None:
        if self._data is not data:
            raise RuntimeError("call reset(data) before step(data)")
        import torch

        model = data.model
        command = self.clip_cmd(cmd)
        with torch.no_grad():
            obs = torch.from_numpy(self.observation(data, command)).unsqueeze(0)
            action = self.policy(obs).detach().numpy().squeeze()
        self.last_action = np.asarray(action, dtype=np.float32)
        data.ctrl[self._leg_ctrl] = self.last_action * self.action_scale + self.default_q
        data.ctrl[self._top_ctrl] = self._top_hold
        for _ in range(max(1, round(self.control_dt / model.opt.timestep))):
            mujoco.mj_step(model, data)
            self.clock += model.opt.timestep
            if on_substep is not None:
                on_substep(model, data)

    def state(self, data: mujoco.MjData, floor_z: float = 0.0) -> BodyState:
        q, v = self._root_qpos, self._root_qvel
        quat = data.qpos[q + 3:q + 7]
        yaw = math.atan2(2 * (quat[0] * quat[3] + quat[1] * quat[2]),
                         1 - 2 * (quat[2] ** 2 + quat[3] ** 2))
        c, s = math.cos(-yaw), math.sin(-yaw)
        world = data.qvel[v:v + 3]
        lin = np.array([c * world[0] - s * world[1], s * world[0] + c * world[1], world[2]])
        gravity = _gravity_in_body(quat)
        pos = np.array(data.qpos[q:q + 3], dtype=float)
        fallen = bool(pos[2] - floor_z < self.fall_height or gravity[2] > -0.5)
        return BodyState(pos=pos, yaw=yaw, lin_vel=lin,
                         ang_vel=np.array(data.qvel[v + 3:v + 6], dtype=float),
                         gravity=gravity, fallen=fallen)

    def teleport(self, data: mujoco.MjData, pose: Pose, z: float = 0.0) -> None:
        self.reset(data, pose)
        q = self._root_qpos
        data.qpos[q + 2] = z + STAND_Z
        mujoco.mj_forward(data.model, data)

    def command_from_action(self, action: Sequence[float]) -> np.ndarray:
        """Keep the repository convention where the forward range is asymmetric.

        `action[0] = -1` has to be the slowest this body goes, not full reverse. With the
        default linear mapping onto an asymmetric range it was full reverse, and it showed:
        the scripted solver's "wall ahead, turn in place" value drove the humanoid
        backwards in a straight line for thirteen seconds until it walked off the room and
        fell, five seeds out of five. Forward maps onto [0, vx_max]; the lateral and turn
        axes stay linear over their symmetric ranges.
        """
        a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        (_, vx_hi), (vy_lo, vy_hi), (wz_lo, wz_hi) = self.cmd_limits
        return np.array([
            (a[0] + 1.0) / 2.0 * vx_hi,
            vy_lo + (a[1] + 1.0) / 2.0 * (vy_hi - vy_lo),
            wz_lo + (a[2] + 1.0) / 2.0 * (wz_hi - wz_lo),
        ])

    def root_body_id(self, model: mujoco.MjModel) -> int:
        return int(model.body(PREFIX + PELVIS).id)

    def history_state(self) -> dict[str, Any]:
        return {"last_action": self.last_action.tolist(), "clock": float(self.clock)}

    def load_history(self, data: mujoco.MjData, state: dict[str, Any]) -> None:
        self.last_action = np.asarray(state["last_action"], dtype=np.float32)
        self.clock = float(state.get("clock", 0.0))
