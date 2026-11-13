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

import copy
from pathlib import Path
from typing import Any, Sequence

import mujoco
import numpy as np

from .. import assets
from .base import BodyAdapter, BodyState, Capabilities, Pose, SubstepFn, rotate_inverse, yaw_from_quat

# Files in the asset cache (keys of upstream.lock.json).
ROBOT_XML = "robot/robot_allcollisions_backlash.xml"
POLICY_ONNX = "policies/velstand.onnx"
# Roller addon (R30, OQ7): the official roller model's wheel bodies, its policies.
ROLLER_XML = "robot/robot_groundcontact_rollers_backlash.xml"
ROLLER_ONNX = "policies/roller.onnx"
CROUCH_ONNX = "policies/roller_crouch.onnx"

# Names in the compiled scene carry PREFIX (MjSpec.attach namespace).
PREFIX = "body/"
TRUNK_BODY = "trunk_base"
FREEJOINT = "trunk_base_freejoint"
GYRO_SENSOR = "imu_ang_vel"
HEAD_SITE = "head_camera"     # MJCF frame site and camera of the head camera (R39)
LEVEL_SETTLE_STEPS = 25       # body steps standing still before reading the level head pose

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
CMD_DIM = 13                # twist(3) + head_pose(4) + body_pose(6); body_pose stays 0
# Head command (R19, OQ2): deltas from HOME for (neck_pitch, head_pitch, head_yaw,
# head_roll). Training (velocity_env_cfg head_pose_range, last stage) samples
# +-1.10, +-1.10, +-1.40, +-0.31 rad, also while walking. `set_head(t)` sends
# t * HEAD_LOW: mouth_tip 0.21 m (t 0) .. 0.135 m (t 1) above the floor, no fall
# standing, walking (vx 0.25-0.4), turning or toggling the head (10/10 seeds each).
HEAD_LOW = np.array([-1.1, 1.1, 0.0, 0.0], dtype=np.float32)
MOUTH_SITE = "mouth_tip"    # beak tip (R18 weld grab, R4 wall buttons)

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


# Action -> command mapping (PuzzleEnv actions in [-1, 1]^3), built around the
# measured velstand response (2026-11-11, 5 s runs, flat floor):
# - vx <= 0.15 alone, vx = -0.2, and turning in place with |wz| <= 0.7 leave the
#   duck standing; vx = 0.2 with wz = 0.3 also stands.
# - vx >= 0.25 walks for every wz in [-1, 1]: vx 0.25 -> 0.12 m/s, 0.3 -> 0.15,
#   0.4 -> 0.22 m/s; with wz = +-1 it turns 0.9-1.0 rad/s while creeping 3-6 cm/s.
#   At vx 0.4, wz 0 drifts -0.2 rad/s (wz 0.3 walks straight).
# - in place, only |wz| = 1.0 turns (0.7-0.9 rad/s).
# - vy alone barely moves (0.3 -> 0.06 m/s); with forward walking it is harmless.
# So: forward a0 < STOP_BELOW -> stand (vx = 0) and turn in place only at full
# rate; a0 in [STOP_BELOW, 1] -> vx linear in WALK_VX (no dead band), wz = a2
# linear (every wz works while walking). vy = a1 * 0.3 linear. Backward walking
# (weak, dead near -0.2) is not used.
STOP_BELOW = -0.6
WALK_VX = (0.25, 0.4)
TURN_IN_PLACE_MIN = 0.5     # |a2| needed to turn in place; mapped to |wz| = 1.0

# Roller addon (R30). The combined model (built only when a room has a `roller`
# station) hangs the official roller model's wheel bodies tire, tire_2 under
# ankle_left and tire_3, tire_4 under ankle_right (OQ7). Mounted, exactly the
# official roller model's colliding geoms collide (13 with the 4 tires, matched
# by body and mesh name); dismounted, the foot model's set, wheels off and hidden.
ROLLER_WHEELS = {"ankle_left": ("tire", "tire_2"), "ankle_right": ("tire_3", "tire_4")}
WHEEL_PREFIX = "w_"
ADDON_NONE, ADDON_ROLLER = "none", "roller"
ROLLER_ACTION_SCALE = 0.8   # manifest.json: roller.onnx and roller_crouch.onnx
# Wheel joint friction [N m]. Upstream's MJCF has 0; OQ7 ran 0.0005-0.003 per seed
# (with friction > 0 a zero command stops the rolling duck within 1 s). Mid value.
WHEEL_FRICTIONLOSS = 0.0015
WHEEL_RGBA = {"tire": (0.12, 0.12, 0.12), "rim": (0.85, 0.85, 0.8)}
# roller.onnx command (OQ7): vx is a push strength in -0.5..0.6 (|vx| <= 0.15
# does not move, 0.6 rolls about 0.33 m/s), vy stays 0, and the third slot is the
# heading error to a target yaw clipped to +-0.5, fed every step (a fixed value
# spins at 1.5-4.4 rad/s). The adapter keeps that target: the command's third
# value is a target-yaw rate [rad/s] it integrates.
ROLLER_CMD_LIMITS = ((-0.5, 0.6), (0.0, 0.0), (-1.0, 1.0))
ROLLER_VX = (0.25, 0.6)     # action[0] in [STOP_BELOW, 1] -> push; below -> 0 (coast to a stop)
HEADING_CLIP = 0.5          # [rad] heading-error slot limit (OQ7)
# roller_crouch.onnx (manifest): episodic 3.5 s, phase command [cos, sin] of
# phase += dt / 5.0 up to 0.7, action_scale 0.8. Only from speed >= crouch_min_speed.
CROUCH_PERIOD = 5.0
CROUCH_END_PHASE = 0.7
CROUCH_MIN_SPEED = 0.2      # [m/s] spec R30 default
# Tow hitch (R31, OQ9): low, so a pull does not tip the trunk back (standing hold 2.2 N
# at 0.04 m, 1.0 N at 0.08 m, 0.6 N at 0.12 m). Trunk frame: under the midpoint of the
# two hip_yaw anchors (x 0.006, y +-0.0175), 0.04 m above the floor in the standing pose
# (trunk 0.1157 m high after 2 s at command 0; measured 0.007, 0.0005, -0.0756, seeds 0-2).
TOW_SITE = "tow_hitch"
TOW_SITE_POS = (0.007, 0.0, -0.0756)


class MicroduckAdapter(BodyAdapter):
    cmd_limits = ((-0.4, 0.4), (-0.3, 0.3), (-1.0, 1.0))   # velstand twist ranges
    control_dt = TIMESTEP * DECIMATION
    timestep = TIMESTEP
    fall_height = FALL_HEIGHT
    footprint_radius = 0.11   # collision geoms span x -0.08..0.11, y +-0.11 m around the trunk
    # R2b. beak_grab: weld grab at the `mouth_tip` site (spec R18); max_lift_mass
    # is the conservative pre-measurement default from spec R18. max_step [m] and
    # max_slope [deg] (R37) are OQ5 measurements (2026-11-11, velstand, vx 0.3,
    # 10 seeds): 1 cm steps up 10/10 (1.2 cm 8/10, 1.4 cm 5/10), 2 cm down
    # without a fall; 5 deg slopes up 10/10 (6 deg 9/10, 7 deg 2/10). beak_height
    # [m]: the beak tip above the floor with the head level (OQ2; `mpg check`
    # asks for the `head` action below it, R19).
    capabilities = Capabilities(names=frozenset({"walk", "beak_grab", "roller"}),
                                values={"max_lift_mass": 0.05, "max_step": 0.01, "max_slope": 5.0,
                                        "beak_height": 0.21})

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
        self._ort_opts = opts
        self.session = ort.InferenceSession(str(self.policy_path), opts,
                                            providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self._asset_root = root
        self._sessions = {"velstand": self.session}
        self.addons: frozenset[str] = frozenset()   # addons the model is built with (R30)
        self.addon = ADDON_NONE
        self.action_scale = ACTION_SCALE
        self.cmd_limits = type(self).cmd_limits
        self.crouch_min_speed = CROUCH_MIN_SPEED
        self.crouch_left = 0          # body steps of crouch left (0: not crouching)
        self._phase = 0.0
        self._yaw_target = 0.0

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
        self.head = np.zeros(4, dtype=np.float32)
        self._prev_torque = np.zeros(len(JOINTS))

    def command_from_action(self, action: Sequence[float]) -> np.ndarray:
        a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        if self.addon == ADDON_ROLLER:
            # Roller: below STOP_BELOW no push (the wheels coast to a stop), else a
            # push in ROLLER_VX; action[2] -> target-yaw rate (+-1 rad/s), lateral unused.
            wz = a[2] * self.cmd_limits[2][1]
            if a[0] < STOP_BELOW:
                return np.array([0.0, 0.0, wz])
            frac = (a[0] - STOP_BELOW) / (1.0 - STOP_BELOW)
            return np.array([ROLLER_VX[0] + frac * (ROLLER_VX[1] - ROLLER_VX[0]), 0.0, wz])
        vy = a[1] * self.cmd_limits[1][1]
        if a[0] < STOP_BELOW:
            wz = np.copysign(self.cmd_limits[2][1], a[2]) if abs(a[2]) >= TURN_IN_PLACE_MIN else 0.0
            return np.array([0.0, vy, wz])
        frac = (a[0] - STOP_BELOW) / (1.0 - STOP_BELOW)
        vx = WALK_VX[0] + frac * (WALK_VX[1] - WALK_VX[0])
        return np.array([vx, vy, a[2] * self.cmd_limits[2][1]])

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
        self._fix_head_camera(spec)
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
        if ADDON_ROLLER in self.addons:
            self._add_wheels(spec)
        return spec

    # ---- addons (R30) --------------------------------------------------------
    def use_addons(self, names) -> None:
        names = frozenset(names)
        unknown = sorted(names - {ADDON_ROLLER})
        if unknown:
            raise ValueError(f"the microduck has no addon {unknown}; it has {[ADDON_ROLLER]}")
        self.addons = names
        for n, rel in (("roller", ROLLER_ONNX), ("crouch", CROUCH_ONNX)):
            if ADDON_ROLLER in names and n not in self._sessions:
                import onnxruntime as ort

                path = self._asset_root / rel
                if not path.exists():
                    raise FileNotFoundError(f"{path} missing; run `mpg fetch-assets` first")
                self._sessions[n] = ort.InferenceSession(str(path), self._ort_opts,
                                                         providers=["CPUExecutionProvider"])

    def _roller_spec(self) -> mujoco.MjSpec:
        path = self._asset_root / ROLLER_XML
        if not path.exists():
            raise FileNotFoundError(f"{path} missing; run `mpg fetch-assets` first")
        rs = mujoco.MjSpec.from_file(str(path))
        mesh_dir = path.parent / (rs.meshdir or "")
        for mesh in rs.meshes:
            if mesh.file:
                mesh.file = str((mesh_dir / mesh.file).resolve())
        rs.meshdir = ""
        return rs

    def _add_wheels(self, spec: mujoco.MjSpec) -> None:
        """Hang the official wheel bodies under the ankles, dismounted: wheel collisions
        off, wheel geoms drawn with their own colour (no material) at alpha 0."""
        self._roller_collide = self._official_collision_set(self._roller_spec())
        rs = self._roller_spec()
        for ankle, tires in ROLLER_WHEELS.items():
            dst = spec.body(ankle)
            for t in tires:
                dst.add_frame().attach_body(rs.body(t), WHEEL_PREFIX, "")
        for ankle, tires in ROLLER_WHEELS.items():
            for t in tires:
                b = spec.body(WHEEL_PREFIX + t)
                for j in b.joints:
                    j.frictionloss = WHEEL_FRICTIONLOSS
                for g in b.geoms:
                    key = "tire" if g.meshname.endswith("tire") else "rim"
                    g.material = ""
                    g.rgba = [*WHEEL_RGBA[key], 0.0]
                    g.contype = 0
                    g.conaffinity = 0

    @staticmethod
    def _official_collision_set(rs: mujoco.MjSpec) -> set[tuple[str, str]]:
        """(body, mesh) names of the official roller model's colliding geoms."""
        m = rs.compile()
        return {(m.body(int(m.geom_bodyid[g])).name,
                 m.mesh(int(m.geom_dataid[g])).name if m.geom_dataid[g] >= 0 else "")
                for g in range(m.ngeom) if m.geom_contype[g] or m.geom_conaffinity[g]}

    @staticmethod
    def _fix_head_camera(spec: mujoco.MjSpec) -> None:
        """Point the MJCF `head_camera` camera along its frame site's +x (the lens
        axis; site +z is up). Upstream's camera quat looks backward into the head
        shell under MuJoCo's camera convention (view along -z, +y up)."""
        site = next((s for s in spec.sites if s.name == HEAD_SITE), None)
        cam = next((c for c in spec.cameras if c.name == HEAD_SITE), None)
        if site is None or cam is None:
            return
        rs = np.zeros(9)
        mujoco.mju_quat2Mat(rs, np.asarray(site.quat, dtype=np.float64))
        rs = rs.reshape(3, 3)
        rc = np.column_stack([-rs[:, 1], rs[:, 2], -rs[:, 0]])   # camera x, y, z
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, rc.ravel())
        cam.pos = site.pos
        cam.quat = q

    def grab_site(self) -> str:
        return PREFIX + MOUTH_SITE

    def tow_site(self, spec: mujoco.MjSpec | None = None) -> str:
        name = PREFIX + TOW_SITE
        if spec is not None and not any(s.name == name for s in spec.sites):
            spec.body(PREFIX + TRUNK_BODY).add_site(name=name, pos=list(TOW_SITE_POS),
                                                    size=[0.006, 0, 0], group=4)
        return name

    def set_head(self, level: float) -> None:
        self.head = HEAD_LOW * float(np.clip(level, 0.0, 1.0))

    def view_camera(self, model: mujoco.MjModel) -> int | None:
        cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, PREFIX + HEAD_SITE)
        return cid if cid >= 0 else None

    def _level_head_frame(self) -> tuple[np.ndarray, np.ndarray]:
        """Head-site position and the head body's rotation relative to the heading
        frame, read once from the body standing still on a flat floor (cached)."""
        if getattr(self, "_level_cache", None) is None:
            from .base import flat_floor_spec

            probe = MicroduckAdapter(asset_dir=self.policy_path.parents[1], vin=self.vin,
                                     vin_drop_gain=self.vin_drop_gain,
                                     action_lag=self.action_lag, robot_xml=self.robot_xml)
            model = probe.spawn(flat_floor_spec()).compile()
            data = mujoco.MjData(model)
            probe.reset(data)
            for _ in range(LEVEL_SETTLE_STEPS):
                probe.step(data, (0.0, 0.0, 0.0))
            sid = model.site(PREFIX + HEAD_SITE).id
            bid = int(model.site_bodyid[sid])
            yaw = probe.state(data).yaw
            c, s = np.cos(yaw), np.sin(yaw)
            r_yaw = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
            r_body = r_yaw.T @ data.xmat[bid].reshape(3, 3)          # head in heading frame
            self._level_cache = (model.site_pos[sid].copy(), r_body)
        return self._level_cache

    def add_rangefinders(self, spec: mujoco.MjSpec, yaws: Sequence[float], prefix: str) -> list[str]:
        site = next((s for s in spec.sites if s.name == PREFIX + HEAD_SITE), None)
        if site is None:
            raise RuntimeError("spawn the body before adding rangefinders")
        head = site.parent
        pos, r_body = self._level_head_frame()
        names = []
        for i, a in enumerate(yaws):
            d_local = r_body.T @ np.array([np.cos(a), np.sin(a), 0.0])   # ray in head frame
            z = np.array([0.0, 0.0, 1.0])
            axis = np.cross(z, d_local)
            ang = float(np.arctan2(np.linalg.norm(axis), d_local @ z))
            n = np.linalg.norm(axis)
            axis = axis / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])
            quat = [np.cos(ang / 2), *(np.sin(ang / 2) * axis)]       # site +z -> ray
            sname = f"{prefix}{i}"
            head.add_site(name=sname, pos=pos, quat=quat, group=4)
            sen = spec.add_sensor(name=sname, type=mujoco.mjtSensor.mjSENS_RANGEFINDER,
                                  objtype=mujoco.mjtObj.mjOBJ_SITE, objname=sname)
            sen.intprm[0] = 1 << int(mujoco.mjtRayDataField.mjRAYDATA_DIST)
            names.append(sname)
        return names

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
        if ADDON_ROLLER in self.addons:
            self._bind_wheels(model)

    def _bind_wheels(self, model: mujoco.MjModel) -> None:
        """Geom sets of the two modes (R30): `_foot_on` (the foot model's colliding geoms,
        with their compiled contype / conaffinity), `_roller_on` (the official roller set),
        the ground-contact geoms of each (ankles, tires) and the wheel geoms to hide."""
        P, W = PREFIX, WHEEL_PREFIX
        tires = {model.body(P + W + t).id for ts in ROLLER_WHEELS.values() for t in ts}
        ankles = {model.body(P + a).id for a in ROLLER_WHEELS}

        def bare(name: str) -> str:
            name = name[len(P):] if name.startswith(P) else name
            return name[len(W):] if name.startswith(W) else name

        body_root = model.body_rootid[model.body(P + TRUNK_BODY).id]
        foot, roller, wheel_geoms, wheel_contact, ankle_contact = {}, {}, [], [], []
        for g in range(model.ngeom):
            b = int(model.geom_bodyid[g])
            if model.body_rootid[b] != body_root:
                continue
            mesh = bare(model.mesh(int(model.geom_dataid[g])).name) if model.geom_dataid[g] >= 0 else ""
            key = (bare(model.body(b).name), mesh)
            if b in tires:
                wheel_geoms.append(g)
                if model.geom_group[g] == 3:          # collision class (visual geoms are group 2)
                    wheel_contact.append(g)
                    roller[g] = (1, 1)
                continue
            ct = (int(model.geom_contype[g]), int(model.geom_conaffinity[g]))
            if ct == (0, 0):
                continue
            foot[g] = ct
            if b in ankles:
                ankle_contact.append(g)
            if key in self._roller_collide:
                roller[g] = ct
        # The compiler ORs geom contype / conaffinity into a per-body filter; the wheels
        # compile with collisions off, so open their bodies' filter once here.
        for b in tires:
            model.body_contype[b], model.body_conaffinity[b] = 1, 1
        self._foot_on, self._roller_on = foot, roller
        self._wheel_geoms = np.array(wheel_geoms, dtype=int)
        self._contact = {ADDON_NONE: ankle_contact, ADDON_ROLLER: wheel_contact}
        self._wheel_rgba = model.geom_rgba[self._wheel_geoms, :3].copy()

    def _apply_addon(self, model: mujoco.MjModel, mode: str) -> None:
        """Collision groups, wheel visibility, lower policy and command ranges of `mode`."""
        if self.addons:
            on = self._roller_on if mode == ADDON_ROLLER else self._foot_on
            for g in set(self._foot_on) | set(self._roller_on):
                model.geom_contype[g], model.geom_conaffinity[g] = on.get(g, (0, 0))
            model.geom_rgba[self._wheel_geoms, 3] = 1.0 if mode == ADDON_ROLLER else 0.0
        name = "velstand" if mode == ADDON_NONE else ("crouch" if self.crouch_left > 0 else "roller")
        self.session = self._sessions[name]
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.action_scale = ACTION_SCALE if mode == ADDON_NONE else ROLLER_ACTION_SCALE
        self.cmd_limits = type(self).cmd_limits if mode == ADDON_NONE else ROLLER_CMD_LIMITS
        self.addon = mode

    def _low_z(self, data: mujoco.MjData, geoms) -> float:
        """Lowest world z of the mesh vertices of `geoms` in the pose now."""
        m = data.model
        zs = []
        for g in geoms:
            mid = int(m.geom_dataid[g])
            if mid < 0:
                zs.append(float(data.geom_xpos[g][2]))
                continue
            a, n = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
            rot = data.geom_xmat[g].reshape(3, 3)
            zs.append(float((m.mesh_vert[a:a + n] @ rot[2]).min() + data.geom_xpos[g][2]))
        return min(zs)

    def set_addon(self, data: mujoco.MjData, mode: str, keep_pose: bool = False) -> float:
        """Switch the body to addon `mode` ("none" or "roller", R30), in this order: trunk
        height (the new contact geoms' lowest point onto the old ones', from the pose
        now), zero trunk velocity, collision groups and wheel visibility, lower policy
        (velstand <-> roller; joints are matched by name, the action order is the same)
        with its action history cleared, then `mj_forward`. Returns the height change.

        `keep_pose` (reset, snapshot restore): the pose in `data` already belongs to
        `mode`; only the model values and the policy are set, nothing moves."""
        if mode not in (ADDON_NONE, *self.addons):
            raise ValueError(f"addon {mode!r} not built into this model (built: {sorted(self.addons)})")
        if self._data is not data:
            self._bind(data)
        m = data.model
        dz = 0.0
        if not keep_pose:
            if mode != self.addon:
                mujoco.mj_forward(m, data)
                dz = self._low_z(data, self._contact[self.addon]) - self._low_z(data, self._contact[mode])
                q, v = self._root_qpos, self._root_qvel
                data.qpos[q + 2] += dz
                data.qvel[v:v + 6] = 0.0
            self.crouch_left = 0
            self.last_action[:] = 0.0
            self._yaw_target = yaw_from_quat(data.qpos[self._root_qpos + 3:self._root_qpos + 7])
        self._apply_addon(m, mode)
        mujoco.mj_forward(m, data)
        return dz

    def wheel_dofs(self, model: mujoco.MjModel) -> list[int]:
        if ADDON_ROLLER not in self.addons:
            return []
        tires = {model.body(PREFIX + WHEEL_PREFIX + t).id for ts in ROLLER_WHEELS.values() for t in ts}
        return [int(model.jnt_dofadr[j]) for j in range(model.njnt) if model.jnt_bodyid[j] in tires]

    def crouch(self, data: mujoco.MjData) -> bool:
        """Start `roller_crouch` (R30): only on wheels, not crouching yet, and at a
        horizontal speed of at least `crouch_min_speed`. Returns whether it started."""
        if self.addon != ADDON_ROLLER or self.crouch_left > 0:
            return False
        v = self._root_qvel
        if float(np.hypot(data.qvel[v], data.qvel[v + 1])) < self.crouch_min_speed:
            return False
        self.crouch_left = int(round(CROUCH_PERIOD * CROUCH_END_PHASE / self.control_dt))
        self._phase = 0.0
        self.last_action[:] = 0.0
        self._apply_addon(data.model, ADDON_ROLLER)
        return True

    @property
    def crouching(self) -> bool:
        return self.crouch_left > 0

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
        self.head = np.zeros(4, dtype=np.float32)
        self._targets = [DEFAULT_POSE.astype(np.float64)] * (self.action_lag + 1)
        self.crouch_left = 0
        self._yaw_target = yaw
        self.set_addon(data, ADDON_NONE, keep_pose=True)   # R30: back on feet, velstand

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
        if self.addon == ADDON_NONE:               # the roller policies run with the head at home
            command[3:7] = self.head
        return np.concatenate([
            data.sensordata[self._gyro],                                 # base_ang_vel (IMU gyro)
            rotate_inverse(quat, np.array([0.0, 0.0, -1.0])),            # projected_gravity
            self.encoder_pos(data) - DEFAULT_POSE,                       # joint_pos rel. default
            self.encoder_vel(data),                                      # joint_vel
            self.last_action,                                            # last action
            command,                                                     # command (13)
        ]).astype(np.float32)

    def teleport(self, data: mujoco.MjData, pose: Pose, z: float = 0.0) -> None:
        mode = self.addon
        self.reset(data, pose)
        if mode != ADDON_NONE:                     # a warp keeps the wheels on (R30)
            self.set_addon(data, mode)
        if z:
            data.qpos[self._root_qpos + 2] += z
            mujoco.mj_forward(data.model, data)

    def kick(self, data: mujoco.MjData, velocity: Sequence[float]) -> None:
        v = self._root_qvel
        data.qvel[v:v + 3] = np.asarray(velocity, dtype=np.float64)

    def shift(self, data: mujoco.MjData, delta: Sequence[float]) -> None:
        q = self._root_qpos
        data.qpos[q:q + 3] += np.asarray(delta, dtype=np.float64)

    def root_body_id(self, model: mujoco.MjModel) -> int:
        return model.body(PREFIX + TRUNK_BODY).id

    def history_state(self) -> dict[str, Any]:
        c = self._ctrl
        return {
            "last_action": self.last_action.copy(),
            "prev_torque": self._prev_torque.copy(),
            "targets": [t.copy() for t in self._targets],
            "q_target": c.q_target.copy(),
            "actuator_state": copy.deepcopy(c.actuator_state),
            "control": copy.deepcopy(c.control),
            "last_ts": c.last_ts,
            "head": self.head.copy(),
            "addon": self.addon,
            "crouch_left": self.crouch_left,
            "phase": self._phase,
            "yaw_target": self._yaw_target,
        }

    def load_history(self, data: mujoco.MjData, state: dict[str, Any]) -> None:
        if self._data is not data:
            self._bind(data)
        c = self._ctrl
        self.crouch_left = int(state.get("crouch_left", 0))
        self._phase = float(state.get("phase", 0.0))
        self._yaw_target = float(state.get("yaw_target", 0.0))
        self.set_addon(data, state.get("addon", ADDON_NONE), keep_pose=True)   # R30
        self.last_action = state["last_action"].copy()
        self._prev_torque = state["prev_torque"].copy()
        self._targets = [t.copy() for t in state["targets"]]
        c.q_target[:] = state["q_target"]
        c.actuator_state = copy.deepcopy(state["actuator_state"])
        c.control = copy.deepcopy(state["control"])
        c.last_ts = state["last_ts"]
        self.head = state.get("head", np.zeros(4, dtype=np.float32)).copy()

    def step(self, data: mujoco.MjData, cmd: Sequence[float],
             on_substep: SubstepFn | None = None) -> None:
        if self._data is not data:
            raise RuntimeError("call reset(data) before step(data)")
        cmd = self.clip_cmd(cmd)
        if self.addon != ADDON_NONE:
            cmd = self._addon_cmd(data, cmd)
        obs = self.observation(data, cmd)
        action = self.session.run([self.output_name], {self.input_name: obs[None, :]})[0][0]
        self.last_action = action.astype(np.float32)
        target = DEFAULT_POSE + self.last_action.astype(np.float64) * self.action_scale
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
            if on_substep is not None:
                on_substep(model, data)
        if self.crouch_left > 0:
            self.crouch_left -= 1
            if self.crouch_left == 0:              # crouch over: back to roller.onnx
                self.last_action[:] = 0.0
                self._yaw_target = yaw_from_quat(data.qpos[self._root_qpos + 3:self._root_qpos + 7])
                self._apply_addon(model, ADDON_ROLLER)

    def _addon_cmd(self, data: mujoco.MjData, cmd: np.ndarray) -> np.ndarray:
        """The policy command on wheels: crouch phase [cos, sin, 0], or roller
        (vx, 0, heading error to the target yaw, which moves at cmd[2] rad/s)."""
        if self.crouch_left > 0:
            out = np.array([np.cos(2 * np.pi * self._phase), np.sin(2 * np.pi * self._phase), 0.0])
            self._phase += self.control_dt / CROUCH_PERIOD
            return out
        yaw = yaw_from_quat(data.qpos[self._root_qpos + 3:self._root_qpos + 7])
        target = self._yaw_target + float(cmd[2]) * self.control_dt
        err = (target - yaw + np.pi) % (2 * np.pi) - np.pi
        err = float(np.clip(err, -HEADING_CLIP, HEADING_CLIP))
        self._yaw_target = yaw + err               # no wind-up past the slot limit
        return np.array([float(cmd[0]), 0.0, err])

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
            # R30: crouching lowers the trunk to about 0.075 m, so only the tilt counts then
            fallen=bool(gravity[2] > FALL_GRAVITY_Z
                        or (self.crouch_left == 0 and pos[2] - floor_z < FALL_HEIGHT)),
        )
