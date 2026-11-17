"""A fixed-base arm as a `BodyAdapter` (M4 R61): the gripper is the body.

The puzzle side of this project asks a body four things - where it is on the plane, which
way it faces, how fast it is going, and whether it has fallen - and it plans on a 2D grid.
Tabletop manipulation is a plane problem too. So this adapter answers those four questions
about the **gripper** rather than about a trunk, and the planar command `(vx, vy, wz)`
becomes the gripper's velocity across the table plus its wrist rotation. Nothing in
`BodyAdapter` had to change, and the M3 judgement that an arm would need a separate base
class was wrong.

What differs from a walking body, and is handled here rather than hidden:

- **The base is bolted down.** The model has no free joint (`nq == nv == 6`), so `reset`
  cannot move the arm to an episode's spawn pose: it restores the home joint angles and
  leaves the base where `spawn` put it. A room that randomizes its spawn therefore
  randomizes nothing for this body; `check_warnings` says so rather than leaving the user
  to wonder why the layout never changes.
- **It cannot fall.** `fallen` is always False and the tilt is always upright, so the
  `fall` reward and termination never fire for it.
- **It cannot walk.** `walk` is not declared, so the 15 built-in parts that need walking
  refuse themselves by name through the capability gate (R2b) instead of silently
  producing an unsolvable room.
- **`grab` is declared, and the name is wrong.** The gate's question is "can this
  body pick an object up with its own end effector", which this arm can; the name is
  anatomy borrowed from the first body, which is the same kind of leak R2 exists to stop.
  Renaming it to `grab` (keeping `grab` as an alias the first body also declares) is
  M4 R60 work. Declaring it here is the honest answer to the question the gate asks.

Measured on the pinned model (2026-11-16, `mj_kinematics` over a 9^4 joint sweep, 6,561
samples): the gripper site reaches 0.001-0.480 m from the base and z -0.223 to 0.520 m,
and within the tabletop band z 0.02-0.15 m it covers radius up to 0.480 m. The shoulder
pan range is +/-1.920 rad, so the envelope is a 220-degree sector, not a full annulus.
No official reach or payload figure is published for this arm, so these are the numbers
this project uses (R64).
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Sequence

import mujoco
import numpy as np

from .base import BodyAdapter, BodyState, Capabilities, Pose, SubstepFn

#: This body's pinned assets (R40): the Menagerie model, no policy. Its paths live under
#: `arm/`, not `robot/`: this body and the quadruped are pinned from the same upstream
#: commit and the asset cache is keyed by that commit, so body-agnostic paths overwrite
#: each other's files. That happened once during this build; `tests/test_assets.py` now
#: refuses the collision.
LOCK = Path(__file__).resolve().parents[1] / "upstream.so101.lock.json"
PREFIX = "so101/"
ROBOT_XML = "arm/so101.xml"
TIMESTEP = 0.005
CONTROL_DT = 0.02
#: Arm joints in model order; `gripper` is the jaw and is driven separately.
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
JAW_JOINT = "gripper"
SITE = "gripperframe"
#: Home joint angles for the five arm joints, chosen by the sweep above to put the gripper
#: in front of the base at (0.215, 0.000, 0.122) m.
HOME = (0.0, -0.873, 0.563, 1.382, 0.0)
#: Gripper height held by the z term while a planar command runs [m].
WORK_Z = 0.12
#: Jaw height at `set_head(1.0)` [m] - the table, where an object lying on it has its
#: handle. The shared `head` channel means "lower the thing that grabs" (`BodyAdapter.
#: set_head`: 0 level, 1 the lowest the body takes), and for this body that is the jaw's
#: height rather than a head pitch. Without it this arm could not pick anything off a
#: table at all: measured, the approach brings the jaw to within 2 mm of a block's handle
#: horizontally and parks 0.102 m above it, against a 0.04 m grab radius, so the weld
#: never armed and every grasping number for this body was really a number about reaching
#: a goal. The kinematic sweep covers the band z 0.02-0.15 m out to the full 0.480 m
#: radius, so the table is inside the envelope; the IK gets there through the same z term.
GRAB_Z = 0.02
#: How far the aim point may run ahead of where the gripper actually is [m].
#:
#: The aim advances by `cmd * control_dt` every step, so while the gripper is blocked -
#: pressing a block against the inside of a box, say - the aim keeps integrating and ends
#: up far ahead of the jaw. The command then stops and the arm *keeps going*, because it
#: is still solving toward an aim that ran away: measured, a block released into a box
#: travelled another 0.22 m and climbed to z 0.146 m, out of the box, with a zero command
#: the whole time. Integrator windup, and this is the anti-windup: ten control steps'
#: worth of travel at the command ceiling (0.15 m/s x 0.02 s x 10).
MAX_LEAD = 0.03
Z_GAIN = 12.0           # [1/s] P gain pulling the gripper back to `WORK_Z`
#: Integral gain and clamp [m.s] on the working-height error (`step`).
#:
#: Swept, with the arm holding a zero command for 3 s and then travelling at the command
#: ceiling for 3 s (settled sag / sag while travelling, mm): 0 and 1 both -1.91 / -8.65,
#: 3 gives -0.005 / -2.61, and 10 and 30 both -0.001 / -1.66. 10 is the first value at the
#: flat end of that sweep, and 30 measuring the same says it is not perched on a cliff.
#: The reset transient (+1.88 mm of overshoot) is the same at every gain, so the integral
#: is not what causes it.
#:
#: The clamp is the anti-windup, and it is on the term's **contribution** in metres, not on
#: the accumulator: without one, a gripper held down - by a lid, pressing a block onto a
#: table, or asked for a pose it cannot hold - integrates for as long as it is held and then
#: springs. 0.02 m is twice the worst sag ever measured here (8.65 mm while travelling).
Z_I_GAIN = 10.0
Z_I_MAX = 0.02
#: Gain [1/s] on the orientation task, when one is asked for (`align_axis`, R114).
#:
#: The solve is position-only by default and the gripper's orientation is whatever the five
#: joints happen to leave - which is why nothing here can grasp by contact: measured, the
#: jaw's opening axis at the grasp height is (0.900, -0.047, 0.434), 43 % vertical, so the
#: two fingers do not straddle an object lying on the table. One goes into the table and the
#: other over the top, and across masses 10-60 g, frictions 0.3-2.0, cube half-widths
#: 10-20 mm and wrist rolls 0-1.2 s the cube never rose.
#:
#: Asking for an axis adds three rows (`mj_jacSite`'s rotational half) to the damped solve.
#: It is **off unless a caller sets `align_axis`**, so every room that ships today takes the
#: same 3-row solve it was measured with. Three control changes measured earlier today each
#: broke a shipped room; this one is written so that it cannot.
ALIGN_GAIN = 6.0
#: Where a grasp closes, as a **fixed** offset in the gripper site's own frame [m] (R120).
#:
#: Measured from the finger-tip geoms with the jaw open, which is the pose an approach
#: straddles an object in. It has to be fixed rather than read live, because the live
#: midpoint travels with the jaw - measured (-0.0116, 0, +0.0226) open, (-0.0010, 0,
#: +0.0069) half open, (-0.0009, 0, -0.0184) shut, so 4 cm along the site's own z - and the
#: jaw joint is **not** in the solve's column set, so that travel is motion the Jacobian
#: cannot see. Servoing the live point made the arm chase it: 7 cm of drift over 8 s with a
#: zero command.
GRASP_OFFSET = (-0.0116, 0.0001, 0.0226)
#: Actuator realism the arm did not model at all (R118). Both default to "none", so every
#: room that ships today runs exactly as it was measured.
#:
#: What the arm had: `kp` 998.22, `kv` 2.731 and a +-2.94 N.m ceiling, all from the real
#: servo, and nothing else - no command latency, no backlash, no servo-to-servo spread. And
#: no randomization layer could add any: `randomize.py`'s `physics` draws **part** values
#: and `body` draws **observation noise**, so neither reaches an actuator. The walking body
#: does have this (its vendor asset carries backlash and friction domain randomization),
#: which is what makes the arm's absence worth naming rather than assuming.
#:
#: `latency_steps` delays the joint targets by whole control steps (0.02 s each), which is
#: what a serial bus and a servo's own loop cost. `servo_spread` scales each joint's `kp` by
#: a factor, for the spread between units of the same part.
LATENCY_MAX = 10
IK_GAIN = 8.0           # [1/s] how hard the solve chases the aim point
DAMPING = 0.05          # damped-least-squares lambda for the inverse kinematics
#: Measured envelope (see the module docstring).
REACH_MAX = 0.48
REACH_MIN = 0.0
#: Shoulder pan range [rad], so the envelope is a sector of +/-110 degrees, not an annulus.
PAN_LIMIT = 1.92
#: The height a part may sit at and still be worked by this body [m] - which is the height
#: the gripper can be *driven* to, not the highest the joints reach.
#:
#: Those are different numbers and the first version declared the wrong one. The joint
#: sweep reaches z 0.520 m, so 0.45 was declared; but the shared command is `(vx, vy, wz)`
#: and has no vertical axis, and `step` holds the gripper at `WORK_Z` through `Z_GAIN`.
#: Driven to every command limit for 4 s the gripper site stays within z 0.111-0.122 m
#: (forward, lateral and wrist alike); the only excursion is a full-reverse command folding
#: the arm in against the base, which lifts the jaw to 0.316 m on the way and is a fold,
#: not a height anyone can aim at. So 0.45 would have admitted a wall plate at 0.40 m that
#: this body can never touch - the body-side version of the declaration lies the M4 audit
#: found on the part side. Declared at the work height, which is what the control holds;
#: `tests/test_so101.py` measures it and fails if the two drift apart.
REACH_HEIGHT = WORK_Z


class So101Adapter(BodyAdapter):
    """The SO-101 arm, driven by inverse kinematics on its position actuators."""

    control_dt = CONTROL_DT
    timestep = TIMESTEP
    #: The gripper's own girth, not an arm span: this is the clearance the planner inflates
    #: walls by, and what moves through a gap is the jaw.
    footprint_radius = 0.05
    #: It is bolted to the table; nothing about it falls.
    fall_height = 0.0
    #: It does not turn by driving, so it never needs a standing turn (R47).
    stand_to_turn_above = math.inf
    #: Measured: the shoulder pan is the only joint that changes where the jaw points, and
    #: it is also the joint that decides where the jaw is, so the gripper always faces
    #: radially outward from the base and its heading cannot be commanded (+0.2 rad of pan
    #: turns the jaw -0.200 rad; every other joint turns it 0.000). The shared steering
    #: drives it by direction instead of by heading (R61).
    holonomic = True
    #: Standing still is the zero command here: the command maps symmetrically onto the
    #: gripper's velocity, so a walking body's `(-1, 0, 0)` would be full speed backwards.
    stand_action = (0.0, 0.0, 0.0)
    #: `set_head` is this body's working height, so a held object rises with it and the
    #: carrier may lift one over a basket wall instead of pressing it against the wall
    #: (`bodies/base.py` `head_lifts_load`). WORK_Z 0.12 m clears a receptacle's default
    #: 0.06 m wall; a taller-walled box is out of this body's range and reads as before.
    head_lifts_load = True
    #: Clearing a placed object means moving the jaw its own footprint plus the object's
    #: size away, not a walking body's 0.35 m, which can be outside this body's envelope.
    place_clearance = 0.10
    #: Standing off to approach a handle: just outside the 0.04 m grab radius, not a
    #: walking body's 0.3 m, which is a third of this body's whole reach.
    approach_clearance = 0.08
    #: The gripper's planar speed [m/s] and wrist rate [rad/s]. Chosen so one control step
    #: moves the gripper at most 3 mm, which the damped least-squares solve tracks without
    #: the arm snapping; `tests/test_so101.py` measures the tracking error.
    cmd_limits = ((-0.15, 0.15), (-0.15, 0.15), (-1.0, 1.0))
    #: `grab` and nothing else. The first version also declared `reach` and `gripper`,
    #: and no part requires either (R67): the gate compares names, so a name nothing asks
    #: for can neither refuse nor admit a room - it reads as a promise the environment
    #: does not keep. The two dimensions they were meant to carry are already compared as
    #: numbers, which is where R2b puts them: height against `reach_height` (0.45 here,
    #: 0.21 on the first body) and mass against `max_lift_mass`. `two_hands` was left
    #: unbuilt on the same reasoning in M4, and `tests/test_capability_names.py` now fails
    #: on either body rather than leaving the rule to be remembered.
    capabilities = Capabilities(
        names=frozenset({"grab"}),
        values={"max_lift_mass": 0.1, "reach_max": REACH_MAX, "reach_min": REACH_MIN,
                "reach_height": REACH_HEIGHT, "walk_speed": 0.0,
                "max_step": 0.0, "max_slope": 0.0},
    )

    def __init__(self, asset_dir: Path | None = None):
        from .. import assets

        base = Path(asset_dir) if asset_dir else Path(assets.fetch(LOCK)["cache_dir"])
        self.robot_xml = base / ROBOT_XML
        if not self.robot_xml.is_file():
            raise FileNotFoundError(f"{self.robot_xml} missing; run `mpg fetch-assets --body so101`")
        self._data: mujoco.MjData | None = None
        self._base_pose: Pose = (0.0, 0.0, 0.0)
        self._aim = np.zeros(3)            # the aim point `step` drives (world frame)
        self._z_i = 0.0                    # integrated working-height error [m.s]
        #: World direction to bring the jaw's opening axis onto, or None for no orientation
        #: task (the default, and what every shipped room uses). Set it to grasp by contact.
        self.align_axis: np.ndarray | None = None
        #: Rows the orientation task adds: 3 pins the whole orientation, 1 only brings the
        #: opening axis onto the plane `align_axis` spans (R120).
        self.align_rows = 3
        #: Control steps of delay on the joint targets (R118); 0 = none, the default.
        self.latency_steps = 0
        self._queue: list[np.ndarray] = []
        self.jaw_target = 0.0
        #: Height the z term holds the jaw at [m]; `set_head` moves it (WORK_Z .. GRAB_Z).
        self.work_z = WORK_Z

    # ---- scene -------------------------------------------------------------

    def robot_spec(self) -> mujoco.MjSpec:
        return mujoco.MjSpec.from_file(str(self.robot_xml))

    def spawn(self, spec: mujoco.MjSpec, pose: Pose = (0.0, 0.0, 0.0)) -> mujoco.MjSpec:
        """Bolt the arm's base to the world at `pose`. Unlike a walking body this is the
        only time its position is decided: `reset` cannot move it afterwards."""
        x, y, yaw = pose
        self._base_pose = (float(x), float(y), float(yaw))
        frame = spec.worldbody.add_frame(
            pos=[x, y, 0.0], quat=[np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])
        robot = self.robot_spec()
        # Scene options do not come along with `attach`, and this model ships its own:
        # an implicit integrator, an elliptic friction cone and a high impratio, which are
        # what hold a grasped object instead of letting it slide out of the jaw. Set them
        # on the parent *before* attaching, or `attach` reports a conflict, keeps the
        # parent's defaults, and the user sees a warning for values we then fix anyway.
        spec.option.timestep = TIMESTEP
        spec.option.integrator = robot.option.integrator
        spec.option.cone = robot.option.cone
        spec.option.impratio = robot.option.impratio
        spec.option.iterations = robot.option.iterations
        spec.option.ls_iterations = robot.option.ls_iterations
        spec.attach(robot, frame=frame, prefix=PREFIX)
        return spec

    # ---- runtime -----------------------------------------------------------

    def _bind(self, data: mujoco.MjData) -> None:
        model = data.model
        self._data = data
        self._qpos = np.array([model.jnt_qposadr[model.joint(PREFIX + n).id] for n in ARM_JOINTS])
        self._qvel = np.array([model.jnt_dofadr[model.joint(PREFIX + n).id] for n in ARM_JOINTS])
        self._ctrl = np.array([model.actuator(PREFIX + n).id for n in ARM_JOINTS])
        self._jaw_q = int(model.jnt_qposadr[model.joint(PREFIX + JAW_JOINT).id])
        self._jaw_ctrl = int(model.actuator(PREFIX + JAW_JOINT).id)
        self._site = int(model.site(PREFIX + SITE).id)
        self._base_body = int(model.body(PREFIX + "base").id)
        self._range = model.jnt_range[[model.joint(PREFIX + n).id for n in ARM_JOINTS]]
        self._fixed_tips = [model.geom(f"{PREFIX}fixed_jaw_sph_tip{i}").id for i in (1, 2, 3)]
        self._moving_tips = [model.geom(f"{PREFIX}moving_jaw_sph_tip{i}").id for i in (1, 2, 3)]
        #: Position-servo gain [N.m/rad] and torque ceiling [N.m], read from the model
        #: rather than written here, so a re-exported arm cannot silently disagree with
        #: the feed-forward (`step`). Measured on the shipped model: 998.22 and 2.94.
        self._kp = model.actuator_gainprm[self._ctrl, 0].copy()
        self._tau_max = np.abs(model.actuator_forcerange[self._ctrl]).max(axis=1)

    def reset(self, data: mujoco.MjData, pose: Pose = (0.0, 0.0, 0.0)) -> None:
        """Restore the home pose. **`pose` is ignored**: the base is welded to the world
        where `spawn` put it, so an episode cannot place this body somewhere else."""
        if self._data is not data:
            self._bind(data)
        self._z_i = 0.0
        self._queue = []
        data.qpos[self._qpos] = HOME
        data.qvel[self._qvel] = 0.0
        data.ctrl[self._ctrl] = HOME
        data.qpos[self._jaw_q] = 0.0
        data.ctrl[self._jaw_ctrl] = 0.0
        self.jaw_target = 0.0
        self.work_z = WORK_Z               # `set_head` is held until changed; reset levels it
        mujoco.mj_forward(data.model, data)
        self._aim = self._clamp(self.control_point(data))

    def _clamp(self, aim: np.ndarray) -> np.ndarray:
        """Hold an aim point inside the measured envelope, in the base's own frame."""
        bx, by, byaw = self._base_pose
        d = np.array([aim[0] - bx, aim[1] - by])
        r = float(np.hypot(*d))
        if r > REACH_MAX:
            d *= REACH_MAX / r
        ang = math.atan2(d[1], d[0]) - byaw
        ang = (ang + math.pi) % (2 * math.pi) - math.pi
        if abs(ang) > PAN_LIMIT:                  # outside the sector: slide along its edge
            edge = math.copysign(PAN_LIMIT, ang) + byaw
            d = np.hypot(*d) * np.array([math.cos(edge), math.sin(edge)])
        return np.array([bx + d[0], by + d[1], self.work_z])

    def step(self, data: mujoco.MjData, cmd: Sequence[float],
             on_substep: SubstepFn | None = None) -> None:
        """Move the gripper across the table at `cmd` for one control period.

        The command drives a **moving aim point**, not the joint targets directly: the aim
        advances by `cmd * control_dt` and the arm solves towards wherever it now is. The
        first version integrated joint targets from the measured angles instead, which
        accumulated the position actuators' lag - a 0.1 m/s command for 2 s travelled
        0.243 m with 0.036 m of sideways drift, and the gripper sagged 18 mm. Tracking a
        setpoint has no such integral error, and clamping the aim to the measured envelope
        is what keeps the solve away from the edge where the Jacobian loses rank.
        """
        if self._data is not data:
            self._bind(data)
        model = data.model
        vx, vy, wz = self.clip_cmd(cmd)
        # The command is in the body's own frame, and here the body is the gripper: `vx` is
        # along the way the jaw faces, `vy` across it. The first version advanced the aim
        # in world coordinates, which happens to agree at the home pose (yaw 0) and
        # disagrees everywhere else - and the shared steering turns to face a target before
        # driving at it, so world coordinates would have sent the arm sideways.
        yaw = self.state(data).yaw
        c, s = math.cos(yaw), math.sin(yaw)
        step_xy = np.array([c * vx - s * vy, s * vx + c * vy, 0.0]) * self.control_dt
        aim = self._clamp(self._aim + step_xy)
        # Anti-windup (`MAX_LEAD`): the aim may not run away from the gripper. Without
        # this the arm carries on travelling after its command stops, by however far the
        # aim got ahead while the jaw was blocked.
        # The point the whole bookkeeping is about (R117). `gripperframe` by default, which
        # is also what this body reports as its position; the middle of the finger gap when
        # an orientation task is on, because that is where a grasp closes and it is not the
        # site: measured 25.0 mm away in xy with the jaw open, 13.8 mm closed, 12 mm above
        # the commanded height at the grab height. Moving only the solve's error to that
        # point - and leaving this clamp on the site - made the two references push against
        # each other and the arm folded inside 150 control steps (grasp point up at z 0.293,
        # aim drifted to (0.0585, 0.0083)). So the lead clamp moves with it.
        now = self.control_point(data)
        lead = aim - now
        n = float(np.linalg.norm(lead[:2]))
        # Only while a command is running (R120). The clamp re-anchors the aim to
        # "wherever the point is now, plus `MAX_LEAD` along the residual", which is right
        # for a blocked jaw under a live command and a **ratchet** without one: if anything
        # holds the residual open - an orientation task spending the same joints, say - the
        # aim is re-planted 0.03 m ahead every step and the arm creeps with a zero command.
        # Measured before this: 76 mm of travel over 8 s, commanded to stand still.
        if n > MAX_LEAD and float(np.hypot(vx, vy)) > 1e-9:
            aim[:2] = now[:2] + lead[:2] * (MAX_LEAD / n)
        self._aim = aim
        steps = max(1, round(self.control_dt / model.opt.timestep))
        for _ in range(steps):
            jacp = np.zeros((3, model.nv))
            jacr = np.zeros((3, model.nv)) if self.align_axis is not None else None
            mujoco.mj_jacSite(model, data, jacp, jacr, self._site)
            here = self.control_point(data)
            if jacr is not None:
                # The Jacobian of that point: `v = v_site + w x r` for a point at world
                # offset `r` from the site, which is this cross product per column.
                jacp = jacp + np.cross(jacr.T, here - np.asarray(data.site_xpos[self._site],
                                                                dtype=float)).T
            err = self._aim - here
            # Height: proportional, plus an integral that is the one thing the position
            # servos cannot do for themselves (R111). A servo torque is `kp * (ctrl - qpos)`
            # and the target here is rebuilt from the *measured* angles each substep, so the
            # lead has to pay for holding the arm up before it buys any motion, and the
            # gripper settles below the height it was asked for: measured 1.91 mm with an
            # empty jaw and 2.47-5.27 mm holding a 30 g block, against a 0.04 m grab radius
            # and a 0.02 m table height. The integral is on **z alone** on purpose. Feeding
            # the whole bias torque forward fixes the same sag (0.02 mm and 0.32-0.64 mm)
            # but hands the arm back its full planar authority too, and the shipped rooms
            # are tuned to the carry it has: measured, `conveyor_triage` went 20/20 -> 0/20
            # (its bins sit where this carry puts a block, by its own header) and
            # `build_a_tower` the same, through a faster approach knocking the carried block
            # out of the jaw. Height alone leaves the planar dynamics untouched.
            # Anti-windup on the **contribution**, not on the accumulator (R117). Clamping
            # `z_i` alone was wrong by a factor of the gain: with `Z_I_GAIN` 10 a saturated
            # 0.4 m.s accumulator asks for 4 m of height error. That never showed while the
            # height was reachable - the rooms keep their numbers either way - but ask for
            # an orientation the pose cannot hold and the z error stops converging, the
            # accumulator saturates, and the solve folds the arm: measured, the controlled
            # point climbed to z 0.293 within 150 control steps.
            cap = Z_I_MAX / Z_I_GAIN if Z_I_GAIN > 0 else 0.0
            self._z_i = float(np.clip(self._z_i + err[2] * model.opt.timestep, -cap, cap))
            err[2] = err[2] * Z_GAIN / IK_GAIN + Z_I_GAIN * self._z_i
            j = jacp[:, self._qvel]
            if jacr is not None:
                # Orientation task (R114): bring the opening axis onto `align_axis`. The
                # axis is carried by the gripper, so turning the site at `w` turns the axis
                # at `w x a`; asking for `a` to move towards the target is asking for
                # `w = a x target`, the cross product below. Three more rows, same solve.
                axis = self.opening_axis(data)
                want = np.asarray(self.align_axis, dtype=float)
                want = want / max(float(np.linalg.norm(want)), 1e-9)
                if self.align_rows == 1:
                    # One row, not three (R120). Three rows pin the whole orientation, and
                    # with three position rows that is six constraints on five joints -
                    # measured, the solve could then not even hold the grab height (site z
                    # 0.0422 against the 0.0200 asked). One row asks for a single scalar
                    # instead: `s = axis . want` to zero, so the opening axis only has to
                    # leave `want`'s direction. `ds/dt = (w x axis) . want = w . (axis x
                    # want)`, which is the row below.
                    row = np.cross(axis, want) @ jacr[:, self._qvel]
                    j = np.vstack([j, row])
                    err = np.append(err, -float(np.dot(axis, want)) * ALIGN_GAIN)
                else:
                    j = np.vstack([j, jacr[:, self._qvel]])
                    err = np.concatenate([err, np.cross(axis, want) * ALIGN_GAIN])
            # Damped least squares: J^T (J J^T + lambda^2 I)^-1 e. The damping is what
            # keeps the solve finite near the envelope's edge, where the Jacobian loses
            # rank and an undamped pseudo-inverse would ask for an enormous joint rate.
            dq = j.T @ np.linalg.solve(j @ j.T + DAMPING ** 2 * np.eye(len(err)),
                                       err * IK_GAIN)
            dq[4] += wz            # the wrist roll spins the jaw about its own axis
            target = data.qpos[self._qpos] + dq * model.opt.timestep
            target = np.clip(target, self._range[:, 0], self._range[:, 1])
            if self.latency_steps > 0:
                # A real servo gets its target late (bus plus its own loop). Hold the last
                # `latency_steps` worth and apply the oldest; before the queue fills, hold
                # the home pose rather than inventing a target.
                depth = int(min(self.latency_steps, LATENCY_MAX)) * max(
                    1, round(self.control_dt / model.opt.timestep))
                self._queue.append(target.copy())
                target = self._queue.pop(0) if len(self._queue) > depth else np.asarray(HOME,
                                                                                       float)
            data.ctrl[self._ctrl] = target
            data.ctrl[self._jaw_ctrl] = self.jaw_target
            mujoco.mj_step(model, data)
            if on_substep is not None:
                on_substep(model, data)

    def set_servo_spread(self, model: mujoco.MjModel, factors) -> None:
        """Scale each arm joint's position gain by `factors` (R118): the spread between
        servos of the same part. 1.0 everywhere is the shipped model."""
        f = np.asarray(factors, dtype=float)
        if f.shape != (len(ARM_JOINTS),) or not np.all(f > 0):
            raise ValueError(f"servo spread needs {len(ARM_JOINTS)} positive factors, got {factors!r}")
        for i, gid in enumerate(self._ctrl):
            model.actuator_gainprm[gid, 0] = self._kp[i] * f[i]
            model.actuator_biasprm[gid, 1] = -self._kp[i] * f[i]

    def control_point(self, data: mujoco.MjData) -> np.ndarray:
        """The point `step` drives: the grasp point when an orientation task is on, else the
        gripper site. One function, so the solve, the lead clamp and the aim agree (R117)."""
        if self.align_axis is None:
            return np.asarray(data.site_xpos[self._site], dtype=float)
        return self.grasp_point(data)

    def grasp_point(self, data: mujoco.MjData) -> np.ndarray:
        """Where a grasp closes, in world coordinates: `GRASP_OFFSET` carried by the site.

        Rigid in the site's frame on purpose (see `GRASP_OFFSET`); `live_grasp_point` is the
        measured one, kept so the two can be measured against each other."""
        site = np.asarray(data.site_xpos[self._site], dtype=float)
        rot = np.asarray(data.site_xmat[self._site], dtype=float).reshape(3, 3)
        return site + rot @ np.asarray(GRASP_OFFSET, dtype=float)

    def live_grasp_point(self, data: mujoco.MjData) -> np.ndarray:
        """Middle of the finger gap as the geometry actually has it, jaw angle and all."""
        a = np.mean([data.geom_xpos[g] for g in self._fixed_tips], axis=0)
        b = np.mean([data.geom_xpos[g] for g in self._moving_tips], axis=0)
        return (np.asarray(a, dtype=float) + np.asarray(b, dtype=float)) / 2.0

    def opening_axis(self, data: mujoco.MjData) -> np.ndarray:
        """Unit world direction the jaw opens along: fixed finger tips to moving ones.

        Read from the geometry rather than declared, because it depends on the jaw angle -
        the moving tip sweeps from +0.065 to -0.017 m in the site's own z as the jaw closes.
        """
        a = np.mean([data.geom_xpos[g] for g in self._fixed_tips], axis=0)
        b = np.mean([data.geom_xpos[g] for g in self._moving_tips], axis=0)
        v = np.asarray(b - a, dtype=float)
        n = float(np.linalg.norm(v))
        return v / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])

    def state(self, data: mujoco.MjData, floor_z: float = 0.0) -> BodyState:
        """Where the gripper is, which way it faces, and how fast it is moving.

        `fallen` is always False and the gravity vector is always upright: a bolted arm
        has no posture to lose, so the fall reward and termination never fire for it.
        """
        if self._data is not data:
            self._bind(data)
        model = data.model
        pos = np.array(data.site_xpos[self._site], dtype=float)
        rot = data.site_xmat[self._site].reshape(3, 3)
        yaw = math.atan2(rot[1, 0], rot[0, 0])
        jacp = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jacp, None, self._site)
        vel = jacp @ data.qvel
        c, s = math.cos(-yaw), math.sin(-yaw)
        lin = np.array([c * vel[0] - s * vel[1], s * vel[0] + c * vel[1], vel[2]])
        return BodyState(pos=pos, yaw=yaw, lin_vel=lin,
                         ang_vel=np.zeros(3), gravity=np.array([0.0, 0.0, -1.0]),
                         fallen=False)

    def teleport(self, data: mujoco.MjData, pose: Pose, z: float = 0.0) -> None:
        """Put the gripper at `pose` by solving for it, since the base cannot move.

        Warps (R14) are the only caller, and a tabletop room has none; it is implemented
        so that a room which does use one gets the arm's nearest honest answer rather than
        a silent no-op. The solve is iterated Jacobian steps, not an analytic inverse, so
        a target outside the measured envelope lands as close as the arm reaches.
        """
        if self._data is not data:
            self._bind(data)
        model = data.model
        target = np.array([pose[0], pose[1], max(z + self.work_z, self.work_z)])
        for _ in range(200):
            mujoco.mj_kinematics(model, data)
            jacp = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, jacp, None, self._site)
            err = target - data.site_xpos[self._site]
            if np.linalg.norm(err) < 1e-4:
                break
            j = jacp[:, self._qvel]
            dq = j.T @ np.linalg.solve(j @ j.T + DAMPING ** 2 * np.eye(3), err)
            data.qpos[self._qpos] = np.clip(data.qpos[self._qpos] + dq,
                                            self._range[:, 0], self._range[:, 1])
        data.qvel[self._qvel] = 0.0
        data.ctrl[self._ctrl] = data.qpos[self._qpos]
        mujoco.mj_forward(model, data)
        self._aim = self._clamp(np.array(data.site_xpos[self._site], dtype=float))

    def set_head(self, level: float) -> None:
        """Lower the jaw: 0 the working height, 1 the table (`GRAB_Z`).

        The same channel a walking body pitches its head with, answered the way this body
        can answer it. The shared carrier already drives it - `carry._hold_tip_at` asks for
        the tip to be at the object's height - so a task that lists `head` in its `actions`
        lets this arm reach down to something lying on the table. A task without it leaves
        the jaw at the working height, exactly as before.
        """
        want = WORK_Z + float(np.clip(level, 0.0, 1.0)) * (GRAB_Z - WORK_Z)
        if want != self.work_z:
            # Setpoint change: drop the integrated height error (R111). Carrying it across
            # a 0.10 m step between the working height and the table makes the next command
            # a spike, and measured, that spike broke the weld grab - `conveyor_triage` left
            # its first block 0.035 m from where it picked it up and stalled on the next
            # grab (0/20), while the room's own bins never moved.
            self._z_i = 0.0
        self.work_z = want

    def view_camera(self, model: mujoco.MjModel) -> int:
        """The arm's own wrist camera, which is the only exteroception this body has (R115).

        The vendor asset ships `wrist_cam` on `camera_mount`, and until this was wired the
        adapter answered `None`, so the `camera` observation term could not be used with
        this body at all - and it has no rangefinder mount either. That left proprioception
        as the whole of its non-privileged observation, which means **no** task for this
        body could be observation-clean: a policy with no exteroception cannot find the
        block. Measured at the home pose: the camera sits at (0.182, 0.054, 0.156), 0.04 m
        behind and above the jaw, with fovy 61 deg, looking along (0.382, -0.547, -0.745) -
        down and across the working area in front of the gripper.
        """
        return int(model.camera(PREFIX + "wrist_cam").id)

    def grab_site(self) -> str:
        """The jaw's own site: what the weld grab (R18) attaches an object to.

        The same site `state` reports as the body's position, because for this body the
        end effector *is* the body. A walking body's grab site is its beak tip, out in
        front of a trunk that the puzzle tracks separately.
        """
        return PREFIX + SITE

    def root_body_id(self, model: mujoco.MjModel) -> int:
        """The arm's base body.

        Caution for callers that compare `model.body_rootid`: every body welded to the
        world has rootid 0, so this body cannot be told from the room's static parts that
        way. Sensing that needs "is this the body" must use this id directly or the
        gripper site. The walking bodies have a free joint and so a root of their own,
        which is why the existing sensing code reads rootid.
        """
        return int(model.body(PREFIX + "base").id)

    def command_from_action(self, action: Sequence[float]) -> np.ndarray:
        """Action to command. Unlike a walking body the middle of the range is standing
        still, because there is no gait to keep going: a gripper at rest is a gripper at
        rest, and the solver's "slowest" command (`action[0] = -1`) is full speed
        backwards rather than a stop."""
        a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        return np.array([a[0] * self.cmd_limits[0][1], a[1] * self.cmd_limits[1][1],
                         a[2] * self.cmd_limits[2][1]])

    #: `wz` rolls the jaw about its own approach axis. It does not change where the jaw
    #: points across the table (measured: 0.000 rad of azimuth for 0.2 rad of roll), so it
    #: is not a steering channel - see `holonomic`.

    def check_warnings(self, env) -> list[str]:
        """A room that randomizes its spawn randomizes nothing for a bolted body."""
        spawn = (getattr(env.room, "spawn", None) or {})
        if any(float(v) for v in (spawn.get("random") or [])):
            return ["the body is bolted to the table, so the room's `spawn: random:` "
                    "changes nothing for it; randomize the parts instead"]
        return []

    def history_state(self) -> dict[str, Any]:
        return {"jaw_target": float(self.jaw_target), "base_pose": list(self._base_pose)}

    def load_history(self, data: mujoco.MjData, state: dict[str, Any]) -> None:
        self.jaw_target = float(state.get("jaw_target", 0.0))
