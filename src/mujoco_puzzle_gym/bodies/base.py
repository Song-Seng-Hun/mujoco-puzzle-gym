"""Body adapter boundary (R2).

Everything outside `bodies/` talks to a walking body only through `BodyAdapter`:
a planar move command (vx, vy, wz) goes in, a `BodyState` comes out. Puzzle,
environment and training code must not name a concrete body, its joints or its
policy files (enforced by tests/test_boundary.py).
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Mapping, Sequence

import mujoco
import numpy as np

Pose = tuple[float, float, float]  # x [m], y [m], yaw [rad] on the floor
#: Addon modes (R30), in the order of signal `body.addon` (0 none, 1 roller) and obs `addon`.
ADDON_MODES = ("none", "roller")
#: Called after every physics step inside `BodyAdapter.step` (R5 `substep` hook).
SubstepFn = Callable[[mujoco.MjModel, mujoco.MjData], None]


@dataclass(frozen=True)
class BodyState:
    pos: np.ndarray       # (3,) trunk position in world [m]
    yaw: float            # heading [rad]
    lin_vel: np.ndarray   # (3,) trunk linear velocity in the yaw frame [m/s] (x forward)
    ang_vel: np.ndarray   # (3,) trunk angular velocity in the body frame [rad/s]
    gravity: np.ndarray   # (3,) unit gravity in the body frame (upright: [0, 0, -1])
    fallen: bool


#: What a numeric limit means when the body never declared it: nothing, not everything.
UNDECLARED_LIMIT = 0.0

#: Capability names and limit keys renamed in R67, old name -> current one. The first
#: body had a beak, so the vocabulary it created said `beak`: a jaw, a hand and a
#: 29-joint humanoid all had to declare `beak_grab` to be allowed to pick anything up,
#: which is the first body's anatomy leaking into shared vocabulary - the same leak R2
#: exists to stop. Renaming would break every room, preset and plugin part written
#: against the old names, so the old names keep working: a requested name and a declared
#: limit key are both read through `canonical` before anything is compared.
ALIASES = {"beak_grab": "grab", "beak_height": "reach_height"}


def canonical(name: str) -> str:
    """The current name for a capability or limit, whichever name the caller used."""
    return ALIASES.get(name, name)


def declared_limit(values: Mapping[str, float] | None, name: str) -> float:
    """A body's numeric capability limit, or `UNDECLARED_LIMIT` when it declares none.

    Callers ask for limits a body may not have: `room.grid` and `check.map_steps` want
    `max_step` and `max_slope`, the beak's grab wants `max_lift_mass`. Reading a missing
    limit as unlimited is the dangerous direction - it let `mpg check` call a room
    solvable that the body cannot walk - and it was only ever safe because the one body
    that existed declared all of them. 0 refuses instead, which reaches the user as a
    reason they can read. Flat ground is unaffected: the terrain checks block a step or
    ramp only when it is *greater* than the limit.
    """
    if values is None:
        return UNDECLARED_LIMIT
    key = canonical(name)
    for k in (key, *(old for old, new in ALIASES.items() if new == key)):
        try:
            return float(values[k])                     # type: ignore[index]
        except (KeyError, TypeError):
            continue
    return UNDECLARED_LIMIT


@dataclass(frozen=True)
class Capabilities:
    """What a body can do (R2b): capability names (e.g. `walk`, `grab`) and
    numeric limits (e.g. `max_lift_mass` [kg]). The body adapter sets both; parts
    state what they need with `requires_body: [...]` and `mpg check` compares."""

    names: frozenset[str] = frozenset()
    values: Mapping[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Store declared names under their current spelling, so a body written against
        the old vocabulary (a plugin's, say) is compared on equal terms with this one."""
        object.__setattr__(self, "names", frozenset(canonical(n) for n in self.names))

    def missing(self, required: Iterable[str]) -> list[str]:
        """Required names this body lacks, in the order given.

        A request is read through `canonical`, so a room or plugin part asking for an
        old name (`beak_grab`) is answered by a body declaring the current one (`grab`).
        The name reported back is the one the caller used, since that is the string they
        can find in their own file.
        """
        return [r for r in required if canonical(r) not in self.names]

    def limit(self, name: str) -> float:
        """This body's `name` limit, 0 when undeclared (see `declared_limit`)."""
        return declared_limit(self.values, name)


class BodyAdapter(ABC):
    """A body that walks on command.

    Lifecycle: `spawn` adds the body to a scene spec; after the caller compiles
    the spec and makes `MjData`, `reset` places the body, and each `step`
    advances the simulation by one `control_dt` while tracking the command.
    """

    #: Command limits ((vx_min, vx_max), (vy_min, vy_max), (wz_min, wz_max)).
    cmd_limits: tuple[tuple[float, float], tuple[float, float], tuple[float, float]]
    #: Seconds of simulated time advanced by one `step` call.
    control_dt: float
    #: Physics timestep the body needs (`model.opt.timestep`).
    timestep: float
    #: Radius [m] of a circle around the body centre that covers its footprint
    #: (clearance for path planning in `mpg check`).
    footprint_radius: float = 0.15
    #: When False, `clip_cmd` passes commands through unclipped. Only for
    #: deliberately broken tasks (`env.command_scale` > 1, `mpg diagnose` tests).
    clip_commands: bool = True
    #: What this body can do (R2b); parts' `requires_body` is checked against it.
    capabilities: Capabilities = Capabilities()
    #: Trunk height [m] above the floor below which the body counts as fallen (R10 uses
    #: it for holes: over a hole the env passes `floor_z = rim - fall_height`).
    fall_height: float = 0.07
    #: Heading error [rad] above which this body should stop and turn on the spot instead
    #: of correcting its heading on the move (read by the scripted solver's steering, R47).
    #: Infinity, the default, means never: correct while moving, whatever the error.
    #:
    #: It belongs to the body because the shared steering cannot know the answer. A body
    #: turns through an arc whose radius is its forward speed over its turn rate, and the
    #: two bodies here differ by three times at the middle of their command range and five
    #: at the top. The one with the tight arc must keep moving: its rooms are timed, a
    #: stand costs about a second per correction, and on wheels or with a load a stand
    #: coasts to a halt, slackens a rope or loses a lure. The one with the wide arc cannot
    #: acquire a small target at all without standing - measured, it circled a 0.6 m floor
    #: button for a whole 40 s episode, passing within 0.07 m of it twice a lap. Neither
    #: answer is right for both, and no geometric rule in the solver separated them: four
    #: were tried (always stand, a heading-error threshold, target closer than the arc, and
    #: the Dubins reachability condition) and each one broke the other body's rooms.
    stand_to_turn_above: float = math.inf
    #: How far this body takes its grab point from something it has just put down [m].
    #:
    #: A walking body backs off a third of a metre, which is about its own length: it has
    #: to get its trunk and its beak clear of what it placed before it turns. A body whose
    #: grab point is its whole self needs only to clear the object - and cannot do more,
    #: because its base never moves and 0.35 m from a placed object can be outside its
    #: envelope entirely. Measured: with the walking number the arm's place step never
    #: ended, the carrier circling in its "away" phase for the rest of the episode after
    #: the block was already resting where it was asked for (0.033 m from the patch centre
    #: against a 0.04 m tolerance). None keeps the shared default (`carry.AWAY`).
    place_clearance: float | None = None
    #: How far out from a handle or a wall plate this body stands to approach it [m].
    #:
    #: The same mistake as `place_clearance`, at the other end of the move: a walking body
    #: goes to a point 0.3 m in front of what it is reaching for and then closes in, which
    #: is about its own length. For a body whose grab point is its whole self, 0.3 m from
    #: the handle is a third of its entire reach away and lands wherever that happens to be.
    #: Measured on the arm: the approach point for a block at (-0.03, -0.12) came out at
    #: (-0.03, +0.202), on the far side of the block and inside the room's receptacle box
    #: (x +-0.12, y 0.02-0.26), where the jaw was trapped against the box and could neither
    #: descend to grab height nor travel - a stall that read as an inverse-kinematics
    #: failure until the tip was driven at the same handle directly and reached it in 30
    #: control steps. None keeps the shared default (`carry.APPROACH`).
    approach_clearance: float | None = None

    #: The command that means "stay where you are", as an action (R78).
    #:
    #: `(-1, 0, 0)` for a walking body, because this repository's convention is that
    #: `action[0] = -1` is the slowest the body goes. On a body whose action maps
    #: symmetrically onto a signed velocity - a fixed-base arm, whose holonomic steering
    #: needs that symmetry to drive backwards along its own facing - the same vector is
    #: full speed *backwards*, so the shared solver and carrier told it to drive away
    #: every time they meant for it to wait. Measured on the arm: at the moment it
    #: released a block into a box, the "stand still" action carried the block 0.066 m
    #: further and lifted it to 0.077 m, out of the box. Third of this shape, after
    #: `SLOW` in the carrier and the humanoid's `command_from_action`.
    stand_action: tuple[float, float, float] = (-1.0, 0.0, 0.0)
    #: True when this body can move in any direction without turning first, so steering it
    #: means picking a direction rather than a heading and a speed (R61).
    #:
    #: A walking body is not holonomic in this sense even though its command has a lateral
    #: channel: it walks forward and turns, and the shared steering drives it that way. A
    #: fixed-base arm is the opposite - measured on the arm in `bodies/so101.py`, the only
    #: joint that changes where its gripper points is the shoulder pan, which is also the
    #: joint that decides where the gripper *is*, so the jaw always faces radially outward
    #: from the base and its heading cannot be commanded at all. Steering it with "turn to
    #: face the target, then drive forward" can only ever drive radially outward; the
    #: lateral channel is the only way to reach anything else.
    holonomic: bool = False

    @abstractmethod
    def spawn(self, spec: mujoco.MjSpec, pose: Pose = (0.0, 0.0, 0.0)) -> mujoco.MjSpec:
        """Add the body to `spec` (a world with a floor) at `pose`; return `spec`."""

    @abstractmethod
    def reset(self, data: mujoco.MjData, pose: Pose = (0.0, 0.0, 0.0)) -> None:
        """Put the body in its standing pose at `pose` and clear controller state."""

    @abstractmethod
    def step(self, data: mujoco.MjData, cmd: Sequence[float],
             on_substep: SubstepFn | None = None) -> None:
        """Advance one `control_dt` tracking cmd = (vx, vy, wz), clipped to `cmd_limits`.

        `on_substep(model, data)`, when given, runs after every physics step
        (`mj_step`) inside it; the env passes the parts' `substep` hooks there.
        """

    @abstractmethod
    def state(self, data: mujoco.MjData, floor_z: float = 0.0) -> BodyState:
        """Read the body state from `data`. `fallen` measures the trunk height from
        `floor_z`, the top of the floor under the body (R10 tile floors)."""

    @abstractmethod
    def teleport(self, data: mujoco.MjData, pose: Pose, z: float = 0.0) -> None:
        """Put the body standing at `pose` (floor height `z`), with zero velocity and
        the walking controller's history cleared (warps, R14)."""

    def kick(self, data: mujoco.MjData, velocity: Sequence[float]) -> None:
        """Set the body's linear velocity (world frame, m/s) as it is; joints, posture and
        the walking controller's history stay (springs, R15). Default: the root free
        joint's linear velocity."""
        m = data.model
        va = int(m.jnt_dofadr[m.body_jntadr[self.root_body_id(m)]])
        data.qvel[va:va + 3] = np.asarray(velocity, dtype=float)

    def shift(self, data: mujoco.MjData, delta: Sequence[float]) -> None:
        """Move the whole body by `delta` (world, m) keeping its velocity, posture and
        controller history (conveyors, R12). Default: the root free joint's position."""
        m = data.model
        qa = int(m.jnt_qposadr[m.body_jntadr[self.root_body_id(m)]])
        data.qpos[qa:qa + 3] += np.asarray(delta, dtype=float)

    @abstractmethod
    def root_body_id(self, model: mujoco.MjModel) -> int:
        """Id of the body's root (free-floating) MuJoCo body; sensors use it to tell
        the walking body from objects (`body_rootid`)."""

    def total_mass(self, model: mujoco.MjModel) -> float:
        """Mass [kg] of the whole body: every MuJoCo body under its root.

        Parts whose numbers are forces or masses read it to say whether their setting
        means anything for the body in the scene (`check_warnings`): a push sized for a
        sub-kilogram body does nothing to one twenty times heavier, and a pressure plate
        whose threshold is below the body's own mass is tripped by standing on it."""
        root = self.root_body_id(model)
        return float(sum(model.body_mass[i] for i in range(model.nbody)
                         if model.body_rootid[i] == root))

    @abstractmethod
    def history_state(self) -> dict[str, Any]:
        """Controller memory outside `MjData` (last action, command queue, actuator
        state) for checkpoint snapshots (R6). Must be safe to keep and reload."""

    @abstractmethod
    def load_history(self, data: mujoco.MjData, state: dict[str, Any]) -> None:
        """Restore `history_state` output; call after the MjData state is restored."""

    # ---- addons (R30) ------------------------------------------------------
    #: Addons the scene was built with (`use_addons`); the current mode is `addon`.
    addons: frozenset[str] = frozenset()
    addon: str = "none"
    #: True while a one-shot crouch runs (the fall check then reads the tilt only).
    crouching: bool = False

    def use_addons(self, names: Iterable[str]) -> None:
        """Build the addons `names` into the body (call before `spawn`): a room with an
        addon station needs its body parts in the model from the start (R30)."""
        if frozenset(names):
            raise ValueError(f"{type(self).__name__} has no addons, asked for {sorted(names)}")

    def set_addon(self, data: mujoco.MjData, mode: str, keep_pose: bool = False) -> float:
        """Switch to addon `mode` ("none" = on its own feet); see the body's docs."""
        if mode != "none":
            raise ValueError(f"{type(self).__name__} has no addon {mode!r}")
        return 0.0

    def crouch(self, data: mujoco.MjData) -> bool:
        """Start the addon's one-shot crouch if allowed now (R30); False when ignored."""
        return False

    def wheel_dofs(self, model: mujoco.MjModel) -> list[int]:
        """DOF indices of the addon wheels' joints (brake strip, R30); empty without wheels."""
        return []

    # ---- tow hitch (R31) ----------------------------------------------------
    def tow_site(self, spec: mujoco.MjSpec | None = None) -> str | None:
        """Name of the body's tow-rope hitch site in the compiled scene, or None when the
        body has none. With `spec` (after `spawn`) the site is added to it first (once):
        only rooms with a rope get it, so other rooms build as before."""
        return None

    # ---- beak (R18, R19) ---------------------------------------------------
    def grab_site(self) -> str | None:
        """Name of the beak-tip site in the compiled scene (weld grab, wall buttons),
        or None when the body cannot grab."""
        return None

    def set_head(self, level: float) -> None:
        """Lower the head: `level` 0 = level (standing pose) .. 1 = lowest the walker
        takes while walking. Held until changed; `reset` sets it back to 0. Bodies
        without a head command ignore it."""

    # ---- sensors (R39, R27) ------------------------------------------------
    def view_camera(self, model: mujoco.MjModel) -> int | None:
        """Id of the body's eye camera in the compiled scene (MuJoCo convention:
        looks along its -z, +y up), or None if the body has no camera."""
        return None

    def view_frame(self, model: mujoco.MjModel, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
        """World position (3,) and rotation (3, 3) of the eye camera (columns: camera
        x, y, z axes; the view direction is -z)."""
        cam = self.view_camera(model)
        if cam is None:
            raise NotImplementedError(f"{type(self).__name__} has no eye camera")
        return data.cam_xpos[cam].copy(), data.cam_xmat[cam].reshape(3, 3).copy()

    def add_rangefinders(self, spec: mujoco.MjSpec, yaws: Sequence[float], prefix: str) -> list[str]:
        """Add one MuJoCo `rangefinder` sensor per yaw offset [rad, + = left] to `spec`
        (after `spawn`): rays from the head, level when the body stands, fanned
        around its heading. Returns the sensor names (`prefix` + index)."""
        raise NotImplementedError(f"{type(self).__name__} has no rangefinder mount")

    def command_from_action(self, action: Sequence[float]) -> np.ndarray:
        """Map a normalised action in [-1, 1]^3 to a command (vx, vy, wz).

        Convention every body keeps: action[0] = forward speed (-1 = slowest or
        stop, +1 = fastest forward), action[1] = lateral (+ = left), action[2] =
        turn rate (+ = counter-clockwise, |1| = fastest). Default: linear onto
        `cmd_limits`. Bodies whose walker ignores parts of that range (dead
        zones) override this so the whole action range does something.
        """
        a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        lo = np.array([lim[0] for lim in self.cmd_limits])
        hi = np.array([lim[1] for lim in self.cmd_limits])
        return lo + (a + 1.0) / 2.0 * (hi - lo)

    def clip_cmd(self, cmd: Sequence[float]) -> np.ndarray:
        if not self.clip_commands:
            return np.asarray(cmd, dtype=np.float64)
        lo = np.array([lim[0] for lim in self.cmd_limits])
        hi = np.array([lim[1] for lim in self.cmd_limits])
        return np.clip(np.asarray(cmd, dtype=np.float64), lo, hi)


def flat_floor_spec() -> mujoco.MjSpec:
    """An empty world with a flat floor (plane, MuJoCo default friction) and a light."""
    spec = mujoco.MjSpec()
    spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05])
    spec.worldbody.add_light(pos=[0, 0, 3.5], dir=[0, 0, -1],
                             type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
    return spec


def yaw_from_quat(q: np.ndarray) -> float:
    w, x, y, z = q
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def rotate_inverse(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate `v` by the inverse of unit quaternion `q` = [w, x, y, z]."""
    w, xyz = q[0], q[1:4]
    t = 2.0 * np.cross(xyz, v)
    return v - w * t + np.cross(xyz, t)
