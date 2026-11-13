"""Body adapter boundary (R2).

Everything outside `bodies/` talks to a walking body only through `BodyAdapter`:
a planar move command (vx, vy, wz) goes in, a `BodyState` comes out. Puzzle,
environment and training code must not name a concrete body, its joints or its
policy files (enforced by tests/test_boundary.py).
"""

from __future__ import annotations

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


@dataclass(frozen=True)
class Capabilities:
    """What a body can do (R2b): capability names (e.g. `walk`, `beak_grab`) and
    numeric limits (e.g. `max_lift_mass` [kg]). The body adapter sets both; parts
    state what they need with `requires_body: [...]` and `mpg check` compares."""

    names: frozenset[str] = frozenset()
    values: Mapping[str, float] = field(default_factory=dict)

    def missing(self, required: Iterable[str]) -> list[str]:
        """Required names this body lacks, in the order given."""
        return [r for r in required if r not in self.names]


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
