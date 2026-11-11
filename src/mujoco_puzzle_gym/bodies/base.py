"""Body adapter boundary (R2).

Everything outside `bodies/` talks to a walking body only through `BodyAdapter`:
a planar move command (vx, vy, wz) goes in, a `BodyState` comes out. Puzzle,
environment and training code must not name a concrete body, its joints or its
policy files (enforced by tests/test_boundary.py).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Sequence

import mujoco
import numpy as np

Pose = tuple[float, float, float]  # x [m], y [m], yaw [rad] on the floor


@dataclass(frozen=True)
class BodyState:
    pos: np.ndarray       # (3,) trunk position in world [m]
    yaw: float            # heading [rad]
    lin_vel: np.ndarray   # (3,) trunk linear velocity in the yaw frame [m/s] (x forward)
    ang_vel: np.ndarray   # (3,) trunk angular velocity in the body frame [rad/s]
    gravity: np.ndarray   # (3,) unit gravity in the body frame (upright: [0, 0, -1])
    fallen: bool


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

    @abstractmethod
    def spawn(self, spec: mujoco.MjSpec, pose: Pose = (0.0, 0.0, 0.0)) -> mujoco.MjSpec:
        """Add the body to `spec` (a world with a floor) at `pose`; return `spec`."""

    @abstractmethod
    def reset(self, data: mujoco.MjData, pose: Pose = (0.0, 0.0, 0.0)) -> None:
        """Put the body in its standing pose at `pose` and clear controller state."""

    @abstractmethod
    def step(self, data: mujoco.MjData, cmd: Sequence[float]) -> None:
        """Advance one `control_dt` tracking cmd = (vx, vy, wz), clipped to `cmd_limits`."""

    @abstractmethod
    def state(self, data: mujoco.MjData) -> BodyState:
        """Read the body state from `data`."""

    def clip_cmd(self, cmd: Sequence[float]) -> np.ndarray:
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
