"""Object pool (R16): prebuilt free bodies parked off-stage and reused.

A part that throws or hands out objects (cannon shells, R16; dispensers, R28)
cannot add bodies while running: the model, and with it the snapshot size,
must stay fixed. So it builds `size` free bodies up front (default 10) and
reuses them:

- parked: far below the room (`PARK_Z`), collisions off (contype and
  conaffinity 0) and gravity compensated, so it stays put with no contacts;
- `launch`: put it at a position with a velocity, collisions and gravity on;
- `park`: back to its parking spot.
When every body is out, `take` reuses the one out longest.

Snapshot safety: positions and velocities are MuJoCo state, the collision bits
and `body_gravcomp` are model values `snapshot.MODEL_FIELDS` holds, and which
bodies are out since when (`out`) goes in the owning part's `save_state`.

`Impacts` is the shell-hit hook (R5 target, R34 `by: shell`): it finds the
physics steps where a shell starts touching given bodies and the shell's speed
one physics step before.
"""

from __future__ import annotations

from typing import Any

import mujoco
import numpy as np

PARK_Z = -5.0        # [m] below the catch plane (floor.CATCH_DEPTH)
PARK_GAP = 0.25      # [m] between parked bodies (radius <= 0.1)


class Pool:
    """`size` free spheres named `<prefix>/<i>`; parked at `park_xy` + i * PARK_GAP along x."""

    def __init__(self, prefix: str, size: int, radius: float, mass: float,
                 rgba=(0.25, 0.25, 0.25, 1.0), park_xy=(0.0, 0.0)):
        self.prefix, self.size = prefix, int(size)
        self.radius, self.mass, self.rgba = float(radius), float(mass), tuple(rgba)
        self.park_xy = np.asarray(park_xy, float)
        self.out: dict[int, float] = {}         # index -> launch time

    def names(self) -> list[str]:
        return [f"{self.prefix}/{i}" for i in range(self.size)]

    def park_pos(self, i: int) -> np.ndarray:
        return np.array([self.park_xy[0] + i * PARK_GAP, self.park_xy[1], PARK_Z])

    # ---- scene ------------------------------------------------------------------
    def build(self, spec: mujoco.MjSpec) -> None:
        for i, name in enumerate(self.names()):
            b = spec.worldbody.add_body(name=name, pos=self.park_pos(i).tolist(), gravcomp=1.0)
            b.add_freejoint(name=name + "/free")
            # Built colliding and parked by `reset`: MuJoCo's broad phase also filters by
            # the body's collision bits, which compile and `mj_setConst` (the walking
            # body's controller calls it once) derive from the geoms' bits then.
            b.add_geom(name=name + "/geom", type=mujoco.mjtGeom.mjGEOM_SPHERE,
                       size=[self.radius, 0, 0], mass=self.mass, rgba=self.rgba)

    def bind(self, model: mujoco.MjModel) -> None:
        self.bids = [model.body(n).id for n in self.names()]
        self.gids = [model.geom(n + "/geom").id for n in self.names()]
        self.qadr = [int(model.jnt_qposadr[model.body_jntadr[b]]) for b in self.bids]
        self.vadr = [int(model.jnt_dofadr[model.body_jntadr[b]]) for b in self.bids]
        self.index = {g: i for i, g in enumerate(self.gids)}

    # ---- runtime ----------------------------------------------------------------
    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        for i in range(self.size):
            self.park(model, data, i)
        self.out = {}

    def take(self) -> int:
        """A parked body's index, or the one out longest."""
        for i in range(self.size):
            if i not in self.out:
                return i
        return min(self.out, key=self.out.get)

    def launch(self, model, data, i: int, pos, vel, now: float) -> None:
        qa, va = self.qadr[i], self.vadr[i]
        data.qpos[qa:qa + 3] = pos
        data.qpos[qa + 3:qa + 7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[va:va + 3] = vel
        data.qvel[va + 3:va + 6] = 0.0
        model.geom_contype[self.gids[i]] = model.geom_conaffinity[self.gids[i]] = 1
        model.body_gravcomp[self.bids[i]] = 0.0
        self.out[i] = float(now)

    def park(self, model, data, i: int) -> None:
        qa, va = self.qadr[i], self.vadr[i]
        data.qpos[qa:qa + 3] = self.park_pos(i)
        data.qpos[qa + 3:qa + 7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[va:va + 6] = 0.0
        model.geom_contype[self.gids[i]] = model.geom_conaffinity[self.gids[i]] = 0
        model.body_gravcomp[self.bids[i]] = 1.0
        self.out.pop(i, None)

    def expire(self, model, data, now: float, lifetime: float) -> None:
        """Park bodies out longer than `lifetime` seconds."""
        for i, t in list(self.out.items()):
            if now - t >= lifetime:
                self.park(model, data, i)

    def pos(self, data, i: int) -> np.ndarray:
        return data.qpos[self.qadr[i]:self.qadr[i] + 3].copy()

    def vel(self, data, i: int) -> np.ndarray:
        return data.qvel[self.vadr[i]:self.vadr[i] + 3].copy()

    def save(self) -> dict[str, Any]:
        return {"out": dict(self.out)}

    def load(self, st: dict[str, Any]) -> None:
        self.out = dict(st["out"])


class Impacts:
    """Shell contact starts on a set of bodies (R5 target; the R34 `by: shell` hook).

    Call `substep` after every physics step (`Part.substep`). A shell (a body
    of kind "shell" in `World.kinds`) that starts touching one of `bodies`
    adds its speed from the physics step before to `events`; it counts again
    only after the contact ends. The owner reads and clears `events` in `update`.
    """

    def __init__(self):
        self.touching: set[int] = set()
        self.speed: dict[int, float] = {}
        self.events: list[float] = []

    def substep(self, model, data, world, bodies: set[int]) -> None:
        shells = [rid for rid, k in world.kinds.items() if k == "shell"]
        if not shells:
            return
        shell_set = set(shells)
        now: set[int] = set()
        for i in range(data.ncon):
            c = data.contact[i]
            b1, b2 = int(model.geom_bodyid[c.geom1]), int(model.geom_bodyid[c.geom2])
            r1, r2 = int(model.body_rootid[b1]), int(model.body_rootid[b2])
            if b1 in bodies and r2 in shell_set:
                now.add(r2)
            elif b2 in bodies and r1 in shell_set:
                now.add(r1)
        for rid in now - self.touching:
            self.events.append(self.speed.get(rid, 0.0))
        self.touching = now
        for rid in shells:
            va = int(model.jnt_dofadr[model.body_jntadr[rid]])
            self.speed[rid] = float(np.linalg.norm(data.qvel[va:va + 3]))
