"""Moving platform and elevator (R11): a slab that follows a path.

`path: [[dx, dy, z], ...]` (offsets from `pos`; z = top face height above the
floor) is a polyline the slab's top centre follows at `speed` [m/s]:
- `mode: loop` runs the closed loop (last point back to the first);
- `pingpong` runs to the end and back;
- `once` runs to the end and stays.
`run: always` moves all the time, `while_signal` only while its `in` channels
are on, `on_signal` starts one trip on each rising edge (once: to the end;
pingpong: to the other end; loop: one lap). `enable` off holds it where it is.
The elevator is a preset with a vertical path.

The slab sits on slide joints (one per axis the path moves along) driven by
position actuators (kp 2e4, critically damped) with gravity compensation, and
does not collide with the static world. The actuator targets move along the
path at `speed`, ramped every physics step (`substep`). Not mocap: a mocap slab moving 0.3 m/s for
3 s left a cube on it 94 cm behind (contacts get no velocity from mocap); the
joint slab carried it with 0.0 cm drift (2026-11-12, OQ1 measurement).

Offsets from `pos` let presets carry a path (the elevator: [[0, 0, 0.02], [0,
0, 0.3]]) and `random` move the slab and its path together. Planning: the cells the slab sweeps are blocked (riding is outside
the walking policy's training, plan Risks).
"""

from __future__ import annotations

import mujoco
import numpy as np

from ... import markers
from ...schema import Field, SchemaError
from ..base import Part

MODES = ("loop", "pingpong", "once")
RUNS = ("always", "while_signal", "on_signal")
KP = 2.0e4           # N/m; with dampratio 1 a loaded slab tracks the moving target
MASS = 2.0           # kg, gravity-compensated


def parse_path(owner: str, raw) -> np.ndarray:
    ok = isinstance(raw, (list, tuple)) and len(raw) >= 2 and all(
        isinstance(p, (list, tuple)) and len(p) == 3
        and all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in p) for p in raw)
    if not ok:
        raise SchemaError(f"{owner}: path must be a list of 2 or more [x, y, z] points, got {raw!r}")
    path = np.asarray(raw, dtype=float)
    if np.any(np.linalg.norm(np.diff(path, axis=0), axis=1) < 1e-6):
        raise SchemaError(f"{owner}: path has two equal points in a row")
    if path[:, 2].min() < 0.0 or path[:, 2].max() > 1.5:
        raise SchemaError(f"{owner}: path heights z = {path[:, 2].tolist()} m are out of range [0, 1.5]")
    return path


class Platform(Part):
    marked = True
    marker_anchor = "/slab"
    marker_faces = markers.FACES_FLAT
    stage = "device"
    state_keys = ("s", "dir", "left", "was_on", "_from", "_to", "_t0")
    fields = (
        Field("path", "any", doc="[[dx, dy, z], ...] from `pos`: points the top centre follows (2 or more)"),
        Field("size", "vec", [0.3, 0.3], length=2, unit="m", min=0.05, max=2.0, doc="slab size [x, y]"),
        Field("thickness", "float", 0.03, unit="m", min=0.01, max=0.2),
        Field("speed", "float", 0.1, unit="m/s", min=0.01, max=1.0, runtime=True,
              doc="speed along the path"),
        Field("mode", "enum", "pingpong", choices=MODES,
              doc="loop: closed loop; pingpong: back and forth; once: to the end and stop"),
        Field("run", "enum", "always", choices=RUNS,
              doc="always; while_signal: while `in` is on; on_signal: one trip per rising edge"),
    )

    def marker_face_size(self, face):
        return float(min(self.cfg["size"]))

    def __init__(self, name, cfg):
        owner = f"part {name!r}"
        super().__init__(name, cfg)
        c = self.cfg
        self.path = path = parse_path(owner, c["path"]) + [*self.base_xy, 0.0]   # room coordinates
        self.mode, self.run = c["mode"], c["run"]
        if self.run != "always" and not self.inputs:
            raise SchemaError(f"{owner}: run: {self.run} needs `in` channels")
        self.half = np.asarray(c["size"], float) / 2
        self.thick = float(c["thickness"])
        pts = np.vstack([path, path[:1]]) if self.mode == "loop" else path
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        self._pts, self._cum = pts, np.concatenate([[0.0], np.cumsum(seg)])
        self.length = float(self._cum[-1])
        self.axes = [a for a in range(3) if np.ptp(path[:, a]) > 1e-9]   # axes with a joint
        # Body origin: [pos, 0]; on axes without a joint the slab sits at the path's value.
        self.origin = np.array([*self.base_xy, 0.0])
        for a in range(3):
            if a not in self.axes:
                self.origin[a] = path[0, a]
        self.shift = np.zeros(3)          # random placement moves the path too
        if self.run == "on_signal":
            self.stateful = True
        self.s, self.dir, self.left, self.was_on = 0.0, 1, 0.0, False
        self._from = self._to = self.path[0] - self.origin
        self._t0 = 0.0

    # ---- scene ------------------------------------------------------------------
    def build(self, spec):
        body = spec.worldbody.add_body(name=self.body_name, pos=self.origin.tolist(), gravcomp=1.0)
        for a in self.axes:
            axis = [0.0, 0.0, 0.0]
            axis[a] = 1.0
            j = body.add_joint(name=f"{self.body_name}/{'xyz'[a]}", type=mujoco.mjtJoint.mjJNT_SLIDE,
                               axis=axis, damping=1.0)
            act = spec.add_actuator(name=f"{self.body_name}/{'xyz'[a]}_motor", target=j.name,
                                    trntype=mujoco.mjtTrn.mjTRN_JOINT)
            act.set_to_position(kp=KP, dampratio=1.0)
        body.add_geom(name=self.body_name + "/slab", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*self.half, self.thick / 2], pos=[0, 0, -self.thick / 2],
                      mass=MASS, rgba=(0.35, 0.55, 0.75, 1.0))
        spec.add_exclude(bodyname1="world", bodyname2=self.body_name)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._j = []
        for a in self.axes:
            j = model.joint(f"{self.body_name}/{'xyz'[a]}")
            self._j.append((a, int(j.qposadr[0]), int(j.dofadr[0]),
                            model.actuator(f"{self.body_name}/{'xyz'[a]}_motor").id))

    def place(self, model, xy):
        super().place(model, xy)
        self.shift = np.array([*(self.xy - self.base_xy), 0.0])
        model.body_pos[self._bid] = self.origin + self.shift

    # ---- path -------------------------------------------------------------------
    def point(self, s: float) -> np.ndarray:
        """Top centre (room coordinates, without the random shift) at path distance `s`."""
        i = int(np.clip(np.searchsorted(self._cum, s, side="right") - 1, 0, len(self._cum) - 2))
        seg = self._cum[i + 1] - self._cum[i]
        t = (s - self._cum[i]) / seg
        return self._pts[i] + t * (self._pts[i + 1] - self._pts[i])

    def top(self, data) -> np.ndarray:
        """Top centre now (from the joints)."""
        return data.xpos[self._bid].copy()

    def _drive(self, model, data) -> None:
        """New target for this body step: `substep` ramps the actuators to it."""
        self._from = self._to
        self._to = self.point(self.s) - self.origin
        self._t0 = data.time
        self.substep(model, data)

    def substep(self, model, data):
        # Ramping the targets every physics step, not once per body step: stepped targets
        # jerk the slab every 20 ms and a cube on it crept 3 cm in 3 s at 0.3 m/s.
        frac = min(1.0, (data.time - self._t0 + model.opt.timestep) / self.bus.dt)
        local = self._from + frac * (self._to - self._from)
        for a, _, _, act in self._j:
            data.ctrl[act] = local[a]

    def reset(self, model, data):
        self.s, self.dir, self.left, self.was_on = 0.0, 1, 0.0, False
        local = self.path[0] - self.origin
        self._from = self._to = local
        self._t0 = data.time
        for a, qa, va, act in self._j:
            data.qpos[qa] = data.ctrl[act] = local[a]
            data.qvel[va] = 0.0

    def _trip(self) -> float:
        """Distance of one `on_signal` trip from here."""
        if self.mode == "loop":
            return self.length
        if self.mode == "once":
            return self.length - self.s
        ahead = self.length - self.s if self.dir > 0 else self.s
        if ahead < 1e-9:                         # at the end it faces: go back
            self.dir = -self.dir
            ahead = self.length - ahead
        return ahead

    def update(self, model, data, body):
        if not self.enabled():
            return
        on = self.inputs_on() if self.inputs else False
        if self.run == "always":
            go = True
        elif self.run == "while_signal":
            go = on
        else:
            if on and not self.was_on:
                self.left = self._trip()
            go = self.left > 1e-9
        self.was_on = on
        if not go:
            return
        step = float(self.live["speed"]) * self.bus.dt
        if self.run == "on_signal":
            step = min(step, self.left)
            self.left -= step
        if self.mode == "loop":
            self.s = (self.s + step) % self.length
        else:
            self.s += self.dir * step
            if self.mode == "once":
                self.s = min(self.s, self.length)
            elif self.s > self.length:
                self.s, self.dir = 2 * self.length - self.s, -1
            elif self.s < 0.0:
                self.s, self.dir = -self.s, 1
        self._drive(model, data)

    def state(self):
        return float(np.clip(self.s / self.length, 0.0, 1.0))

    # ---- planning ---------------------------------------------------------------
    def _swept(self, grid):
        lo = self.path[:, :2].min(axis=0) + self.shift[:2] - self.half
        hi = self.path[:, :2].max(axis=0) + self.shift[:2] + self.half
        return grid.rect((lo + hi) / 2, tuple((hi - lo) / 2))

    def hole_mask(self, grid, get):
        return self._swept(grid)

    def hole_now(self, grid):
        return self._swept(grid)

    def target_mask(self, grid):
        return self._swept(grid)

