"""Cannon (R16): fires balls (shells) from a pool.

Firing: with `fire: auto` it fires a burst of `burst` shells (`BURST_GAP`
apart) every `period` seconds while active (trap base: its `in` channels on);
with `fire: edge` (preset `trap/cannon_turret`) one burst on each rising edge
of its `in` channels. Each shell leaves the muzzle (`height` above `pos`) at
`power` [m/s] toward the aim, turned by a random angle inside a cone of
`spread` degrees, and is parked again after `lifetime` seconds.

Aim (`aim`):
- `fixed`: heading `yaw` + `aim_yaw`, elevation `aim_pitch` (deg);
- `at_body`: at the walking body's trunk where it is when firing (the low
  ballistic arc; 45 deg, the farthest, when out of reach);
- `lead`: at where the trunk will be after the flight time at its velocity now;
- `signal`: like fixed, with `aim_yaw` / `aim_pitch` meant to follow channels
  through `bind` (R13); the ballistic check below reads the bind ranges.

Shells are bodies of kind "shell" from a `Pool` (`pool` bodies, default 10;
the longest-flying one is reused when all are out). Buttons, plates and zones
never count shells (`TRIGGERS`); targets (R5) and R34 `by: shell` do
(`pool.Impacts`). `shell_mass` is fixed when the room is built: MuJoCo keeps
the mass matrix of a lone free body from compile time (`mj_setConst`), so a
shell whose `body_mass` changed later falls at mass_new / mass_old x g
(measured 2026-11-12: 39 m/s^2 for 0.1 -> 0.4 kg).

Default strength 0.2 kg at 5 m/s (1.0 N s): in the 2026-11-12 measurement
(plan step 12) one such shell knocked the standing body over 6 times out of 6
from the side and the front (4 m/s: 4 of 6); 0.5 N s or less never did.

Outputs: `<name>.fired` is 1 in the body step a shell left; `<name>.hits`
counts shells that touched the walking body (once per shell, `substep`).

Ballistic check (`mpg check` stage `parts`, run by each target): with a
cannon aiming by `signal` and a target in the room, the shell path (gravity,
no drag, from the muzzle) must hit the target's board at its `min_speed` or
faster for at least one aim in the channel ranges: a bound value takes its
`map` output range (per channel value when the source part lists them in
`channel_cells(channel)`, e.g. lever detents, R28; None: continuous), an unbound one its value.
With `spread` > 0 the whole cone (centre and 8 edge directions) must hit.
"""

from __future__ import annotations

import math
import zlib

import mujoco
import numpy as np

from ...layout import Segment
from ...schema import Field, SchemaError
from ..pool import Pool
from .base import Trap, yaw_quat

BURST_GAP = 0.15     # [s] between shells of one burst
BASE = 0.06          # [m] half size of the base block
MUZZLE = 0.13        # [m] from the muzzle centre to where a shell starts (plus its radius)
G = 9.81
AIM_STEP = 0.25      # [deg] aim grid of the ballistic check
CONE = 8             # edge directions of the spread cone in the check


def low_arc(dist: float, dz: float, speed: float) -> float | None:
    """Elevation [rad] of the low arc that reaches (dist, dz) at `speed`; None if out of reach."""
    v2 = speed * speed
    disc = v2 * v2 - G * (G * dist * dist + 2 * dz * v2)
    if disc < 0 or dist < 1e-6:
        return None
    return math.atan2(v2 - math.sqrt(disc), G * dist)


def direction(yaw: np.ndarray, pitch: np.ndarray) -> np.ndarray:
    """Unit vectors (..., 3) for yaw / pitch in radians."""
    return np.stack([np.cos(pitch) * np.cos(yaw), np.cos(pitch) * np.sin(yaw), np.sin(pitch)], axis=-1)


def turn(d: np.ndarray, angle: float, side: np.ndarray) -> np.ndarray:
    """`d` (..., 3) turned by `angle` [rad] toward side angles `side` [rad] around it
    (result shape: side.shape + d.shape)."""
    up = np.where(np.abs(d[..., 2:3]) < 0.9, [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])
    e1 = np.cross(d, up)
    e1 /= np.linalg.norm(e1, axis=-1, keepdims=True)
    e2 = np.cross(d, e1)
    side = np.asarray(side, float).reshape(-1)
    return np.stack([math.cos(angle) * d + math.sin(angle) * (math.cos(a) * e1 + math.sin(a) * e2)
                     for a in side])


class Cannon(Trap):
    kind = "shell"
    suffixes = (".fired", ".hits")
    state_keys = ("next_shot", "next_t", "queue", "was_on", "hits", "hit_done", "rng")
    fields = (
        Field("yaw", "float", 0.0, unit="deg", min=-180.0, max=180.0, doc="heading the cannon faces"),
        Field("height", "float", 0.15, unit="m", min=0.05, max=1.5, doc="muzzle height above the floor"),
        Field("power", "float", 5.0, unit="m/s", min=0.5, max=15.0, runtime=True, doc="launch speed"),
        Field("shell_mass", "float", 0.2, unit="kg", min=0.005, max=1.0, doc="shell mass"),
        Field("shell_radius", "float", 0.025, unit="m", min=0.01, max=0.1),
        Field("period", "float", 2.0, unit="s", min=0.2, max=60.0, runtime=True,
              doc="fire: auto: time between bursts"),
        Field("burst", "int", 1, min=1, max=10, runtime=True, doc="shells per burst"),
        Field("spread", "float", 0.0, unit="deg", min=0.0, max=30.0, runtime=True,
              doc="half angle of the random cone around the aim"),
        Field("aim", "enum", "fixed", choices=("fixed", "at_body", "lead", "signal"),
              doc="fixed: yaw + aim_yaw, aim_pitch; at_body / lead: at the body (now / ahead); "
                  "signal: aim_yaw, aim_pitch from channels (bind)"),
        Field("aim_yaw", "float", 0.0, unit="deg", min=-180.0, max=180.0, runtime=True,
              doc="aim left / right of `yaw`"),
        Field("aim_pitch", "float", 10.0, unit="deg", min=-30.0, max=80.0, runtime=True,
              doc="aim elevation"),
        Field("lifetime", "float", 3.0, unit="s", min=0.2, max=30.0, doc="a shell is parked after this"),
        Field("pool", "int", 10, min=1, max=50, doc="shells built in advance (reused)"),
        Field("fire", "enum", "auto", choices=("auto", "edge"),
              doc="auto: every `period` while active; edge: one burst per rising edge of `in`"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        self.yaw, self.height = float(c["yaw"]), float(c["height"])
        self.aim, self.fire = c["aim"], c["fire"]
        self.radius = float(c["shell_radius"])
        self.lifetime = float(c["lifetime"])
        if self.fire == "edge" and not self.inputs:
            raise SchemaError(f"part {name!r}: fire: edge needs `in` channels")
        if self.fire == "edge":
            self.stateful = True
        self.pool = Pool(self.body_name + "/shell", int(c["pool"]), self.radius, float(c["shell_mass"]),
                         park_xy=(self.base_xy[0], self.base_xy[1]))
        self._reset_state()

    def _reset_state(self) -> None:
        self.next_shot = self.next_t = 0.0
        self.queue = 0
        self.was_on = False
        self.hits = 0
        self.hit_done: set[int] = set()
        self.fired = 0
        self.rng = np.random.default_rng(zlib.crc32(self.name.encode()))

    # ---- scene ------------------------------------------------------------------
    def build(self, spec):
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, 0.0],
                                       quat=yaw_quat(self.yaw))
        top = self.height - self.radius - 0.03
        body.add_geom(name=self.body_name + "/base", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[BASE, BASE, top / 2], pos=[0, 0, top / 2], rgba=(0.3, 0.3, 0.32, 1.0))
        body.add_geom(name=self.body_name + "/barrel", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[self.radius + 0.01, 0, 0], fromto=[0, 0, self.height, MUZZLE, 0, self.height],
                      rgba=(0.15, 0.15, 0.15, 1.0), contype=0, conaffinity=0, mass=0.0)
        self.pool.build(spec)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self.pool.bind(model)

    def root_body_ids(self, model):
        return list(self.pool.bids)

    # ---- runtime ----------------------------------------------------------------
    def reset(self, model, data):
        self.pool.reset(model, data)
        self._reset_state()

    def muzzle_centre(self) -> np.ndarray:
        return np.array([*self.xy, self.height])

    def aim_dir(self, body) -> np.ndarray:
        """Aim direction now (before the spread)."""
        yaw = math.radians(self.yaw + float(self.live["aim_yaw"]))
        pitch = math.radians(float(self.live["aim_pitch"]))
        if self.aim in ("at_body", "lead") and body is not None:
            c, v = self.muzzle_centre(), float(self.live["power"])
            goal = np.asarray(body.pos, float).copy()
            vel = np.zeros(3)
            if self.aim == "lead":
                cy, sy = math.cos(body.yaw), math.sin(body.yaw)
                lv = np.asarray(body.lin_vel, float)
                vel = np.array([cy * lv[0] - sy * lv[1], sy * lv[0] + cy * lv[1], 0.0])
            start = c
            for _ in range(4):          # fixed point: launch point (muzzle tip) and, for lead, flight time
                d = goal - start
                dist = math.hypot(d[0], d[1])
                yaw = math.atan2(d[1], d[0])
                arc = low_arc(dist, d[2], v)
                pitch = arc if arc is not None else math.pi / 4
                t = dist / max(v * math.cos(pitch), 1e-6)
                goal = np.asarray(body.pos, float) + vel * t
                start = c + direction(np.array(yaw), np.array(pitch)) * (MUZZLE + self.radius)
        return direction(np.array(yaw), np.array(pitch))

    def _shoot(self, model, data, body) -> None:
        d = self.aim_dir(body)
        s = math.radians(float(self.live["spread"]))
        if s > 0:                       # uniform over the cone's solid angle
            angle = math.acos(1.0 - self.rng.random() * (1.0 - math.cos(s)))
            d = turn(d, angle, self.rng.random() * 2 * math.pi)[0]
        i = self.pool.take()
        pos = self.muzzle_centre() + d * (MUZZLE + self.radius)
        self.pool.launch(model, data, i, pos, d * float(self.live["power"]), data.time)
        self.hit_done.discard(i)
        self.fired += 1

    def update(self, model, data, body):
        now = float(data.time)
        self.pool.expire(model, data, now, self.lifetime)
        self.fired = 0
        on = self.active()
        if self.fire == "edge":
            edge = self.inputs_on()
            if edge and not self.was_on and self.enabled() and int(self.live["enabled"]) == 1:
                self.queue += int(self.live["burst"])
            self.was_on = edge
        elif on:
            if now >= self.next_shot - 1e-9:
                self.queue += int(self.live["burst"])
                self.next_shot = now + float(self.live["period"])
        else:
            self.queue = 0
        if self.queue > 0 and now >= self.next_t - 1e-9:
            self._shoot(model, data, body)
            self.queue -= 1
            self.next_t = now + BURST_GAP
        self.write(float(self.fired > 0), self.name + ".fired")
        self.write(float(self.hits), self.name + ".hits")

    def substep(self, model, data):
        pool = self.pool
        if not pool.out or self.world is None:
            return
        kinds = self.world.kinds
        for k in range(data.ncon):
            c = data.contact[k]
            i, other = pool.index.get(c.geom1), c.geom2
            if i is None:
                i, other = pool.index.get(c.geom2), c.geom1
            if i is None or i not in pool.out or i in self.hit_done:
                continue
            if kinds.get(int(model.body_rootid[model.geom_bodyid[other]])) == "body":
                self.hit_done.add(i)
                self.hits += 1

    def state(self):
        return float(bool(self.pool.out))

    def save_state(self):
        return {**super().save_state(), "pool": self.pool.save()}

    def load_state(self, model, data, st):
        st = dict(st)
        self.pool.load(st.pop("pool"))
        super().load_state(model, data, st)

    # ---- planning ---------------------------------------------------------------
    def blockers(self, is_open):
        a = math.radians(self.yaw)
        d = np.array([math.cos(a), math.sin(a)]) * BASE
        return [Segment(tuple(self.xy - d), tuple(self.xy + d), 2 * BASE)]

    # ---- ballistic check (R16 cannon_turret) ---------------------------------------
    def aim_range(self, key: str, parts) -> np.ndarray:
        """Aim values [deg] `key` (aim_yaw / aim_pitch) can take: its bind's output
        range (per source cell when the source lists them), else its value."""
        f = self._schema[key]
        if key not in self.binds:
            return np.array([float(self.live[key])])
        ch, (i0, i1, o0, o1) = self.binds[key]
        src = parts.get(self.bus.writers.get(ch)) if self.bus is not None else None
        cells = getattr(src, "channel_cells", None)
        cells = cells(ch) if callable(cells) else None
        if cells is not None:
            ins = np.asarray(cells, float)
        else:
            n = int(math.ceil(abs(o1 - o0) / AIM_STEP)) + 1
            ins = np.linspace(i0, i1, max(n, 2))
        out = o0 + (ins - i0) * (o1 - o0) / (i1 - i0)
        return np.unique(np.clip(out, f.min, f.max))

    def hits_board(self, dirs: np.ndarray, target) -> np.ndarray:
        """Per direction (..., 3): does the shell path hit `target`'s board fast enough?"""
        tc, n, u, (hw, hh) = target.face()
        v = float(self.live["power"])
        p0 = self.muzzle_centre() + dirs * (MUZZLE + self.radius)
        vel = dirs * v
        nv = vel @ n
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((tc - p0) @ n) / nv
        q = p0 + vel * t[..., None]
        q[..., 2] -= 0.5 * G * t * t
        speed = np.sqrt((vel[..., 0] ** 2 + vel[..., 1] ** 2) + (vel[..., 2] - G * t) ** 2)
        ok = ((np.abs(nv) > 1e-9) & (t > 0) & (np.abs((q - tc) @ u) <= hw)
              & (np.abs(q[..., 2] - tc[2]) <= hh) & (q[..., 2] >= self.radius)
              & (p0[..., 2] >= self.radius) & (speed >= float(target.live["min_speed"])))
        return ok

    def can_hit(self, target, parts) -> bool:
        yaws = np.radians(self.yaw + self.aim_range("aim_yaw", parts))
        pitches = np.radians(self.aim_range("aim_pitch", parts))
        Y, P = np.meshgrid(yaws, pitches, indexing="ij")
        d = direction(Y, P)
        ok = self.hits_board(d, target)
        s = math.radians(float(self.live["spread"]))
        if s > 0:
            for e in turn(d, s, np.linspace(0.0, 2 * math.pi, CONE, endpoint=False)):
                ok &= self.hits_board(e, target)
        return bool(ok.any())

    def aim_text(self, parts) -> str:
        r = {k: self.aim_range(k, parts) for k in ("aim_yaw", "aim_pitch")}
        return ", ".join(f"{k} {v.min():g}..{v.max():g} deg" for k, v in r.items())
