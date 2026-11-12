"""Gaze sensor (R27): "the body is looking at that marker", by geometry.

Not a recognition sensor: no image is processed. A marker face (R26) is seen
from the body adapter's eye camera (`BodyAdapter.view_frame`)
when all four hold:
- its centre is within `fov / 2` of the view direction;
- it is at most `max_dist` away;
- the face points at the eye (angle between face normal and the line to the eye
  within `max_tilt`);
- a ray (`mj_ray`) from the eye to the centre is not blocked by another object
  (the walking body's own geoms and the marked part's own geoms do not block).

Room value (all optional; defaults shown):

    gaze: {fov: 60, max_dist: 2.0, max_tilt: 70, dwell: 0.5}   # degrees, m, degrees, s

Channel `<part>.seen`: 1 once the part's marker has been seen for `dwell`
seconds without a break, 0 otherwise ("a door that opens when looked at" is a
door with `in: [<part>.seen]`). The room adds a hidden `_gaze` part that writes
these channels only for parts whose `.seen` some part reads.

Planning (R20): each watched part also gets a hidden switch `_look_<part>`
(no channel of its own): the solution order turns it on to make
`<part>.seen` 1 (`GazeWatch.plan_values`). Its target area is the ring
LOOK_NEAR..LOOK_FAR m round the part, and it counts as done once `.seen` was
1 in the episode. The scripted solver looks with the `demo:` sub-goal `look
<part>` and for a `_look_` switch in the grid answer (carry.py).

Obs item `markers_seen` (fixed length): one slot per marker id used in the room,
ascending: (seen now, x, y of the marker in the heading frame [m], x, y of the
seen face's normal in the heading frame; 0 for a floor face), zeros when not
seen. No detection noise is simulated; it prepares a camera student policy.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np

from . import markers
from .parts.base import Part
from .registry import register_obs
from .schema import Field

GAZE_PART = "_gaze"
SEEN = ".seen"
LOOK_PREFIX = "_look_"
LOOK_NEAR, LOOK_FAR = 0.35, 0.8   # [m] planning: where the body stands to look at a part
MAX_SKIPS = 8          # robot geoms a ray may pass through on its way out of the head


@dataclass(frozen=True)
class GazeConfig:
    fov: float = 60.0         # [deg] full cone angle
    max_dist: float = 2.0     # [m]
    max_tilt: float = 70.0    # [deg] (floor markers: a standing body sees them 0.43-0.69 m away)
    dwell: float = 0.5        # [s]


LIMITS = {"fov": (1.0, 180.0), "max_dist": (0.05, 20.0), "max_tilt": (0.0, 90.0), "dwell": (0.0, 60.0)}


def parse_config(raw: Any) -> GazeConfig:
    """Room `gaze:` mapping -> GazeConfig; ValueError names a bad key or value."""
    if raw is None:
        return GazeConfig()
    if not isinstance(raw, dict) or set(raw) - set(LIMITS):
        raise ValueError(f"`gaze:` takes {sorted(LIMITS)}, got {raw!r}")
    vals = {}
    for k, v in raw.items():
        lo, hi = LIMITS[k]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not lo <= v <= hi:
            raise ValueError(f"gaze.{k} = {v!r} is out of range [{lo:g}, {hi:g}]")
        vals[k] = float(v)
    return GazeConfig(**vals)


class Eye:
    """Visibility tests from the body's eye in one compiled model."""

    def __init__(self, body, model: mujoco.MjModel, cfg: GazeConfig):
        self.body = body
        self.cfg = cfg
        self.root = int(model.body_rootid[body.root_body_id(model)])
        self.cos_fov = math.cos(math.radians(cfg.fov) / 2)
        self.cos_tilt = math.cos(math.radians(cfg.max_tilt))
        self._gid = np.zeros(1, dtype=np.int32)

    def frame(self, model, data) -> tuple[np.ndarray, np.ndarray]:
        return self.body.view_frame(model, data)

    def sees(self, model, data, eye: np.ndarray, rot: np.ndarray, centre: np.ndarray,
             normal: np.ndarray, owner_body: int) -> bool:
        """Whether the face at `centre` with outward `normal` on body `owner_body` is seen."""
        v = centre - eye
        dist = float(np.linalg.norm(v))
        if dist > self.cfg.max_dist or dist < 1e-9:
            return False
        u = v / dist
        if float(u @ -rot[:, 2]) < self.cos_fov:
            return False
        if float(-u @ normal) < self.cos_tilt:
            return False
        start, left = eye.copy(), dist
        for _ in range(MAX_SKIPS):
            hit = mujoco.mj_ray(model, data, start, u, None, 1, -1, self._gid)
            g = int(self._gid[0])
            if hit < 0 or g < 0:
                return True
            b = int(model.geom_bodyid[g])
            if b == owner_body:
                return True
            if hit >= left - 1e-4:
                return True
            if int(model.body_rootid[b]) != self.root:
                return False
            step = hit + 1e-4
            start, left = start + u * step, left - step
        return False

    def seen_face(self, model, data, eye, rot, part: Part, gids: list[int]):
        """(centre, normal) of the first face of `part`'s marker that is seen, else None."""
        for _, c, n in markers.face_poses(data, gids):
            if self.sees(model, data, eye, rot, c, n, part._bid):
                return c, n
        return None


class GazeWatch(Part):
    """Hidden part writing `<part>.seen` for the watched marked parts (added by the room)."""

    type_name = "gaze"
    stage = "input"
    has_body = False
    state_keys = ("timers",)
    fields = (Field("pos", "vec", [0.0, 0.0], length=2, unit="m", doc="unused (no body)"),)

    def __init__(self, name: str, watched: list[Part], cfg: GazeConfig):
        super().__init__(name, {"out": [p.name + SEEN for p in watched]})
        self.watched = watched
        self.gaze = cfg
        self.timers = [0.0] * len(watched)
        self._eye: Eye | None = None

    def bind(self, model, data):
        super().bind(model, data)
        self._gids = [markers.geom_ids(model, p) for p in self.watched]
        self._eye = None

    def reset(self, model, data):
        self.timers = [0.0] * len(self.watched)
        for ch in self.outputs:
            self.write(0.0, ch)

    def plan_values(self, on, get):
        return {p.name + SEEN: float(LOOK_PREFIX + p.name in on) for p in self.watched}

    def update(self, model, data, body):
        w = self.world
        if w is None or getattr(w, "body", None) is None:
            return
        if self._eye is None:
            self._eye = Eye(w.body, model, self.gaze)
        eye, rot = self._eye.frame(model, data)
        dt = self.bus.dt
        for i, p in enumerate(self.watched):
            seen = self._eye.seen_face(model, data, eye, rot, p, self._gids[i]) is not None
            self.timers[i] = self.timers[i] + dt if seen else 0.0
            self.write(float(self.timers[i] >= self.gaze.dwell - 1e-9 and seen), self.outputs[i])


class Look(Part):
    """Hidden planning switch "look at `watched`" (see the module docstring): done once
    `<watched>.seen` was 1 this episode."""

    type_name = "look"
    role = "switch"
    stage = "input"
    has_body = False
    state_keys = ("done",)
    fields = (Field("pos", "vec", [0.0, 0.0], length=2, unit="m", doc="unused (no body)"),)

    def __init__(self, watched: Part):
        self.watched = watched
        super().__init__(LOOK_PREFIX + watched.name, {"in": [watched.name + SEEN]})
        self.done = False

    @property
    def xy(self):
        """The watched part's position (no body of its own)."""
        return self.watched.xy

    @xy.setter
    def xy(self, value):
        pass

    def reset(self, model, data):
        self.done = False

    def update(self, model, data, body):
        self.done = self.done or self.bus.on(self.watched.name + SEEN)

    def satisfied(self):
        return self.done

    def target_mask(self, grid):
        return grid.disk(self.watched.xy, LOOK_FAR) & ~grid.disk(self.watched.xy, LOOK_NEAR)


def add_watch(parts: dict[str, Part], cfg: GazeConfig) -> None:
    """Add the `_gaze` part (and a `_look_<part>` switch per watched part) when some
    part reads a `<part>.seen` channel.

    ValueError when the channel names a part without a marker."""
    reads = []
    for p in parts.values():
        for ch in [*p.inputs, *([p.enable] if p.enable else [])]:
            if ch.endswith(SEEN) and ch[:-len(SEEN)] in parts and ch not in reads:
                reads.append(ch)
    if not reads:
        return
    if GAZE_PART in parts:
        raise ValueError(f"part name {GAZE_PART!r} is reserved for the gaze sensor")
    watched = []
    for ch in reads:
        target = parts[ch[:-len(SEEN)]]
        if target.marker is None:
            raise ValueError(f"channel {ch!r}: part {target.name!r} has no marker to look at "
                             f"(give it `marker:`)")
        watched.append(target)
    parts[GAZE_PART] = GazeWatch(GAZE_PART, watched, cfg)
    for p in watched:
        if LOOK_PREFIX + p.name in parts:
            raise ValueError(f"part name {LOOK_PREFIX + p.name!r} is reserved for the gaze sensor")
        parts[LOOK_PREFIX + p.name] = Look(p)


@register_obs("markers_seen", replace=True)
def markers_seen(env):
    """Markers seen now (R27): per used id (ascending) (seen, x, y, nx, ny), heading frame."""
    by_id: dict[int, list] = {}
    for p in env.parts.values():
        if p.marker is not None:
            by_id.setdefault(p.marker.id, []).append((p, markers.geom_ids(env.model, p)))
    slots = sorted(by_id.items())
    eye_ = Eye(env.body, env.model, env.room.gaze)
    s = env.body_state
    c, sn = math.cos(s.yaw), math.sin(s.yaw)
    out = np.zeros((len(slots), 5))
    if not slots:
        return out.ravel()
    eye, rot = eye_.frame(env.model, env.data)
    for i, (_, members) in enumerate(slots):
        best = None
        for p, gids in members:
            face = eye_.seen_face(env.model, env.data, eye, rot, p, gids)
            if face is not None:
                d = float(np.linalg.norm(face[0] - eye))
                if best is None or d < best[0]:
                    best = (d, face)
        if best is None:
            continue
        centre, normal = best[1]
        dx, dy = centre[0] - s.pos[0], centre[1] - s.pos[1]
        out[i] = [1.0, c * dx + sn * dy, -sn * dx + c * dy,
                  c * normal[0] + sn * normal[1], -sn * normal[0] + c * normal[1]]
    return out.ravel()
