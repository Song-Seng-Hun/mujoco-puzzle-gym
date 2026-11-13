"""Addon station (R30): a floor pad that puts an addon on the walking body or takes it off.

A room with a `roller` station builds the body with its wheels from the start
(`BodyAdapter.use_addons`, before the body spawns); the station only switches
modes. It switches when the body's trunk is inside the pad, its horizontal
speed is below `max_speed`, no crouch is running (it waits for the crouch
to end, still armed) and the signal comes:
- with `in` channels: when they turn on (`require` all / any), or later while
  they stay on, once the body is on the pad and slow (one switch per turn-on);
- without `in`: on entering the pad (one switch per visit; leave and come back
  for another).
`mode: attach` puts the addon on (nothing if it is on), `detach` takes it off,
`toggle` flips it. `enable` off holds the station; after a switch it waits
`cooldown` seconds before the next. The env does the switch after the body
step (`World.requests` "addon"): `BodyAdapter.set_addon` (trunk height, zero
velocity, collision groups, lower policy). The pad is drawn on the floor (no
collision) and lights up while the body wears this station's addon.

Static check (`mpg check` stage `parts`, a warning): the room's first station
floods the floor (every switch on) from each attach or toggle pad, with the
other detach and toggle pads blocked; raised or lowered floor it reaches (step
cells of a letter map, floor tiles with a height, `STEP_EPS`) is a way onto
steps on wheels, which the roller policy does not climb. Lay a detach pad
across the way before the steps.

Signals: `<name>.attached` (1 while the body wears the addon) and `body.addon`
(the body's mode: 0 none, 1 roller; written by the room's first station).
"""

from __future__ import annotations

import mujoco
import numpy as np

from ... import markers
from ...bodies import ADDON_MODES
from ...schema import Field
from ..base import Part
from ..input._sense import in_rect

ADDONS = ("roller",)
MODES = ("attach", "detach", "toggle")
BODY_ADDON = "body.addon"
RGBA_OFF = (0.35, 0.35, 0.45, 1.0)
RGBA_ON = (0.95, 0.75, 0.2, 1.0)
STEP_EPS = 0.003     # [m] floor this far above or below the room floor counts as a step


class AddonStation(Part):
    marked = True
    marker_anchor = "/pad"
    marker_faces = markers.FACES_FLAT
    stage = "device"
    suffixes = (".attached",)
    state_keys = ("armed", "inside", "was_on", "until", "uses")
    fields = (
        Field("size", "vec", [0.3, 0.3], length=2, unit="m", min=0.2, max=2.0, doc="pad size [x, y]"),
        Field("addon", "enum", "roller", choices=ADDONS, doc="the addon it puts on or takes off"),
        Field("mode", "enum", "attach", choices=MODES, doc="attach, detach or toggle"),
        Field("cooldown", "float", 1.0, unit="s", min=0.0, max=60.0,
              doc="time after a switch before the station switches again"),
        Field("max_speed", "float", 0.1, unit="m/s", min=0.01, max=1.0,
              doc="the body must be slower than this (horizontal) to switch"),
    )

    def marker_face_size(self, face):
        return float(min(self.cfg["size"])) * 0.6

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        self.half = np.asarray(c["size"], float) / 2
        self.addon = c["addon"]
        self.mode = c["mode"]
        self.cooldown = float(c["cooldown"])
        self.max_speed = float(c["max_speed"])
        self.needs_body("walk", self.addon)
        self.writes_body = False       # the room's first station also writes `body.addon`
        self._geoms: list[int] = []
        self.reset(None, None)

    @classmethod
    def add_room_parts(cls, parts):
        first = next((p for p in parts.values() if isinstance(p, AddonStation)), None)
        if first is not None and not first.writes_body:
            first.writes_body = True
            first.outputs.append(BODY_ADDON)

    def build(self, spec):
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/pad", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*self.half, 0.003], pos=[0, 0, 0.003], rgba=RGBA_OFF,
                      contype=0, conaffinity=0)
        for i, sx in enumerate((-1, 1)):            # two wheel marks
            body.add_geom(name=f"{self.body_name}/wheel{i}", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                          size=[min(self.half) * 0.22, 0.0035],
                          pos=[sx * self.half[0] * 0.45, 0, 0.0035],
                          rgba=(0.12, 0.12, 0.12, 1.0), contype=0, conaffinity=0)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._geoms = [model.geom(self.body_name + "/pad").id]

    def reset(self, model, data):
        self.armed = False
        self.inside = False
        self.was_on = False
        self.until = -np.inf
        self.uses = 0

    def _body_mode(self) -> str:
        body = self.world.body if self.world is not None else None
        return getattr(body, "addon", "none") if body is not None else "none"

    def _target(self, now: str) -> str | None:
        """The mode a switch here would give, None when it changes nothing."""
        want = {"attach": self.addon, "detach": "none",
                "toggle": "none" if now == self.addon else self.addon}[self.mode]
        return want if want != now else None

    def update(self, model, data, body):
        now = self._body_mode()
        attached = now == self.addon
        self.write(float(attached), self.name + ".attached")
        if self.writes_body:
            self.write(float(ADDON_MODES.index(now)), BODY_ADDON)
        for g in self._geoms:
            model.geom_rgba[g] = RGBA_ON if attached else RGBA_OFF
        inside = body is not None and in_rect(body.pos[:2], self.xy, self.half)
        if self.inputs:
            on = self.inputs_on()
            if on and not self.was_on:
                self.armed = True
            elif not on:
                self.armed = False
            self.was_on = on
        elif inside and not self.inside:
            self.armed = True
        elif not inside:
            self.armed = False
        self.inside = inside
        if not (self.armed and inside and self.enabled()) or self.world is None:
            return
        if data.time < self.until or float(np.hypot(*body.lin_vel[:2])) >= self.max_speed:
            return
        if getattr(self.world.body, "crouching", False):   # a crouch runs: switch once it ends
            return
        target = self._target(now)
        self.armed = False
        if target is None:
            return
        self.until = data.time + self.cooldown
        self.uses += 1
        self.world.requests.append(("addon", self, target))

    def state(self):
        return float(self._body_mode() == self.addon)

    def check_warnings(self, env) -> list[str]:
        """Steps reachable on wheels (R30 mode_switch_route): see the module docs."""
        if not self.writes_body:
            return []
        from ...logic import free_mask

        stations = [p for p in env.parts.values() if isinstance(p, AddonStation)]
        sources = [p for p in stations if p.mode in ("attach", "toggle")]
        if not sources:
            return []
        grid = env.grid
        steps = np.zeros(grid.X.shape, dtype=bool)
        if env.floor is not None:
            for iy, ix in np.ndindex(*steps.shape):
                top, _ = env.floor_height(grid.center(iy, ix))
                steps[iy, ix] = top is not None and abs(top) > STEP_EPS
        for p in env.parts.values():         # rising floor tiles count at their `on` height too
            on_h = p.cfg.get("on_height") if getattr(p, "floor_part", False) else None
            if on_h is not None and abs(float(on_h)) > STEP_EPS:
                steps |= grid.rect(p.base_xy, tuple(p.half))
        if not steps.any():
            return []
        on = {p.name for p in env.parts.values() if p.role in ("switch", "warp")}
        free = free_mask(grid, env.walls, env.parts, on)
        out = []
        for src in sources:
            mask = free.copy()
            for st in stations:
                if st is not src and st.mode in ("detach", "toggle"):
                    mask &= ~grid.rect(st.base_xy, tuple(st.half))
            hit = grid.flood(mask, src.base_xy) & steps
            if hit.any():
                iy, ix = np.argwhere(hit)[0]
                x, y = grid.center(iy, ix)
                out.append(f"part {src.name!r}: on wheels from this station the body can reach steps "
                           f"(e.g. at ({x:.2f}, {y:.2f})) without passing a detach station; the roller "
                           f"policy does not climb steps (lay a detach pad across the way)")
        return out

    def target_mask(self, grid):
        return grid.rect(self.xy, tuple(np.maximum(self.half - 0.05, 0.03)))
