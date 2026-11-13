"""Pit (R16): a hole in the floor.

A floor part (tile floor, R10) with no tile: the cell is "no floor", so a body
whose trunk drops below the rim has fallen (floor.py). `on_fall` says what
then happens: `fail` ends the episode (`failed()`, also without the fall
termination), `reset` puts the body back at the last checkpoint or the start
(env.py), like room `on_fall: reset` (R37). What falls in lands on the catch
plane.

Without `in` the pit is always open and cannot be switched: `enable`,
`enabled: 0` and a setter, `bind` or difficulty step on `enabled` are
rejected (give the pit an `in`). With `in` a lid covers it and slides away
while the inputs are on (and `enable` and `enabled`, trap base); the lid is a
collision toggle (contype / conaffinity, kept by snapshots), drawn only while
closed.

Planning: an open pit is a hole (`hole_mask`, `hole_now`), so solution
orders and solver paths go round it.

Depth (R16, R31 `pit_bridge`): without `depth` the pit is open down to the catch
plane. With `depth` a bottom plate lies at that depth (side walls under the rim keep
what falls in from sliding under the floor; the plate's friction, geom priority 3,
wins over a hitch object's, priority 2, so a box in it stays put when walked on). A cube (R17) resting on the plate,
tilted at most FILL_TILT, with its top at the floor height within FILL_TOL fills the
pit: the pit's cells are floor (the fall height is measured from the cube's top,
`height_at`) and the live planner no longer sees a hole (`hole_now`). When the cube
leaves, the pit is a hole again. "Filled" is read from the physics state each time
(not stored), so snapshots and same-seed runs need nothing extra. The static order
(`hole_mask`) counts a pit with `depth` as fillable, not a hole, when the room has a
cube with `hitch: true` that a rope can tie to and that fits it (`fits`: height
within FILL_TOL of `depth`; each side of the opening 0 to FIT_SLACK wider than the
box's largest horizontal size, handles included, so it drops in either way round); the static
check (`rope.pit_fill_problems`) holds a box the `demo:` drags in to the same fit.
"""

from __future__ import annotations

import mujoco
import numpy as np

from ...floor import CATCH_DEPTH, TILE_THICKNESS
from ...schema import Field, SchemaError
from ..floor.tile import FloorPart
from ..input._sense import in_rect
from .base import Trap

LID_RGBA = (0.45, 0.4, 0.35, 1.0)
PLATE_RGBA = (0.2, 0.2, 0.22, 1.0)
FILL_TILT = 5.0          # [deg] a filling cube tilts at most this much
FILL_TOL = 0.01          # [m] its top within this of the floor height, its bottom of the plate
FIT_SLACK = 0.04         # [m] each side of the opening is 0 to this much wider than the box
WALL_T = 0.01            # [m] half thickness of the side walls under the rim
PLATE_FRICTION = 1.0     # the plate's friction wins (priority 3 > a hitch object's 2): a filling box stays put
DARK = (0.08, 0.08, 0.08, 1.0)
NO_SWITCH = ("a pit without `in` is always open, so `enable` / `enabled` cannot switch it; "
             "give the pit an `in` to switch it")


class Pit(FloorPart, Trap):
    state_keys = ("open", "fell")
    fields = (
        Field("enabled", "int", 1, min=0, max=1, runtime=True,
              doc="0 closes the lid; only a pit with `in` can be switched"),
        Field("on_fall", "enum", "fail", choices=("fail", "reset"),
              doc="fail: the episode ends; reset: back to the last checkpoint or the start"),
        Field("depth", "float", None, unit="m", min=0.06, max=0.45,
              doc="bottom plate this deep (none: open down to the catch plane); a flush cube on it fills the pit"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.on_fall = self.cfg["on_fall"]
        if not self.inputs and (self.cfg.get("enable") or int(self.cfg["enabled"]) != 1
                                or "enabled" in self.binds):
            raise SchemaError(f"part {name!r}: {NO_SWITCH}")
        self.open = not self.inputs
        self.fell = False
        self.depth = None if self.cfg.get("depth") is None else float(self.cfg["depth"])
        self.cubes: list = []          # cubes that can fill it (`add_room_parts`, depth only)
        self.fillable = False          # planning: a tiable hitch cube fits (module docstring)
        self._data = None

    @classmethod
    def add_room_parts(cls, parts):
        from ..device.rope import Rope
        from ..object.cube import Cube

        cubes = [p for p in parts.values() if isinstance(p, Cube)]
        ropes = [p for p in parts.values() if isinstance(p, Rope)]
        for p in parts.values():
            if isinstance(p, Pit) and p.depth is not None:
                p.cubes = cubes
                p.fillable = any(c.cfg["hitch"] and p.fits(c.extent()) is None
                                 and any(c.name in r.ends_cfg or None in r.ends_cfg for r in ropes)
                                 for c in cubes)

    @property
    def full_depth(self) -> float:
        """Rim to the plate, or to the catch plane without `depth` [m]."""
        return self.depth if self.depth is not None else CATCH_DEPTH

    def fits(self, extent) -> str | None:
        """Why a box of `extent` (x, y, z) [m] does not fill this pit flush, or None."""
        depth = self.full_depth
        if abs(extent[2] - depth) > FILL_TOL:
            return (f"leaves its top {abs(depth - extent[2]):.2f} m {'below' if extent[2] < depth else 'above'} "
                    f"the floor; a box filling a pit must be flush within {FILL_TOL * 100:.0f} cm")
        size, box = 2 * self.half, float(max(extent[0], extent[1]))
        slack = size - box
        if (slack < -1e-9).any() or (slack > FIT_SLACK + 1e-9).any():
            return (f"does not fit: each side of the opening ({size[0]:.3f} x {size[1]:.3f} m) must be "
                    f"0-{FIT_SLACK * 100:.0f} cm wider than the box ({box:.3f} m across, handles included)")
        return None

    def claim_value(self, key):
        if key == "enabled" and not self.inputs:
            raise SchemaError(f"part {self.name!r}: {NO_SWITCH}")
        super().claim_value(key)

    def why_not_stepped(self, key):
        return NO_SWITCH if key == "enabled" and not self.inputs else super().why_not_stepped(key)

    def build(self, spec):
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, 0.0])
        if self.depth is None:
            body.add_geom(name=self.body_name + "/dark", type=mujoco.mjtGeom.mjGEOM_BOX,
                          size=[*self.half, 0.005], pos=[0, 0, -0.25], rgba=DARK,
                          contype=0, conaffinity=0)
        else:
            d, (hx, hy) = self.depth, self.half
            body.add_geom(name=self.body_name + "/plate", type=mujoco.mjtGeom.mjGEOM_BOX,
                          size=[hx + 2 * WALL_T, hy + 2 * WALL_T, 0.01], pos=[0, 0, -d - 0.01], rgba=PLATE_RGBA,
                          friction=[PLATE_FRICTION, 0.005, 0.0001], priority=3)
            hz, cz = (d - TILE_THICKNESS) / 2, -(d + TILE_THICKNESS) / 2
            for k, (pos, size) in enumerate((([hx + WALL_T, 0], [WALL_T, hy + 2 * WALL_T]),
                                              ([-hx - WALL_T, 0], [WALL_T, hy + 2 * WALL_T]),
                                              ([0, hy + WALL_T], [hx, WALL_T]),
                                              ([0, -hy - WALL_T], [hx, WALL_T]))):
                body.add_geom(name=f"{self.body_name}/side{k}", type=mujoco.mjtGeom.mjGEOM_BOX,
                              size=[*size, hz], pos=[*pos, cz], rgba=PLATE_RGBA)
        if self.inputs:
            body.add_geom(name=self.body_name + "/lid", type=mujoco.mjtGeom.mjGEOM_BOX,
                          size=[*self.half, TILE_THICKNESS / 2], pos=[0, 0, -TILE_THICKNESS / 2],
                          priority=1, rgba=LID_RGBA)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._lid = model.geom(self.body_name + "/lid").id if self.inputs else -1
        self._data = data

    def _set_lid(self, model, closed: bool) -> None:
        if self._lid >= 0:
            model.geom_contype[self._lid] = model.geom_conaffinity[self._lid] = int(closed)
            model.geom_rgba[self._lid] = LID_RGBA if closed else (*LID_RGBA[:3], 0.0)

    def reset(self, model, data):
        self.open = not self.inputs
        self.fell = False
        self._set_lid(model, not self.open)

    def update(self, model, data, body):
        if self.inputs:
            now = self.active()
            if now != self.open:
                self.open = now
                self._set_lid(model, not now)
        if body is not None and body.fallen and self.open and in_rect(body.pos, self.xy, self.half) \
                and self.filler_top() is None:
            self.fell = True

    def filler_top(self) -> float | None:
        """Top height [m] of a cube filling the pit now (module docstring), else None."""
        if not self.cubes or self._data is None:
            return None
        data, cos_max = self._data, np.cos(np.radians(FILL_TILT))
        for c in self.cubes:
            if c.parked:
                continue
            p = data.xpos[c._bid]
            if not in_rect(p, self.xy, self.half):
                continue
            if np.abs(data.xmat[c._bid].reshape(3, 3)[2]).max() < cos_max:
                continue
            h = c.size / 2
            if abs(p[2] + h) <= FILL_TOL and abs(p[2] - h + self.depth) <= FILL_TOL:
                return float(p[2] + h)
        return None

    def height_at(self, xy):
        if not self.open:
            return 0.0, 0.0
        top = self.filler_top()
        return (None, 0.0) if top is None else (top, 0.0)

    def failed(self):
        return self.fell and self.on_fall == "fail"

    def state(self):
        return float(self.open)

    def hole_mask(self, grid, get):
        if self.fillable or (self.inputs and not self.plan_inputs_on(get)):
            return None
        return grid.rect(self.xy, tuple(self.half))

    def hole_now(self, grid):
        return grid.rect(self.xy, tuple(self.half)) if self.open and self.filler_top() is None else None
