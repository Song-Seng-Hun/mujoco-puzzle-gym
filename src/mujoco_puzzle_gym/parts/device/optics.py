"""Optical panels (R99-R101): the mirror, and the two panels that share its tracer.

All three are thin upright panels that stand in the beam's way and collide like a
wall. `yaw` is the direction the panel **faces** - its normal - so a mirror facing
90 deg sends a beam travelling +x off towards +y. The panel itself spans `width`
across that normal. `yaw` can change while the room runs, so a room can let the
body aim a mirror through a `setter` or a `bind`.

  - `mirror` reflects, keeping `1 - loss` of the power.
  - `filter` lets the light through and keeps `transmission` of it. It does not
    reflect. A wall or a closed door already *stops* a beam (measured in the probe
    room: a beam with an object on the far side read 0 with an inner wall across
    it and 1 without), so there would be nothing here for an on-off shutter. What
    a filter does that a door cannot is leave *some* light, which is only a puzzle
    beside a receiver that asks how much (`photocell.min_power`).
  - `splitter` does both at once: `split` of the power carries straight on and
    `(1 - split) * (1 - loss)` goes off the normal. Expressions on the signal bus
    refuse arithmetic between channels ("BinOp is not allowed", locked in
    `tests/test_signals.py`), so dividing one beam's power in a ratio has nowhere
    else to live.
"""

from __future__ import annotations

import mujoco
import numpy as np

from ... import optics
from ...schema import Field
from ..base import Part

THICK = 0.008               # panel thickness [m]
HEIGHT = 0.12               # panel height [m]
MIRROR_RGBA = (0.72, 0.78, 0.85, 1.0)
FILTER_RGBA = (0.35, 0.75, 0.45, 0.55)
SPLIT_RGBA = (0.6, 0.65, 0.8, 0.6)
EDGE_RGBA = (0.25, 0.25, 0.28, 1.0)


class Panel(Part):
    """A thin upright panel that the tracer can ask about."""

    stage = "device"
    rgba = MIRROR_RGBA
    fields = (
        Field("yaw", "float", 0.0, unit="deg", min=-180.0, max=180.0, runtime=True,
              doc="direction the panel faces (its normal; 0: +x)"),
        Field("width", "float", 0.12, unit="m", min=0.03, max=1.0, doc="width across the normal"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.width = float(self.cfg["width"])
        self._yaw_set = None

    def build(self, spec):
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/panel", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[THICK / 2, self.width / 2, HEIGHT / 2],
                      pos=[0, 0, HEIGHT / 2], rgba=self.rgba)
        for s in (-1, 1):                       # edge posts, so the panel reads as a panel
            body.add_geom(name=f"{self.body_name}/edge{s}", type=mujoco.mjtGeom.mjGEOM_BOX,
                          size=[THICK / 2, THICK, HEIGHT / 2],
                          pos=[0, s * self.width / 2, HEIGHT / 2], rgba=EDGE_RGBA)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._gid = model.geom(self.body_name + "/panel").id
        self.apply_yaw(model)

    def reset(self, model, data):
        self.apply_yaw(model)

    def apply_yaw(self, model) -> None:
        """Turn the body to `yaw`; the tracer reads the pose, not this number."""
        yaw = float(self.live["yaw"]) * np.pi / 180.0
        if yaw == self._yaw_set:
            return
        model.body_quat[self._bid] = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
        self._yaw_set = yaw

    def update(self, model, data, body):
        self.apply_yaw(model)

    def optical_geoms(self, model) -> list[int]:
        return [self._gid]

    def normal(self, data) -> np.ndarray:
        return optics.normal_of(data, self._bid)

    def past(self, data, point, direction) -> np.ndarray:
        """A point on the far side of the panel, so the next ray does not hit it again.

        The step is the chord through a slab, `THICK / |d . n|`, not `THICK`: with a flat
        step a panel at 45 deg was crossed twice, because 8.6 mm along the ray is still
        inside an 8 mm panel met at an angle (the splitter then measured 0.36 straight
        through instead of 0.60, and fed its reflected cell twice)."""
        d = np.asarray(direction, float)
        cos = abs(float(np.dot(d, self.normal(data))))
        return point + d * (THICK / max(cos, 0.1) + 2 * optics.EPS)

    def blockers(self, is_open: bool):
        """Planning: the panel is a wall across its width."""
        from ...layout import Segment
        yaw = float(self.live["yaw"]) * np.pi / 180.0
        along = np.array([-np.sin(yaw), np.cos(yaw)]) * self.width / 2
        return [Segment(tuple(self.xy - along), tuple(self.xy + along), THICK)]


class Mirror(Panel):
    rgba = MIRROR_RGBA
    fields = (
        Field("loss", "float", 0.05, min=0.0, max=0.9, runtime=True,
              doc="share of the power lost in the reflection"),
    )

    def interact(self, model, data, gid, point, direction, power):
        out = optics.reflect(direction, self.normal(data))
        kept = power * (1.0 - float(self.live["loss"]))
        return [(point + out * optics.EPS, out, kept, 1)]


class Filter(Panel):
    rgba = FILTER_RGBA
    fields = (
        Field("transmission", "float", 0.5, min=0.0, max=1.0, runtime=True,
              doc="share of the power that gets through"),
    )

    def interact(self, model, data, gid, point, direction, power):
        kept = power * float(self.live["transmission"])
        return [(self.past(data, point, direction), np.asarray(direction, float), kept, 0)]


class Splitter(Panel):
    rgba = SPLIT_RGBA
    fields = (
        Field("split", "float", 0.5, min=0.0, max=1.0, runtime=True,
              doc="share of the power that carries straight on; the rest reflects"),
        Field("loss", "float", 0.1, min=0.0, max=0.9, runtime=True,
              doc="share of the reflected power lost"),
    )

    def interact(self, model, data, gid, point, direction, power):
        split = float(self.live["split"])
        through = power * split
        side = power * (1.0 - split) * (1.0 - float(self.live["loss"]))
        d = np.asarray(direction, float)
        out = optics.reflect(d, self.normal(data))
        return [(self.past(data, point, d), d, through, 0),
                (point + out * optics.EPS, out, side, 1)]
