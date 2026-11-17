"""Balance beam (R90): 1 while a freely pivoting plank rests level.

A plank on a central hinge with a spring that pulls it back to level. The physics solves the
torques, so what the part reads is the *angle*, and the angle already answers
"mass x distance": it settles where the spring balances the net torque of everything on the
plank. That is the
one thing this package could not ask before: `pressure` sums the mass on a plate and a gate
expression can compare or window two of those sums (measured 2026-11-17: `pan_b.mass >
pan_a.mass` and `pan_b.mass >= 0.04 and pan_b.mass <= 0.06` both work), but no part and no
expression can weigh a lever arm, because expressions refuse arithmetic between channels
("BinOp is not allowed"). Two equal cubes at unequal distances tip this plank; no
combination of the existing parts notices.

On when both hold:

- `|angle| <= balance_band`, the plank level within its band;
- the hinge rate has stayed under `REST_RATE` for `settle` seconds, so a plank swinging
  through level on its way past does not read as balanced.

Channels: its own (or `out`) 0/1, and `<name>.angle` [deg, + = the +x end down; measured].

The angle is a torque reading, not just a direction. One 20 g cube resting at x = +0.122 m
is 0.0239 N m about the hinge, and the plank settled at 0.2527 rad against the default
0.09 N m/rad spring, which is 0.0227 N m - within 5 %. Two equal cubes at equal distances
(+-0.08, +-0.10 and +-0.12 m) all read 0.00 deg and on; move one of them in and it reads off.

Two deliberate narrowings against the first draft of spec R90, both measured rather than
guessed:

- **No `accept` list.** Every receiving part in the package filters what counts, because it
  reads positions and has to know what it is looking at. This one reads a hinge: gravity
  already weighs whatever is on the plank, including the body, and the settle test asks the
  hinge's own rate rather than each object's speed. A filter would be a lie - the plank
  cannot ignore a mass that is really pressing on it.
- **A spring, not a free pivot.** The first build left the hinge free, the way a seesaw
  pivots. Measured 2026-11-17: that is not a balance. A plank pivoted at its centre with
  loads resting *on top* is in unstable equilibrium - two equal cubes at equal distances
  (+-0.10 m) still ran to -14.48 deg, because the first degree of tilt slides the loads
  downhill and makes the next degree worse. Real balances hang their pans below the pivot
  for exactly this reason. Hanging pans would need the scripted place to aim at a pan rather
  than a part, which is the problem that withdrew spec R89, so this part keeps one flat
  surface and adds the spring. The angle then measures net torque instead of announcing
  which way it fell, which is strictly more than the requirement asked for.

- **The plank is not a bridge.** The spec sketched a levelled plank as a path to walk. It
  is not offered as one: the walking policy is trained to `max_slope` 5 deg (duck) and 6 deg
  (go2) on *static* ground, and the planner has no notion of a surface that tilts under the
  body. A room that wants a crossing should raise a static `platform` from this signal.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from ...schema import Field, SchemaError
from ..base import Part

REST_RATE = 20.0      # [deg/s] the hinge counts as settled under this
PLANK_MASS = 0.08     # [kg] a light plank: a 20 g cube has to move it
POST_GAP = 0.006      # [m] the post stops this far under the hinge (see `build`)
POST_RGBA = (0.42, 0.42, 0.46, 1.0)
OFF_RGBA = (0.75, 0.6, 0.3, 1.0)
ON_RGBA = (0.2, 0.8, 0.35, 1.0)


class Beam(Part):
    role = "sensor"
    stage = "input"
    emits = True
    suffixes = (".angle",)
    state_keys = ("on", "angle", "_since")
    fields = (
        Field("span", "float", 0.24, unit="m", min=0.08, max=1.5,
              doc="plank length end to end"),
        Field("width", "float", 0.05, unit="m", min=0.02, max=0.5),
        Field("pivot_height", "float", 0.035, unit="m", min=0.01, max=0.4,
              doc="hinge height above the floor"),
        Field("tilt_limit", "float", 18.0, unit="deg", min=3.0, max=60.0,
              doc="how far each end can drop before the hinge stops"),
        Field("balance_band", "float", 3.0, unit="deg", min=0.2, max=30.0, runtime=True,
              doc="level within this reads on"),
        Field("settle", "float", 0.4, unit="s", min=0.0, max=5.0,
              doc="the hinge must be this quiet before it reads on"),
        Field("lip", "float", 0.005, unit="m", min=0.0, max=0.03,
              doc="rim around the plank so a tilted load stays on it"),
        Field("stiffness", "float", 0.09, unit="N m/rad", min=0.005, max=5.0,
              doc="the spring that pulls the plank back to level; the angle measures torque"),
        Field("damping", "float", 0.02, unit="N m s/rad", min=0.0, max=1.0,
              doc="hinge damping: too little and a light load never settles"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        if float(c["balance_band"]) >= float(c["tilt_limit"]):
            raise SchemaError(f"part {name!r}: balance_band {c['balance_band']:g} deg must be "
                              f"under tilt_limit {c['tilt_limit']:g} deg, or it always reads level")
        self.span = float(c["span"])
        self.width = float(c["width"])
        self.pivot = float(c["pivot_height"])
        self.limit = float(c["tilt_limit"])
        self.half = np.array([self.span / 2, self.width / 2])
        self.on = False
        self.angle = 0.0
        self._since = None

    def build(self, spec):
        body = super().build(spec)
        # The post stops POST_GAP under the hinge. Measured 2026-11-17: with its top flush at
        # `pivot_height` the plank's underside rested on it, and that contact's friction held
        # the hinge still - a 20 g cube at 0.12 m against one at 0.04 m read 0.00 deg, and a
        # 2 rad/s kick decayed to 0.008 rad/s in 20 steps. The gap and the exclude below are
        # what make this part a balance rather than a fixed plank.
        body.add_geom(name=self.body_name + "/post", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[0.014, 0.014, (self.pivot - POST_GAP) / 2],
                      pos=[0, 0, (self.pivot - POST_GAP) / 2], rgba=POST_RGBA)
        plank = body.add_body(name=self.body_name + "/plank", pos=[0, 0, self.pivot])
        plank.add_joint(name=self.body_name + "/hinge", type=mujoco.mjtJoint.mjJNT_HINGE,
                        axis=[0, 1, 0], range=[-self.limit, self.limit],
                        limited=mujoco.mjtLimited.mjLIMITED_TRUE,
                        stiffness=float(self.cfg["stiffness"]), springref=0.0,
                        damping=float(self.cfg["damping"]), armature=0.0005)
        plank.add_geom(name=self.body_name + "/plank", type=mujoco.mjtGeom.mjGEOM_BOX,
                       size=[self.span / 2, self.width / 2, 0.006], pos=[0, 0, 0.006],
                       mass=PLANK_MASS, rgba=OFF_RGBA)
        lip = float(self.cfg["lip"])
        if lip:
            # A rim, not a wall. R70 measured that this arm's own links foul a 0.06 m
            # receptacle wall (2.111 N against it); 5 mm is an order of magnitude under the
            # 0.04 m grab radius and well under GRAB_Z, and without it a plank at a few
            # degrees loses its load (measured: a 20 g cube ran off to x=-0.162 m).
            for sx in (-1, 1):
                plank.add_geom(name=f"{self.body_name}/lip{'px' if sx > 0 else 'mx'}",
                               type=mujoco.mjtGeom.mjGEOM_BOX,
                               size=[0.004, self.width / 2, lip / 2],
                               pos=[sx * (self.span / 2 - 0.004), 0, 0.012 + lip / 2],
                               mass=0.002, rgba=OFF_RGBA)
            for sy in (-1, 1):
                plank.add_geom(name=f"{self.body_name}/lip{'py' if sy > 0 else 'my'}",
                               type=mujoco.mjtGeom.mjGEOM_BOX,
                               size=[self.span / 2, 0.004, lip / 2],
                               pos=[0, sy * (self.width / 2 - 0.004), 0.012 + lip / 2],
                               mass=0.002, rgba=OFF_RGBA)
        spec.add_exclude(bodyname1=self.body_name, bodyname2=plank.name)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        j = model.joint(self.body_name + "/hinge")
        self._qa, self._va = int(j.qposadr[0]), int(j.dofadr[0])
        self._gid = model.geom(self.body_name + "/plank").id

    def reset(self, model, data):
        data.qpos[self._qa] = data.qvel[self._va] = 0.0
        self.on, self.angle, self._since = False, 0.0, None
        model.geom_rgba[self._gid] = OFF_RGBA
        for ch in self.outputs:
            self.write(0.0, ch)

    def rate(self, data) -> float:
        """Hinge rate now [deg/s]."""
        return math.degrees(float(data.qvel[self._va]))

    def update(self, model, data, body):
        self.angle = math.degrees(float(data.qpos[self._qa]))
        if not self.enabled():
            return
        level = abs(self.angle) <= float(self.live["balance_band"])
        quiet = abs(self.rate(data)) < REST_RATE
        now = float(data.time)
        if level and quiet:
            self._since = now if self._since is None else self._since
        else:
            self._since = None
        on = self._since is not None and now - self._since >= float(self.cfg["settle"])
        if on != self.on:
            self.on = on
            model.geom_rgba[self._gid] = ON_RGBA if on else OFF_RGBA
        for ch in self.outputs[:-1]:
            self.write(float(self.on), ch)
        self.write(round(self.angle, 2), self.outputs[-1])

    def state(self):
        return float(self.on)

    def satisfied(self):
        return self.on

    def place_tolerance(self, obj) -> float:
        """Scripted `place`: the plank is narrow, so let go close to where it was aimed."""
        return 0.02

    def target_mask(self, grid):
        return grid.rect(self.xy, tuple(self.half))
