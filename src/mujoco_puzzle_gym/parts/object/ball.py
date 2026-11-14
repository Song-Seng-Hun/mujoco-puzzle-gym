"""Ball and pin (R38): free objects the body pushes with its body (dribbling).

`ball`: a free sphere (kind "object", object kind "ball"), `condim` 6 so it
has rolling friction. Values:
- `radius`, `mass`: Open question 10 (2026-11-11, default walking body, flat floor,
  body steered at the ball at full speed for 10 s, 3 lateral offsets x
  elasticity 0 / 0.5 / 0.9): radius >= 0.05 m with mass <= 0.3 kg, and radius
  >= 0.08 m up to 1 kg, never fell; radius <= 0.05 m with >= 0.5 kg (0.03-0.04
  m from 0.3 kg) tripped it. Pushed balls move at 0.18-0.24 m/s for
  elasticity 0.5 (free walking 0.22 m/s), slower for heavy or dead balls.
  Default 0.05 m, 0.05 kg: 0.23 m/s. (Measured with roll_friction 0.002; the
  default 0.005 keeps kicked balls from rolling into corners: mini_soccer
  solver 19/20 against 16/20.)
- `elasticity`: coefficient of restitution e in [0, 0.95], set through the
  contact `solref` in its direct form (-stiffness, -damping) = (-w^2, -2 z w)
  with w = `CONTACT_OMEGA` and damping ratio z = -ln e / sqrt(pi^2 + ln^2 e)
  (e = 0: z = 1, no bounce). The ball geom has contact priority 1, so its
  solref and friction win over the floor's and the body's. A drop from 0.3 m
  bounces to e^2 of the height within 0.02 (0, 0.3, 0.5, 0.7, 0.9 measured,
  any mass; tests/test_ball.py). The (timeconst, dampratio) form is not used:
  its stiffness grows as 1/dampratio^2 and blows up at 5 ms steps.
- `friction` (sliding), `roll_friction` (m), `roll_damping` (on the three
  rotational dofs only), `color` (goal nets' `accepts` match it).

`pin`: a free upright cylinder; `<name>.down` is 1 once it tilts more than
`down_angle` (latched until reset). A pushing target like a goal net.

Pushing (R38 `push <ball> to <target>`): parts with `push_target(env)` (goal
nets, pins) give the scripted solver a point to push the nearest ball they
accept to; see `push.py`.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from ...schema import Field
from ..base import Part
from ..pool import PARK_Z

COLORS: dict[str, tuple[float, float, float, float]] = {
    "white": (0.95, 0.95, 0.95, 1.0),
    "red": (0.9, 0.15, 0.15, 1.0),
    "blue": (0.15, 0.35, 0.95, 1.0),
    "green": (0.15, 0.75, 0.25, 1.0),
    "yellow": (0.95, 0.85, 0.1, 1.0),
    "orange": (1.0, 0.55, 0.1, 1.0),
    "purple": (0.6, 0.25, 0.8, 1.0),
    "black": (0.1, 0.1, 0.1, 1.0),
}
CONTACT_OMEGA = 100.0        # contact natural frequency [rad/s]: ~6 steps of 5 ms per impact,
                             # 1 mm resting sink, 4-20 mm at a 0.3 m drop


def damping_ratio(e: float) -> float:
    """Contact damping ratio giving restitution `e` for a linear spring-damper."""
    if e <= 0.0:
        return 1.0
    ln = math.log(e)
    return -ln / math.sqrt(math.pi ** 2 + ln ** 2)


def contact_solref(e: float) -> list[float]:
    """Direct-form contact solref (-stiffness, -damping) for restitution `e`."""
    w = CONTACT_OMEGA
    return [-w * w, -2.0 * damping_ratio(e) * w]


class FreeObject(Part):
    """A part that is one free body placed at `pos` on reset (ball, pin).

    `park` takes it off stage (a dispenser's copy, R28, or an object a receptacle
    with `consume` took, R5): at a given point (below the floor), collisions off
    and gravity compensated, so it rests there touching nothing; `unpark` gives
    back its collision bits. `body_gravcomp` acts only in a model that has a body
    with gravcomp at compile time (MuJoCo `ngravcomp`); dispenser copies and
    consuming receptacles build one. The bits and `body_gravcomp` are snapshot
    fields, `parked` is in `save_state`."""

    kind = "object"
    object_kind = "object"
    stage = "device"
    floor_z = 0.0          # floor top under `pos`, set by the env before reset (R10/R37 heights)

    def _rest_z(self) -> float:
        raise NotImplementedError

    def bind(self, model, data):
        super().bind(model, data)
        j = model.body_jntadr[self._bid]
        self._q = int(model.jnt_qposadr[j])
        self._v = int(model.jnt_dofadr[j])
        gids = np.nonzero(model.geom_bodyid == self._bid)[0]
        self._bits = [(int(g), int(model.geom_contype[g]), int(model.geom_conaffinity[g])) for g in gids]

    def place(self, model, xy):
        self.xy = np.asarray(xy, dtype=float)

    def reset(self, model, data):
        self.parked = False
        self.put(data, [*self.xy, self.floor_z + self._rest_z()])

    def park(self, model, data, xyz) -> None:
        """Off stage at `xyz`: collisions off, gravity compensated (call mj_forward after).
        Tow ropes (R31) tied to it come loose first."""
        if self.world is not None and self._bid >= 0:
            self.world.untie(model, data, int(model.body_rootid[self._bid]))
        self.put(data, xyz)
        for g, _, _ in self._bits:
            model.geom_contype[g] = model.geom_conaffinity[g] = 0
        model.body_gravcomp[self._bid] = 1.0
        self.parked = True

    def unpark(self, model) -> None:
        for g, ct, ca in self._bits:
            model.geom_contype[g], model.geom_conaffinity[g] = ct, ca
        model.body_gravcomp[self._bid] = 0.0
        self.parked = False

    def vanish(self, model, data) -> None:
        """A receptacle consumed it (R5 `consume`): parked PARK_Z below where it is."""
        self.park(model, data, [*self.pos(data)[:2], PARK_Z])

    def shelve(self, model, data, spot) -> None:
        """R35 storage shelf: a free object is parked below the shelf spot (`park`),
        which also unties a rope from it and stops it falling."""
        self.park(model, data, [float(spot[0]), float(spot[1]), PARK_Z])

    def unshelve(self, model, data) -> None:
        self.unpark(model)

    def save_state(self):
        return {**super().save_state(), "parked": self.parked}

    def put(self, data, xyz, vel=None) -> None:
        """Move the object upright to `xyz` with linear velocity `vel` (call mj_forward after)."""
        data.qpos[self._q:self._q + 3] = xyz
        data.qpos[self._q + 3:self._q + 7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[self._v:self._v + 6] = 0.0
        if vel is not None:
            data.qvel[self._v:self._v + 3] = vel

    def pos(self, data) -> np.ndarray:
        return data.xpos[self._bid].copy()

    def vel(self, data) -> np.ndarray:
        """Linear velocity of the body origin in world axes."""
        return data.qvel[self._v:self._v + 3].copy()


class Ball(FreeObject):
    object_kind = "ball"
    fields = (
        Field("radius", "float", 0.05, unit="m", min=0.02, max=0.2,
              doc="OQ10: >= 0.05 m pushes without falling (<= 0.3 kg)"),
        Field("mass", "float", 0.05, unit="kg", min=0.005, max=5.0,
              doc="OQ10: <= 0.3 kg pushes without falling (radius >= 0.05 m)"),
        Field("elasticity", "float", 0.5, min=0.0, max=0.95,
              doc="restitution (0 = no bounce), through contact solref"),
        Field("friction", "float", 0.8, min=0.0, max=2.0, doc="sliding friction"),
        Field("roll_friction", "float", 0.005, unit="m", min=0.0, max=0.05,
              doc="rolling friction (0.005: a 0.5 m/s ball stops in about 0.2 m)"),
        Field("roll_damping", "float", 0.0, unit="N m s/rad", min=0.0, max=1.0,
              doc="damping on the rotational dofs"),
        Field("color", "enum", "white", choices=tuple(COLORS)),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        self.radius = float(c["radius"])
        self.mass = float(c["mass"])
        self.color = c["color"]
        self.needs_body("walk")

    def _rest_z(self) -> float:
        return self.radius

    def build(self, spec):
        c = self.cfg
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, self.radius])
        body.add_freejoint(name=self.body_name + "/free")
        body.add_geom(name=self.body_name + "/geom", type=mujoco.mjtGeom.mjGEOM_SPHERE,
                      size=[self.radius, 0, 0], mass=self.mass, condim=6, priority=1,
                      friction=[c["friction"], 0.005, c["roll_friction"]],
                      solref=contact_solref(c["elasticity"]),
                      rgba=COLORS[self.color])
        return body

    def bind(self, model, data):
        super().bind(model, data)
        model.dof_damping[self._v + 3:self._v + 6] = self.cfg["roll_damping"]

    def check_static(self, env) -> list[str]:
        """Static reasons the ball cannot be pushed (R38): outside the room, in a
        wall, or where the body cannot get next to it with every gate open."""
        from ...logic import reach_with_warps

        x, y = self.base_xy
        where = f"ball {self.name!r} at [{x:g}, {y:g}]"
        if not any(r.contains(self.base_xy, -self.radius) for r in env.room.zone_extents()):
            return [f"{where} is outside the room"]
        grid = env.grid
        if grid.blocked(env.walls, inflate=self.radius)[grid.cell(self.base_xy)]:
            return [f"{where} is inside a wall"]
        switches = {p.name for p in env.parts.values() if p.role == "switch"}
        reach = reach_with_warps(grid, env.walls, env.parts, switches, env.room.spawn_pos)
        if not (grid.disk(self.base_xy, self.radius + 0.15) & reach).any():
            return [f"{where} is out of the body's reach"]
        return []


FOLLOW_THROUGH = 0.3    # [m] pin push aim beyond the pin (plan step 8 bowling tune)


class Pin(FreeObject):
    object_kind = "pin"
    role = "switch"
    stage = "input"
    suffixes = (".down",)
    state_keys = ("down",)
    fields = (
        Field("radius", "float", 0.02, unit="m", min=0.01, max=0.1),
        Field("height", "float", 0.12, unit="m", min=0.03, max=0.5),
        Field("mass", "float", 0.03, unit="kg", min=0.005, max=2.0),
        Field("down_angle", "float", 45.0, unit="deg", min=5.0, max=90.0,
              doc="tilt from upright that counts as down"),
        Field("color", "enum", "white", choices=tuple(COLORS)),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        self.radius, self.height = float(c["radius"]), float(c["height"])
        self.cos_down = math.cos(math.radians(c["down_angle"]))
        self.down = False
        self.needs_body("walk")

    def _rest_z(self) -> float:
        return self.height / 2

    def build(self, spec):
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, self.height / 2])
        body.add_freejoint(name=self.body_name + "/free")
        body.add_geom(name=self.body_name + "/geom", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[self.radius, self.height / 2, 0], mass=self.cfg["mass"],
                      rgba=COLORS[self.cfg["color"]])
        return body

    def reset(self, model, data):
        super().reset(model, data)
        self.down = False
        self.write(0.0)

    def update(self, model, data, body):
        if not self.down and self.enabled():
            up_z = data.xmat[self._bid][8]            # world z of the pin's axis
            if up_z < self.cos_down:
                self.down = True
                self.write(1.0)

    def state(self):
        return float(self.down)

    def satisfied(self):
        return self.down

    def push_target(self, env, ball_xy) -> np.ndarray:
        """A point `FOLLOW_THROUGH` beyond the pin on the ball -> pin line: the pusher
        keeps its fast push through the pin instead of slowing down near it."""
        pin = self.pos(env.data)[:2]
        d = pin - np.asarray(ball_xy, dtype=float)
        n = float(np.linalg.norm(d))
        return pin + d / n * FOLLOW_THROUGH if n > 1e-6 else pin

    def push_accepts(self, ball: Ball) -> bool:
        return True

    def target_mask(self, grid):
        return grid.disk(self.xy, self.radius + 0.06)
