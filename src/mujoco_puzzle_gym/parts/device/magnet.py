"""Magnet (R98): pulls the objects it accepts to itself and holds them there.

A flat pad at `pos`. While the magnet is enabled and its `in` channels are on
(no `in`: always), every object named in `accept` (empty: every object) whose
centre is within `radius` of the pad and lower than `height` gets a force of
`power` [N] toward the post, in the floor plane, through `xfrc_applied` on its
root body - the same add-and-take-back bookkeeping as `trap/wind.py`, so two
magnets add up and switching one off removes only its share. Inside `hold` the
force tapers to zero at the centre, which is what makes the object *stop* on
the pad instead of being pushed past it. Its channel is 1 while an accepted
object sits inside `hold`. Switching the magnet off only stops the pull: what
was already moving coasts on and friction stops it (measured: released at
0.472 m it rested at 0.370 m, released at 0.359 m it reached 0.076 m), so
"off" means "no longer arriving", not "frozen where it is".

Three things separate it from a wind blowing the same object across the floor,
and all three were measured before this part was written (2026-11-17, probe
rooms in `docs/specs/2026-11-17-magnets-and-optics/intent.md`):

  - **The direction follows the object.** A wind's `yaw` is one number for the
    whole zone, so it can pull along a line; a magnet collects from any side.
  - **It chooses.** A wind's `trigger` names kinds (object, body, npc), never
    one object. `accept` names objects, so a magnet can bring one cube and
    leave its neighbour where it is.
  - **It can hold.** A constant force never stops: 0.5 N on a 0.05 kg cube
    pulled it 1.1 m in one second and then on into the far wall (x 1.1 ->
    -1.07 m), and the zone it crossed on the way read 1 for a moment and 0
    again. With the taper the same cube comes to rest on the pad.

`power` is a force, so what it does depends on the object: a cube on the floor
needs about its own weight to break friction loose (0.05 kg -> 0.49 N), which
is why the preset is 1.0 N and why `check_warnings` names a setting that
cannot move what it accepts.

`damp` is there because pull alone does not deliver. A constant force over the
radius hands the object as much energy as friction can take back over the same
distance, so above about 2 mu m g it crosses the pad and leaves the field on
the far side. Measured on a 0.03 kg box pulled 0.39 m (2026-11-17), as the
distance it came to rest from the pad centre:

    power [N]   damp 0        damp 0.4      damp 1.0
    0.6         0.009 m       0.011 m       0.011 m
    1.0         0.269 m *     0.004 m       0.003 m
    2.0         0.506 m *     0.001 m       0.001 m
    4.0         1.391 m *     0.101 m *     0.000 m

    (*) ended outside the field, channel 0: thrown past the pad, not delivered.

Without damping only one setting in four delivers; with the preset 0.4 N s/m
three of four land within 4 mm. So the force against the object's own speed is
what makes a magnet a place to put something rather than a slingshot.
"""

from __future__ import annotations

import mujoco
import numpy as np

from ...schema import Field
from ..base import Part
from ..input._sense import candidates

PAD_R = 0.05                        # pad radius [m]
PAD_H = 0.004                       # pad thickness [m]
COIL_RGBA = (0.75, 0.2, 0.2, 1.0)
PAD_RGBA = (0.35, 0.35, 0.38, 1.0)
ON_RGBA = (1.0, 0.55, 0.2, 1.0)


class Magnet(Part):
    stage = "device"
    emits = True
    state_keys = ("held",)
    fields = (
        Field("radius", "float", 0.35, unit="m", min=0.05, max=2.0,
              doc="objects nearer than this are pulled"),
        Field("power", "float", 1.0, unit="N", min=0.0, max=20.0, runtime=True,
              doc="force on each accepted object, toward the post"),
        Field("hold", "float", 0.06, unit="m", min=0.02, max=0.5,
              doc="inside this the pull tapers to zero and the channel reads 1"),
        Field("height", "float", 0.2, unit="m", min=0.02, max=2.0,
              doc="objects count while their centre is lower than this"),
        Field("damp", "float", 0.4, unit="N s/m", min=0.0, max=20.0, runtime=True,
              doc="force against the object's own speed, so it settles instead of overshooting"),
        Field("accept", "list", [], doc="object parts it pulls (empty: every object)"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.radius = float(self.cfg["radius"])
        self.hold = float(self.cfg["hold"])
        self.height = float(self.cfg["height"])
        self.accept = list(self.cfg["accept"])
        if self.hold >= self.radius:
            raise ValueError(f"part {name!r}: hold {self.hold:g} m must be inside "
                             f"radius {self.radius:g} m")
        self.held = False
        self.applied: dict[int, np.ndarray] = {}

    def build(self, spec):
        """A flat pad, not a post. The first build put a 9 cm post at `pos`: the object
        arrived and held (measured: 0.35 m away -> resting at the centre in 1.0 s), and
        then the arm could not pick it up - the jaw fouled the post, the same failure
        R70 measured on a receptacle's rim. Nothing here stands above 4 mm, and the pad
        does not collide at all."""
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/pad", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[PAD_R, PAD_H / 2], pos=[0, 0, PAD_H / 2], rgba=PAD_RGBA,
                      contype=0, conaffinity=0, mass=0.0)
        for i, r in enumerate((0.75, 0.45)):
            body.add_geom(name=f"{self.body_name}/coil{i}", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                          size=[PAD_R * r, PAD_H], pos=[0, 0, PAD_H], rgba=COIL_RGBA,
                          contype=0, conaffinity=0, mass=0.0)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._coils = [model.geom(f"{self.body_name}/coil{i}").id for i in range(2)]

    def reset(self, model, data):
        self.held = False
        self.applied = {}
        self._paint(model, False)
        for ch in self.outputs:
            self.write(0.0, ch)

    def _paint(self, model, on: bool) -> None:
        for gid in getattr(self, "_coils", ()):
            model.geom_rgba[gid] = ON_RGBA if on else COIL_RGBA

    def pulling(self) -> bool:
        return self.enabled() and (not self.inputs or self.inputs_on())

    def force_on(self, pos, vel=None) -> np.ndarray | None:
        """Force [N] on an object at `pos` moving at `vel` (world), or None when it is out
        of the field: `power` toward the pad, tapered inside `hold`, minus `damp` times the
        object's own planar speed."""
        d = self.xy - np.asarray(pos[:2], dtype=float)
        dist = float(np.linalg.norm(d))
        if dist > self.radius or pos[2] > self.height:
            return None
        pull = np.zeros(2)
        if dist >= 1e-6:
            pull = d / dist * float(self.live["power"]) * min(1.0, dist / self.hold)
        if vel is not None:
            pull = pull - float(self.live["damp"]) * np.asarray(vel[:2], dtype=float)
        return np.array([*pull, 0.0])

    def update(self, model, data, body):
        if self.world is None:
            return
        want: dict[int, np.ndarray] = {}
        held = False
        if self.pulling():
            for key, pos in candidates(self, data, body, "object", self.accept or None):
                f = self.force_on(pos, data.cvel[int(key), 3:6])
                if f is None:
                    continue
                want[int(key)] = f
                held |= float(np.linalg.norm(self.xy - pos[:2])) <= self.hold
        zero = np.zeros(3)
        for rid in set(self.applied) | set(want):
            data.xfrc_applied[rid, :3] += want.get(rid, zero) - self.applied.get(rid, zero)
        self.applied = want
        if held != self.held:
            self.held = held
            self._paint(model, held)
        for ch in self.outputs:
            self.write(float(self.held), ch)

    def state(self):
        return float(self.held)

    def satisfied(self):
        return self.held

    def check_warnings(self, env) -> list[str]:
        """A force under the weight of what it accepts does not break floor friction
        loose (the default contact friction is 1.0, so the pull has to carry about
        `m g` to start the object sliding). Reported, not failed: a magnet too weak to
        slide a box can still hold one that arrives some other way."""
        power = float(self.live["power"])
        g = abs(float(env.model.opt.gravity[2]))
        out = []
        for name in self.accept or []:
            part = env.parts.get(name)
            if part is None or getattr(part, "_bid", -1) < 0:
                continue
            mass = float(env.model.body_subtreemass[part._bid])
            if not (power and mass):
                continue
            if power < mass * g:
                out.append(f"part {self.name!r}: power {power:g} N is under the weight of "
                           f"{name!r} ({mass * g:.2f} N); it will not slide it along the floor")
            elif not float(self.live["damp"]) and power > 2 * mass * g:
                out.append(f"part {self.name!r}: power {power:g} N with no `damp` is over twice "
                           f"the weight of {name!r} ({mass * g:.2f} N); it will throw it past "
                           f"the pad and out of the field")
        return out

    def target_mask(self, grid):
        """Cells next to the post (where a body would stand to take what arrived)."""
        return np.hypot(grid.X - self.xy[0], grid.Y - self.xy[1]) <= self.radius / 2
