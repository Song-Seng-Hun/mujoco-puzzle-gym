"""Light: an emitter that casts a path and a photocell that reads what arrives (R99).

    - {name: lamp, type: emitter, pos: [-0.5, 0.0], yaw: 0.0, power: 1.0, bounces: 2}
    - {name: eye, type: photocell, pos: [0.5, 0.4], min_power: 0.3}

The emitter casts one ray at `height` in the direction `yaw` and follows it through
the optical panels (`mirror`, `filter`, `splitter`) with `src/mujoco_puzzle_gym/optics.py`.
Its own channel is 1 while that path reaches any photocell. A photocell writes
`<name>` (1 while the power that arrived is at least `min_power`) and
`<name>.power` (that power, a float), so a room can ask *whether* the light got
there and *how much* of it did.

Why these are parts rather than a room written with `laser`. `laser` (R5) is a
beam between two posts that answers "did something cut it", and that answer is
the same 0 whether nothing cut it or a wall hides the whole beam - measured in
the probe room before this was written, with the object on the far side of an
inner wall. Nothing in the package could ask whether light *arrived*, and nothing
could bend it. `laser` is untouched: rooms that use it as a trip wire keep working.

The emitter updates in the `input` stage and hands each photocell its power
directly; the cells watch the emitter's channel, so the bus updates the emitter
first and a cell's channel is current within the same body step (the ordering
`device/setter.py` uses for its target).
"""

from __future__ import annotations

import mujoco
import numpy as np

from ... import optics
from ...schema import Field
from ..base import Part

POST = 0.022                 # emitter / photocell post half width [m]
POST_H = 0.1                 # post height [m]
BEAM_R = 0.004               # drawn beam radius [m]
LAMP_RGBA = (0.95, 0.85, 0.3, 1.0)
CELL_OFF_RGBA = (0.3, 0.32, 0.38, 1.0)
CELL_ON_RGBA = (0.3, 1.0, 0.45, 1.0)
BEAM_RGBA = (1.0, 0.35, 0.2, 0.75)


class Photocell(Part):
    """What the light is aimed at: 1 while `min_power` arrives."""

    role = "sensor"
    stage = "input"
    emits = True
    is_photocell = True
    suffixes = (".power",)
    state_keys = ("on", "power")
    fields = (
        Field("min_power", "float", 0.3, min=0.0, max=100.0, runtime=True,
              doc="power needed to read 1"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.on = False
        self.power = 0.0
        self._beams: dict[str, float] = {}

    def build(self, spec):
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/cell", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[POST, POST, POST_H / 2], pos=[0, 0, POST_H / 2],
                      rgba=CELL_OFF_RGBA)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._gid = model.geom(self.body_name + "/cell").id

    def optical_geoms(self, model) -> list[int]:
        return [self._gid]

    def take(self, source: str, power: float) -> None:
        """An emitter reports the power its path delivered here this step."""
        self._beams[source] = float(power)

    def reset(self, model, data):
        self.on, self.power, self._beams = False, 0.0, {}
        model.geom_rgba[self._gid] = CELL_OFF_RGBA
        self.write(0.0)
        self.write(0.0, self.name + ".power")

    def update(self, model, data, body):
        power = float(sum(self._beams.values()))
        if not self.enabled():
            return
        on = power >= float(self.live["min_power"]) and power > optics.MIN_POWER
        if on != self.on:
            model.geom_rgba[self._gid] = CELL_ON_RGBA if on else CELL_OFF_RGBA
        self.on, self.power = on, power
        self.write(float(on))
        self.write(power, self.name + ".power")

    def state(self):
        return float(self.on)

    def satisfied(self):
        return self.on


class Emitter(Part):
    """The source: one ray, `bounces` turns, and whatever the panels do to it."""

    role = "sensor"
    stage = "input"
    emits = True
    state_keys = ("on",)
    fields = (
        Field("yaw", "float", 0.0, unit="deg", min=-180.0, max=180.0, runtime=True,
              doc="direction the ray leaves in (0: +x)"),
        Field("power", "float", 1.0, min=0.0, max=100.0, runtime=True, doc="power it sends"),
        Field("bounces", "int", 2, min=0, max=8,
              doc="how often one path may turn; going straight through a filter is free"),
        Field("height", "float", 0.06, unit="m", min=0.02, max=0.5, doc="beam height"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.height = float(self.cfg["height"])
        self.bounces = int(self.cfg["bounces"])
        self.on = False
        self.optical: list[Part] = []          # panels and cells, filled when the room loads
        self.aim_setters: list[Part] = []      # setters that aim a panel in this room
        self._maps: tuple[dict, dict] | None = None

    @classmethod
    def add_room_parts(cls, parts):
        """Every emitter learns the room's optical parts, every cell watches the emitters
        (so the bus updates the emitter before the cell it feeds), and every cell gets this
        emitter as its `plan_driver`.

        That last one is a modelling decision, not geometry. The planner works on reachable
        cells and channels and cannot trace a ray, so a photocell left on its own reads as
        an input nothing can turn on ("unreachable: ['eye']", measured while building
        `puzzles/light/aim_the_mirror`) and the room is refused before it runs. What the
        emitter claims instead is: the light arrives once the aiming the room declares has
        been done - the inputs of any setter that points at one of the room's optical panels (a setter is
        a device, so it writes nothing while planning and its own channel cannot be read
        here). `any`, not `all`: a room may offer several settings of which one is right,
        and the planner has no way to tell which - that is what the solver measures. So
        `mpg check`'s `order` stage says "the light needs the controls operated", never
        "it needs this value". The claim is
        coarse on purpose; the scripted solver, not the planner, is what measures whether
        the beam really lands."""
        emitters = [p for p in parts.values() if isinstance(p, cls)]
        if not emitters:
            return
        optical = [p for p in parts.values()
                   if hasattr(p, "interact") or getattr(p, "is_photocell", False)]
        channels = [e.outputs[0] for e in emitters if e.outputs]
        panels = {n for n, p in parts.items() if hasattr(p, "interact")}
        aim = [p for p in parts.values()
               if p.type_name == "setter"
               and str(p.cfg.get("target", "")).rpartition(".")[0] in panels]
        for e in emitters:
            e.optical = optical
            e.aim_setters = list(aim)
        for p in optical:
            if getattr(p, "is_photocell", False):
                p.watch = list(dict.fromkeys([*p.watch, *channels]))
                p.plan_driver = emitters[0]

    def plan_values(self, on: set[str], get) -> dict[str, float]:
        """Planning: an emitter is not something the body switches on, so its channel
        follows the same claim it makes for its cells."""
        return {ch: float(self.plan_drives(get)) for ch in self.outputs}

    def plan_drives(self, get) -> bool:
        """Planning: this emitter lights its cells once its own `in` and every aiming
        setter in the room are on."""
        if self.inputs and not self.plan_inputs_on(get):
            return False
        return not self.aim_setters or any(s.plan_inputs_on(get) for s in self.aim_setters)

    def build(self, spec):
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/lamp", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[POST, POST, POST_H / 2], pos=[0, 0, POST_H / 2], rgba=LAMP_RGBA)
        for i in range(optics.MAX_SEGMENTS):
            body.add_geom(name=f"{self.body_name}/beam{i}", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                          size=[BEAM_R, BEAM_R, 0], pos=[0, 0, self.height],
                          rgba=(*BEAM_RGBA[:3], 0.0), contype=0, conaffinity=0, mass=0.0)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._beams = [model.geom(f"{self.body_name}/beam{i}").id
                       for i in range(optics.MAX_SEGMENTS)]
        self._maps = None

    def maps(self):
        if self._maps is None:
            self._maps = optics.collect(self.optical)
        return self._maps

    def direction(self) -> np.ndarray:
        yaw = float(self.live["yaw"]) * np.pi / 180.0
        return np.array([np.cos(yaw), np.sin(yaw), 0.0])

    def reset(self, model, data):
        self.on = False
        self._maps = None
        self.draw(model, [])
        self.write(0.0)

    def draw(self, model, segs) -> None:
        """Show the path: one capsule per segment, the rest with no alpha."""
        base = np.array([*self.xy, 0.0])
        for i, gid in enumerate(self._beams):
            if i < len(segs):
                seg = segs[i]
                half = max(seg.length / 2, 1e-4)
                model.geom_pos[gid] = (seg.a + seg.b) / 2 - base
                model.geom_quat[gid] = optics.quat_for(seg.b - seg.a)
                model.geom_size[gid, 1] = half
                alpha = float(np.clip(seg.power / max(float(self.live["power"]), 1e-6), 0.1, 1.0))
                model.geom_rgba[gid] = (*BEAM_RGBA[:3], BEAM_RGBA[3] * alpha)
            else:
                model.geom_rgba[gid] = (*BEAM_RGBA[:3], 0.0)
                model.geom_size[gid, 1] = 1e-4

    def update(self, model, data, body):
        surfaces, cells = self.maps()
        power = float(self.live["power"])
        segs, lit = [], {}
        if self.enabled() and (not self.inputs or self.inputs_on()) and power > optics.MIN_POWER:
            origin = np.array([*self.xy, self.height]) + self.direction() * (POST + optics.EPS)
            segs, lit = optics.trace(model, data, origin, self.direction(), power,
                                     self.bounces, surfaces, cells, exclude=self._bid)
        self.draw(model, segs)
        for part in self.optical:
            if getattr(part, "is_photocell", False):
                part.take(self.name, lit.get(part.name, 0.0))
        self.on = bool(lit)
        self.write(float(self.on))

    def state(self):
        return float(self.on)

    def satisfied(self):
        return self.on
