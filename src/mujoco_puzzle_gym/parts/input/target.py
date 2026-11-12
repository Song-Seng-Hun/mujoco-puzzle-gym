"""Target (R5): a board that turns on when a cannon shell hits it.

Only shells count (`trigger: shell`; the walking body, objects and NPCs bumping
it do nothing), and only hits at `min_speed` [m/s] or faster. The speed is
the shell's in the physics step before the contact started, and one contact
of one shell counts once until it ends (`pool.Impacts` through the `substep`
hook). Output `mode: pulse` writes 1 for the body step of a hit, `latch` stays
1 after the first hit; `<name>.hits` counts hits.

The board (`size` [width, height], centre `height` above the floor) stands on
a thin post and faces `yaw` (deg, 0 = its face looks along +x). The ballistic
check of a cannon that aims by signal (R16 `cannon_turret`) reads `face()`.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from ... import markers
from ...schema import Field
from ..base import Part
from ..pool import Impacts

THICK = 0.02
OFF_RGBA = (0.9, 0.9, 0.9, 1.0)
ON_RGBA = (0.95, 0.25, 0.2, 1.0)


class Target(Part):
    role = "sensor"
    marked = True
    marker_anchor = "/board"
    marker_faces = markers.FACES_PANEL
    stage = "input"
    emits = True
    suffixes = (".hits",)
    state_keys = ("on", "hits", "impacts")
    fields = (
        Field("size", "vec", [0.2, 0.2], length=2, unit="m", min=0.03, max=2.0, doc="board [width, height]"),
        Field("height", "float", 0.2, unit="m", min=0.05, max=2.0, doc="board centre above the floor"),
        Field("yaw", "float", 0.0, unit="deg", min=-180.0, max=180.0, doc="direction the face looks"),
        Field("min_speed", "float", 1.0, unit="m/s", min=0.0, max=30.0, runtime=True,
              doc="slowest hit that counts"),
        Field("mode", "enum", "latch", choices=("pulse", "latch"),
              doc="pulse: 1 for the body step of a hit; latch: stays 1"),
        Field("trigger", "enum", "shell", choices=("shell",), doc="only cannon shells count"),
    )

    def marker_face_size(self, face):
        return float(min(self.cfg["size"]))

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.w, self.h = (float(v) for v in self.cfg["size"])
        self.height = float(self.cfg["height"])
        self.yaw = float(self.cfg["yaw"])
        self.mode = self.cfg["mode"]
        if self.mode == "latch":
            self.stateful = True
        self.on = False
        self.hits = 0
        self.impacts = Impacts()

    def face(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, tuple[float, float]]:
        """(centre [x, y, z], normal, horizontal in-plane axis, half [width, height])."""
        a = math.radians(self.yaw)
        n = np.array([math.cos(a), math.sin(a), 0.0])
        u = np.array([-math.sin(a), math.cos(a), 0.0])
        return np.array([*self.xy, self.height]), n, u, (self.w / 2, self.h / 2)

    def build(self, spec):
        a = math.radians(self.yaw) / 2
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, 0.0],
                                       quat=[math.cos(a), 0, 0, math.sin(a)])
        body.add_geom(name=self.body_name + "/board", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[THICK / 2, self.w / 2, self.h / 2], pos=[0, 0, self.height], rgba=OFF_RGBA)
        body.add_geom(name=self.body_name + "/bull", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[min(self.w, self.h) * 0.2, 0.001], pos=[THICK / 2 + 0.001, 0, self.height],
                      quat=[math.sqrt(0.5), 0, math.sqrt(0.5), 0], rgba=ON_RGBA, contype=0, conaffinity=0)
        bottom = self.height - self.h / 2
        if bottom > 0.002:
            body.add_geom(name=self.body_name + "/post", type=mujoco.mjtGeom.mjGEOM_BOX,
                          size=[0.01, 0.01, bottom / 2], pos=[-THICK, 0, bottom / 2],
                          rgba=(0.4, 0.4, 0.4, 1.0))
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._gid = model.geom(self.body_name + "/board").id

    def reset(self, model, data):
        self.on = False
        self.hits = 0
        self.impacts = Impacts()
        model.geom_rgba[self._gid] = OFF_RGBA

    def substep(self, model, data):
        if self.world is not None:
            self.impacts.substep(model, data, self.world, {self._bid})

    def update(self, model, data, body):
        events, self.impacts.events = self.impacts.events, []
        if not self.enabled():
            return
        new = sum(1 for v in events if v >= float(self.live["min_speed"]))
        self.hits += new
        on = self.on if self.mode == "latch" and self.on else new > 0
        if on != self.on:
            model.geom_rgba[self._gid] = ON_RGBA if on else OFF_RGBA
        self.on = on
        for ch in self.outputs:
            self.write(float(self.hits) if ch == self.name + ".hits" else float(self.on), ch)

    def state(self):
        return float(self.on)

    def check_static(self, env):
        """R16 ballistic check: a cannon aiming by signal must be able to hit this target."""
        from ..trap.cannon import Cannon

        turrets = [p for p in env.parts.values() if isinstance(p, Cannon) and p.aim == "signal"]
        if not turrets or any(c.can_hit(self, env.parts) for c in turrets):
            return []
        aims = "; ".join(f"{c.name}: {c.aim_text(env.parts)}, power {float(c.live['power']):g} m/s"
                         + (f", spread {float(c.live['spread']):g} deg (whole cone)" if c.live["spread"] else "")
                         for c in turrets)
        return [f"target {self.name!r}: no aim of the signal-aimed cannons hits it at "
                f"min_speed {float(self.live['min_speed']):g} m/s or faster ({aims}; gravity, no drag)"]

    def satisfied(self):
        return self.on
