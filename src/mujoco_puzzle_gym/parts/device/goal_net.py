"""Goal net (R38): a frame closed on three sides that counts the balls inside.

Geometry: an inner area `width` x `depth` centred at `pos`, open on the side
`facing` points to (degrees, 0 = +x, 90 = +y), walls `height` high on the
other three sides (back and two posts; no roof). The walls are dead (no
bounce, contact priority over balls) so a ball stays in. A ball is in when its centre
is inside that area and below `height`; balls the body holds never count.

`accepts: [colour ...]` counts only balls of those colours (empty: any ball).

Channels:
- `<name>`: 1 while at least `need` counted balls are in (links, `in:`);
- `<name>.scored`: 1 for one body step when a counted ball comes in;
- `<name>.count`: counted balls in now.

Role `switch` for the solution order: the net is reached when the area in
front of its mouth is; the scripted solver pushes the nearest accepted ball
into it (`push_target`, see push.py). Static check (`check_static`): fewer
than `need` accepted balls fit the mouth, or the net or a ball sits in a wall,
outside the room or out of reach.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from ...layout import Segment
from ...schema import Field
from ..base import Part
from ..object.ball import contact_solref

RGBA = (0.85, 0.85, 0.9, 1.0)
RGBA_IN = (0.3, 0.9, 0.4, 1.0)
FLOOR_RGBA = (0.9, 0.9, 0.95, 0.35)
MOUTH_LEAD = 0.15      # push target in front of the mouth for balls beside the net [m]


class GoalNet(Part):
    role = "switch"
    stage = "input"
    emits = True
    suffixes = (".scored", ".count")
    state_keys = ("inside",)
    fields = (
        Field("width", "float", 0.36, unit="m", min=0.08, max=2.0, doc="mouth width (inside)"),
        Field("depth", "float", 0.2, unit="m", min=0.05, max=1.0, doc="inside depth"),
        Field("height", "float", 0.12, unit="m", min=0.03, max=0.5, doc="wall height"),
        Field("facing", "float", 180.0, unit="deg", min=-180.0, max=180.0,
              doc="direction the open side faces"),
        Field("thickness", "float", 0.02, unit="m", min=0.005, max=0.1, doc="wall thickness"),
        Field("accepts", "list", [], doc="ball colours that count (empty: any)"),
        Field("need", "int", 1, min=1, max=100, doc="balls in for the main channel"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        self.width, self.depth, self.height = float(c["width"]), float(c["depth"]), float(c["height"])
        self.thick = float(c["thickness"])
        a = math.radians(c["facing"])
        self.u = np.array([math.cos(a), math.sin(a)])       # out of the mouth
        self.v = np.array([-self.u[1], self.u[0]])
        self.accepts_colors = list(c["accepts"])
        self.need = int(c["need"])
        self.inside: list[str] = []
        self.needs_body("walk")

    # ---- geometry --------------------------------------------------------------------
    def _local(self, xy) -> tuple[float, float]:
        d = np.asarray(xy[:2], dtype=float) - self.xy
        return float(d @ self.u), float(d @ self.v)

    def contains(self, p) -> bool:
        a, b = self._local(p)
        return abs(a) <= self.depth / 2 and abs(b) <= self.width / 2 and p[2] <= self.height

    def _walls_local(self) -> list[tuple[float, float, float, float]]:
        """(centre a, centre b, half a, half b) of back wall and two side walls."""
        t, hd, hw = self.thick, self.depth / 2, self.width / 2
        return [(-hd - t / 2, 0.0, t / 2, hw + t),
                (0.0, hw + t / 2, hd, t / 2),
                (0.0, -hw - t / 2, hd, t / 2)]

    def mouth_xy(self) -> np.ndarray:
        return self.xy + self.u * self.depth / 2

    def build(self, spec):
        body = super().build(spec)
        yaw = math.atan2(self.u[1], self.u[0])
        body.quat = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        for i, (a, b, ha, hb) in enumerate(self._walls_local()):
            body.add_geom(name=f"{self.body_name}/wall{i}", type=mujoco.mjtGeom.mjGEOM_BOX,
                          size=[ha, hb, self.height / 2], pos=[a, b, self.height / 2], rgba=RGBA,
                          priority=2, solref=contact_solref(0.0))
        body.add_geom(name=self.body_name + "/area", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[self.depth / 2, self.width / 2, 0.001], pos=[0, 0, 0.001],
                      rgba=FLOOR_RGBA, contype=0, conaffinity=0)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._area = model.geom(self.body_name + "/area").id

    # ---- balls ---------------------------------------------------------------------
    def push_accepts(self, ball) -> bool:
        return getattr(ball, "object_kind", None) == "ball" and (
            not self.accepts_colors or getattr(ball, "color", None) in self.accepts_colors)

    def balls(self):
        """(root body id, ball part) of balls this net counts, not held."""
        w = self.world
        if w is None:
            return []
        return [(rid, o) for rid, o in w.objects.items()
                if rid not in w.held and self.push_accepts(o)]

    def push_target(self, env, ball_xy) -> np.ndarray:
        """Where the solver pushes a ball at `ball_xy`: the middle of the inside area
        once the ball is in front of the mouth, else a point in front of the mouth."""
        a, b = self._local(ball_xy)
        if a > self.depth / 2 and abs(b) < self.width / 2 - 0.02:
            return self.xy.copy()
        return self.mouth_xy() + self.u * MOUTH_LEAD

    # ---- runtime -------------------------------------------------------------------
    def reset(self, model, data):
        self.inside = []
        self._write(model, scored=False)

    def _write(self, model, scored: bool) -> None:
        n = len(self.inside)
        o = self.outputs
        self.write(float(n >= self.need), o[0])
        self.write(float(scored), o[1])
        self.write(float(n), o[2])
        if model is not None:
            model.geom_rgba[self._area] = RGBA_IN if n >= self.need else FLOOR_RGBA

    def update(self, model, data, body):
        if not self.enabled():
            return
        now = [o.name for rid, o in self.balls() if self.contains(data.xpos[rid])]
        scored = any(n not in self.inside for n in now)
        self.inside = now
        self._write(model, scored)

    def state(self):
        return float(len(self.inside) >= self.need)

    def satisfied(self):
        return len(self.inside) >= self.need

    # ---- planning ------------------------------------------------------------------
    def blockers(self, is_open):
        segs = []
        for a, b, ha, hb in self._walls_local():
            c = self.xy + a * self.u + b * self.v
            if ha >= hb:
                d, th = ha * self.u, 2 * hb
            else:
                d, th = hb * self.v, 2 * ha
            segs.append(Segment(tuple(c - d), tuple(c + d), th))
        return segs

    def target_mask(self, grid):
        """In front of the mouth: where the body stands when the ball goes in."""
        return grid.disk(self.mouth_xy() + self.u * 0.12, 0.08)

    def check_static(self, env) -> list[str]:
        """Static reasons this net cannot be scored (R38)."""
        why = []
        fits = [o for o in env.parts.values() if self.push_accepts(o)
                and 2 * o.radius <= self.width and o.radius <= self.height]
        if len(fits) < self.need:
            big = [f"{o.name} (diameter {2 * o.radius:g} m)" for o in env.parts.values()
                   if self.push_accepts(o) and o not in fits]
            why.append(f"goal net {self.name!r}: needs {self.need} accepted balls that fit its "
                       f"mouth (width {self.width:g} m, height {self.height:g} m), has {len(fits)}"
                       + (f"; too big: {', '.join(big)}" if big else ""))
        hx, hy = env.room.size[0] / 2, env.room.size[1] / 2
        if (np.abs(self.xy) > [hx, hy]).any():
            why.append(f"goal net {self.name!r} at {self.xy.tolist()} is outside the room")
        elif not (self.target_mask(env.grid) & ~env.grid.blocked(env.walls)).any():
            why.append(f"goal net {self.name!r}: its mouth faces a wall")
        return why
