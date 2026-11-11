"""Warp pad (R14): moves what steps on it.

The moment a body or object (`trigger`, default `body`) enters the pad it is
sent to the destination:
- `to: {pos: [x, y], yaw: a}`: that place (the body faces `yaw` [rad]); puzzle
  state (counters, buttons) stays as it is;
- `to: reset`: back to the last checkpoint snapshot (R6), or to the episode
  start without one (counters keep their value only with `persist: true`);
- `to: next_stage`: the start of the next room in the main room's `stages:`
  list (one model, rooms side by side; `info["stage_changed"]`).
`routes: [{when: "laps.count >= 3", to: next_stage}, {else: reset}]` picks the
first route whose `when` (a safe expression, R7) is true at that moment.
The chosen route's index goes out on `<name>.route` (-1: none) and colours the
pad. After a warp the moved body or object ignores every warp pad for
`cooldown` seconds.

The body moves with `BodyAdapter.teleport` (walking controller history and
velocity cleared) onto the floor height at the destination; an object keeps
its height above the floor and its orientation, with zero velocity. The env
carries out the move (`World.requests`). Carrying held objects and ropes along
comes with grabbing and ropes (E3).
"""

from __future__ import annotations

import math
from typing import Any

import mujoco
import numpy as np

from ...expr import ExprError, compile_expr
from ...schema import Field, SchemaError
from ..base import TRIGGERS, Part
from ..input._sense import candidates, in_rect

DESTS = ("reset", "next_stage")
ROUTE_RGBA = ((0.55, 0.3, 0.85, 1.0), (0.2, 0.75, 0.85, 1.0), (0.95, 0.55, 0.15, 1.0),
              (0.85, 0.25, 0.45, 1.0))
NONE_RGBA = (0.4, 0.4, 0.4, 1.0)


def parse_dest(owner: str, d: Any) -> Any:
    if d in DESTS:
        return d
    if isinstance(d, dict) and "pos" in d:
        try:
            x, y = (float(v) for v in d["pos"])
            yaw = float(d.get("yaw", 0.0))
        except (TypeError, ValueError):
            pass
        else:
            return {"pos": [x, y], "yaw": yaw}
    raise SchemaError(f"{owner}: warp destination must be {{pos: [x, y], yaw}}, reset or "
                      f"next_stage, got {d!r}")


class Warp(Part):
    role = "warp"
    stage = "device"
    suffixes = (".route",)
    state_keys = ("uses", "inside")
    fields = (
        Field("size", "vec", [0.25, 0.25], length=2, unit="m", min=0.1, max=2.0, doc="pad size [x, y]"),
        Field("trigger", "enum", "body", choices=tuple(TRIGGERS),
              doc="what it moves: body, object, npc or any"),
        Field("to", "any", None, doc="destination: {pos: [x, y], yaw}, reset or next_stage"),
        Field("routes", "any", None,
              doc="conditional destinations: [{when: expr, to: dest}, ..., {else: dest}]"),
        Field("cooldown", "float", 1.0, unit="s", min=0.0, max=60.0,
              doc="time a warped body or object ignores warp pads"),
        Field("max_height", "float", 0.2, unit="m", min=0.01, max=1.0,
              doc="objects count when their centre is lower than this"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        c = self.cfg
        owner = f"part {name!r}"
        self.half = np.asarray(c["size"], float) / 2
        self.trigger = c["trigger"]
        self.cooldown = float(c["cooldown"])
        self.max_height = float(c["max_height"])
        if (c["to"] is None) == (c["routes"] is None):
            raise SchemaError(f"{owner}: a warp needs exactly one of `to` and `routes`")
        self.routes: list[tuple[Any, Any]] = []     # (compiled expr or None for else, dest)
        if c["to"] is not None:
            self.routes.append((None, parse_dest(owner, c["to"])))
        else:
            if not isinstance(c["routes"], list) or not c["routes"]:
                raise SchemaError(f"{owner}: `routes` must be a list of {{when, to}} / {{else}}")
            for i, r in enumerate(c["routes"]):
                if isinstance(r, dict) and set(r) == {"else"}:
                    if i != len(c["routes"]) - 1:
                        raise SchemaError(f"{owner}: routes[{i}]: `else` must come last")
                    self.routes.append((None, parse_dest(owner, r["else"])))
                elif isinstance(r, dict) and set(r) == {"when", "to"}:
                    try:
                        ex = compile_expr(str(r["when"]))
                    except ExprError as exc:
                        raise SchemaError(f"{owner}: routes[{i}].when: {exc}") from None
                    self.routes.append((ex, parse_dest(owner, r["to"])))
                    self.inputs = list(dict.fromkeys(self.inputs + ex.channels))
                else:
                    raise SchemaError(f"{owner}: routes[{i}] must be {{when, to}} or {{else: dest}}, "
                                      f"got {r!r}")
        if any(e is not None and e.stateful for e, _ in self.routes):
            self.stateful = True
        self.needs_body("walk")
        self.zone = 0                  # set by the room (stages)
        self.next_stage_pose = None    # spawn of the next stage (set by the room)
        self.uses = 0
        self.inside: list = []
        self.route = -1

    @classmethod
    def shift_cfg(cls, cfg, dx, dy):
        out = super().shift_cfg(cfg, dx, dy)

        def shift(d):
            if isinstance(d, dict) and isinstance(d.get("pos"), (list, tuple)) and len(d["pos"]) == 2:
                d = {**d, "pos": [float(d["pos"][0]) + dx, float(d["pos"][1]) + dy]}
            return d

        if out.get("to") is not None:
            out["to"] = shift(out["to"])
        if isinstance(out.get("routes"), list):
            out["routes"] = [{k: (shift(v) if k in ("to", "else") else v) for k, v in r.items()}
                             if isinstance(r, dict) else r for r in out["routes"]]
        return out

    # ---- scene ------------------------------------------------------------------
    def build(self, spec):
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/pad", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[*self.half, 0.003], pos=[0, 0, 0.003], rgba=ROUTE_RGBA[0],
                      contype=0, conaffinity=0)
        body.add_geom(name=self.body_name + "/ring", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[min(self.half) * 0.6, 0.004], pos=[0, 0, 0.004], rgba=(1, 1, 1, 0.6),
                      contype=0, conaffinity=0)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._gid = model.geom(self.body_name + "/pad").id

    def reset(self, model, data):
        for ex, _ in self.routes:
            if ex is not None:
                ex.reset()
        self.uses = 0
        self.inside = []
        self._route(model, self._choose(self.read))

    # ---- runtime ----------------------------------------------------------------
    def _choose(self, get, plan: bool = False) -> int:
        for i, (ex, _) in enumerate(self.routes):
            if ex is None:
                return i
            v = ex.plan(get) if plan else ex.eval(get, self.bus.dt)
            if v > 0.5:
                return i
        return -1

    def _route(self, model, idx: int) -> None:
        self.route = idx
        if model is not None:
            model.geom_rgba[self._gid] = ROUTE_RGBA[idx % len(ROUTE_RGBA)] if idx >= 0 else NONE_RGBA
        self.write(float(idx), self.name + ".route")

    def update(self, model, data, body):
        idx = self._choose(self.read)
        if idx != self.route:
            self._route(model, idx)
        else:
            self.write(float(idx), self.name + ".route")
        now = []
        for key, pos in candidates(self, data, body, self.trigger):
            if in_rect(pos, self.xy, self.half) and (key == "body" or pos[2] < self.max_height):
                now.append(key)
        entered = [k for k in now if k not in self.inside]
        self.inside = now
        if not self.enabled() or idx < 0 or self.world is None:
            return
        for key in entered:
            if data.time < self.world.warp_until.get(key, -math.inf):
                continue
            self.uses += 1
            self.world.requests.append(("warp", self, key, self.routes[idx][1]))

    def state(self):
        return float(self.uses > 0)

    def satisfied(self):
        return self.uses > 0

    def target_mask(self, grid):
        return grid.rect(self.xy, tuple(self.half * 0.8))

    # ---- planning ---------------------------------------------------------------
    def plan_dest(self, get) -> tuple[float, float] | None:
        """Where the body lands when it steps on the pad (planning), None for reset."""
        idx = self._choose(get, plan=True)
        if idx < 0:
            return None
        d = self.routes[idx][1]
        if d == "next_stage":
            return None if self.next_stage_pose is None else tuple(self.next_stage_pose[:2])
        if d == "reset":
            return None
        return tuple(d["pos"])
