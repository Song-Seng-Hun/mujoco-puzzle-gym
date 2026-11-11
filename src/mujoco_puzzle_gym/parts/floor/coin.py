"""Coins (R36): floor pickups, coin groups and the coin lock.

- `coin`: picked when the walking body's centre comes within `radius` of it;
  it then disappears. No collision. Channels: `<name>` (0 -> 1 when picked,
  stays 1 until the episode ends) and `<name>.picked` (a one-step pulse when
  picked). A reset puts every coin back; a checkpoint restore puts back the
  coins picked after the snapshot. With `enable` off the coin cannot be picked.
- coin group: coins with the same `group` (default `coins`) form one; the room
  adds a part named after the group (type `coin_group`, no body) that writes
  `<group>.count` (coins picked), `<group>.total` (coins in the group) and
  `<group>.picked` (one pulse per picked coin; two coins picked in the same or
  in neighbouring steps give two pulses with a zero step between them, so a
  counter on it counts both). The group is what the solution order plans
  with: it is a switch, on once `need` coins are picked; its target area is the
  coins not picked yet, so the scripted solver walks to the nearest of them.
- `coin_lock`: an R8b counter whose `up` is `<group>.picked` and whose target
  is `need` (`all`, the default, = every coin of the group, or a number).
  Channels `<name>.count` and `<name>.reached`; a door with
  `in: [<name>.reached]` opens once enough coins are picked.

`mpg check` (order stage) rejects a coin the body can never reach (inside a
wall, behind a door that never opens) and a group whose `need` coins are not
reachable when the order needs them (behind the door they open):
`unreachable_coins`.
"""

from __future__ import annotations

from typing import Any

import mujoco
import numpy as np

from ...schema import Field, SchemaError
from ..base import Part
from ..logic.counter import Counter

COIN_RGBA = (0.98, 0.78, 0.12, 1.0)
GROUP_TYPE = "coin_group"


class Coin(Part):
    stage = "input"
    emits = True
    suffixes = (".picked",)
    state_keys = ("picked",)
    fields = (
        Field("radius", "float", 0.08, unit="m", min=0.02, max=0.5,
              doc="pick-up distance from the body centre"),
        Field("group", "str", "coins", doc="coin group (channels <group>.count, <group>.total)"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.radius = float(self.cfg["radius"])
        self.group = self.cfg["group"]
        self.needs_body("walk")
        self.picked = False
        self._gid = -1

    @classmethod
    def add_room_parts(cls, parts: dict[str, Part]) -> None:
        ensure_groups(parts)

    def build(self, spec):
        body = super().build(spec)
        body.add_geom(name=self.body_name + "/coin", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[0.035, 0.004], pos=[0, 0, 0.012], rgba=COIN_RGBA,
                      contype=0, conaffinity=0)
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self._gid = model.geom(self.body_name + "/coin").id

    def _show(self, model) -> None:
        if model is not None and self._gid >= 0:
            model.geom_rgba[self._gid, 3] = 0.0 if self.picked else COIN_RGBA[3]

    def _write(self, pulse: bool) -> None:
        if self.bus is not None:
            self.write(float(self.picked), self.outputs[0])
            self.write(float(pulse), self.outputs[-1])

    def reset(self, model, data):
        self.picked = False
        self._show(model)
        self._write(False)

    def update(self, model, data, body):
        pulse = (not self.picked and self.enabled()
                 and bool(np.hypot(*(body.pos[:2] - self.xy)) < self.radius))
        if pulse:
            self.picked = True
            self._show(model)
        self._write(pulse)

    def load_state(self, model, data, st):
        super().load_state(model, data, st)
        self._show(model)

    def state(self):
        return float(self.picked)

    def satisfied(self):
        return self.picked

    def target_mask(self, grid):
        return grid.disk(self.xy, self.radius * 0.8)


class CoinGroup(Part):
    """The coins of one `group` (added by the room; see the module doc)."""

    role = "switch"
    stage = "input"
    has_body = False
    suffixes = (".count", ".total", ".picked")
    state_keys = ("count", "pending", "last")
    fields = (
        Field("pos", "vec", [0.0, 0.0], length=2, unit="m",
              doc="where guides draw the group (it has no body)"),
    )

    def __init__(self, name, cfg):
        super().__init__(name, cfg)
        self.coins: list[Coin] = []
        self.need = 0              # coins the order needs (largest coin_lock need, else all)
        self.count = 0
        self.pending = 0           # picks not pulsed yet
        self.last = False          # `.picked` was 1 last step
        self.picks = 0             # picks this episode, kept across restores (reward coin_pick)
        self.rewarded = 0

    def add_coin(self, coin: Coin) -> None:
        self.coins.append(coin)
        self.inputs.append(coin.outputs[0])
        self.need = len(self.coins)

    @property
    def total(self) -> int:
        return len(self.coins)

    def remaining(self) -> list[Coin]:
        return [c for c in self.coins if not c.picked]

    def reset(self, model, data):
        self.count, self.pending, self.last = 0, 0, False
        self.picks = self.rewarded = 0
        self.xy = np.mean([c.xy for c in self.coins], axis=0)
        self._write()

    def _write(self) -> None:
        if self.bus is not None:
            self.write(float(self.count), self.outputs[0])
            self.write(float(self.total), self.outputs[1])
            self.write(float(self.last), self.outputs[2])

    def update(self, model, data, body):
        count = sum(c.picked for c in self.coins)
        if count > self.count:
            self.pending += count - self.count
            self.picks += count - self.count
        self.count = count
        self.last = self.pending > 0 and not self.last
        self.pending -= int(self.last)
        left = self.remaining()
        if left and body is not None:
            d = [np.hypot(*(c.xy - body.pos[:2])) for c in left]
            self.xy = left[int(np.argmin(d))].xy.copy()
        self._write()

    def load_state(self, model, data, st):
        super().load_state(model, data, st)
        self._write()

    def state(self):
        return float(min(self.count / self.need, 1.0)) if self.need else 1.0

    def picked_ratio(self) -> float:
        return self.count / self.total if self.total else 0.0

    def satisfied(self):
        return self.count >= self.need

    def target_key(self):
        return self.count

    def target_mask(self, grid):
        left = self.remaining()
        if not left:
            return grid.disk(self.xy, 0.05)
        mask = np.zeros_like(grid.X, dtype=bool)
        for c in left:
            mask |= c.target_mask(grid)
        return mask

    def plan_values(self, on, get):
        n = self.need if self.name in on else 0
        return {self.outputs[0]: float(n), self.outputs[1]: float(self.total),
                self.outputs[2]: float(n > 0)}


class CoinLock(Counter):
    fields = (
        Field("group", "str", "coins", doc="coin group whose picks it counts"),
        Field("need", "int", None, min=1, max=10_000,
              doc="coins needed to reach; `all` (default) = every coin of the group"),
        Field("up", "str", None, doc="set to <group>.picked"),
    )

    def __init__(self, name, cfg):
        cfg = dict(cfg)
        if cfg.get("need") == "all":
            cfg["need"] = None
        group = cfg.get("group") or "coins"
        if cfg.get("up") not in (None, f"{group}.picked"):
            raise SchemaError(f"part {name!r}: a coin_lock counts `{group}.picked`; drop `up`")
        cfg["up"] = f"{group}.picked"
        super().__init__(name, cfg)
        self.group = self.cfg["group"]
        self.need = self.cfg["need"]

    @classmethod
    def add_room_parts(cls, parts: dict[str, Part]) -> None:
        ensure_groups(parts)


def ensure_groups(parts: dict[str, Part]) -> None:
    """Add a `coin_group` part per coin group and set coin lock targets (room load)."""
    groups: dict[str, CoinGroup] = {}
    for p in parts.values():
        if isinstance(p, CoinGroup):
            groups[p.name] = p
    for p in list(parts.values()):
        if not isinstance(p, Coin) or any(p is c for g in groups.values() for c in g.coins):
            continue
        g = groups.get(p.group)
        if g is None:
            if p.group in parts:
                raise ValueError(f"coin {p.name!r}: group {p.group!r} is also the name of part "
                                 f"{p.group!r}; rename one")
            g = groups[p.group] = CoinGroup(p.group, {})
            g.type_name = GROUP_TYPE
            parts[p.group] = g
        g.add_coin(p)
    for g in groups.values():
        g.base_xy = g.xy = np.mean([c.base_xy for c in g.coins], axis=0) if g.coins else g.xy
    locks = [p for p in parts.values() if isinstance(p, CoinLock)]
    for g in groups.values():
        g.need = max([lk.need or g.total for lk in locks if lk.group == g.name], default=g.total)
    for lk in locks:
        g = groups.get(lk.group)
        if g is None or not g.coins:
            raise ValueError(f"part {lk.name!r}: no coins in group {lk.group!r}")
        need = lk.need or g.total
        if need > g.total:
            raise ValueError(f"part {lk.name!r}: need {need} but group {lk.group!r} has "
                             f"{g.total} coins")
        lk.target = need
        lk.hi = max(lk.hi, need)


def unreachable_coins(env: Any, order: list[str], start) -> list[str]:
    """Static check (R36): reasons for coins the body cannot reach (empty = fine).

    Reachability from `start` uses the room grid with walls and closed gates grown by the
    body footprint (as the scripted solver plans), also through warp pads (R14):
    every coin must be reachable
    with every switch of `order` on, and a group in `order` must have `need`
    coins reachable with only the switches before it on.
    """
    from ...logic import reach_with_warps
    from ...solver import CLEARANCE

    grid, walls, parts = env.grid, env.walls, env.parts
    inflate = env.body.footprint_radius + CLEARANCE

    def reach(on: set[str]) -> np.ndarray:
        return reach_with_warps(grid, walls, parts, on, start, inflate)

    why = []
    final = reach(set(order))
    lost = [p.name for p in parts.values()
            if isinstance(p, Coin) and not (p.target_mask(grid) & final).any()]
    if lost:
        why.append(f"coins the body cannot reach (inside a wall or behind a closed door): {lost}")
    for i, name in enumerate(order):
        g = parts[name]
        if isinstance(g, CoinGroup):
            r = reach(set(order[:i]))
            ok = [c.name for c in g.coins if (c.target_mask(grid) & r).any()]
            if len(ok) < g.need:
                behind = [c.name for c in g.coins if c.name not in ok]
                why.append(f"group {name!r} needs {g.need} coins but only {len(ok)} are reachable "
                           f"before it; {behind} are behind doors that open later")
    return why
