"""Episode randomization (R35): the samplers a room declares and the env draws from.

A room says **what may change and how far**, a task says **how much of that to use
this run**. The same room can be trained fixed or randomized.

    parts:
      - name: switch_a
        type: switch
        pos: [-0.3, 0.5]
        randomize:
          pos: {region: [[-1.0, -0.1, 0.0, 0.8]], min_gap: 0.12}
      - name: cannon_a
        preset: trap/cannon
        randomize:
          power: {range: 0.5, holdout_scale: 1.5}
          pitch: {candidates: [10, 20, 30], holdout: [40]}
    spawn: {pos: [-0.9, 0.0], yaw: 0.0, randomize: {pos: {range: [0.15, 0.3]}, yaw: {range: 3.14}}}
    max_resample: 20

Samplers (same names for positions and for values):

- `range`: the nominal value +- a half-range. M1's `random: [dx, dy]` is this
  sampler written short, and the two cannot both be given for one part.
- `region`: anywhere inside a list of boxes `[x0, x1, y0, y1]`, keeping `min_gap`
  from the walls and from the other parts. Positions only.
- `candidates`: one entry of a list, optionally with `weights`. Positions and values.

Layers and strengths (task.yaml `randomize:`): a position or spawn draw belongs to
the `layout` layer, a part value to `physics`, and observation noise to `body`
(obs.py). The task scales each layer: `{layout: 1.0, physics: 0.5}` uses half of
every range a value declares, and `0` pins the layer to its nominal value. Every
implemented layer defaults to 1.0, so a room that declares a range is randomized
unless the task turns it off. `structure`, `look` and `rooms` are not implemented
yet and are refused above 0.

Draw modes: `train` (the declared ranges), `nominal` (no draw at all) and `wide`
(the evaluation-only values only: `holdout` entries, `holdout_region` boxes, and
for `range` the band from one to `holdout_scale` times the half-range).

Every draw comes from the episode's own generator in one fixed order (the spawn,
then the parts in room order, each part's targets by name), so the same seed gives
the same room. A draw that cannot be solved is thrown away and drawn again, up to
the room's `max_resample` (default 20); `mpg check` measures how often that
happens (`check.check_resample`).
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .parts.base import CUE_OFF_RGBA

#: Randomization layers (R35). `structure`, `look` and `rooms` are declared by the
#: spec but not implemented yet.
LAYERS = ("layout", "structure", "physics", "body", "look", "rooms")
DONE_LAYERS = ("layout", "structure", "physics", "body")
MODES = ("nominal", "train", "wide")
KINDS = ("range", "region", "candidates")
DEFAULT_MAX_RESAMPLE = 20
HOLDOUT_SCALE = 1.5          # `range` wide band: 1 .. this times the half-range
REGION_TRIES = 50            # `region` attempts to satisfy `min_gap` within one draw
#: Colours a drawn link is painted with (R35 cue), one per `shuffle: {links: ...}` entry.
CUE_RGBA = ((0.95, 0.55, 0.15, 1.0), (0.30, 0.75, 0.95, 1.0), (0.65, 0.40, 0.90, 1.0),
            (0.35, 0.85, 0.45, 1.0))


class RandomizeError(ValueError):
    """A room's `randomize:` block does not make sense (raised at the `values` stage)."""


# ---- samplers ---------------------------------------------------------------------
@dataclass
class Sampler:
    """One drawn thing: `where` names it in messages, `layer` is the task strength."""

    where: str
    layer: str

    def draw(self, rng: np.random.Generator, nominal, scale: float, mode: str,
             ctx=None, owner: str | None = None):
        raise NotImplementedError

    def wide_ok(self) -> bool:
        """Whether this sampler has an evaluation-only value to draw in `wide` mode."""
        return False

    def extremes(self, nominal) -> list[np.ndarray]:
        """Edge positions a draw can reach, for the static checks that must hold
        everywhere the part can land (letter-map placement, R37)."""
        return []

    def width(self) -> int:
        """Numbers this sampler contributes to the `randomized` observation."""
        return 1

    def norm(self, drawn, nominal) -> np.ndarray:
        """The drawn value as numbers in [-1, 1] (obs item `randomized`)."""
        return np.zeros(self.width())


@dataclass
class Range(Sampler):
    amount: Any = 0.0                     # float, or [dx, dy] for a position
    holdout_scale: float = HOLDOUT_SCALE
    lo: float | None = None               # schema range, drawn values are clipped
    hi: float | None = None
    integer: bool = False                 # a whole-number value (`int` field)

    def draw(self, rng, nominal, scale, mode, ctx=None, owner=None):
        a = np.asarray(self.amount, dtype=float) * float(scale)
        base = np.asarray(nominal, dtype=float)
        if mode == "wide":
            band = np.abs(a) * (self.holdout_scale - 1.0)
            sign = np.where(rng.random(a.shape if a.ndim else None) < 0.5, -1.0, 1.0)
            out = base + sign * (np.abs(a) + rng.random(a.shape if a.ndim else None) * band)
        else:
            out = base + rng.uniform(-a, a)
        out = np.clip(out, -np.inf if self.lo is None else self.lo,
                      np.inf if self.hi is None else self.hi)
        if self.integer:
            out = np.round(out)
        if np.ndim(nominal):
            return out.astype(int).tolist() if self.integer else out
        return int(out) if self.integer else float(out)

    def wide_ok(self) -> bool:
        return float(np.max(np.abs(self.amount))) > 0.0 and self.holdout_scale > 1.0

    def extremes(self, nominal):
        a = np.abs(np.asarray(self.amount, dtype=float)) * self.holdout_scale
        if a.ndim == 0 or len(a) != 2:
            return []
        c = np.asarray(nominal, dtype=float)
        return [c + np.array([sx * a[0], sy * a[1]])
                for sx in (-1, 1) for sy in (-1, 1)]

    def width(self) -> int:
        return int(np.size(self.amount))

    def norm(self, drawn, nominal) -> np.ndarray:
        a = np.abs(np.asarray(self.amount, dtype=float))
        d = np.asarray(drawn, dtype=float) - np.asarray(nominal, dtype=float)
        out = np.divide(d, a, out=np.zeros(np.shape(d), dtype=float), where=a > 0)
        return np.atleast_1d(np.clip(out, -self.holdout_scale, self.holdout_scale))


@dataclass
class Region(Sampler):
    boxes: list[tuple[float, float, float, float]] = field(default_factory=list)
    min_gap: float = 0.0
    holdout: list[tuple[float, float, float, float]] = field(default_factory=list)

    def draw(self, rng, nominal, scale, mode, ctx=None, owner=None):
        boxes = self.holdout if mode == "wide" else self.boxes
        if float(scale) <= 0.0:
            return np.asarray(nominal, dtype=float)
        areas = np.array([(b[1] - b[0]) * (b[3] - b[2]) for b in boxes], dtype=float)
        best = None
        for _ in range(REGION_TRIES):
            b = boxes[int(rng.choice(len(boxes), p=areas / areas.sum()))]
            xy = np.array([rng.uniform(b[0], b[1]), rng.uniform(b[2], b[3])])
            best = xy if best is None else best
            if ctx is None or ctx.clear(xy, self.min_gap, owner):
                return xy
        return best            # the caller's validity check throws this draw away

    def wide_ok(self) -> bool:
        return bool(self.holdout)

    def extremes(self, nominal):
        return [np.array([x, y]) for b in (*self.boxes, *self.holdout)
                for x in (b[0], b[1]) for y in (b[2], b[3])]


@dataclass
class Candidates(Sampler):
    options: list[Any] = field(default_factory=list)
    weights: list[float] | None = None
    holdout: list[Any] = field(default_factory=list)

    def draw(self, rng, nominal, scale, mode, ctx=None, owner=None):
        if float(scale) <= 0.0:
            return copy.deepcopy(nominal)
        opts = self.holdout if mode == "wide" else self.options
        w = None if (mode == "wide" or self.weights is None) else \
            np.asarray(self.weights, dtype=float) / float(sum(self.weights))
        return copy.deepcopy(opts[int(rng.choice(len(opts), p=w))])

    def wide_ok(self) -> bool:
        return bool(self.holdout)

    def extremes(self, nominal):
        out = []
        for o in (*self.options, *self.holdout):
            try:                                  # value candidates are not positions
                p = np.asarray(o, dtype=float)
            except (TypeError, ValueError):
                continue
            if p.shape == (2,):
                out.append(p)
        return out

    def norm(self, drawn, nominal) -> np.ndarray:
        """Which entry was drawn, spread over [-1, 1] (training entries then the
        evaluation-only ones, so a `wide` draw lands past the training values)."""
        opts = [*self.options, *self.holdout]
        for i, o in enumerate(opts):
            if _same(o, drawn):
                return np.array([2.0 * i / max(len(opts) - 1, 1) - 1.0])
        return np.zeros(1)


@dataclass
class Places(Sampler):
    """`shuffle: {places: [...]}`: the listed parts swap their nominal places.

    The parts must be of one type, so a policy cannot tell them apart by shape."""

    names: list[str] = field(default_factory=list)

    def permute(self, rng, pos: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
        order = list(rng.permutation(len(self.names)))
        return {self.names[i]: pos[self.names[j]].copy() for i, j in enumerate(order)}


@dataclass
class LinkSource(Sampler):
    """`shuffle: {links: <reader>, from: [...]}`: which part feeds `reader` this episode.

    The reader keeps the rest of its `in` channels; the drawn source's channel
    replaces whichever candidate the room wired. `hidden: true` leaves the cue
    (the matching colour and the `part_links` observation) off, which `mpg check`
    warns about: the room then needs memory or search to solve."""

    reader: str = ""
    options: list[str] = field(default_factory=list)
    holdout: list[str] = field(default_factory=list)
    hidden: bool = False
    nominal: str = ""                     # the source the room file wired

    def pick(self, rng, mode: str) -> str:
        opts = self.holdout if mode == "wide" else self.options
        return str(opts[int(rng.choice(len(opts)))])

    def wide_ok(self) -> bool:
        return bool(self.holdout)

    def width(self) -> int:
        return len(self.options) + len(self.holdout)

    def bits(self, drawn: str | None) -> np.ndarray:
        """The cue in the observation: a one-hot over the candidates, zeros when the
        cue is hidden or nothing was drawn."""
        out = np.zeros(self.width())
        opts = [*self.options, *self.holdout]
        if not self.hidden and drawn in opts:
            out[opts.index(drawn)] = 1.0
        return out


def _same(a, b) -> bool:
    """Whether two drawn candidate entries are the same value (numbers or anything else)."""
    try:
        return bool(np.allclose(np.asarray(a, dtype=float), np.asarray(b, dtype=float)))
    except (TypeError, ValueError):
        return a == b


# ---- parsing ----------------------------------------------------------------------
def _boxes(where: str, key: str, raw) -> list[tuple[float, float, float, float]]:
    if not isinstance(raw, (list, tuple)) or not raw:
        raise RandomizeError(f"{where}: `{key}` is a list of boxes [x0, x1, y0, y1]")
    out = []
    for b in raw:
        try:
            x0, x1, y0, y1 = (float(v) for v in b)
        except (TypeError, ValueError):
            raise RandomizeError(f"{where}: `{key}` box must be [x0, x1, y0, y1], "
                                 f"got {b!r}") from None
        if x1 <= x0 or y1 <= y0:
            raise RandomizeError(f"{where}: `{key}` box [x0, x1, y0, y1] needs x1 > x0 and "
                                 f"y1 > y0, got {list(b)}")
        out.append((x0, x1, y0, y1))
    return out


def _xy_list(where: str, key: str, raw) -> list[np.ndarray]:
    if not isinstance(raw, (list, tuple)) or not raw:
        raise RandomizeError(f"{where}: `{key}` is a list of [x, y] points")
    out = []
    for p in raw:
        try:
            x, y = (float(v) for v in p)
        except (TypeError, ValueError):
            raise RandomizeError(f"{where}: `{key}` entry must be [x, y], got {p!r}") from None
        out.append(np.array([x, y]))
    return out


def _amount(where: str, raw, length: int | None) -> Any:
    """`range: 0.5` or `range: [dx, dy]` (positions), always non-negative."""
    try:
        v = np.asarray(raw, dtype=float)
    except (TypeError, ValueError):
        raise RandomizeError(f"{where}: `range` must be a number or a list of numbers, "
                             f"got {raw!r}") from None
    if v.ndim > 1 or (length is not None and (v.ndim != 1 or len(v) != length)):
        raise RandomizeError(f"{where}: `range` for a position is [dx, dy], got {raw!r}")
    if np.any(v < 0) or not np.all(np.isfinite(v)):
        raise RandomizeError(f"{where}: `range` must be finite and >= 0, got {raw!r}")
    return v if v.ndim else float(v)


def _one_kind(where: str, spec: dict) -> str:
    kinds = [k for k in KINDS if k in spec]
    if len(kinds) != 1:
        raise RandomizeError(f"{where}: give exactly one of {list(KINDS)}, got "
                             f"{sorted(spec) or 'nothing'}")
    return kinds[0]


def _check_keys(where: str, spec: dict, kind: str) -> None:
    allowed = {"range": {"range", "holdout_scale"},
               "region": {"region", "min_gap", "holdout_region"},
               "candidates": {"candidates", "weights", "holdout", "pick"}}[kind]
    extra = sorted(set(spec) - allowed)
    if extra:
        raise RandomizeError(f"{where}: `{kind}` takes {sorted(allowed)}, got extra {extra}")


def parse_position(where: str, spec: Any, layer: str = "layout") -> Sampler:
    """A `pos` sampler: `range`, `region` or `candidates` of [x, y] points."""
    if not isinstance(spec, dict):
        raise RandomizeError(f"{where}: write {{range: [dx, dy]}}, {{region: [[x0, x1, y0, y1]]}} "
                             f"or {{candidates: [[x, y], ...]}}, got {spec!r}")
    kind = _one_kind(where, spec)
    _check_keys(where, spec, kind)
    if kind == "range":
        return Range(where, layer, _amount(where, spec["range"], 2),
                     _holdout_scale(where, spec))
    if kind == "region":
        gap = spec.get("min_gap", 0.0)
        if isinstance(gap, bool) or not isinstance(gap, (int, float)) or gap < 0:
            raise RandomizeError(f"{where}: `min_gap` must be a number >= 0 [m], got {gap!r}")
        return Region(where, layer, _boxes(where, "region", spec["region"]), float(gap),
                      _boxes(where, "holdout_region", spec["holdout_region"])
                      if "holdout_region" in spec else [])
    pts = _xy_list(where, "candidates", spec["candidates"])
    return Candidates(where, layer, pts, _weights(where, spec, len(pts)),
                      _xy_list(where, "holdout", spec["holdout"]) if "holdout" in spec else [])


def parse_value(where: str, spec: Any, f, layer: str = "physics") -> Sampler:
    """A part value sampler, checked against that value's schema field `f`."""
    if not isinstance(spec, dict):
        raise RandomizeError(f"{where}: write {{range: <amount>}} or {{candidates: [...]}}, "
                             f"got {spec!r}")
    kind = _one_kind(where, spec)
    _check_keys(where, spec, kind)
    if kind == "region":
        raise RandomizeError(f"{where}: `region` draws a position, not a value; "
                             f"use `range` or `candidates`")
    if kind == "range":
        if f.kind not in ("float", "int", "vec"):
            raise RandomizeError(f"{where}: `range` needs a number value ({f.name} is a {f.kind}); "
                                 f"use `candidates`")
        amount = _amount(where, spec["range"], f.length if f.kind == "vec" else None)
        return Range(where, layer, amount, _holdout_scale(where, spec),
                     None if f.min is None else float(f.min),
                     None if f.max is None else float(f.max), f.kind == "int")
    opts = list(spec["candidates"])
    if len(opts) < 1:
        raise RandomizeError(f"{where}: `candidates` needs at least one entry")
    if "pick" in spec and spec["pick"] != 1:
        raise RandomizeError(f"{where}: `pick` must be 1 here; picking several entries is "
                             f"for part counts, which are not implemented yet")
    hold = list(spec["holdout"]) if "holdout" in spec else []
    for v in (*opts, *hold):
        try:
            f.check(where, v)             # a candidate out of the value's range is refused
        except ValueError as exc:
            raise RandomizeError(str(exc)) from None
    return Candidates(where, layer, opts, _weights(where, spec, len(opts)), hold)


def _holdout_scale(where: str, spec: dict) -> float:
    v = spec.get("holdout_scale", HOLDOUT_SCALE)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 1.0:
        raise RandomizeError(f"{where}: `holdout_scale` must be a number > 1, got {v!r}")
    return float(v)


def _weights(where: str, spec: dict, n: int) -> list[float] | None:
    if "weights" not in spec:
        return None
    w = spec["weights"]
    if (not isinstance(w, (list, tuple)) or len(w) != n
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or v < 0 for v in w)
            or sum(w) <= 0):
        raise RandomizeError(f"{where}: `weights` must be {n} numbers >= 0 that are not all 0, "
                             f"got {w!r}")
    return [float(v) for v in w]


# ---- the room's whole randomization ------------------------------------------------
@dataclass
class Entry:
    """One sampler in the fixed draw order."""

    owner: str              # "spawn" or a part name
    target: str             # "pos", "yaw" or a part value
    sampler: Sampler


class Clearance:
    """`min_gap` helper: where everything else stands in this draw, and the walls.

    `taken` is keyed by owner so a part is never measured against its own place."""

    def __init__(self, walls, taken: dict[str, np.ndarray]):
        self.walls = list(walls)
        self.taken = dict(taken)

    def clear(self, xy, gap: float, owner: str | None = None) -> bool:
        if gap <= 0.0:
            return True
        if any(float(np.hypot(*(np.asarray(xy, float) - t))) < gap
               for n, t in self.taken.items() if n != owner):
            return False
        return all(_seg_distance(xy, w) >= gap for w in self.walls)


def _seg_distance(xy, seg) -> float:
    """Distance from a point to a wall segment's surface (its thickness removed)."""
    p0, p1 = np.asarray(seg.p0, float), np.asarray(seg.p1, float)
    v = p1 - p0
    n = float(v @ v)
    t = 0.0 if n <= 1e-12 else float(np.clip((np.asarray(xy, float) - p0) @ v / n, 0.0, 1.0))
    return float(np.hypot(*(np.asarray(xy, float) - (p0 + t * v)))) - seg.thickness / 2


class Randomizer:
    """A room's declared randomization plus the task's strengths (R35)."""

    def __init__(self, room, parts: dict[str, Any], task: dict[str, Any] | None = None):
        self.max_resample = _max_resample(room)
        self.strength = strengths(task)
        self.entries: list[Entry] = []
        block = room.raw.get("randomize")
        if block is not None:
            if not isinstance(block, dict) or set(block) - {"shuffle"}:
                raise RandomizeError("the room's `randomize:` takes only `shuffle:`; a part's "
                                     "own `randomize:` goes on that part")
            for sampler in parse_shuffle(block.get("shuffle"), parts):
                self.entries.append(Entry("room", sampler.where, sampler))
        spawn = (room.raw.get("spawn") or {}).get("randomize")
        if spawn is not None:
            if not isinstance(spawn, dict):
                raise RandomizeError("spawn `randomize:` must be a mapping of pos / yaw")
            if np.any(room.spawn_random) and "pos" in spawn:
                raise RandomizeError("spawn: give either `random: [dx, dy, dyaw]` or "
                                     "`randomize: {pos: ...}`, not both")
            for target in sorted(spawn):
                where = f"spawn randomize.{target}"
                if target == "pos":
                    self.entries.append(Entry("spawn", "pos", parse_position(where, spawn["pos"])))
                elif target == "yaw":
                    self.entries.append(Entry("spawn", "yaw", _spawn_yaw(where, spawn["yaw"])))
                else:
                    raise RandomizeError(f"spawn `randomize:` takes pos and yaw, got {target!r}")
        for name, part in parts.items():
            raw = part.cfg.get("randomize")
            if raw is None:
                continue
            if not isinstance(raw, dict) or not raw:
                raise RandomizeError(f"part {name!r}: `randomize:` must be a mapping of "
                                     f"pos / value names to samplers, got {raw!r}")
            schema = part.full_schema()
            for target in sorted(raw):
                where = f"part {name!r} randomize.{target}"
                if target == "pos":
                    if np.any(part.random):
                        raise RandomizeError(f"part {name!r}: give either `random: [dx, dy]` or "
                                             f"`randomize: {{pos: ...}}`, not both")
                    self.entries.append(Entry(name, "pos", parse_position(where, raw["pos"])))
                    continue
                f = schema.get(target)
                if f is None or not (f.runtime or f.episode):
                    ok = sorted(k for k, g in schema.items() if g.runtime or g.episode)
                    raise RandomizeError(
                        f"{where}: only `pos` and values marked run-time or episode-start can be "
                        f"randomized ({name}: {ok})")
                if target in part.binds:
                    raise RandomizeError(f"{where}: this value is driven by a `bind`, which would "
                                         f"overwrite the drawn value every body step")
                self.entries.append(Entry(name, target, parse_value(where, raw[target], f)))
        self.values = sorted({(e.owner, e.target) for e in self.entries
                              if e.owner not in ("spawn", "room") and e.target != "pos"})
        #: `shuffle: {links: ...}` entries, in declaration order (the `part_links` item).
        self.links = [e.sampler for e in self.entries if isinstance(e.sampler, LinkSource)]

    # ---- drawing ------------------------------------------------------------------
    def __bool__(self) -> bool:
        return bool(self.entries)

    def active(self) -> bool:
        """Whether any entry has a non-zero strength this run."""
        return any(self.strength.get(e.sampler.layer, 0.0) > 0.0 for e in self.entries)

    def check_difficulty(self, steps) -> None:
        """R21: a value cannot be both a difficulty step and randomized (the step would
        set it and the draw would scatter it, silently undoing the step)."""
        both = sorted({(s.part, s.key) for s in steps} & set(self.values))
        if both:
            names = ", ".join(f"{p}.{k}" for p, k in both)
            raise RandomizeError(f"{names}: a value cannot be both a difficulty step and "
                                 f"randomized; drop one of the two")

    def _live(self) -> list[Entry]:
        return [e for e in self.entries if self.strength.get(e.sampler.layer, 0.0) > 0.0]

    def wide_missing(self) -> list[str]:
        """Live samplers with no evaluation-only value; those stay nominal in `wide`."""
        return [e.sampler.where for e in self._live() if not e.sampler.wide_ok()]

    def wide_ok(self) -> bool:
        """Whether `wide` has anything to draw (a room with nothing is refused)."""
        return any(e.sampler.wide_ok() for e in self._live())

    def extremes_of(self, owner: str, nominal) -> list[np.ndarray]:
        """Edge positions the samplers of `owner` ("spawn" or a part) can draw."""
        return [p for e in self.entries if e.owner == owner and e.target == "pos"
                for p in e.sampler.extremes(nominal)]

    def draw(self, room, parts: dict[str, Any], rng: np.random.Generator,
             mode: str = "train") -> dict[str, Any]:
        """One draw: spawn, part positions and part values, in the fixed order."""
        if mode not in MODES:
            raise RandomizeError(f"randomize mode must be one of {list(MODES)}, got {mode!r}")
        if mode == "wide" and not self.wide_ok():
            raise RandomizeError(
                "`wide` needs an evaluation-only value: give a `holdout`, `holdout_region` "
                "or `holdout_scale` to at least one sampler this run draws")
        spawn = [*map(float, room.spawn_pos), room.spawn_yaw]
        pos = {n: p.base_xy.copy() for n, p in parts.items()}
        values: dict[str, dict[str, Any]] = {}
        if mode == "nominal":
            return {"spawn": tuple(spawn), "parts": pos, "values": values, "links": {},
                    "mode": mode}
        if mode == "train":            # M1 `random:` shorthand: no evaluation-only band,
            d = rng.uniform(-room.spawn_random, room.spawn_random)   # so `wide` leaves it alone
            spawn = [spawn[0] + d[0], spawn[1] + d[1], spawn[2] + d[2]]
            for n, p in parts.items():
                if np.any(p.random):
                    pos[n] = p.base_xy + rng.uniform(-p.random, p.random)
        ctx = Clearance(room.walls(), {"spawn": np.array(spawn[:2]), **pos})
        links: dict[str, str] = {}
        for e in self.entries:
            scale = float(self.strength.get(e.sampler.layer, 0.0))
            if scale <= 0.0 or (mode == "wide" and not e.sampler.wide_ok()):
                continue          # `wide`: a sampler with no evaluation-only value stays nominal
            if isinstance(e.sampler, Places):
                swapped = e.sampler.permute(rng, pos)
                pos.update(swapped)
                ctx.taken.update(swapped)
                continue
            if isinstance(e.sampler, LinkSource):
                links[e.sampler.reader] = e.sampler.pick(rng, mode)
                continue
            if e.owner == "spawn":
                if e.target == "pos":
                    xy = e.sampler.draw(rng, np.array(spawn[:2]), scale, mode, ctx, "spawn")
                    spawn[0], spawn[1] = float(xy[0]), float(xy[1])
                    ctx.taken["spawn"] = np.array(spawn[:2])
                else:
                    spawn[2] = float(e.sampler.draw(rng, spawn[2], scale, mode, ctx))
                continue
            if e.target == "pos":
                pos[e.owner] = np.asarray(
                    e.sampler.draw(rng, pos[e.owner], scale, mode, ctx, e.owner), dtype=float)
                ctx.taken[e.owner] = pos[e.owner]
            else:
                nominal = parts[e.owner].room_value(e.target)   # never the last draw
                values.setdefault(e.owner, {})[e.target] = \
                    e.sampler.draw(rng, nominal, scale, mode, ctx)
        return {"spawn": tuple(float(v) for v in spawn), "parts": pos, "values": values,
                "links": links, "mode": mode}

    def apply_links(self, parts: dict[str, Any], layout: dict[str, Any]) -> None:
        """Set each shuffled reader's `in` to the source this episode drew (R35). The
        caller wires the bus again afterwards, which checks the drawn wiring."""
        for sampler in self.links:
            drawn = (layout.get("links") or {}).get(sampler.reader) or sampler.nominal
            cand = {_channel_of(parts[n]) for n in (*sampler.options, *sampler.holdout)}
            want = _channel_of(parts[drawn])
            reader = parts[sampler.reader]
            reader.inputs = [want if ch in cand else ch for ch in reader.inputs]

    def paint_cues(self, model, parts: dict[str, Any], layout: dict[str, Any]) -> None:
        """Mark a drawn link on the cue mats (R35): the reader and the source it reads
        this episode get the entry's colour, every other candidate the off colour. The
        mats are always there and the same size, so only the colour carries the cue; a
        `hidden: true` entry leaves them all off."""
        for i, sampler in enumerate(self.links):
            drawn = (layout.get("links") or {}).get(sampler.reader) or sampler.nominal
            rgba = CUE_RGBA[i % len(CUE_RGBA)]
            for name in (sampler.reader, *sampler.options, *sampler.holdout):
                part = parts.get(name)
                if part is None:
                    continue
                lit = not sampler.hidden and name in (sampler.reader, drawn)
                part.paint_cue(model, rgba if lit else CUE_OFF_RGBA)

    def link_observation(self, layout: dict[str, Any]) -> np.ndarray:
        """Obs item `part_links` (R35): per shuffled reader a one-hot over its candidate
        sources, so a policy can see which part feeds it. Hidden entries give zeros."""
        drawn = layout.get("links") or {}
        rows = [s.bits(drawn.get(s.reader) or s.nominal) for s in self.links]
        return np.concatenate(rows) if rows else np.zeros(0)

    def hidden_cues(self) -> list[str]:
        """Live link shuffles whose cue is off (`mpg check` warns: the room then needs
        memory or search, so a feed-forward policy has a success-rate ceiling)."""
        return [e.sampler.where for e in self._live()
                if isinstance(e.sampler, LinkSource) and e.sampler.hidden]

    def observation(self, layout: dict[str, Any], parts: dict[str, Any]) -> np.ndarray:
        """Obs item `randomized` (R35): the drawn run-time values, normalized. Fixed
        length per room; a nominal episode gives zeros."""
        out = []
        drawn = layout.get("values") or {}
        for owner, target in self.values:
            e = next(e for e in self.entries if e.owner == owner and e.target == target)
            v = drawn.get(owner, {}).get(target)
            nominal = parts[owner].room_value(target)
            out.append(np.zeros(e.sampler.width()) if v is None
                       else np.asarray(e.sampler.norm(v, nominal), dtype=float).ravel())
        return np.concatenate(out) if out else np.zeros(0)

    def gap_reasons(self, room, layout: dict[str, Any]) -> list[str]:
        """Reasons a drawn layout breaks a `min_gap` (the draw is then thrown away)."""
        out = []
        walls = room.walls()
        for e in self.entries:
            gap = float(getattr(e.sampler, "min_gap", 0.0) or 0.0)
            if gap <= 0.0 or e.target != "pos":
                continue
            xy = np.array(layout["spawn"][:2]) if e.owner == "spawn" else layout["parts"][e.owner]
            taken = {**layout["parts"], "spawn": np.array(layout["spawn"][:2])}
            if not Clearance(walls, taken).clear(xy, gap, e.owner):
                out.append(f"{e.sampler.where}: drawn closer than min_gap {gap:.2f} m to a wall "
                           f"or another part")
        return out


def _spawn_yaw(where: str, spec: Any) -> Sampler:
    """The spawn heading: `range` (radians) or `candidates` of headings."""
    if not isinstance(spec, dict):
        raise RandomizeError(f"{where}: write {{range: <radians>}} or {{candidates: [...]}}, "
                             f"got {spec!r}")
    kind = _one_kind(where, spec)
    _check_keys(where, spec, kind)
    if kind == "range":
        return Range(where, "layout", _amount(where, spec["range"], None),
                     _holdout_scale(where, spec))
    if kind == "candidates":
        opts = [float(v) for v in spec["candidates"]]
        hold = [float(v) for v in spec["holdout"]] if "holdout" in spec else []
        return Candidates(where, "layout", opts, _weights(where, spec, len(opts)), hold)
    raise RandomizeError(f"{where}: `region` draws a position, not a heading")


def assign_cues(block: Any, parts: dict[str, Any]) -> None:
    """Give every part a link shuffle names a cue mat slot (R35). Called while the room
    loads, before the scene is built, so the mats are in the model; the declaration is
    checked later by `parse_shuffle`, which gives the real messages."""
    if not isinstance(block, dict):
        return
    for i, e in enumerate(block.get("shuffle") or []):
        if not isinstance(e, dict) or "links" not in e:
            continue
        names = [e.get("links"), *(e.get("from") or []), *(e.get("holdout") or [])]
        for n in names:
            if isinstance(n, str) and n in parts and parts[n].cue_slot is None:
                parts[n].cue_slot = i


def parse_shuffle(raw: Any, parts: dict[str, Any]) -> list[Sampler]:
    """The room's `randomize: {shuffle: [...]}` entries (R35 structure layer)."""
    if not isinstance(raw, list) or not raw:
        raise RandomizeError("`randomize: {shuffle: [...]}` is a list of "
                             "{places: [...]} or {links: <part>, from: [...]} entries")
    out: list[Sampler] = []
    for i, e in enumerate(raw):
        where = f"randomize.shuffle[{i}]"
        if not isinstance(e, dict) or ("places" in e) == ("links" in e):
            raise RandomizeError(f"{where}: write {{places: [...]}} or "
                                 f"{{links: <part>, from: [...]}}, got {e!r}")
        if "places" in e:
            extra = sorted(set(e) - {"places"})
            if extra:
                raise RandomizeError(f"{where}: `places` takes no other key, got extra {extra}")
            names = e["places"]
            if (not isinstance(names, list) or len(names) < 2
                    or len(set(names)) != len(names)
                    or any(not isinstance(n, str) for n in names)):
                raise RandomizeError(f"{where}: `places` is a list of at least two different "
                                     f"part names, got {names!r}")
            for n in names:
                if n not in parts:
                    raise RandomizeError(f"{where}: `places` names {n!r}, which is not a part")
                if np.any(parts[n].random) or (parts[n].cfg.get("randomize") or {}).get("pos"):
                    raise RandomizeError(f"{where}: {n!r} already draws its own place; a part "
                                         f"either swaps places or draws one, not both")
            kinds = {type(parts[n]).type_name for n in names}
            if len(kinds) != 1:
                raise RandomizeError(f"{where}: `places` swaps parts of one type, got "
                                     f"{sorted(kinds)}; a policy would tell them apart")
            out.append(Places(where, "structure", list(names)))
            continue
        extra = sorted(set(e) - {"links", "from", "holdout", "hidden"})
        if extra:
            raise RandomizeError(f"{where}: `links` takes from, holdout and hidden, "
                                 f"got extra {extra}")
        reader, opts = e["links"], e.get("from")
        if not isinstance(reader, str) or reader not in parts:
            raise RandomizeError(f"{where}: `links` names the part that reads the channel, "
                                 f"got {reader!r}")
        if (not isinstance(opts, list) or len(opts) < 2 or len(set(opts)) != len(opts)
                or any(not isinstance(n, str) for n in opts)):
            raise RandomizeError(f"{where}: `from` is a list of at least two different part "
                                 f"names, got {opts!r}")
        hold = e.get("holdout") or []
        if not isinstance(hold, list) or any(not isinstance(n, str) for n in hold):
            raise RandomizeError(f"{where}: `holdout` is a list of part names, got {hold!r}")
        for n in (*opts, *hold):
            if n not in parts:
                raise RandomizeError(f"{where}: {n!r} is not a part of this room")
            if not parts[n].outputs:
                raise RandomizeError(f"{where}: {n!r} writes no channel, so it cannot feed "
                                     f"{reader!r}")
        wired = [n for n in (*opts, *hold) if _channel_of(parts[n]) in parts[reader].inputs]
        if len(wired) != 1:
            raise RandomizeError(f"{where}: {reader!r} must read exactly one of the candidates "
                                 f"in the room file (its `in` reads {list(parts[reader].inputs)}); "
                                 f"that one is the nominal wiring")
        hidden = bool(e.get("hidden", False))
        out.append(LinkSource(where, "structure", reader, list(opts), list(hold), hidden,
                              wired[0]))
    return out


def _channel_of(part) -> str:
    """The channel a candidate source feeds a reader with: its first output."""
    return part.outputs[0]


def _max_resample(room) -> int:
    v = room.raw.get("max_resample", DEFAULT_MAX_RESAMPLE)
    if isinstance(v, bool) or not isinstance(v, int) or v < 1:
        raise RandomizeError(f"`max_resample` must be a whole number >= 1, got {v!r}")
    return int(v)


def strengths(task: dict[str, Any] | None) -> dict[str, float]:
    """The task's per-layer strengths, every implemented layer defaulting to 1.0."""
    raw = (task or {}).get("randomize") or {}
    out = {k: 1.0 for k in DONE_LAYERS}
    for k, v in raw.items():
        out[k] = float(v) if not isinstance(v, bool) else (1.0 if v else 0.0)
    return out


def validate_strengths(raw: Any) -> None:
    """task.yaml `randomize:` (task.py calls this; TaskError is raised there)."""
    if not isinstance(raw, dict):
        raise RandomizeError("`randomize` must be a mapping, e.g. {layout: 1.0, physics: 0.5}")
    for k, v in raw.items():
        if k not in LAYERS:
            raise RandomizeError(f"randomize.{k}: unknown layer; the layers are {list(LAYERS)}")
        if isinstance(v, bool):
            v = 1.0 if v else 0.0
        if not isinstance(v, (int, float)) or v < 0 or not math.isfinite(v):
            raise RandomizeError(f"randomize.{k} must be a number >= 0 or a true/false, got {v!r}")
        if k not in DONE_LAYERS and v > 0:
            raise RandomizeError(f"randomize.{k} is not available yet; the layers that work are "
                                 f"{list(DONE_LAYERS)}")
