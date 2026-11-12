"""Room files (R5): read and validate room.yaml, build body + room with MjSpec, draw a top view.

room.yaml (lengths in metres, angles in radians; the room is centred at the origin):

    name: switch_door
    size: [2.6, 1.8]              # interior x, y; outer walls are added
    wall: {thickness: 0.06, height: 0.25}
    walls:                        # inner walls as segments
      - {from: [0.3, -0.9], to: [0.3, -0.25]}
    spawn: {pos: [-0.9, 0.0], yaw: 0.0, random: [0.15, 0.3, 3.14]}
    parts:                        # `type` is a registered part (register_part)
      - {name: switch_a, type: switch, pos: [-0.3, 0.5], size: [0.25, 0.25], random: [0.1, 0.1]}
      - {name: door_b, type: door, pos: [0.3, 0.0], width: 0.5, axis: y}
      - {name: goal, type: goal, pos: [0.9, 0.0], radius: 0.15, random: [0.1, 0.4]}
    links:                        # switch -> gate
      - {from: switch_a, to: door_b}
    floor: {friction: [1.0, 0.005, 0.0001]}   # optional: default floor friction (R10)
    stages: [room2.yaml]          # optional: next rooms, side by side in one model (R14)

Presets (R3, catalog.py): a part entry `{name, preset: input/floor_oneshot, ...}`
takes the preset's type and values and overrides some; a top-level `preset:`
(name or list, `puzzles/<topic>/<name>`) merges assembly presets under the
room (parts by name). Overridden values end up in `task.resolved.yaml`
(`room_parts`, `resolved_parts`).

`random` entries are uniform half-ranges (+-) applied on every env reset.
A link is a signal channel (R1, R2): it adds the switch's channel to the
door's `in`, the same as writing `in: [switch_a]` on the door.

Floors (R10): a room with floor parts (`floor_tile`, `trapdoor`) gets a tile
floor without gaps (floor.py) and a catch plane below; otherwise the M1 plane.
`floor.friction` sets the default floor friction for either.

Letter maps (R37): `map:` (with `cell`, `step_height`, `legend`; roommap.py)
replaces `size` and `walls` (using both is an error): no outer walls are
added, the floor is tiles of the map's heights and ramps, and cliff cells have
no floor. `on_fall: fail | reset` (default fail) says what a fall into a
cliff or open trapdoor does: end the episode, or go back to the last
checkpoint (the episode start without one).

Stages (R14): `stages:` lists further room files (paths relative to this
one). Each is built in its own zone to the +x side of the previous one, in
the same model, so observation and snapshot sizes stay fixed; a warp pad with
`to: next_stage` moves the body to the next zone's spawn. Part names must be
unique over all stages; channels are shared. The goal requirement holds for
the stages together (an earlier stage may have none).

Demo (R20): `demo:` lists the scripted solver's sub-goals for `mpg check`, in
order (strings): `goto <part>`, `press <part>`, `wait <seconds>`, `push <ball>
to <net or pin>`, `herd <npc> <part or x y>` (R29), `grab <object>`, `place
<object> <part>` (R18; a hook: hang it), `drop <object> <part>` (let go from
the level head, R28), `turn <lever> <value>` (a push lever to that cell, R28),
`turn <dial> <cell>` (R28), `pull <cord or drawer>` (R28), `look <part>` (until `<part>.seen`, R27). Objects are grabbable parts (cube, laundry, key, ...). After the last one the solver goes on with the grid
answer (the solution order), so a demo may stop before the goal. Without
`demo:` the solver uses the grid answer only.

Markers and gaze (R26, R27): marked parts get ArUco markers (markers/);
`gaze: {fov, max_dist, max_tilt, dwell}` sets the gaze sensor (gaze.py), and a
part reading `<part>.seen` adds the hidden `_gaze` part writing it.

Errors: `RoomError.stage` names the `mpg check` static stage a load error
belongs to (`values`: a value out of its range, R3; `channels`: a channel no
part writes, two writers, a stateless loop or a link to a missing part, R1;
`demo`: a bad `demo:` entry); None for a file that cannot be read at all.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import yaml

from . import catalog
from . import floor as floor_mod
from . import gaze as gaze_mod
from . import markers
from .bodies import BodyAdapter, flat_floor_spec
from .floor import FloorMap, Rect, StaticTile
from .layout import Grid, Segment
from .roommap import MapError, RoomMap
from .registry import get_part
from .signals import SignalBus, SignalError, wire

WALL_RGBA = (0.75, 0.75, 0.8, 1.0)
STAGE_GAP = 1.0      # [m] between the zones of `stages:` rooms


def _shift(seg: Segment, dx: float, dy: float = 0.0) -> Segment:
    return Segment((seg.p0[0] + dx, seg.p0[1] + dy), (seg.p1[0] + dx, seg.p1[1] + dy), seg.thickness)


class RoomError(ValueError):
    """Invalid room file; the message says what to fix. `stage`: see the module docstring."""

    def __init__(self, msg: str, stage: str | None = None):
        super().__init__(msg)
        self.stage = stage


DEMO_COMMANDS = ("goto", "press", "wait", "push", "herd", "grab", "place", "drop", "turn", "pull", "look")
MAX_WAIT = 60.0                        # [s] longest `wait`


def parse_demo(raw: Any, parts: dict[str, Any]) -> list[tuple]:
    """`demo:` entries as tuples: ("goto"|"press", part), ("wait", seconds),
    ("push", ball, target), ("herd", npc, part or (x, y)), ("grab", object),
    ("place" | "drop", object, part), ("turn", lever, value), ("pull", cord), ("look", part).
    Raises RoomError (stage `demo`) naming the bad entry."""
    if raw is None:
        return []
    if not isinstance(raw, list):
        raise RoomError("`demo:` must be a list of sub-goals, e.g. [press switch_a, goto goal]", "demo")
    out: list[tuple] = []

    def part(i: int, name: str):
        if name not in parts:
            raise RoomError(f"demo[{i}]: no part named {name!r} (parts: {sorted(parts)})", "demo")
        return name

    for i, entry in enumerate(raw):
        words = str(entry).split() if isinstance(entry, str) else []
        if not words:
            raise RoomError(f"demo[{i}]: must be a string like 'goto goal', got {entry!r}", "demo")
        cmd, args = words[0], words[1:]
        if cmd not in DEMO_COMMANDS:
            raise RoomError(f"demo[{i}]: unknown sub-goal {cmd!r}; use one of {DEMO_COMMANDS}", "demo")
        if cmd in ("goto", "press"):
            if len(args) != 1:
                raise RoomError(f"demo[{i}]: `{cmd} <part>` takes one part name, got {entry!r}", "demo")
            p = part(i, args[0])
            if cmd == "press" and parts[p].role not in ("switch", "checkpoint", "goal"):
                raise RoomError(f"demo[{i}]: cannot press {p!r} (a {parts[p].role}); press "
                                f"switches, buttons, sensors and flags", "demo")
            out.append((cmd, p))
        elif cmd == "wait":
            try:
                sec = float(args[0]) if len(args) == 1 else -1.0
            except ValueError:
                sec = -1.0
            if not 0.0 < sec <= MAX_WAIT:
                raise RoomError(f"demo[{i}]: `wait <seconds>` needs one number in (0, {MAX_WAIT:g}], "
                                f"got {entry!r}", "demo")
            out.append(("wait", sec))
        elif cmd == "herd":
            out.append(_parse_herd(i, entry, args, parts, part))
        elif cmd in ("turn", "pull", "look"):
            out.append(_parse_device(i, entry, cmd, args, parts, part))
        elif cmd in ("grab", "place", "drop"):
            n = 1 if cmd == "grab" else 2
            if len(args) != n:
                raise RoomError(f"demo[{i}]: write `" + ("grab <object>" if n == 1 else f"{cmd} <object> <part>")
                                + f"`, got {entry!r}", "demo")
            obj = part(i, args[0])
            if not hasattr(parts[obj], "handle_points"):
                raise RoomError(f"demo[{i}]: {obj!r} is not a grabbable object (nothing to grab)", "demo")
            out.append((cmd, obj, *([part(i, args[1])] if n == 2 else [])))
        else:
            if len(args) != 3 or args[1] != "to":
                raise RoomError(f"demo[{i}]: write `push <ball> to <net or pin>`, got {entry!r}", "demo")
            ball, target = part(i, args[0]), part(i, args[2])
            if getattr(parts[ball], "object_kind", None) != "ball":
                raise RoomError(f"demo[{i}]: {ball!r} is not a ball", "demo")
            if not hasattr(parts[target], "push_target"):
                raise RoomError(f"demo[{i}]: cannot push a ball to {target!r} "
                                f"(push to a goal net or a pin)", "demo")
            if not parts[target].push_accepts(parts[ball]):
                raise RoomError(f"demo[{i}]: {target!r} does not accept {ball!r}", "demo")
            out.append(("push", ball, target))
    return out


def _parse_device(i: int, entry: str, cmd: str, args: list[str], parts: dict[str, Any], part) -> tuple:
    """("turn", lever or dial, value) / ("pull", cord or drawer) / ("look", part) (R28, R27)."""
    from .gaze import LOOK_PREFIX

    n = 2 if cmd == "turn" else 1
    if len(args) != n:
        form = {"turn": "turn <lever> <value>", "pull": "pull <cord or drawer>", "look": "look <part>"}[cmd]
        raise RoomError(f"demo[{i}]: write `{form}`, got {entry!r}", "demo")
    name = part(i, args[0])
    p = parts[name]
    kind = getattr(p, "object_kind", None)
    if cmd == "pull":
        if kind not in ("pull_cord", "drawer"):
            raise RoomError(f"demo[{i}]: cannot pull {name!r}; pull a pull_cord or a drawer", "demo")
        return ("pull", name)
    if cmd == "look":
        if LOOK_PREFIX + name not in parts:
            raise RoomError(f"demo[{i}]: cannot look at {name!r}: no part reads {name + '.seen'!r} "
                            f"(give it a marker and read that channel)", "demo")
        return ("look", name)
    if kind == "dial":
        ok = [str(c) for c in range(p.n)]
        if args[1] not in ok:
            raise RoomError(f"demo[{i}]: `turn {name} <cell>` needs one of its cells 0..{p.n - 1}, "
                            f"got {entry!r}", "demo")
        return ("turn", name, float(args[1]))
    if kind != "lever" or p.mode != "push":
        raise RoomError(f"demo[{i}]: cannot turn {name!r}; the solver turns levers with mode: push "
                        f"(holding a grab lever's knob stalls the sidestep, plan E3b-2)", "demo")
    try:
        value = float(args[1])
    except ValueError:
        value = math.nan
    ok = p.cells if p.cells is not None else None
    if not -1.0 <= value <= 1.0 or (ok is not None and not np.isclose(ok, value).any()):
        want = f"one of its cells {[float(v) for v in ok]}" if ok is not None else "a value in -1..1"
        raise RoomError(f"demo[{i}]: `turn {name} <value>` needs {want}, got {entry!r}", "demo")
    return ("turn", name, value)


def _parse_herd(i: int, entry: str, args: list[str], parts: dict[str, Any], part) -> tuple:
    """("herd", npc, part name or (x, y)) for `herd <npc> <part>` / `herd <npc> <x> <y>` (R29)."""
    from .parts.npc.behaviors import HERDABLE, can_herd

    if len(args) not in (2, 3):
        raise RoomError(f"demo[{i}]: write `herd <npc> <part>` or `herd <npc> <x> <y>`, "
                        f"got {entry!r}", "demo")
    npc = part(i, args[0])
    if not can_herd(parts[npc], parts):
        raise RoomError(f"demo[{i}]: cannot herd {npc!r}; herd an npc robot with behavior "
                        f"{' / '.join(HERDABLE)} (lure: with a grabbable bait)", "demo")
    if len(args) == 2:
        return ("herd", npc, part(i, args[1]))
    try:
        return ("herd", npc, (float(args[1]), float(args[2])))
    except ValueError:
        raise RoomError(f"demo[{i}]: `herd <npc> <x> <y>` needs two numbers, got {entry!r}",
                        "demo") from None


class Parts(dict):
    """Part objects by name (room order) plus the signal bus wiring them (R1)."""

    bus: SignalBus


@dataclass
class Room:
    name: str
    size: tuple[float, float]
    wall_thickness: float
    wall_height: float
    inner_walls: list[Segment]
    spawn_pos: np.ndarray
    spawn_yaw: float
    spawn_random: np.ndarray
    part_cfgs: list[dict[str, Any]]
    links: list[dict[str, str]]
    path: Path | None = None
    raw: dict[str, Any] = field(default_factory=dict)
    floor_friction: tuple[float, float, float] | None = None
    #: `stages:` rooms as (x offset, room), zone 1, 2, ... (R14)
    stages: list[tuple[float, "Room"]] = field(default_factory=list)
    #: zone index of each entry of `part_cfgs` (0: this room)
    part_zones: list[int] = field(default_factory=list)
    map: RoomMap | None = None          # letter map (R37)
    on_fall: str = "fail"               # fall into a hole: fail | reset (R37, R16)
    demo: list[tuple] = field(default_factory=list)   # scripted solver sub-goals (R20)
    gaze: gaze_mod.GazeConfig = field(default_factory=gaze_mod.GazeConfig)   # R27
    part_presets: dict[str, str] = field(default_factory=dict)   # part name -> preset (R3)

    # ---- loading ----------------------------------------------------------
    @classmethod
    def load(cls, path: str | Path) -> "Room":
        path = Path(path)
        try:
            raw = yaml.safe_load(path.read_text())
        except (OSError, yaml.YAMLError) as exc:
            raise RoomError(f"cannot read room file {path}: {exc}") from exc
        return cls.from_dict(raw, path=path)

    @classmethod
    def from_dict(cls, raw: dict[str, Any], path: Path | None = None, stage: bool = False) -> "Room":
        if not isinstance(raw, dict):
            raise RoomError("room file must be a mapping")
        try:
            raw = catalog.expand_room(raw)
            expanded = [catalog.expand_part(c) for c in raw.get("parts") or []]
        except catalog.PresetError as exc:
            raise RoomError(str(exc), "values") from None
        raw = {**raw, "parts": [c for c, _ in expanded]} if expanded else raw
        wall = raw.get("wall", {}) or {}
        thick = float(wall.get("thickness", 0.06))
        height = float(wall.get("height", 0.25))
        rmap = None
        if "map" in raw:
            if "size" in raw or raw.get("walls"):
                raise RoomError("`map` replaces `size` and `walls`; use one or the other")
            if raw.get("stages"):
                raise RoomError("`map` rooms cannot use `stages:` yet; draw connected rooms "
                                "in one map with walls and doors")
            try:
                rmap = RoomMap(raw["map"], cell=raw.get("cell", 0.1),
                               step_height=raw.get("step_height", 0.01),
                               wall_height=height, legend=raw.get("legend"))
            except MapError as exc:
                raise RoomError(str(exc)) from None
            ext = rmap.extent()
            raw = {**raw, "size": [ext.x1 - ext.x0, ext.y1 - ext.y0]}
        elif any(k in raw for k in ("cell", "legend", "step_height")):
            raise RoomError("`cell`, `step_height` and `legend` need a `map`")
        try:
            size = tuple(float(v) for v in raw["size"])
            assert len(size) == 2 and min(size) > 0
        except (KeyError, TypeError, ValueError, AssertionError):
            raise RoomError("`size: [x, y]` with positive values is required") from None
        on_fall = raw.get("on_fall", "fail")
        if on_fall not in ("fail", "reset"):
            raise RoomError(f"`on_fall` must be fail or reset, got {on_fall!r}")
        walls = []
        for i, w in enumerate(raw.get("walls", []) or []):
            try:
                walls.append(Segment(tuple(map(float, w["from"])), tuple(map(float, w["to"])),
                                     float(w.get("thickness", thick))))
            except (KeyError, TypeError, ValueError):
                raise RoomError(f"walls[{i}]: needs `from: [x, y]` and `to: [x, y]`") from None
        spawn = raw.get("spawn") or {}
        if "pos" not in spawn:
            raise RoomError("`spawn: {pos: [x, y]}` is required")
        rnd = list(spawn.get("random", [0.0, 0.0, 0.0])) + [0.0] * 3
        cfgs = raw.get("parts") or []
        links = raw.get("links") or []
        fl = raw.get("floor") or {}
        friction = None
        if not isinstance(fl, dict) or set(fl) - {"friction"}:
            raise RoomError("`floor:` takes only `friction: [slide, torsion, roll]`")
        if "friction" in fl:
            try:
                friction = tuple(float(v) for v in fl["friction"])
                assert len(friction) == 3 and all(0.0 <= v <= 5.0 for v in friction)
            except (TypeError, ValueError, AssertionError):
                raise RoomError("floor.friction must be 3 numbers in [0, 5] "
                                f"(slide, torsion, roll), got {fl['friction']!r}") from None
        name = raw.get("name") or (path.stem if path else "room")
        name = str(name) if isinstance(name, (int, float)) else name
        # the name becomes a folder (runs/check/<name>/checkpoints): no path parts
        if not isinstance(name, str) or name in (".", "..") or any(c in name for c in "/\\\0"):
            raise RoomError(f"`name` must be a plain name without / or \\, got {name!r}", "values")
        room = cls(
            name=name,
            size=size, wall_thickness=thick, wall_height=height, inner_walls=walls,
            spawn_pos=np.asarray(spawn["pos"], dtype=float),
            spawn_yaw=float(spawn.get("yaw", 0.0)),
            spawn_random=np.asarray(rnd[:3], dtype=float),
            part_cfgs=[dict(c) for c in cfgs], links=[dict(l) for l in links],
            path=path, raw=raw, floor_friction=friction, part_zones=[0] * len(cfgs),
            map=rmap, on_fall=on_fall,
            part_presets={c["name"]: n for c, n in expanded if n and isinstance(c, dict) and "name" in c},
        )
        if stage:
            if raw.get("stages"):
                raise RoomError("a `stages:` room cannot list stages itself")
            return room
        try:
            room.gaze = gaze_mod.parse_config(raw.get("gaze"))
        except ValueError as exc:
            raise RoomError(str(exc), "values") from None
        room._add_stages(raw.get("stages") or [])
        parts = room.make_parts()          # validates parts and links
        room.demo = parse_demo(raw.get("demo"), parts)
        return room

    def _add_stages(self, paths: list[Any]) -> None:
        if not isinstance(paths, list):
            raise RoomError("`stages:` must be a list of room files")
        base = self.path.parent if self.path else Path(".")
        right = self.extent().x1
        for k, rel in enumerate(paths, start=1):
            p = base / str(rel)
            try:
                raw = yaml.safe_load(p.read_text())
            except (OSError, yaml.YAMLError) as exc:
                raise RoomError(f"stages[{k - 1}]: cannot read room file {p}: {exc}") from None
            sub = Room.from_dict(raw, path=p, stage=True)
            dx = right + STAGE_GAP - sub.extent().x0
            right = dx + sub.extent().x1
            self.stages.append((dx, sub))
            for cfg in sub.part_cfgs:
                try:
                    cls = get_part(cfg.get("type"))
                except KeyError as exc:
                    raise RoomError(f"stages[{k - 1}] part {cfg.get('name')!r}: {exc.args[0]}") from None
                self.part_cfgs.append(cls.shift_cfg(cfg, dx, 0.0))
                self.part_zones.append(k)
            self.links += [dict(l) for l in sub.links]
            self.part_presets.update(sub.part_presets)

    def resolved_parts(self) -> list[dict[str, Any]]:
        """Every room part as loaded: name, type, preset and its checked values with
        defaults filled in (`task.resolved.yaml` `room_parts`, R3)."""
        import json

        parts = self.make_parts()
        out = []
        for cfg in self.part_cfgs:
            p = parts[cfg["name"]]
            entry = {"name": p.name, "type": p.type_name}
            if p.name in self.part_presets:
                entry["preset"] = self.part_presets[p.name]
            entry.update(p.cfg)
            out.append(json.loads(json.dumps(entry, default=lambda o: o.tolist()
                                             if hasattr(o, "tolist") else str(o))))
        return out

    def zone_spawns(self) -> list[tuple[float, float, float]]:
        """Spawn pose of every zone (this room, then each `stages:` room)."""
        out = [(*map(float, self.spawn_pos), self.spawn_yaw)]
        for dx, sub in self.stages:
            out.append((float(sub.spawn_pos[0]) + dx, float(sub.spawn_pos[1]), sub.spawn_yaw))
        return out

    def make_parts(self) -> Parts:
        """Fresh part objects (each env owns its own), with links and channels wired."""
        parts = Parts()
        for i, cfg in enumerate(self.part_cfgs):
            name, kind = cfg.get("name"), cfg.get("type")
            if not name or not kind:
                raise RoomError(f"parts[{i}]: needs `name` and `type`")
            if name in parts:
                raise RoomError(f"duplicate part name {name!r}")
            try:
                cls = get_part(kind)
            except KeyError as exc:
                raise RoomError(f"part {name!r}: {exc.args[0]}") from None
            try:
                parts[name] = cls(name, {k: v for k, v in cfg.items() if k not in ("name", "type")})
                parts[name].marker = markers.parse(parts[name], parts[name].cfg.get("marker"))
            except (ValueError, TypeError) as exc:
                raise RoomError(str(exc), "values") from None
            parts[name].zone = self.part_zones[i] if i < len(self.part_zones) else 0
        for kind in {type(p): None for p in list(parts.values())}:
            try:
                kind.add_room_parts(parts)
            except (ValueError, TypeError) as exc:
                raise RoomError(str(exc), "values") from None
        spawns = self.zone_spawns()
        for n, p in parts.items():
            if p.role == "warp":
                nxt = p.zone + 1
                p.next_stage_pose = spawns[nxt] if nxt < len(spawns) else None
                if p.next_stage_pose is None and any(d == "next_stage" for _, d in p.routes):
                    raise RoomError(f"part {n!r}: warp to next_stage, but no stage follows "
                                    f"(add the next room to `stages:`)")
        floors = [p for p in parts.values() if getattr(p, "floor_part", False)]
        for i, a in enumerate(floors):
            for b in floors[i + 1:]:
                if a.floor_rect().overlaps(b.floor_rect()):
                    raise RoomError(f"floor parts {a.name!r} and {b.name!r} overlap")
        if not any(p.role == "goal" for p in parts.values()):
            raise RoomError("the room needs at least one part with role goal (e.g. type: goal)")
        flags: dict[int, str] = {}
        for n, p in parts.items():
            if p.role == "checkpoint":
                if p.order in flags:
                    raise RoomError(f"checkpoints {flags[p.order]!r} and {n!r} both have order {p.order}")
                flags[p.order] = n
        for i, link in enumerate(self.links):
            src, dst = link.get("from"), link.get("to")
            for end, ref in (("from", src), ("to", dst)):
                if ref not in parts:
                    raise RoomError(f"links[{i}].{end}: no part named {ref!r} "
                                    f"(parts: {sorted(parts)})", "channels")
            if parts[src].role != "switch":
                raise RoomError(f"links[{i}]: from {src!r} is a {parts[src].role}, must be a switch")
            if parts[dst].role != "gate":
                raise RoomError(f"links[{i}]: to {dst!r} is a {parts[dst].role}, must be a gate (door)")
            if not parts[src].outputs:
                raise RoomError(f"links[{i}]: {src!r} writes no channel (`out: []`)")
            if parts[src].outputs[0] not in parts[dst].inputs:
                parts[dst].inputs.append(parts[src].outputs[0])
        try:
            markers.assign_ids(parts)
        except ValueError as exc:
            raise RoomError(str(exc), "values") from None
        try:
            gaze_mod.add_watch(parts, self.gaze)
        except ValueError as exc:
            raise RoomError(str(exc), "channels") from None
        try:
            parts.bus = wire(parts)
        except SignalError as exc:
            raise RoomError(str(exc), "channels") from None
        return parts

    # ---- geometry ---------------------------------------------------------
    def outer_walls(self) -> list[Segment]:
        if self.map is not None:
            return []
        hx, hy = self.size[0] / 2, self.size[1] / 2
        t = self.wall_thickness
        ex, ey = hx + t / 2, hy + t / 2
        corners = [(-ex, -ey), (ex, -ey), (ex, ey), (-ex, ey)]
        return [Segment(corners[i], corners[(i + 1) % 4], t) for i in range(4)]

    def own_walls(self) -> list[Segment]:
        if self.map is not None:
            return [_rect_segment(r) for r, _ in self.map.wall_rects()]
        return self.outer_walls() + self.inner_walls

    def walls(self) -> list[Segment]:
        out = self.own_walls()
        for dx, sub in self.stages:
            out += [_shift(s, dx) for s in sub.own_walls()]
        return out

    def extent(self) -> Rect:
        """Floor area of this room alone (interior plus walls), room coordinates."""
        if self.map is not None:
            return self.map.extent()
        hx, hy = self.size[0] / 2 + self.wall_thickness, self.size[1] / 2 + self.wall_thickness
        return Rect(-hx, hx, -hy, hy)

    def zone_extents(self) -> list[Rect]:
        return [self.extent()] + [sub.extent().shifted(dx) for dx, sub in self.stages]

    def grid(self, res: float | None = None, body_values: dict | None = None) -> Grid:
        """Planner grid. A map room's grid is aligned with its cells, and its walls,
        cliffs, steps above `max_step` and ramps above `max_slope` (from
        `body_values`, the body's capability values; no limit without) are blocked."""
        kw = {"res": res} if res else {}
        if self.map is not None:
            m = self.map
            r = res or m.cell / math.ceil(m.cell / 0.04 - 1e-9)
            ext = m.extent()
            grid = Grid((ext.x1 - ext.x0, ext.y1 - ext.y0), res=r)
            vals = body_values or {}
            grid.terrain, _ = m.terrain(grid, float(vals.get("max_step", math.inf)),
                                        float(vals.get("max_slope", 90.0)))
            return grid
        if not self.stages:
            t = 2 * self.wall_thickness
            return Grid((self.size[0] + t, self.size[1] + t), **kw)
        ext = self.zone_extents()
        x0, x1 = min(e.x0 for e in ext), max(e.x1 for e in ext)
        y0, y1 = min(e.y0 for e in ext), max(e.y1 for e in ext)
        return Grid((x1 - x0, y1 - y0), center=((x0 + x1) / 2, (y0 + y1) / 2), **kw)

    # ---- floor (R10) --------------------------------------------------------
    def needs_tiles(self, parts: dict[str, Any]) -> bool:
        return self.map is not None or any(getattr(p, "floor_part", False) for p in parts.values())

    def static_tiles(self, parts: dict[str, Any]) -> list[StaticTile]:
        """Boxes filling every zone's floor around the floor parts (no gaps)."""
        holes = [p.floor_rect() for p in parts.values() if getattr(p, "floor_part", False)]
        fr = self.floor_friction or floor_mod.DEFAULT_FRICTION
        rooms = [(0.0, self)] + self.stages
        out = []
        if self.map is not None:
            for t in self.map.floor_tiles():
                f = t.friction if t.friction != floor_mod.DEFAULT_FRICTION else fr
                out += [StaticTile(r, t.top, f) for r in floor_mod.cover(t.rect, holes)]
            return out
        for dx, room in rooms:
            f = room.floor_friction or fr
            out += [StaticTile(r, 0.0, f) for r in floor_mod.cover(room.extent().shifted(dx), holes)]
        return out

    def floor_map(self, parts: dict[str, Any]) -> FloorMap | None:
        """Floor height under a point (fall rule, R10); None for the M1 plane floor."""
        if not self.needs_tiles(parts):
            return None
        fm = FloorMap()
        if self.map is not None:
            fm.outside_rim = self.map.rim
            fm.add(self.map.extent(), self.map.height_at)
        else:
            for t in self.static_tiles(parts):
                fm.add_static(t)
        for p in parts.values():
            if getattr(p, "floor_part", False):
                fm.add(p.floor_rect(), p.height_at)
        return fm

    def sample_spawn(self, rng: np.random.Generator | None) -> tuple[float, float, float]:
        if rng is None:
            return (*map(float, self.spawn_pos), self.spawn_yaw)
        d = rng.uniform(-self.spawn_random, self.spawn_random)
        return float(self.spawn_pos[0] + d[0]), float(self.spawn_pos[1] + d[1]), self.spawn_yaw + float(d[2])

    # ---- scene ------------------------------------------------------------
    def build_spec(self, body: BodyAdapter, parts: dict[str, Any]) -> mujoco.MjSpec:
        spec = flat_floor_spec()
        if self.map is not None:
            self.map.build(spec, WALL_RGBA, floor_mod.TILE_RGBA[0])
        else:
            _add_walls(spec, self.own_walls(), self.wall_height, "wall")
        for k, (dx, sub) in enumerate(self.stages, start=1):
            _add_walls(spec, [_shift(s, dx) for s in sub.own_walls()], sub.wall_height, f"wall/s{k}")
        if self.needs_tiles(parts):
            floor_mod.lower_catch_plane(spec)
            floor_mod.add_tiles(spec, self.static_tiles(parts))
        elif self.floor_friction is not None:
            spec.geom("floor").friction = list(self.floor_friction)
            spec.geom("floor").priority = 1
        for part in parts.values():
            pb = part.build(spec)
            markers.attach(spec, part)
            if self.map is not None and pb is not None and not getattr(part, "floor_part", False):
                top = self.map.height_at(part.base_xy)[0]     # stand parts on raised cells
                if top:
                    pb.pos = [pb.pos[0], pb.pos[1], pb.pos[2] + top]
        body.spawn(spec, self.sample_spawn(None))
        for part in parts.values():
            part.attach_body(spec, body)
        spec.visual.global_.offwidth = 960
        spec.visual.global_.offheight = 720
        return spec


def _rect_segment(r: Rect) -> Segment:
    """A wall rectangle as a segment along its long side (thickness = short side)."""
    (cx, cy), (hx, hy) = r.centre, r.half
    if hx >= hy:
        return Segment((cx - hx + hy, cy), (cx + hx - hy, cy), 2 * hy)
    return Segment((cx, cy - hy + hx), (cx, cy + hy - hx), 2 * hx)


def _add_walls(spec: mujoco.MjSpec, segs: list[Segment], height: float, prefix: str) -> None:
    for i, seg in enumerate(segs):
        p0, p1 = np.asarray(seg.p0), np.asarray(seg.p1)
        mid, d = (p0 + p1) / 2, p1 - p0
        ang = math.atan2(d[1], d[0])
        spec.worldbody.add_geom(
            name=f"{prefix}/{i}", type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[np.linalg.norm(d) / 2 + seg.thickness / 2, seg.thickness / 2, height / 2],
            pos=[mid[0], mid[1], height / 2],
            quat=[math.cos(ang / 2), 0, 0, math.sin(ang / 2)], rgba=WALL_RGBA)


def render_top(model: mujoco.MjModel, data: mujoco.MjData, room: Room, path: Path,
               width: int = 960, height: int = 720) -> Path:
    """Top-down PNG with the offscreen renderer (MUJOCO_GL picks the backend)."""
    import imageio.v3 as iio

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    ext = room.zone_extents()
    x0, x1 = min(e.x0 for e in ext), max(e.x1 for e in ext)
    y0, y1 = min(e.y0 for e in ext), max(e.y1 for e in ext)
    cam.lookat[:] = [(x0 + x1) / 2, (y0 + y1) / 2, 0.0]
    cam.elevation = -90.0
    cam.azimuth = 90.0
    fovy = math.radians(model.vis.global_.fovy)
    extent = max((x1 - x0) * height / width, y1 - y0) / 2 + room.wall_thickness
    cam.distance = extent / math.tan(fovy / 2) * 1.05
    with mujoco.Renderer(model, height=height, width=width) as renderer:
        renderer.update_scene(data, camera=cam)
        img = renderer.render()
    path.parent.mkdir(parents=True, exist_ok=True)
    iio.imwrite(path, img)
    return path


def build(room_path: str | Path, out_dir: str | Path | None = None, png: bool = True) -> dict[str, Any]:
    """`mpg build`: compile body + room, write the combined MJCF and a top-view PNG."""
    from .bodies import make_body

    room = Room.load(room_path)
    body = make_body()
    parts = room.make_parts()
    spec = room.build_spec(body, parts)
    model = spec.compile()
    data = mujoco.MjData(model)
    body.reset(data, room.sample_spawn(None))
    for p in parts.values():
        p.bind(model, data)
        p.reset(model, data)
    mujoco.mj_forward(model, data)
    out = Path(out_dir) if out_dir else Path("build") / room.name
    out.mkdir(parents=True, exist_ok=True)
    xml_path = out / f"{room.name}.xml"
    xml_path.write_text(spec.to_xml())
    result: dict[str, Any] = {
        "room": room.name,
        "mjcf": str(xml_path.resolve()),
        "parts": [{"name": n, "type": p.type_name, "role": p.role} for n, p in parts.items()],
        "links": room.links,
        "model": {"nbody": model.nbody, "ngeom": model.ngeom, "nu": model.nu},
    }
    if png:
        result["png"] = str(render_top(model, data, room, out / f"{room.name}.png").resolve())
    return result
