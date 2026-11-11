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

`random` entries are uniform half-ranges (+-) applied on every env reset.
A link is a signal channel (R1, R2): it adds the switch's channel to the
door's `in`, the same as writing `in: [switch_a]` on the door.

Floors (R10): a room with floor parts (`floor_tile`, `trapdoor`) gets a tile
floor without gaps (floor.py) and a catch plane below; otherwise the M1 plane.
`floor.friction` sets the default floor friction for either.

Stages (R14): `stages:` lists further room files (paths relative to this
one). Each is built in its own zone to the +x side of the previous one, in
the same model, so observation and snapshot sizes stay fixed; a warp pad with
`to: next_stage` moves the body to the next zone's spawn. Part names must be
unique over all stages; channels are shared. The goal requirement holds for
the stages together (an earlier stage may have none).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import yaml

from . import floor as floor_mod
from .bodies import BodyAdapter, flat_floor_spec
from .floor import FloorMap, Rect, StaticTile
from .layout import Grid, Segment
from .registry import get_part
from .signals import SignalBus, SignalError, wire

WALL_RGBA = (0.75, 0.75, 0.8, 1.0)
STAGE_GAP = 1.0      # [m] between the zones of `stages:` rooms


def _shift(seg: Segment, dx: float, dy: float = 0.0) -> Segment:
    return Segment((seg.p0[0] + dx, seg.p0[1] + dy), (seg.p1[0] + dx, seg.p1[1] + dy), seg.thickness)


class RoomError(ValueError):
    """Invalid room file; the message says what to fix."""


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
            size = tuple(float(v) for v in raw["size"])
            assert len(size) == 2 and min(size) > 0
        except (KeyError, TypeError, ValueError, AssertionError):
            raise RoomError("`size: [x, y]` with positive values is required") from None
        wall = raw.get("wall", {}) or {}
        thick = float(wall.get("thickness", 0.06))
        height = float(wall.get("height", 0.25))
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
        room = cls(
            name=str(raw.get("name") or (path.stem if path else "room")),
            size=size, wall_thickness=thick, wall_height=height, inner_walls=walls,
            spawn_pos=np.asarray(spawn["pos"], dtype=float),
            spawn_yaw=float(spawn.get("yaw", 0.0)),
            spawn_random=np.asarray(rnd[:3], dtype=float),
            part_cfgs=[dict(c) for c in cfgs], links=[dict(l) for l in links],
            path=path, raw=raw, floor_friction=friction, part_zones=[0] * len(cfgs),
        )
        if stage:
            if raw.get("stages"):
                raise RoomError("a `stages:` room cannot list stages itself")
            return room
        room._add_stages(raw.get("stages") or [])
        room.make_parts()          # validates parts and links
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
            except (ValueError, TypeError) as exc:
                raise RoomError(str(exc)) from None
            parts[name].zone = self.part_zones[i] if i < len(self.part_zones) else 0
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
                                    f"(parts: {sorted(parts)})")
            if parts[src].role != "switch":
                raise RoomError(f"links[{i}]: from {src!r} is a {parts[src].role}, must be a switch")
            if parts[dst].role != "gate":
                raise RoomError(f"links[{i}]: to {dst!r} is a {parts[dst].role}, must be a gate (door)")
            if not parts[src].outputs:
                raise RoomError(f"links[{i}]: {src!r} writes no channel (`out: []`)")
            if parts[src].outputs[0] not in parts[dst].inputs:
                parts[dst].inputs.append(parts[src].outputs[0])
        try:
            parts.bus = wire(parts)
        except SignalError as exc:
            raise RoomError(str(exc)) from None
        return parts

    # ---- geometry ---------------------------------------------------------
    def outer_walls(self) -> list[Segment]:
        hx, hy = self.size[0] / 2, self.size[1] / 2
        t = self.wall_thickness
        ex, ey = hx + t / 2, hy + t / 2
        corners = [(-ex, -ey), (ex, -ey), (ex, ey), (-ex, ey)]
        return [Segment(corners[i], corners[(i + 1) % 4], t) for i in range(4)]

    def own_walls(self) -> list[Segment]:
        return self.outer_walls() + self.inner_walls

    def walls(self) -> list[Segment]:
        out = self.own_walls()
        for dx, sub in self.stages:
            out += [_shift(s, dx) for s in sub.own_walls()]
        return out

    def extent(self) -> Rect:
        """Floor area of this room alone (interior plus walls), room coordinates."""
        hx, hy = self.size[0] / 2 + self.wall_thickness, self.size[1] / 2 + self.wall_thickness
        return Rect(-hx, hx, -hy, hy)

    def zone_extents(self) -> list[Rect]:
        return [self.extent()] + [sub.extent().shifted(dx) for dx, sub in self.stages]

    def grid(self, res: float | None = None) -> Grid:
        kw = {"res": res} if res else {}
        if not self.stages:
            t = 2 * self.wall_thickness
            return Grid((self.size[0] + t, self.size[1] + t), **kw)
        ext = self.zone_extents()
        x0, x1 = min(e.x0 for e in ext), max(e.x1 for e in ext)
        y0, y1 = min(e.y0 for e in ext), max(e.y1 for e in ext)
        return Grid((x1 - x0, y1 - y0), center=((x0 + x1) / 2, (y0 + y1) / 2), **kw)

    # ---- floor (R10) --------------------------------------------------------
    def needs_tiles(self, parts: dict[str, Any]) -> bool:
        return any(getattr(p, "floor_part", False) for p in parts.values())

    def static_tiles(self, parts: dict[str, Any]) -> list[StaticTile]:
        """Boxes filling every zone's floor around the floor parts (no gaps)."""
        holes = [p.floor_rect() for p in parts.values() if getattr(p, "floor_part", False)]
        fr = self.floor_friction or floor_mod.DEFAULT_FRICTION
        rooms = [(0.0, self)] + self.stages
        out = []
        for dx, room in rooms:
            f = room.floor_friction or fr
            out += [StaticTile(r, 0.0, f) for r in floor_mod.cover(room.extent().shifted(dx), holes)]
        return out

    def floor_map(self, parts: dict[str, Any]) -> FloorMap | None:
        """Floor height under a point (fall rule, R10); None for the M1 plane floor."""
        if not self.needs_tiles(parts):
            return None
        fm = FloorMap()
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
            part.build(spec)
        body.spawn(spec, self.sample_spawn(None))
        spec.visual.global_.offwidth = 960
        spec.visual.global_.offheight = 720
        return spec


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
