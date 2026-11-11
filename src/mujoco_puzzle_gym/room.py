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

`random` entries are uniform half-ranges (+-) applied on every env reset.
A link is a signal channel (R1, R2): it adds the switch's channel to the
door's `in`, the same as writing `in: [switch_a]` on the door.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import yaml

from .bodies import BodyAdapter, flat_floor_spec
from .layout import Grid, Segment
from .registry import get_part
from .signals import SignalBus, SignalError, wire

WALL_RGBA = (0.75, 0.75, 0.8, 1.0)


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
    def from_dict(cls, raw: dict[str, Any], path: Path | None = None) -> "Room":
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
        room = cls(
            name=str(raw.get("name") or (path.stem if path else "room")),
            size=size, wall_thickness=thick, wall_height=height, inner_walls=walls,
            spawn_pos=np.asarray(spawn["pos"], dtype=float),
            spawn_yaw=float(spawn.get("yaw", 0.0)),
            spawn_random=np.asarray(rnd[:3], dtype=float),
            part_cfgs=[dict(c) for c in cfgs], links=[dict(l) for l in links],
            path=path, raw=raw,
        )
        room.make_parts()          # validates parts and links
        return room

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
        if not any(p.role == "goal" for p in parts.values()):
            raise RoomError("the room needs at least one part with role goal (e.g. type: goal)")
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

    def walls(self) -> list[Segment]:
        return self.outer_walls() + self.inner_walls

    def grid(self, res: float | None = None) -> Grid:
        t = 2 * self.wall_thickness
        return Grid((self.size[0] + t, self.size[1] + t), **({"res": res} if res else {}))

    def sample_spawn(self, rng: np.random.Generator | None) -> tuple[float, float, float]:
        if rng is None:
            return (*map(float, self.spawn_pos), self.spawn_yaw)
        d = rng.uniform(-self.spawn_random, self.spawn_random)
        return float(self.spawn_pos[0] + d[0]), float(self.spawn_pos[1] + d[1]), self.spawn_yaw + float(d[2])

    # ---- scene ------------------------------------------------------------
    def build_spec(self, body: BodyAdapter, parts: dict[str, Any]) -> mujoco.MjSpec:
        spec = flat_floor_spec()
        for i, seg in enumerate(self.walls()):
            p0, p1 = np.asarray(seg.p0), np.asarray(seg.p1)
            mid, d = (p0 + p1) / 2, p1 - p0
            ang = math.atan2(d[1], d[0])
            spec.worldbody.add_geom(
                name=f"wall/{i}", type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[np.linalg.norm(d) / 2 + seg.thickness / 2, seg.thickness / 2, self.wall_height / 2],
                pos=[mid[0], mid[1], self.wall_height / 2],
                quat=[math.cos(ang / 2), 0, 0, math.sin(ang / 2)], rgba=WALL_RGBA)
        for part in parts.values():
            part.build(spec)
        body.spawn(spec, self.sample_spawn(None))
        spec.visual.global_.offwidth = 960
        spec.visual.global_.offheight = 720
        return spec


def render_top(model: mujoco.MjModel, data: mujoco.MjData, room: Room, path: Path,
               width: int = 960, height: int = 720) -> Path:
    """Top-down PNG with the offscreen renderer (MUJOCO_GL picks the backend)."""
    import imageio.v3 as iio

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.0, 0.0, 0.0]
    cam.elevation = -90.0
    cam.azimuth = 90.0
    fovy = math.radians(model.vis.global_.fovy)
    extent = max(room.size[0] * height / width, room.size[1]) / 2 + 2 * room.wall_thickness
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
