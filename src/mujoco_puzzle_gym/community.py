"""`mpg check --community`: limits for rooms shared through `community/rooms/` (community spec R2).

A room file cannot run code (YAML is read with `safe_load`, logic expressions go
through `expr.py`'s AST allow-list), but a shared room could still use parts a
plugin module registers, read room files outside its folder through `stages:`,
or be large enough to stall CI. The `community` stage rejects those:

- a top-level `plugins` key;
- a `stages:` path that leaves the room file's folder (checked before loading);
- a part type that is not built in (registered by a module outside this package);
- more than MAX_PARTS parts (stages included);
- a floor wider or deeper than MAX_SIDE metres (stages included);
- a letter map over MAX_MAP cells on a side.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from .registry import get_part

MAX_PARTS = 64
MAX_SIDE = 10.0          # [m]
MAX_MAP = 40             # cells per side
BUILTIN_PREFIX = "mujoco_puzzle_gym."


def check_raw(path: str | Path) -> list[str]:
    """Problems visible in the YAML alone, before the room (and its stage files) load."""
    path = Path(path)
    try:
        raw = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as exc:
        return [f"cannot read room file {path}: {exc}"]
    if not isinstance(raw, dict):
        return ["room file must be a mapping"]
    out = []
    if "plugins" in raw:
        out.append("`plugins` is not allowed in a shared room (plugin modules run code)")
    folder = path.resolve().parent
    stages = raw.get("stages") or []
    for k, rel in enumerate(stages if isinstance(stages, list) else []):
        p = (folder / str(rel)).resolve()
        if not p.is_relative_to(folder):
            out.append(f"stages[{k}] {rel!r} points outside the room's folder")
    return out


def check_loaded(room: Any) -> list[str]:
    """Problems in the loaded room: plugin parts, size limits."""
    out = []
    for cfg in room.part_cfgs:
        module = get_part(cfg.get("type")).__module__
        if not module.startswith(BUILTIN_PREFIX):
            out.append(f"part {cfg.get('name')!r}: type {cfg.get('type')!r} comes from plugin "
                       f"module {module}; shared rooms use built-in parts only")
    if len(room.part_cfgs) > MAX_PARTS:
        out.append(f"{len(room.part_cfgs)} parts; the limit is {MAX_PARTS}")
    zones = room.zone_extents()
    w = max(z.x1 for z in zones) - min(z.x0 for z in zones)
    d = max(z.y1 for z in zones) - min(z.y0 for z in zones)
    if max(w, d) > MAX_SIDE:
        out.append(f"floor is {w:.2f} x {d:.2f} m; the limit is {MAX_SIDE:g} m a side")
    if room.map is not None and max(room.map.ncols, room.map.nrows) > MAX_MAP:
        out.append(f"map is {room.map.ncols} x {room.map.nrows} cells; the limit is "
                   f"{MAX_MAP} a side")
    return out


def stage(problems: list[str]) -> dict[str, Any]:
    return {"ok": not problems, "problems": problems} if problems else {"ok": True}
