"""`mpg check --community`: limits for rooms shared through `community/rooms/` (community spec R2).

A room file cannot run code (YAML is read with `safe_load`, logic expressions go
through `expr.py`'s AST allow-list), but a shared room could still use parts a
plugin module registers, read room files outside its folder through `stages:`
or `randomize.rooms`,
or be large enough to stall CI. The `community` stage rejects those:

- a top-level `plugins` key;
- a `stages:` or `randomize.rooms.candidates` path that leaves the room file's
  folder (checked before loading);
- a part type that is not built in (registered by a module outside this package);
- more than MAX_PARTS parts (stages included);
- a floor wider or deeper than MAX_SIDE metres (stages included; for a room that
  draws its room the largest candidate counts, because only one is in play);
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


class _NoAliasLoader(yaml.SafeLoader):
    """SafeLoader without anchors and aliases: a 64 KB file of nested aliases
    expands to billions of nodes once something walks it."""

    def compose_node(self, parent, index):
        if self.check_event(yaml.AliasEvent):
            raise yaml.composer.ComposerError(
                None, None, "YAML aliases (*name) are not allowed in shared rooms",
                self.peek_event().start_mark)
        return super().compose_node(parent, index)


def check_raw(path: str | Path) -> list[str]:
    """Problems visible in the YAML alone, before the room (and its stage files) load."""
    path = Path(path)
    try:
        raw = yaml.load(path.read_text(), Loader=_NoAliasLoader)
    except (OSError, yaml.YAMLError) as exc:
        return [f"cannot read room file {path}: {exc}"]
    if not isinstance(raw, dict):
        return ["room file must be a mapping"]
    out = []
    if "plugins" in raw:
        out.append("`plugins` is not allowed in a shared room (plugin modules run code)")
    folder = path.resolve().parent
    rnd = raw.get("randomize") if isinstance(raw.get("randomize"), dict) else {}
    rooms = (rnd.get("rooms") or {}) if isinstance(rnd.get("rooms"), dict) else {}
    files = [("stages", raw.get("stages") or []),
             ("randomize.rooms.candidates", rooms.get("candidates") or [])]
    for where, entries in files:
        for k, rel in enumerate(entries if isinstance(entries, list) else []):
            if not isinstance(rel, str):
                out.append(f"{where}[{k}] must be a file name")
                continue
            p = (folder / rel).resolve()
            if not p.is_relative_to(folder):
                out.append(f"{where}[{k}] {rel!r} points outside the room's folder")
                continue
            try:
                yaml.load(p.read_text(), Loader=_NoAliasLoader)
            except (OSError, yaml.YAMLError) as exc:
                out.append(f"{where}[{k}] {rel!r}: {exc}")
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
    if room.zone_names:      # R35 `rooms`: the zones are alternatives, one per episode,
        w = max(z.x1 - z.x0 for z in zones)        # so the largest room is the floor
        d = max(z.y1 - z.y0 for z in zones)
    else:                    # `stages:`: the zones are played one after another
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
