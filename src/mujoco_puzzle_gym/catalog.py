"""Presets and the parts catalog (R3, R23): `preset:` in room.yaml and `mpg parts`.

Part presets are a part type plus default values, one YAML file each under
`presets/parts/<category>/<name>.yaml` (categories: CATEGORIES). A room's part
entry loads one and overrides some values:

    parts:
      - {name: b1, preset: input/floor_oneshot, pos: [0.2, 0.5], size: [0.3, 0.3]}

Assembly presets (several parts, or a whole room) live under
`presets/puzzles/<topic>/<name>.yaml`. A room loads them with a top-level
`preset:` (one name or a list), and its own keys go on top:

    preset: puzzles/coins/coin_lock     # parts: coins, lock, lock_door
    size: [2.4, 1.8]
    parts:
      - {name: lock_door, pos: [0.3, 0.0]}    # same name: these values change
      - {name: goal, type: goal, pos: [0.85, 0.0]}   # new name: added

Merge rules (in list order, the room last): top-level keys replace; `parts`
merge by `name` (an entry with `type` or `preset` replaces the part, without
them its values update the part's); `links` add up. A preset's `name` is not
inherited. Presets have no code (spec "presets are a part type + defaults").

`markdown()` is the value table of `mpg parts --markdown` (docs/parts-guide.md).
"""

from __future__ import annotations

import copy
import re
from pathlib import Path
from typing import Any

import yaml

from .registry import PARTS, load_builtins
from .schema import Field

PRESET_ROOT = Path(__file__).resolve().parent / "presets"
PART_ROOT = PRESET_ROOT / "parts"
PUZZLE_ROOT = PRESET_ROOT / "puzzles"
CATEGORIES = ("input", "logic", "device", "floor", "trap", "object", "beak", "npc")
_PART_NAME = re.compile(r"^([a-z]+)/([a-z0-9_]+)$")
_PUZZLE_NAME = re.compile(r"^puzzles/([a-z0-9_]+)/([a-z0-9_]+)$")
TABLE_BEGIN = "<!-- mpg parts --markdown: begin -->"
TABLE_END = "<!-- mpg parts --markdown: end -->"


class PresetError(ValueError):
    """A missing or malformed preset; the message names it and what exists."""


def _read(path: Path, what: str) -> dict[str, Any]:
    try:
        raw = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise PresetError(f"cannot read {what} {path}: {exc}") from None
    if not isinstance(raw, dict):
        raise PresetError(f"{what} {path} must be a mapping")
    return raw


def part_preset_names() -> list[str]:
    """Every part preset as `<category>/<name>`."""
    return sorted(f"{p.parent.name}/{p.stem}" for p in PART_ROOT.glob("*/*.yaml"))


def puzzle_preset_names() -> list[str]:
    """Every assembly preset as `puzzles/<topic>/<name>`."""
    return sorted(f"puzzles/{p.parent.name}/{p.stem}" for p in PUZZLE_ROOT.glob("*/*.yaml"))


def load_part_preset(name: Any) -> dict[str, Any]:
    """The preset's values (with `type`)."""
    m = _PART_NAME.match(name) if isinstance(name, str) else None
    if not m or m.group(1) not in CATEGORIES:
        raise PresetError(f"preset {name!r}: write `<category>/<name>` with a category in "
                          f"{CATEGORIES}, e.g. input/floor_oneshot")
    path = PART_ROOT / m.group(1) / f"{m.group(2)}.yaml"
    if not path.is_file():
        have = [n for n in part_preset_names() if n.startswith(m.group(1) + "/")]
        raise PresetError(f"no part preset {name!r}; {m.group(1)} presets: {have}")
    raw = _read(path, "part preset")
    if not isinstance(raw.get("type"), str):
        raise PresetError(f"part preset {name!r} needs `type: <part type>`")
    if "name" in raw or "preset" in raw:
        raise PresetError(f"part preset {name!r} must not set `name` or `preset`")
    return raw


def load_puzzle_preset(name: Any) -> dict[str, Any]:
    m = _PUZZLE_NAME.match(name) if isinstance(name, str) else None
    if not m:
        raise PresetError(f"room preset {name!r}: write `puzzles/<topic>/<name>`, "
                          f"e.g. puzzles/coins/coin_lock")
    path = PUZZLE_ROOT / m.group(1) / f"{m.group(2)}.yaml"
    if not path.is_file():
        raise PresetError(f"no assembly preset {name!r}; presets: {puzzle_preset_names()}")
    raw = _read(path, "assembly preset")
    if "preset" in raw:
        raise PresetError(f"assembly preset {name!r} must not load presets itself")
    return raw


def expand_part(entry: Any) -> tuple[Any, str | None]:
    """(part entry with its preset's values filled in, preset name or None)."""
    if not isinstance(entry, dict) or "preset" not in entry:
        return entry, None
    name = entry["preset"]
    base = load_part_preset(name)
    if "type" in entry and entry["type"] != base["type"]:
        raise PresetError(f"part {entry.get('name')!r}: preset {name!r} is a {base['type']!r}, "
                          f"not a {entry['type']!r}; drop `type`")
    own = {k: v for k, v in entry.items() if k != "preset"}
    return {**copy.deepcopy(base), **own}, name


def _merge_parts(base: list, over: list) -> list:
    out = [dict(e) if isinstance(e, dict) else e for e in base]
    index = {e.get("name"): i for i, e in enumerate(out) if isinstance(e, dict)}
    for e in over:
        name = e.get("name") if isinstance(e, dict) else None
        if name in index:
            i = index[name]
            out[i] = dict(e) if ("type" in e or "preset" in e) else {**out[i], **e}
        else:
            out.append(e)
            if name is not None:
                index[name] = len(out) - 1
    return out


def _merge_room(base: dict, over: dict) -> dict:
    out = dict(base)
    for k, v in over.items():
        if k == "parts":
            out[k] = _merge_parts(list(base.get(k) or []), list(v or []))
        elif k == "links":
            out[k] = list(base.get(k) or []) + list(v or [])
        else:
            out[k] = v
    return out


def expand_room(raw: dict[str, Any]) -> dict[str, Any]:
    """`raw` with its top-level `preset:` (assembly presets) merged under it."""
    names = raw.get("preset")
    if names is None:
        return raw
    if isinstance(names, str):
        names = [names]
    if not isinstance(names, list) or not names:
        raise PresetError("`preset:` takes a preset name or a list of names, "
                          "e.g. puzzles/coins/coin_lock")
    merged: dict[str, Any] = {}
    for n in names:
        p = copy.deepcopy(load_puzzle_preset(n))
        p.pop("name", None)
        merged = _merge_room(merged, p)
    return _merge_room(merged, {k: v for k, v in raw.items() if k != "preset"})


# ---- `mpg parts` ------------------------------------------------------------------------
def builtin_parts() -> dict[str, type]:
    """Registered part types from this package (not plugins), by name."""
    load_builtins()
    return {n: c for n, c in sorted(PARTS.items()) if c.__module__.startswith("mujoco_puzzle_gym.")}


def catalog(include_plugins: bool = True) -> dict[str, Any]:
    """`mpg parts --json`: registered parts with their value schemas, and the presets."""
    load_builtins()
    table = PARTS if include_plugins else builtin_parts()
    parts = {}
    for n, cls in sorted(table.items()):
        parts[n] = {"role": cls.role, "stage": cls.stage, "module": cls.__module__,
                    "doc": (cls.__doc__ or "").strip().split("\n")[0],
                    "schema": {f.name: f.describe() for f in cls.full_schema().values()}}
    presets = {n: load_part_preset(n) for n in part_preset_names()}
    return {"parts": parts, "presets": presets, "assemblies": puzzle_preset_names()}


def _cell(v: Any) -> str:
    return str(v).replace("|", "\\|").replace("\n", " ")


def _default(f: Field) -> str:
    if f.required:
        return "필수"
    if f.default is None:
        return "—"
    return f"`{_cell(f.default)}`"


def _range(f: Field) -> str:
    if f.kind == "enum":
        return ", ".join(f"`{_cell(c)}`" for c in f.choices)
    if f.kind in ("float", "int", "vec") and (f.min is not None or f.max is not None):
        return _cell(f.range_text())
    return ""


def _field_rows(fields) -> list[str]:
    rows = ["| 값 | 종류 | 단위 | 기본값 | 범위 | 실행 중 변경 | 설명 |", "|---|---|---|---|---|---|---|"]
    for f in fields:
        rows.append(f"| `{f.name}` | {f.kind} | {_cell(f.unit)} | {_default(f)} | {_range(f)} | "
                    f"{'예' if f.runtime else ''} | {_cell(f.doc)} |")
    return rows


def markdown() -> str:
    """Value tables of every built-in part and the preset list (docs/parts-guide.md)."""
    from .parts.base import Part

    common = Part.full_schema()
    out = ["### 모든 부품의 공통 값", "", *_field_rows(common.values()), ""]
    for name, cls in builtin_parts().items():
        own = [f for f in cls.full_schema().values() if common.get(f.name) != f]
        out += [f"### `{name}`", "", f"역할 `{cls.role}`, 갱신 단계 `{cls.stage}`.", ""]
        out += (_field_rows(own) if own else ["공통 값만 받습니다."]) + [""]
    out += ["### 부품 프리셋", "", "| 프리셋 | 부품 종류 | 값 |", "|---|---|---|"]
    for n in part_preset_names():
        p = load_part_preset(n)
        vals = ", ".join(f"`{k}: {_cell(v)}`" for k, v in p.items() if k != "type")
        out.append(f"| `{n}` | `{p['type']}` | {vals} |")
    out += ["", "### 조립 프리셋", ""]
    out += [f"- `{n}`" for n in puzzle_preset_names()]
    return "\n".join(out) + "\n"
