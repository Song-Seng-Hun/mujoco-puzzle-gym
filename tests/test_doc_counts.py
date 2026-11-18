"""The catalog counts the docs state, pinned to the catalog itself.

Six documents in three languages say how many part types, part presets and room
presets this repository has, and nothing checked those numbers against the
package. Adding a room or a part leaves them untouched, and a reader has no way
to tell a stale number from a real one. These tests fail when the catalog grows
and a document still carries the old count.

A room preset brings its own extent (`size` or `map`); a puzzle preset without
one is an assembly a room lays down with `preset:` (today only
`coins/coin_lock`), so the file count under `presets/puzzles` is one higher than
the number of rooms. Counting files instead of rooms is exactly how the room
count gets misreported.
"""
from __future__ import annotations

import re
from pathlib import Path

import pytest
import yaml

from mujoco_puzzle_gym import catalog

REPO = Path(__file__).resolve().parent.parent

PART_TYPES = len(catalog.builtin_parts())
PART_PRESETS = len(catalog.part_preset_names())


def _rooms_and_assemblies() -> tuple[list[str], list[str]]:
    rooms, assemblies = [], []
    for name in sorted(catalog.puzzle_preset_names()):
        raw = yaml.safe_load((catalog.PRESET_ROOT / f"{name}.yaml").read_text()) or {}
        (rooms if ("size" in raw or "map" in raw) else assemblies).append(name)
    return rooms, assemblies


ROOMS, ASSEMBLIES = _rooms_and_assemblies()
ROOM_PRESETS = len(ROOMS)

# Each pattern locates a stated count by the words around it. A pattern that
# matches nothing in a document that should carry it is itself a failure: the
# wording changed and this list has to follow, rather than silently stop
# checking.
ROOM_PATTERNS = {
    "ko": (r"방 프리셋[^0-9\n]{0,4}\*{0,2}(\d+)", r"방 (\d+)개", r"(\d+)방 사다리"),
    "en": (r"(\d+) room presets", r"all (\d+) rooms", r"over all (\d+)"),
    "ja": (r"部屋プリセット[^0-9\n]{0,4}\*{0,2}(\d+)", r"(\d+)部屋"),
}
PART_TYPE_PATTERNS = {
    "ko": (r"내장 부품 \*\*(\d+)종",),
    "en": (r"\*\*(\d+) built-in part types\*\*",),
    "ja": (r"内蔵部品\*\*(\d+)種",),
}
PART_PRESET_PATTERNS = {
    "ko": (r"\*\*프리셋 (\d+)종",),
    "en": (r"\*\*(\d+) presets\*\*",),
    "ja": (r"\*\*プリセット(\d+)種",),
}

# (path, language, does it state the part and preset counts as well)
DOCS = [
    ("README.md", "ko", True),
    ("docs/challenges.md", "ko", False),
    ("docs/README.en.md", "en", True),
    ("docs/challenges.en.md", "en", False),
    ("docs/README.ja.md", "ja", True),
    ("docs/challenges.ja.md", "ja", False),
]


def _stated(text: str, patterns: tuple[str, ...]) -> list[int]:
    return [int(n) for p in patterns for n in re.findall(p, text)]


def test_the_assembly_is_not_counted_as_a_room():
    """45 puzzle preset files, 44 rooms: the odd one out is an assembly."""
    assert len(catalog.puzzle_preset_names()) == ROOM_PRESETS + len(ASSEMBLIES)
    assert ASSEMBLIES == ["puzzles/coins/coin_lock"], ASSEMBLIES
    assert ROOM_PRESETS == 44, ROOM_PRESETS


@pytest.mark.parametrize("doc,lang,full", DOCS, ids=[d[0] for d in DOCS])
def test_the_docs_state_the_real_room_count(doc, lang, full):
    text = (REPO / doc).read_text(encoding="utf-8")
    stated = _stated(text, ROOM_PATTERNS[lang])
    assert stated, f"{doc}: no room-count sentence matched; update ROOM_PATTERNS"
    assert set(stated) == {ROOM_PRESETS}, f"{doc}: states {sorted(set(stated))}, real {ROOM_PRESETS}"


@pytest.mark.parametrize("doc,lang,full", DOCS, ids=[d[0] for d in DOCS])
def test_the_docs_state_the_real_part_counts(doc, lang, full):
    if not full:
        pytest.skip("this document does not state the part counts")
    text = (REPO / doc).read_text(encoding="utf-8")
    types = _stated(text, PART_TYPE_PATTERNS[lang])
    presets = _stated(text, PART_PRESET_PATTERNS[lang])
    assert types, f"{doc}: no part-type count matched; update PART_TYPE_PATTERNS"
    assert presets, f"{doc}: no part-preset count matched; update PART_PRESET_PATTERNS"
    assert set(types) == {PART_TYPES}, f"{doc}: states {sorted(set(types))}, real {PART_TYPES}"
    assert set(presets) == {PART_PRESETS}, f"{doc}: states {sorted(set(presets))}, real {PART_PRESETS}"
