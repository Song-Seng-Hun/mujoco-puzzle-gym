#!/usr/bin/env python3
"""Rewrite the generated value table in docs/parts-guide{,.en,.ja}.md from the part schemas.

Run it after adding or changing a part:

    .venv/bin/python tools/refresh_parts_tables.py

`tests/test_guide.py` checks the Korean file against `catalog.markdown()`, and nothing
checked the other two, so they drifted: on 2026-11-17 the Japanese table was 35 rows short
of the parts that existed, one English row still carried a Korean word, and a hand edit
replaced the Japanese headers with Korean ones outright. All three are generated from the
same schemas now, and only the wording in `catalog.LANGS` differs between them - every
field's description comes from the part and is English in all three.
"""

from __future__ import annotations

import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from mujoco_puzzle_gym import catalog     # noqa: E402

FILES = {"ko": "docs/parts-guide.md", "en": "docs/parts-guide.en.md",
         "ja": "docs/parts-guide.ja.md"}


def refresh(lang: str, name: str) -> int:
    """Replace the table between the markers; returns the line count written."""
    path = ROOT / name
    text = path.read_text()
    if text.count(catalog.TABLE_BEGIN) != 1 or text.count(catalog.TABLE_END) != 1:
        raise SystemExit(f"{name}: expected one {catalog.TABLE_BEGIN} and one "
                         f"{catalog.TABLE_END}")
    head, rest = text.split(catalog.TABLE_BEGIN + "\n", 1)
    _, tail = rest.split(catalog.TABLE_END, 1)
    table = catalog.markdown(lang)
    path.write_text(head + catalog.TABLE_BEGIN + "\n" + table + catalog.TABLE_END + tail)
    return len(table.splitlines())


def main() -> None:
    for lang, name in FILES.items():
        print(f"{name}: {refresh(lang, name)} lines")


if __name__ == "__main__":
    main()
