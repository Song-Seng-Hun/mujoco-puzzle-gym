"""R23: docs/parts-guide.md names every built-in part and preset, and its value
tables are exactly the `mpg parts --markdown` output. To refresh the tables, put
that output between the two marker lines."""

from pathlib import Path

from mujoco_puzzle_gym import catalog

GUIDE = Path(__file__).resolve().parents[1] / "docs/parts-guide.md"


def test_guide_names_every_part_and_preset():
    text = GUIDE.read_text()
    missing = [n for n in [*catalog.builtin_parts(), *catalog.part_preset_names(),
                           *catalog.puzzle_preset_names()] if f"`{n}`" not in text]
    assert not missing


def test_guide_tables_match_mpg_parts_markdown():
    text = GUIDE.read_text()
    assert text.count(catalog.TABLE_BEGIN) == 1 and text.count(catalog.TABLE_END) == 1
    table = text.split(catalog.TABLE_BEGIN + "\n")[1].split(catalog.TABLE_END)[0]
    assert table == catalog.markdown(), "refresh docs/parts-guide.md with `mpg parts --markdown`"
