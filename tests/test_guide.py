"""R23: docs/parts-guide.md names every built-in part and preset, and its value
tables are exactly the `mpg parts --markdown` output. To refresh the tables, put
that output between the two marker lines.

R25: the agent entry points (AGENTS.md, skills/mujoco-puzzle-gym/SKILL.md) exist and
every relative link in the documents, headings included, points at something real."""

import re
from pathlib import Path

import pytest

from mujoco_puzzle_gym import catalog

ROOT = Path(__file__).resolve().parents[1]
GUIDE = ROOT / "docs/parts-guide.md"
ENTRY_POINTS = ["AGENTS.md", "skills/mujoco-puzzle-gym/SKILL.md"]
LINKED = [*ENTRY_POINTS, "README.md", *(f"docs/{n}" for n in (
    "guide.md", "guide.en.md", "guide.ja.md", "parts-guide.md", "parts-guide.en.md",
    "parts-guide.ja.md", "parts.md", "community.md"))]
LINK = re.compile(r"\[[^\]]*\]\(([^)\s]+)\)")


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


# ---- R25: the agent entry points and their links ------------------------------------
def _slug(heading: str) -> str:
    """The anchor GitHub gives a heading: no markup, lowercase, spaces to hyphens."""
    text = re.sub(r"[`*]", "", heading.strip()).lower()
    return re.sub(r"[^\w\- ]", "", text).replace(" ", "-")


def test_the_agent_entry_points_exist_and_lead_to_the_guides():
    for name in ENTRY_POINTS:
        text = (ROOT / name).read_text()
        assert "docs/guide.md" in text and "parts-guide.md" in text, name


@pytest.mark.parametrize("name", LINKED)
def test_every_relative_link_points_at_something_real(name):
    path = ROOT / name
    broken = []
    for target in LINK.findall(path.read_text()):
        if target.startswith(("http://", "https://", "mailto:")):
            continue
        rel, _, anchor = target.partition("#")
        dest = (path.parent / rel).resolve() if rel else path
        if not dest.exists():
            broken.append(target)
        elif anchor and dest.suffix == ".md":
            headings = {_slug(line.lstrip("#")) for line in dest.read_text().splitlines()
                        if line.startswith("#")}
            if anchor not in headings:
                broken.append(target)
    assert not broken, f"{name}: {broken}"
