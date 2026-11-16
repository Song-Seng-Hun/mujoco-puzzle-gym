"""R67: a capability name no part asks for is a decoration, and this fails on it.

M4's R60 already said it - its own check reads "쓰이지 않는 능력 이름이 없다(표에서 열이 비면
테스트 실패)" - and the test was never written, so the arm shipped `reach` and `gripper` that
nothing requires while the same milestone refused to add `two_hands` for exactly that reason.
One rule, applied on one side only, because nothing executed it. Hence a test.

What counts as "asked for" is collected two ways, and the union is used:

1. every string literal inside a `needs_body(...)` call in the package. A literal is found
   even in a branch no default configuration reaches - `button` asks for `beak_grab` only
   when it is wall-mounted, `zone` only when its role is not `switch` - which matters
   because the failure mode to avoid is deleting a name that *is* required somewhere.
2. the schema choices of a part that requires a name it reads from its own config
   (`addon_station` does `needs_body("walk", self.addon)`), since those names appear in no
   literal at all.

Both directions are checked here: a declared name nothing asks for, and a required name no
body has. The second already lived in `tests/test_capability_gate.py` for two bodies; it is
restated here over every body in `BODIES` so adding a fifth cannot quietly skip it.
"""

import ast
import inspect
from pathlib import Path

import pytest

from mujoco_puzzle_gym.bodies import BODIES, make_body
from mujoco_puzzle_gym.catalog import builtin_parts

SRC = Path(__file__).resolve().parents[1] / "src/mujoco_puzzle_gym"


def _calls(tree: ast.AST):
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
                and node.func.attr == "needs_body":
            yield node


def _values(node: ast.AST, fields_of: dict[str, tuple]) -> set[str]:
    """Every string this argument expression can hand to `needs_body`.

    Parsed rather than matched, because the strings in a call are not all capabilities:
    `needs_body("walk" if "body" in TRIGGERS[self.trigger] else "beak_grab")` passes
    `walk` or `beak_grab` and only *tests* `body`. A regex over that line cannot tell the
    two apart and collects the trigger names as capabilities.
    """
    if isinstance(node, ast.Constant):
        return {node.value} if isinstance(node.value, str) else set()
    if isinstance(node, ast.IfExp):                       # a or b, never the condition
        return _values(node.body, fields_of) | _values(node.orelse, fields_of)
    if isinstance(node, (ast.List, ast.Tuple, ast.Set)):
        return set().union(set(), *(_values(e, fields_of) for e in node.elts))
    if isinstance(node, ast.Starred):
        return _values(node.value, fields_of)
    if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) \
            and node.value.id == "self":
        # A name the part reads from its own config (`needs_body("walk", self.addon)`):
        # the possible values are that field's schema choices, and they appear in no
        # literal anywhere.
        for f in fields_of.get(node.attr, ()):
            return {str(c) for c in f.choices}
    return set()


def asked() -> set[str]:
    """Every capability name some built-in part can require, over every configuration."""
    by_module: dict[str, dict[str, tuple]] = {}
    for cls in builtin_parts().values():
        path = inspect.getsourcefile(cls)
        if path:
            fields = by_module.setdefault(path, {})
            for f in getattr(cls, "fields", ()):
                if getattr(f, "choices", None):
                    fields.setdefault(f.name, ()); fields[f.name] = (*fields[f.name], f)
    out: set[str] = set()
    for path in sorted(SRC.rglob("*.py")):
        tree = ast.parse(path.read_text())
        for call in _calls(tree):
            for arg in call.args:
                out |= _values(arg, by_module.get(str(path), {}))
    return out


@pytest.mark.parametrize("body", BODIES)
def test_no_body_declares_a_capability_no_part_asks_for(body, all_asset_dirs):
    """The one that fails on the arm's `reach` and `gripper` before R67 is applied.

    A name only the body side knows is not a gate: `mpg check` compares names, so a
    capability nothing requires can never refuse or admit anything. It reads as a promise
    the environment does not keep - the humanoid's `two_hands` was left unbuilt on this
    reasoning, and the same reasoning has to hold for every body.
    """
    declared = set(make_body(body).capabilities.names)
    assert declared <= asked(), (
        f"{body} declares {sorted(declared - asked())}, which no part requires: "
        "either a part must ask for it or the body must stop declaring it")


def test_no_part_asks_for_a_capability_no_body_has(all_asset_dirs):
    """The converse: a required name no body declares would refuse every room silently."""
    have: set[str] = set()
    for body in BODIES:
        have |= set(make_body(body).capabilities.names)
    assert asked() <= have, f"no body declares {sorted(asked() - have)}"


def test_the_scan_sees_a_requirement_only_a_branch_reaches():
    """Guard for the scan itself: the names that live in a conditional branch, which an
    audit built from default configurations would miss, are in the set it returns."""
    found = asked()
    assert {"walk", "grab", "roller", "tow", "chest_push"} <= found
    # `addon_station` requires `self.addon`, whose value is a schema choice, and the
    # trigger/role names that only appear in a condition are not capabilities.
    assert "roller" in found
    assert not ({"body", "object", "npc", "any", "switch", "sensor"} & found)


def test_the_scan_notices_a_name_that_is_asked_for_nowhere():
    """Guard the other way: a name absent from every `needs_body` call is reported as
    unused, so this test fails if the scan ever returns everything."""
    assert "jetpack" not in asked()


def test_the_old_name_still_works(all_asset_dirs):
    """R67 renamed `beak_grab` to `grab` and `beak_height` to `reach_height`; a room,
    preset or plugin part written against the old names keeps running.

    Checked on the bodies that have the capability and on one that does not, because an
    alias that quietly admitted everything would be worse than the leak it removes.
    """
    from mujoco_puzzle_gym.bodies.base import canonical

    assert canonical("beak_grab") == "grab" and canonical("walk") == "walk"
    duck, arm, walker = (make_body(b).capabilities for b in ("microduck", "so101", "go2"))
    for caps in (duck, arm):
        assert caps.missing(["beak_grab"]) == []          # old name, current body
        assert caps.missing(["grab"]) == []
    assert walker.missing(["beak_grab"]) == ["beak_grab"]  # reported as the caller wrote it
    assert duck.limit("beak_height") == duck.limit("reach_height") == 0.21
    assert arm.limit("beak_height") == arm.limit("reach_height") == 0.45


def test_a_body_declaring_only_the_old_name_is_read_as_the_new_one():
    """The alias works from the body's side too, so a plugin body written before the
    rename is compared on equal terms rather than silently lacking the capability."""
    from mujoco_puzzle_gym.bodies.base import Capabilities

    old = Capabilities(names=frozenset({"walk", "beak_grab"}), values={"beak_height": 0.3})
    assert old.missing(["grab"]) == [] and old.missing(["beak_grab"]) == []
    assert old.limit("reach_height") == 0.3
