"""R46: what a part says it needs from the body is true, and `mpg check` says so.

The first test is the one that keeps `docs/bodies.md` honest: its part table is the
written record of the audit, so if a part's `requires_body` changes and the table does
not (or the other way round), this fails and names the part.

The rest are the cases the audit found (docs/bodies.md section 4): parts that were worked
by something the body may not have - a chest, a tow hitch, a beak - and said nothing, so
a room using them passed every `mpg check` stage for a body that could never solve it.
Each is checked from both ends: refused for a body without the capability, and still
accepted for one with it.
"""

import re
from pathlib import Path

import pytest

from mujoco_puzzle_gym.bodies import make_body
from mujoco_puzzle_gym.catalog import builtin_parts
from mujoco_puzzle_gym.check import check_body, check_parts
from mujoco_puzzle_gym.env import PuzzleEnv

#: The parts this package ships. Not `registry.PARTS`: other test modules register
#: parts of their own into it, and a plugin's part is not this repo's to audit.
PARTS = builtin_parts()

DOC = Path(__file__).resolve().parents[1] / "docs/bodies.md"
BODIES = ("microduck", "go2")

#: Smallest config that builds each part (its required fields only).
MINIMAL = {
    "laser": {"to": [0.0, 0.3]}, "gate": {"expr": "1"}, "counter": {"up": "p"},
    "transmitter": {"in": ["p"], "band": "x"}, "receiver": {"band": "x"},
    "wire": {"carries": "p", "to": ["q"], "mount": "wall"},
    "setter": {"in": ["p"], "target": "q.pos", "value": [0, 0]},
    "rope": {"a": "x", "b": "y"}, "destroyer": {"in": ["p"]},
    "platform": {"path": [[0, 0, 0.0], [0.5, 0, 0.1]]}, "warp": {"to": {"pos": [1.0, 0.0]}},
    "relay": {"in": ["p"]},
}
ROW = re.compile(r"^\| `(\w+)` \| ([^|]+?) \|")


def declared(name: str) -> list[str]:
    part = PARTS[name]("p", {"pos": [0.0, 0.0], **MINIMAL.get(name, {})})
    return list(part.requires_body)


def table() -> dict[str, list[str]]:
    """{part: requires_body} as docs/bodies.md writes it (section 2 only: the later
    sections hold the same part names with their pre-audit or per-setting values)."""
    body = DOC.read_text().split("\n## 2.")[1].split("\n## 3.")[0]
    out = {}
    for line in body.splitlines():
        m = ROW.match(line)
        if m and m.group(1) in PARTS:
            cell = m.group(2).strip()
            out[m.group(1)] = [] if cell == "-" else re.findall(r"`(\w+)`", cell)
    return out


# ---- the table is the record ---------------------------------------------------------
def test_the_doc_lists_every_part_once():
    assert set(table()) == set(PARTS), "docs/bodies.md section 2 misses or invents a part"


@pytest.mark.parametrize("name", sorted(PARTS))
def test_each_part_declares_what_the_doc_says(name):
    assert declared(name) == table()[name], \
        f"{name}: code says {declared(name)}, docs/bodies.md says {table()[name]}"


# ---- the capabilities the audit added ------------------------------------------------
def test_the_bodies_declare_the_capabilities_parts_ask_for():
    """Every capability any built-in part can require is one some body has: a name no
    body declares would refuse every room silently rather than name a real gap."""
    asked = {c for name in PARTS for c in declared(name)}
    have = set().union(*(make_body(b).capabilities.names for b in BODIES))
    assert asked <= have, f"no body declares {sorted(asked - have)}"


@pytest.mark.parametrize("cfg, capability", [
    ({"type": "lever", "mode": "push", "facing": 180.0}, "chest_push"),
    ({"type": "dial"}, "chest_push"),
    ({"type": "hitch_station"}, "tow"),
    ({"type": "low_bar"}, "roller"),
    ({"type": "wire", "carries": "s", "to": ["g"], "mount": "wall",
      "breakable": True, "by": ["peck"]}, "grab"),
], ids=["push_lever", "dial", "hitch_station", "low_bar", "peck_wire"])
def test_a_part_the_body_cannot_work_is_refused_by_name(cfg, capability):
    part = PARTS[cfg["type"]]("p", {"pos": [0.0, 0.0], **{k: v for k, v in cfg.items()
                                                          if k != "type"}})
    duck, other = (make_body(b).capabilities for b in BODIES)
    assert capability in part.requires_body
    assert duck.missing(part.requires_body) == []            # the body it was measured on
    assert capability in other.missing(part.requires_body)   # the one that cannot work it


# ---- end to end: the gate fires in a real room ---------------------------------------
def _room(tmp_path, name, parts, body):
    import yaml

    path = tmp_path / f"{name}.yaml"
    path.write_text(yaml.safe_dump(
        {"name": name, "size": [5.0, 4.0], "wall": {"thickness": 0.06, "height": 0.5},
         "spawn": {"pos": [-1.8, 0.0], "yaw": 0.0}, "parts": parts}))
    return PuzzleEnv(task={"room": path, "body": body, "obs": ["gravity"],
                           "env": {"hz": 10, "episode_seconds": 20}})


GOAL = {"name": "g", "type": "goal", "pos": [1.5, 0.0], "radius": 0.2}
#: One room per capability the second body lacks (R45): it must be refused by name.
GATED_ROOMS = {
    "chest_push": [{"name": "lv", "type": "lever", "pos": [0.0, 0.0], "mode": "push",
                    "facing": 180.0}, GOAL],
    "grab": [{"name": "cb", "type": "cube", "pos": [0.4, 0.0]}, GOAL],
    "roller": [{"name": "bar", "type": "low_bar", "pos": [0.4, 0.0]}, GOAL],
    # The station is not itself a hitch: the rope runs between a towable and a post, and
    # the station is where the body would take the near end over.
    "tow": [{"name": "hs", "type": "hitch_station", "pos": [0.4, 0.0], "mode": "attach"},
            {"name": "an", "type": "anchor", "pos": [0.0, 0.8]},
            {"name": "sl", "type": "npc", "pos": [0.4, 0.5], "hitch": True},
            {"name": "rp", "type": "rope", "pos": [0.2, 0.65], "a": "sl", "b": "an",
             "length": 0.9, "hitch_radius": 0.5}, GOAL],
}


@pytest.mark.parametrize("capability", sorted(GATED_ROOMS))
def test_a_room_the_body_cannot_solve_is_refused_by_name(asset_dir, go2_asset_dir,
                                                        tmp_path, capability):
    env = _room(tmp_path, f"gate_{capability}", GATED_ROOMS[capability], "go2")
    res = check_body(env)
    assert not res["ok"], f"{capability}: the gate let the room through"
    part = next(n for n, caps in res["missing"].items() if capability in caps)
    assert f"'{part}'" in res["reason"] and capability in res["reason"]
    env.close()


@pytest.mark.parametrize("capability", sorted(GATED_ROOMS))
def test_the_same_rooms_pass_for_the_body_they_were_measured_on(asset_dir, tmp_path,
                                                                capability):
    """The gate refuses what this body cannot do, not every room it is given."""
    env = _room(tmp_path, f"ok_{capability}", GATED_ROOMS[capability], "microduck")
    assert check_body(env)["ok"]
    env.close()


# ---- numbers that only mean something against a body --------------------------------
def test_a_wind_too_weak_for_the_body_is_reported(asset_dir, go2_asset_dir, tmp_path):
    """The preset 1.0 N is 14 % of one body's weight and 0.7 % of the other's."""
    parts = [{"name": "w", "type": "wind", "pos": [0.0, 0.0], "power": 1.0},
             {"name": "g", "type": "goal", "pos": [1.5, 0.0], "radius": 0.2}]
    heavy = _room(tmp_path, "wind_heavy", parts, "go2")
    res = check_parts(heavy)
    assert res["ok"] and any("'w'" in w and "weight" in w for w in res["warnings"])
    heavy.close()
    light = _room(tmp_path, "wind_light", parts, "microduck")
    assert "warnings" not in check_parts(light)       # unchanged for the measured body
    light.close()


def test_a_plate_narrower_than_the_body_is_reported(asset_dir, go2_asset_dir, tmp_path):
    parts = [{"name": "pl", "type": "pressure", "pos": [0.0, 0.0], "trigger": "any",
              "min_mass": 1.0},
             {"name": "g", "type": "goal", "pos": [1.5, 0.0], "radius": 0.2}]
    env = _room(tmp_path, "plate", parts, "go2")
    res = check_parts(env)
    assert res["ok"]
    assert any("footprint" in w for w in res["warnings"])        # reads a fraction of it
    assert any("own mass" in w for w in res["warnings"])         # and arriving is enough
    env.close()


def test_the_npc_speed_warning_is_measured_against_this_body(asset_dir, go2_asset_dir,
                                                             tmp_path):
    """0.5 m/s outruns one body and not the other; the warning used to be one number."""
    parts = [{"name": "n", "type": "npc", "pos": [0.5, 0.5], "speed": 0.5},
             {"name": "g", "type": "goal", "pos": [1.5, 0.0], "radius": 0.2}]
    slow = _room(tmp_path, "npc_slow", parts, "microduck")
    assert any("'n'" in w and "cannot catch" in w for w in check_parts(slow)["warnings"])
    slow.close()
    fast = _room(tmp_path, "npc_fast", parts, "go2")
    assert "warnings" not in check_parts(fast)
    fast.close()
