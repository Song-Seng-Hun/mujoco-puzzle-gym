"""R2b: body capability tags; `mpg check` fails a room whose parts need a capability
the body lacks, naming the part and the capability."""

import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

from mujoco_puzzle_gym.bodies import BodyAdapter, Capabilities, make_body
from mujoco_puzzle_gym.check import check_body
from mujoco_puzzle_gym.env import PuzzleEnv

ROOM = "examples/switch_door.yaml"
# M1 example rooms and the M2 example rooms (examples/rooms/, step 8).
ROOMS = [ROOM, "examples/broken/door_unlinked.yaml", "examples/broken/switch_in_wall.yaml",
         *sorted(str(p.relative_to(Path(__file__).resolve().parents[1]))
                 for p in (Path(__file__).resolve().parents[1] / "examples/rooms").glob("*.yaml"))]


def _variant(repo_root, tmp_path, edit):
    raw = yaml.safe_load((repo_root / ROOM).read_text())
    edit(raw)
    path = tmp_path / "room.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


def test_capabilities_missing():
    caps = Capabilities(frozenset({"walk"}), {"max_lift_mass": 0.05})
    assert caps.missing(["walk", "grip", "fly"]) == ["grip", "fly"]
    assert BodyAdapter.capabilities.names == frozenset()


def test_default_body_capabilities(asset_dir):
    caps = make_body().capabilities
    assert {"walk", "beak_grab"} <= caps.names
    assert caps.values["max_lift_mass"] > 0


@pytest.mark.parametrize("room", ROOMS)
def test_example_rooms_pass_body_check(asset_dir, repo_root, room):
    env = PuzzleEnv(room=repo_root / room)
    assert check_body(env)["ok"]
    env.close()


def test_required_capability_present_passes(asset_dir, repo_root, tmp_path):
    path = _variant(repo_root, tmp_path, lambda r: r["parts"][0].update(requires_body=["walk"]))
    env = PuzzleEnv(room=path)
    assert check_body(env)["ok"]
    env.close()


def test_missing_capability_fails_check(asset_dir, repo_root, tmp_path):
    path = _variant(repo_root, tmp_path,
                    lambda r: r["parts"][2].update(requires_body=["walk", "grip"]))
    out = subprocess.run([sys.executable, "-m", "mujoco_puzzle_gym.cli", "check", str(path),
                          "--json", "--seeds", "2"], capture_output=True, text=True)
    assert out.returncode != 0, out.stderr
    res = json.loads(out.stdout)
    assert res["ok"] is False and "body" in res["failed"]
    assert res["body"]["missing"] == {"goal": ["grip"]}
    assert "'goal'" in res["body"]["reason"] and "grip" in res["body"]["reason"]
    assert res["solver"]["successes"] == 0 and "skipped" in res["solver"]


# M2 parts declare what they need from the body, using only Microduck capabilities.
M2_PARTS = [
    ({"type": "button", "pos": [0, 0]}, ["walk"]),
    ({"type": "button", "pos": [0, 0], "trigger": "object"}, ["beak_grab"]),
    ({"type": "zone", "pos": [0, 0]}, ["walk"]),
    ({"type": "laser", "pos": [0, -0.3], "to": [0, 0.3]}, ["walk"]),
    ({"type": "pressure", "pos": [0, 0]}, ["beak_grab"]),
    ({"type": "receptacle", "pos": [0, 0]}, ["beak_grab"]),
    ({"type": "checkpoint", "pos": [0, 0]}, ["walk"]),
    ({"type": "gate", "expr": "1"}, []),
    ({"type": "counter", "up": "p"}, []),
    ({"type": "transmitter", "pos": [0, 0], "in": ["p"], "band": "x"}, []),
]


@pytest.mark.parametrize("cfg, needs", M2_PARTS, ids=[c["type"] for c, _ in M2_PARTS])
def test_m2_parts_need_microduck_capabilities(asset_dir, cfg, needs):
    from mujoco_puzzle_gym.registry import get_part
    part = get_part(cfg["type"])("p", {k: v for k, v in cfg.items() if k != "type"})
    assert part.requires_body == needs
    assert make_body().capabilities.missing(part.requires_body) == []


def test_m2_parts_merge_declared_capabilities(asset_dir, repo_root, tmp_path):
    def edit(raw):
        raw["parts"].append({"name": "chk", "type": "checkpoint", "pos": [-0.7, -0.6],
                             "requires_body": ["grip"]})
    env = PuzzleEnv(room=_variant(repo_root, tmp_path, edit))
    assert env.parts["chk"].requires_body == ["walk", "grip"]
    res = check_body(env)
    assert not res["ok"] and res["missing"] == {"chk": ["grip"]}
    env.close()
