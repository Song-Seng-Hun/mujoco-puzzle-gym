"""R2b: body capability tags; `mpg check` fails a room whose parts need a capability
the body lacks, naming the part and the capability."""

import json
import subprocess
import sys

import pytest
import yaml

from mujoco_puzzle_gym.bodies import BodyAdapter, Capabilities, make_body
from mujoco_puzzle_gym.check import check_body
from mujoco_puzzle_gym.env import PuzzleEnv

ROOM = "examples/switch_door.yaml"
# M1 example rooms. M2 example rooms (examples/rooms/) join this list when they exist (step 8).
ROOMS = [ROOM, "examples/broken/door_unlinked.yaml", "examples/broken/switch_in_wall.yaml"]


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
