"""R17: a user-registered part and reward term work end to end without library edits.

register -> build -> check runs here. The R17 "1-minute train" step joins in
PR C (see TODO below); `task.resolved.yaml` writing is tested here already.
"""

import yaml
import pytest

from mujoco_puzzle_gym import registry
from mujoco_puzzle_gym.check import check
from mujoco_puzzle_gym.parts import Switch
from mujoco_puzzle_gym.registry import register_part, register_reward
from mujoco_puzzle_gym.room import build
from mujoco_puzzle_gym.task import RESOLVED_NAME, load_task, write_resolved


class BigPad(Switch):
    """A user part: a wide switch that defaults to 0.4 m square."""

    def __init__(self, name, cfg):
        super().__init__(name, {"size": [0.4, 0.4], **cfg})


@pytest.fixture
def user_terms():
    register_part("big_pad", BigPad)
    register_reward("alive", lambda env: 0.0 if env.events["fallen"] else 1.0)
    yield
    registry.PARTS.pop("big_pad", None)
    registry.REWARDS.pop("alive", None)


def test_register_build_check(asset_dir, repo_root, tmp_path, user_terms):
    raw = yaml.safe_load((repo_root / "examples/switch_door.yaml").read_text())
    raw["name"] = "pad_room"
    raw["parts"][0] = {"name": "pad", "type": "big_pad", "pos": [-0.25, 0.45]}
    raw["links"] = [{"from": "pad", "to": "door_b"}]
    room = tmp_path / "pad_room.yaml"
    room.write_text(yaml.safe_dump(raw))

    built = build(room, out_dir=tmp_path / "build")
    assert {"name": "pad", "type": "big_pad", "role": "switch"} in built["parts"]

    task = {"room": str(room), "reward": {"progress": 10.0, "success": 10.0, "alive": 0.01}}
    res = check(room, task=task, seeds=5, need=5)
    assert res["ok"], res
    assert res["order"]["order"] == ["pad", "door_b", "goal"]

    resolved = write_resolved(load_task(task), tmp_path / "run")
    assert resolved.name == RESOLVED_NAME
    saved = yaml.safe_load(resolved.read_text())
    assert saved["reward"]["alive"] == 0.01 and saved["env"]["hz"] == 10.0
    # TODO(PR C, R17): `mpg train` this task for 1 minute here and assert that
    # runs/<id>/task.resolved.yaml exists and the "alive" term shows up in the
    # per-term reward log.


def test_duplicate_registration_needs_replace():
    with pytest.raises(ValueError, match="already registered"):
        register_part("switch", BigPad)
