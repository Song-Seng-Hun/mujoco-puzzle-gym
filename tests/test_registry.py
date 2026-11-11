"""R17: a user-registered part and reward term work end to end without library edits.

The user's code lives in a plugin module named by task.yaml `plugins:`; it is
imported here and in every training worker process (SubprocVecEnv, spawn), so
register -> build -> check -> train all see the same part and term.
"""

import json
import sys
import textwrap

import pytest
import yaml

from mujoco_puzzle_gym import registry
from mujoco_puzzle_gym.check import check
from mujoco_puzzle_gym.parts import Switch
from mujoco_puzzle_gym.registry import register_part
from mujoco_puzzle_gym.room import build
from mujoco_puzzle_gym.task import RESOLVED_NAME, load_task
from mujoco_puzzle_gym.train import train

PLUGIN = "user_plugin_r17"
PLUGIN_SRC = textwrap.dedent("""
    from mujoco_puzzle_gym.parts import Switch
    from mujoco_puzzle_gym.registry import register_part, register_reward


    @register_part("big_pad")
    class BigPad(Switch):
        \"\"\"A user part: a wide switch that defaults to 0.4 m square.\"\"\"

        def __init__(self, name, cfg):
            super().__init__(name, {"size": [0.4, 0.4], **cfg})


    @register_reward("alive")
    def alive(env):
        return 0.0 if env.events["fallen"] else 1.0
""")


@pytest.fixture
def plugin_dir(tmp_path, monkeypatch):
    (tmp_path / f"{PLUGIN}.py").write_text(PLUGIN_SRC)
    monkeypatch.syspath_prepend(str(tmp_path))      # spawn workers inherit sys.path
    yield tmp_path
    registry.PARTS.pop("big_pad", None)
    registry.REWARDS.pop("alive", None)
    sys.modules.pop(PLUGIN, None)


@pytest.mark.parametrize("minutes", [
    pytest.param(0.1, id="short"),
    pytest.param(1.0, id="one-minute", marks=pytest.mark.slow),     # the R17 check as written
])
def test_register_build_check_train(asset_dir, repo_root, plugin_dir, minutes):
    tmp_path = plugin_dir
    raw = yaml.safe_load((repo_root / "examples/switch_door.yaml").read_text())
    raw["name"] = "pad_room"
    raw["parts"][0] = {"name": "pad", "type": "big_pad", "pos": [-0.25, 0.45]}
    raw["links"] = [{"from": "pad", "to": "door_b"}]
    room = tmp_path / "pad_room.yaml"
    room.write_text(yaml.safe_dump(raw))
    task_path = tmp_path / "task.yaml"
    task_path.write_text(yaml.safe_dump({
        "room": "pad_room.yaml", "plugins": [PLUGIN], "n_envs": 2,
        "reward": {"progress": 10.0, "success": 10.0, "alive": 0.01},
        "algo_kwargs": {"n_steps": 64, "batch_size": 128}}))

    task = load_task(task_path)                     # imports the plugin
    assert "big_pad" in registry.PARTS and "alive" in registry.REWARDS

    built = build(room, out_dir=tmp_path / "build")
    assert {"name": "pad", "type": "big_pad", "role": "switch"} in built["parts"]

    res = check(room, task=task, seeds=5, need=5)
    assert res["ok"], res
    assert res["order"]["order"] == ["pad", "door_b", "goal"]

    summary = train(task_path, minutes=minutes, runs_dir=tmp_path / "runs", run_id="r17")
    run = tmp_path / "runs" / "r17"
    saved = yaml.safe_load((run / RESOLVED_NAME).read_text())
    assert saved["reward"]["alive"] == 0.01 and saved["plugins"] == [PLUGIN]
    assert saved["env"]["hz"] == 10.0 and saved["n_envs"] == 2
    eps = [json.loads(line) for line in (run / "episodes.jsonl").read_text().splitlines()]
    assert eps, "no finished training episode"
    assert all(e["terms"]["alive"] > 0 for e in eps)            # the worker-side user term ran
    assert "alive" in summary["last_window"]["mean_terms"]


def test_plugin_import_error_is_a_task_error(repo_root):
    from mujoco_puzzle_gym.task import TaskError

    with pytest.raises(TaskError, match="no_such_plugin"):
        load_task({"room": str(repo_root / "examples/switch_door.yaml"),
                   "plugins": ["no_such_plugin"]})


def test_duplicate_registration_needs_replace():
    class Other(Switch):
        pass

    with pytest.raises(ValueError, match="already registered"):
        register_part("switch", Other)
