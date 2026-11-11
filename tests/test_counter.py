"""R8b counter: one count per turn-on, down/reset/wrap/clamp, target, enable,
snapshot with and without `persist`, and the `counters` observation term.

The lap counter run with the walking body (zones + sequence lock + counter) is
in tests/test_sensors.py next to the zone sensor it needs.
"""

import pytest
import yaml

from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.parts.base import Part
from mujoco_puzzle_gym.parts.logic import Counter
from mujoco_puzzle_gym.registry import register_part
from mujoco_puzzle_gym.signals import wire

ROOM = "examples/switch_door.yaml"


class Src(Part):
    stage, emits, has_body = "input", True, False
    value = 0.0

    def update(self, model, data, body):
        self.write(self.value)


register_part("t_csrc", Src, replace=True)


def rig(**cfg):
    parts = {n: Src(n, {"pos": [0, 0]}) for n in ("u", "d", "r", "p")}
    parts["c"] = Counter("c", {"up": "u", **cfg})
    bus = wire(parts)
    for p in parts.values():
        p.reset(None, None)
    return parts, bus


def feed(parts, bus, frames):
    counts = []
    for f in frames:
        for n, v in f.items():
            parts[n].value = v
        bus.step(None, None, None)
        counts.append(int(bus.get("c.count")))
    return counts


def taps(name, n):
    return [{name: 1}, {name: 0}] * n


def test_counts_each_turn_on_once():
    parts, bus = rig()
    assert feed(parts, bus, [{"u": 1}] * 5) == [1] * 5            # held: no more counts
    assert feed(parts, bus, taps("u", 3)[1:]) == [1, 2, 2, 3, 3]


def test_down_reset_and_target():
    parts, bus = rig(down="d", reset="r", target=2)
    feed(parts, bus, taps("u", 3))
    assert bus.get("c.count") == 3 and bus.get("c.reached") == 1
    feed(parts, bus, taps("d", 2))
    assert bus.get("c.count") == 1 and bus.get("c.reached") == 0
    assert feed(parts, bus, [{"r": 1, "u": 1}]) == [0]            # reset wins in the same step


def test_clamp_and_wrap():
    parts, bus = rig(down="d", max=2)
    assert feed(parts, bus, taps("u", 3))[-1] == 2
    assert feed(parts, bus, taps("d", 4))[-1] == 0
    parts, bus = rig(down="d", min=1, max=3, start=1, wrap=True)
    assert feed(parts, bus, taps("u", 3))[::2] == [2, 3, 1]
    assert feed(parts, bus, taps("d", 1))[0] == 3


def test_start_and_bad_range():
    parts, bus = rig(start=4, target=4)
    assert bus.get("c.count") == 4 and bus.get("c.reached") == 1
    with pytest.raises(ValueError, match="min <= start <= max"):
        Counter("c", {"up": "u", "start": 5, "max": 3})


def test_enable_freezes_count():
    parts, bus = rig(enable="p")
    assert feed(parts, bus, taps("u", 2))[-1] == 0
    parts["p"].value = 1
    assert feed(parts, bus, taps("u", 2))[-1] == 2


@pytest.mark.parametrize("persist, want", [(False, 1), (True, 3)])
def test_snapshot_and_persist(persist, want):
    parts, bus = rig(persist=persist)
    c = parts["c"]
    feed(parts, bus, taps("u", 1))
    saved, values = c.save_state(), bus.snapshot()
    feed(parts, bus, taps("u", 2))
    bus.load(values)
    c.load_state(None, None, saved)          # restore order: bus values, then parts
    assert c.count == want and bus.get("c.count") == want


def test_counters_obs(asset_dir, repo_root, tmp_path):
    raw = yaml.safe_load((repo_root / ROOM).read_text())
    raw["parts"].append({"name": "laps", "type": "counter", "up": "switch_a", "target": 4})
    path = tmp_path / "room.yaml"
    path.write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(task={"obs": ["counters"]}, room=path)
    obs, _ = env.reset(seed=0)
    assert obs.shape == (1,) and obs[0] == 0.0
    env.parts["laps"].count = 2
    assert env._obs()[0] == pytest.approx(0.5)
    env.close()
