"""R36 coins: pick-up, group channels, coin lock (all / need), reset, snapshots,
the `coins` observation and `coin_pick` reward, the static reachability check,
the `coin_lock` preset, and the example room with the scripted solver (slow).
"""

import copy
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml
from rooms import stand_at, tick

import mujoco_puzzle_gym
from mujoco_puzzle_gym.check import check_order
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.obs import COIN_OBS_K
from mujoco_puzzle_gym.room import Room, RoomError

EXAMPLE = "examples/rooms/coin_room.yaml"
PRESET = Path(mujoco_puzzle_gym.__file__).parent / "presets/puzzles/coins/coin_lock.yaml"
FAR = (-0.9, -0.7)                   # spawn corner, away from every coin


@pytest.fixture
def make(asset_dir, repo_root, tmp_path):
    """Env of the example room after `edit(raw)`, at the nominal layout, body at FAR."""
    envs = []

    def _make(edit=None, task=None):
        raw = yaml.safe_load((repo_root / EXAMPLE).read_text())
        raw["spawn"] = {"pos": list(FAR), "yaw": 0.0}
        if edit:
            edit(raw)
        path = tmp_path / f"room{len(envs)}.yaml"
        path.write_text(yaml.safe_dump(raw))
        env = PuzzleEnv(task=task, room=path)
        env.reset(seed=0, options={"layout": env.sample_layout(None)})
        envs.append(env)
        return env

    yield _make
    for e in envs:
        e.close()


def part(raw, name):
    return next(p for p in raw["parts"] if p["name"] == name)


def pick(env, *names):
    for n in names:
        stand_at(env, *env.parts[n].xy)
        stand_at(env, *FAR)


def visible(env, name):
    c = env.parts[name]
    return bool(env.model.geom_rgba[c._gid, 3] > 0)


def door_wants_open(env):
    return env.parts["door_b"].inputs_on()


@pytest.mark.parametrize("picked, opens", [(("coin_a", "coin_b", "coin_c"), True),
                                           (("coin_a", "coin_c"), False)])
def test_all_coins_open_the_door(make, picked, opens):
    env = make()
    pick(env, *picked)
    bus = env.signals
    assert bus.get("coins.count") == len(picked) and bus.get("coins.total") == 3
    assert bus.get("lock.count") == len(picked)
    assert door_wants_open(env) is opens
    for n in ("coin_a", "coin_b", "coin_c"):
        assert visible(env, n) is (n not in picked)
        assert bus.get(n) == float(n in picked)


def test_need_two(make):
    env = make(lambda raw: part(raw, "lock").update(need=2))
    pick(env, "coin_b")
    assert not door_wants_open(env)
    pick(env, "coin_c")
    assert door_wants_open(env) and env.signals.get("lock.reached") == 1.0
    assert env.parts["coins"].satisfied()


def test_coin_is_picked_once_inside_radius(make):
    env = make()
    c = env.parts["coin_a"]
    stand_at(env, c.xy[0] + c.radius + 0.01, c.xy[1])
    assert not c.picked
    stand_at(env, c.xy[0] + c.radius - 0.01, c.xy[1])
    assert c.picked and env.signals.get("coin_a.picked") == 1.0
    tick(env)
    assert env.signals.get("coin_a.picked") == 0.0 and env.signals.get("coin_a") == 1.0


def test_reset_puts_coins_back(make):
    env = make()
    pick(env, "coin_a", "coin_b", "coin_c")
    env.reset(seed=1)
    assert all(visible(env, n) for n in ("coin_a", "coin_b", "coin_c"))
    assert env.signals.get("coins.count") == 0 and env.signals.get("lock.count") == 0
    assert not any(env.parts[n].picked for n in ("coin_a", "coin_b", "coin_c"))


def test_neighbouring_picks_count_twice(make):
    """Two coins picked in the same or in neighbouring steps give two counts."""
    def edit(raw):
        part(raw, "coin_b")["pos"] = [-0.4, 0.4]      # same place as coin_a
        part(raw, "coin_c")["pos"] = [-0.25, 0.4]
        part(raw, "coin_c")["random"] = [0.0, 0.0]
    env = make(edit)
    stand_at(env, -0.4, 0.4)                          # a and b in one step
    pulses = [env.signals.get("coins.picked")]
    stand_at(env, -0.31, 0.4)                         # c the next step
    pulses.append(env.signals.get("coins.picked"))
    for _ in range(4):
        tick(env)
        pulses.append(env.signals.get("coins.picked"))
    assert pulses == [1.0, 0.0, 1.0, 0.0, 1.0, 0.0]
    assert env.signals.get("lock.count") == 3 and door_wants_open(env)


def test_snapshot_restores_picked_coins(make):
    env = make()
    pick(env, "coin_a")
    snap = env.save_snapshot()
    pick(env, "coin_b", "coin_c")
    assert door_wants_open(env)
    env.restore_snapshot(snap)
    assert env.parts["coin_a"].picked and not env.parts["coin_b"].picked
    assert visible(env, "coin_b") and not visible(env, "coin_a")
    assert env.signals.get("coins.count") == 1 and env.signals.get("lock.count") == 1
    tick(env)
    assert not door_wants_open(env)


def test_group_is_the_planned_target(make):
    env = make()
    assert env.plan["order"] == ["coins", "door_b", "goal"]
    group = env.parts["coins"]
    assert env.current_target() is group and group.type_name == "coin_group"
    pick(env, "coin_a")
    stand_at(env, 0.0, 0.0)                           # nearest left: coin_b or coin_c
    left = {n: np.hypot(*env.parts[n].xy) for n in ("coin_b", "coin_c")}
    assert np.allclose(group.xy, env.parts[min(left, key=left.get)].xy)
    mask = group.target_mask(env.grid)
    assert not (mask & env.parts["coin_a"].target_mask(env.grid)).any()


def test_obs_and_reward(make, tmp_path):
    task = {"obs": ["coins"], "reward": {"coin_pick": 1.0}}
    env = make(task=task)
    obs = env._obs()
    assert obs.shape == (1 + 3 * COIN_OBS_K,)
    assert obs[0] == 0.0 and list(obs[3::3]) == [1.0] * COIN_OBS_K
    pick(env, "coin_a", "coin_b")
    obs = env._obs()
    assert obs[0] == pytest.approx(2 / 3)
    assert list(obs[3::3]) == [1.0, 0.0, 0.0] and list(obs[4:]) == [0.0] * 6
    # picks pay once each, counted at the next env step
    _, r, *_ = env.step(np.array([-1.0, 0.0, 0.0]))
    assert r == 2.0
    _, r, *_ = env.step(np.array([-1.0, 0.0, 0.0]))
    assert r == 0.0


def test_progress_has_no_jump_on_pick(make):
    """Picking a coin switches the progress field to the next coin without a penalty."""
    env = make(task={"reward": {"progress": 1.0}})
    a = env.parts["coin_a"]
    env.body.teleport(env.data, (a.xy[0] - 0.2, a.xy[1], 0.0))
    env.step(np.array([-1.0, 0.0, 0.0]))
    env.body.teleport(env.data, (*a.xy, 0.0))
    _, r, *_ = env.step(np.array([-1.0, 0.0, 0.0]))
    assert a.picked and r > 0.05


def test_coin_in_wall_fails_static_check(make):
    def edit(raw):
        part(raw, "coin_c").update(pos=[0.3, -0.6], random=[0.0, 0.0])   # inside the divider
    res = check_order(make(edit))
    assert not res["ok"] and "coin_c" in res["reason"] and "cannot reach" in res["reason"]


def test_coin_behind_its_own_door_fails_static_check(make):
    def edit(raw):
        part(raw, "coin_c").update(pos=[0.8, 0.5], random=[0.0, 0.0])    # behind door_b
    res = check_order(make(edit))
    assert not res["ok"] and "coin_c" in res["reason"] and "behind" in res["reason"]


def test_need_two_tolerates_coin_behind_door(make):
    def edit(raw):
        part(raw, "coin_c").update(pos=[0.8, 0.5], random=[0.0, 0.0])
        part(raw, "lock")["need"] = 2
    assert check_order(make(edit))["ok"]


@pytest.mark.parametrize("edit, msg", [
    (lambda raw: part(raw, "lock").update(need=4), "need 4"),
    (lambda raw: part(raw, "lock").update(group="gold"), "no coins in group 'gold'"),
    (lambda raw: raw["parts"].append({"name": "coins", "type": "goal", "pos": [0, 0]}),
     "also the name of part"),
])
def test_bad_lock_rejected(repo_root, edit, msg):
    raw = yaml.safe_load((repo_root / EXAMPLE).read_text())
    edit(raw)
    with pytest.raises(RoomError, match=msg):
        Room.from_dict(raw)


def test_coin_needs_walk(asset_dir):
    from mujoco_puzzle_gym.bodies import make_body
    from mujoco_puzzle_gym.registry import get_part
    coin = get_part("coin")("c", {"pos": [0, 0]})
    assert coin.requires_body == ["walk"] and make_body().capabilities.missing(["walk"]) == []


def test_preset_builds_and_orders(asset_dir, repo_root, tmp_path):
    preset = yaml.safe_load(PRESET.read_text())
    raw = yaml.safe_load((repo_root / EXAMPLE).read_text())
    raw["parts"] = copy.deepcopy(preset["parts"]) + [part(raw, "goal")]
    path = tmp_path / "preset_room.yaml"
    path.write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(room=path)
    assert check_order(env)["order"] == ["coins", "lock_door", "goal"]
    assert env.parts["lock"].target == 3
    env.close()


@pytest.mark.slow
def test_example_room_check(asset_dir, repo_root):
    out = subprocess.run([sys.executable, "-m", "mujoco_puzzle_gym.cli", "check", EXAMPLE, "--json"],
                         cwd=repo_root, capture_output=True, text=True, timeout=1800)
    res = json.loads(out.stdout)
    assert res["ok"], res
    assert res["order"]["order"] == ["coins", "door_b", "goal"]
    assert res["solver"]["successes"] >= 18


def test_coins_on_map_and_stage_rooms(asset_dir, repo_root, tmp_path):
    """Coins in a letter-map room (R37) and in the next zone of a `stages:` room (R14)."""
    raw = yaml.safe_load((repo_root / "examples/rooms/two_rooms_map.yaml").read_text())
    raw["parts"] = [{"name": "coin_a", "type": "coin", "pos": [-0.75, -0.4]},
                    {"name": "lock", "type": "coin_lock", "pos": [-0.2, 0.4]},
                    {"name": "door", "type": "door", "pos": [-0.05, 0.05], "width": 0.5, "axis": "y",
                     "in": ["lock.reached"]},
                    {"name": "goal", "type": "goal", "pos": [0.65, 0.35], "radius": 0.12}]
    (tmp_path / "map.yaml").write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(room=tmp_path / "map.yaml")
    assert check_order(env)["order"] == ["coins", "door", "goal"]
    env.close()
    raw["parts"][0]["pos"] = [0.95, -0.5]                # a cliff cell: no floor
    (tmp_path / "map.yaml").write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(room=tmp_path / "map.yaml")
    assert "coin_a' at (0.95, -0.50) has no floor under it" in check_order(env)["reason"]
    env.close()

    second = {"name": "second", "size": [1.6, 1.2], "spawn": {"pos": [-0.5, 0.0], "yaw": 0.0},
              "walls": [{"from": [0.0, -0.6], "to": [0.0, -0.2]}, {"from": [0.0, 0.2], "to": [0.0, 0.6]}],
              "parts": [{"name": "coin_a", "type": "coin", "pos": [-0.5, 0.4]},
                        {"name": "lock", "type": "coin_lock", "pos": [-0.3, 0.4]},
                        {"name": "d2", "type": "door", "pos": [0.0, 0.0], "width": 0.4,
                         "in": ["lock.reached"]},
                        {"name": "goal", "type": "goal", "pos": [0.5, 0.0], "radius": 0.12}]}
    first = {"name": "first", "size": [1.6, 1.2], "spawn": {"pos": [-0.5, 0.0], "yaw": 0.0},
             "stages": ["second.yaml"],
             "parts": [{"name": "w", "type": "warp", "pos": [0.4, 0.0], "to": "next_stage"}]}
    (tmp_path / "second.yaml").write_text(yaml.safe_dump(second))
    (tmp_path / "first.yaml").write_text(yaml.safe_dump(first))
    env = PuzzleEnv(room=tmp_path / "first.yaml")
    res = check_order(env)
    assert res["ok"], res
    assert res["order"] == ["w", "coins", "d2", "goal"]
    env.close()
