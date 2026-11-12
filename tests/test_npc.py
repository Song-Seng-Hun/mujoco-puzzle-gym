"""R29 NPC robots: behaviours, leash, determinism, button presses, snapshots,
the speed warning, `herd` and the example room `herd.yaml` (slow).

Behaviour tests hold the body still (zero command) and read the sign of the
robot -> body distance change after 2 s. `shy_gaze` uses the R27 gaze sensor
(the standing body looks straight ahead). `lure` follows the bait while the
beak holds it (R18 grab fills `World.held`) and is herded after `grab <bait>`.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from mujoco_puzzle_gym.check import check, check_order, check_parts
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.obs import npcs
from mujoco_puzzle_gym.room import RoomError
from mujoco_puzzle_gym.solver import ScriptedSolver, run_episode

from rooms import body_steps, open_room, tick
from test_doors import Source  # noqa: F401  (registers t_signal)

ROOT = Path(__file__).resolve().parents[1]
HERD = ROOT / "examples/rooms/herd.yaml"
DUCK = (-1.0, -1.0)


def robot(env, name="bot"):
    return env.parts[name]


def gap(env, name="bot") -> float:
    return float(np.linalg.norm(robot(env, name).pos(env.data) - env.body_state.pos[:2]))


def change_after(env, seconds=2.0, name="bot") -> float:
    d0 = gap(env, name)
    body_steps(env, seconds)
    return gap(env, name) - d0


def npc_env(tmp_path, name="npc", extra=(), **bot):
    entry = {"name": "bot", "type": "npc", **bot}
    return open_room(tmp_path, [entry, *extra], spawn=DUCK, name=name)


# ---- behaviours ------------------------------------------------------------------------------
def test_flee_runs_away(asset_dir, tmp_path):
    env = npc_env(tmp_path, behavior="flee", pos=[-0.7, -1.0])
    assert change_after(env) > 0.1
    assert robot(env).acting is False                     # out of sense now: stands


def test_flee_out_of_sense_stays(asset_dir, tmp_path):
    env = npc_env(tmp_path, behavior="flee", pos=[0.0, -1.0])
    assert abs(change_after(env)) < 0.01


def test_follow_comes_and_keeps_its_distance(asset_dir, tmp_path):
    env = npc_env(tmp_path, behavior="follow", pos=[-0.3, -1.0], sense=0.8, keep=0.3)
    assert change_after(env) < -0.1
    body_steps(env, 4.0)
    assert gap(env) == pytest.approx(0.3, abs=0.04)


def test_lure_comes_to_the_bait_only_while_it_is_held(asset_dir, tmp_path):
    from test_grab import CUBE, grab, to_beak

    env = npc_env(tmp_path, behavior="lure", bait="c", pos=[-0.4, -1.0], sense=0.8, keep=0.2,
                  extra=[CUBE])
    assert abs(change_after(env)) < 0.01                 # bait on the floor far off: stays
    to_beak(env, "c")
    grab(env)
    assert change_after(env) < -0.1                       # held: comes towards it
    body_steps(env, 3.0)
    bait = env.parts["c"].pos(env.data)[:2]
    assert np.linalg.norm(robot(env).pos(env.data) - bait) == pytest.approx(0.2, abs=0.04)
    grab(env, on=False, seconds=0.5)
    assert abs(change_after(env)) < 0.01                 # let go: stops


def test_patrol_drives_its_path_and_waits_for_the_body(asset_dir, tmp_path):
    # path from (0, -1) towards the body at (-1, -1): it stops within sense of it
    env = npc_env(tmp_path, behavior="patrol", pos=[-0.3, -1.0], path=[[0.0, 0.0], [-0.65, 0.0]],
                  sense=0.4)
    assert change_after(env) < -0.1
    body_steps(env, 3.0)
    assert 0.36 < gap(env) < 0.42 and robot(env).acting  # waiting in front of the body


def test_patrol_loops_when_free(asset_dir, tmp_path):
    env = npc_env(tmp_path, behavior="patrol", pos=[0.3, 0.5], path=[[0.0, 0.0], [0.3, 0.0]])
    xs = []
    body_steps(env, 12.0, every=lambda e: xs.append(robot(e).pos(e.data)[0]))
    far = next(i for i, x in enumerate(xs) if x > 0.57)                   # out to the far point
    assert max(xs) < 0.6 + 0.005 and min(xs[far:]) < 0.33                  # and back


def test_guard_steps_in_front_of_the_body(asset_dir, tmp_path):
    env = npc_env(tmp_path, behavior="guard", pos=[-0.4, -1.0], sense=0.4, reach=0.25)
    assert change_after(env) < -0.1
    body_steps(env, 3.0)
    assert robot(env).pos(env.data)[0] == pytest.approx(-0.65, abs=0.02)   # `reach` out


def test_guard_stays_on_post_without_the_body(asset_dir, tmp_path):
    env = npc_env(tmp_path, behavior="guard", pos=[0.4, 0.4], sense=0.4, reach=0.25)
    body_steps(env, 2.0)
    assert np.linalg.norm(robot(env).pos(env.data) - [0.4, 0.4]) < 0.01


def test_seek_button_goes_on_signal_and_presses_it(asset_dir, tmp_path):
    sig = {"name": "s", "type": "t_signal", "pos": [1.0, -1.0]}
    btn = {"name": "b", "type": "button", "kind": "momentary", "trigger": "npc",
           "pos": [0.5, 0.5], "size": [0.2, 0.2], "marker": {"id": 7}}
    env = npc_env(tmp_path, behavior="seek_button", pos=[-0.2, 0.0], button="b", **{"in": ["s"]},
                  extra=[sig, btn])
    bot, b = robot(env), env.parts["b"]
    d_btn = lambda: float(np.linalg.norm(bot.pos(env.data) - [0.5, 0.5]))  # noqa: E731
    d0 = d_btn()
    body_steps(env, 2.0)
    assert d_btn() == pytest.approx(d0, abs=0.005) and not b.on
    env.parts["s"].value = 1.0
    body_steps(env, 2.0)
    assert d_btn() < d0 - 0.1
    on = []
    body_steps(env, 12.0, every=lambda e: on.append((b.on, robot(e).on_id, e.signals.get("bot.on"))))
    assert d_btn() < 0.02
    inside = [o for o in on if o[1] == 7]
    assert inside and all(o[0] and o[2] == 7.0 for o in inside)   # on while it stands on it
    assert all(not o[0] and o[2] == -1.0 for o in on if o[1] == -1)


def test_sleepy_moves_then_stops(asset_dir, tmp_path):
    env = npc_env(tmp_path, behavior="sleepy", pos=[0.0, 0.5], path=[[0.0, 0.0], [0.8, 0.0]],
                  awake=2.0, asleep=2.0)
    bot = robot(env)
    p0 = bot.pos(env.data)
    body_steps(env, 2.0)
    p1 = bot.pos(env.data)
    body_steps(env, 0.4)                                  # brakes at its accel
    p2 = bot.pos(env.data)
    body_steps(env, 1.5)
    p3 = bot.pos(env.data)
    body_steps(env, 2.0)
    assert p1[0] - p0[0] > 0.1                            # awake: drives
    assert np.linalg.norm(p3 - p2) < 0.005                # asleep: still
    assert bot.pos(env.data)[0] - p3[0] > 0.05            # awake again


def test_shy_gaze_creeps_unseen_and_freezes_when_looked_at(asset_dir, tmp_path):
    side = npc_env(tmp_path, name="side", behavior="shy_gaze", pos=[-1.0, -0.45], keep=0.25)
    assert change_after(side) < -0.1                      # beside the body: not seen, comes
    front = npc_env(tmp_path, name="front", behavior="shy_gaze", pos=[-0.45, -1.0], keep=0.25)
    body_steps(front, 2.0)                                # comes until the body sees its marker
    assert front.signals.get("bot.seen") == 1.0
    assert abs(change_after(front)) < 0.005               # then freezes


def test_enable_off_stops_in_place(asset_dir, tmp_path):
    sig = {"name": "s", "type": "t_signal", "pos": [1.0, -1.0]}
    env = npc_env(tmp_path, behavior="flee", pos=[-0.7, -1.0], enable="s", extra=[sig])
    assert abs(change_after(env)) < 0.005
    env.parts["s"].value = 1.0
    assert change_after(env) > 0.1


def test_moving_channel(asset_dir, tmp_path):
    env = npc_env(tmp_path, behavior="flee", pos=[-0.7, -1.0])
    seen = []
    body_steps(env, 2.0, every=lambda e: seen.append(e.signals.get("bot.moving")))
    assert seen[0] == 0.0 and 1.0 in seen


# ---- leash, walls ----------------------------------------------------------------------------
def test_never_leaves_its_leash(asset_dir, tmp_path):
    box = [-0.9, -1.1, -0.5, -0.7]
    env = npc_env(tmp_path, behavior="flee", pos=[-0.75, -0.95], leash=box)
    bot = robot(env)
    seen = []
    body_steps(env, 4.0, every=lambda e: seen.append(bot.pos(e.data)))
    env.push("kick", bot._bid, [2.0, 2.0, 0.0])           # a hard shove out of the box
    body_steps(env, 2.0, every=lambda e: seen.append(bot.pos(e.data)))
    xy = np.array(seen)
    tol = 0.005                                           # soft joint limit under the shove
    assert (xy[:, 0] >= box[0] - tol).all() and (xy[:, 0] <= box[2] + tol).all()
    assert (xy[:, 1] >= box[1] - tol).all() and (xy[:, 1] <= box[3] + tol).all()
    assert np.linalg.norm(xy[len(seen) // 2] - [-0.75, -0.95]) > 0.05     # it did run


def test_cornered_flee_slides_and_stops(asset_dir, tmp_path):
    """Fleeing into a wall it slides along it, and stops short of the corner walls."""
    env = open_room(tmp_path, [{"name": "bot", "type": "npc", "behavior": "flee", "sense": 1.0,
                                "pos": [-1.25, -1.0]}], spawn=(-1.0, -1.25), name="corner")
    body_steps(env, 6.0)
    p = robot(env).pos(env.data)
    assert p[1] > -0.75                                    # slid up the left wall
    assert -1.5 + 0.06 < p[0] < -1.5 + 0.15                # along it, not into it


def test_stands_on_a_raised_floor_tile(asset_dir, tmp_path):
    tile = {"name": "tile", "type": "floor_tile", "pos": [0.3, 0.3], "size": [0.8, 0.8], "height": 0.04}
    env = npc_env(tmp_path, behavior="patrol", pos=[0.3, 0.3], path=[[0.0, 0.0], [0.2, 0.0]],
                  extra=[tile])
    bot = robot(env)
    chassis = env.model.geom(bot.body_name + "/chassis").id
    bottom = env.data.geom_xpos[chassis][2] - bot.half[2]
    assert bottom == pytest.approx(0.04 + 0.006, abs=1e-6)            # CLEAR above the tile top
    xs = []
    body_steps(env, 4.0, every=lambda e: xs.append(bot.pos(e.data)[0]))
    assert max(xs) > 0.48                                                # drives its path on it


# ---- env moves (conveyor R12, warp R14) ----------------------------------------------------
def test_conveyor_carries_a_robot(asset_dir, tmp_path):
    belt = {"name": "c", "preset": "device/conveyor", "pos": [0.5, 0.5], "size": [1.2, 0.3],
            "speed": 0.1}
    env = npc_env(tmp_path, behavior="flee", pos=[0.2, 0.5], extra=[belt])   # body far: it stands
    x0 = robot(env).pos(env.data)[0]
    body_steps(env, 2.0)
    assert robot(env).pos(env.data)[0] - x0 == pytest.approx(0.2, abs=0.02)


def test_warp_pad_teleports_a_robot(asset_dir, tmp_path):
    pad = {"name": "w", "type": "warp", "pos": [0.5, 0.5], "trigger": "npc",
           "to": {"pos": [-0.5, 0.8], "yaw": 1.0}}
    env = npc_env(tmp_path, behavior="flee", pos=[0.5, 0.5], extra=[pad])
    tick(env)
    env._run_requests()
    bot = robot(env)
    assert np.linalg.norm(bot.pos(env.data) - [-0.5, 0.8]) < 0.005
    assert bot.yaw(env.data) == pytest.approx(1.0) and np.allclose(bot.vel(env.data), 0.0)


# ---- determinism, snapshots ----------------------------------------------------------------
def run_traj(path, seed):
    env = PuzzleEnv(room=path)
    env.reset(seed=seed)
    bot = env.parts["bot"]
    out = []
    for _ in range(30):
        env.step(np.array([1.0, 0.0, 0.2], dtype=np.float32))
        out.append(np.concatenate([bot.pos(env.data), bot.vel(env.data)]))
    env.close()
    return np.array(out)


def test_same_seed_same_trajectory(asset_dir):
    a, b = run_traj(HERD, 3), run_traj(HERD, 3)
    assert np.array_equal(a, b)
    assert not np.allclose(run_traj(HERD, 4), a)          # the seed moves its start


def test_snapshot_restores_robot_and_memory(asset_dir, tmp_path):
    env = npc_env(tmp_path, behavior="sleepy", pos=[0.0, 0.5], path=[[0.0, 0.0], [0.3, 0.0]],
                  awake=2.5, asleep=1.0)
    bot = robot(env)
    body_steps(env, 3.0)                                   # past a waypoint, asleep
    snap = env.save_snapshot()
    state = bot.save_state()
    a = []
    body_steps(env, 3.0, every=lambda e: a.append(bot.pos(e.data)))
    env.restore_snapshot(snap)
    assert bot.wp == state["wp"] and bot.timer == pytest.approx(state["timer"])
    b = []
    body_steps(env, 3.0, every=lambda e: b.append(bot.pos(e.data)))
    assert np.abs(np.array(a) - np.array(b)).max() < 0.002


def test_snapshot_keeps_the_leash_of_its_layout(asset_dir):
    env = PuzzleEnv(room=HERD)
    env.reset(seed=1)
    bot = env.parts["bot"]
    rng = env.model.jnt_range[bot._j[0]].copy()
    snap = env.save_snapshot()
    env.reset(seed=2)                                     # another placement, other limits
    assert not np.allclose(env.model.jnt_range[bot._j[0]], rng)
    env.restore_snapshot(snap)
    assert np.allclose(env.model.jnt_range[bot._j[0]], rng)
    env.close()


# ---- other parts, obs, checks ---------------------------------------------------------------
def test_pressure_plate_counts_robot_mass(asset_dir, tmp_path):
    plate = {"name": "pl", "type": "pressure", "trigger": "npc", "pos": [0.5, 0.5],
             "size": [0.3, 0.3], "min_mass": 0.25}
    env = npc_env(tmp_path, behavior="seek_button", pos=[-0.2, 0.0], button="pl", mass=0.3,
                  extra=[plate])
    body_steps(env, 12.0)
    assert env.parts["pl"].on and env.parts["pl"].mass == pytest.approx(0.3, abs=0.01)


def test_npc_obs_item(asset_dir, tmp_path):
    env = npc_env(tmp_path, behavior="flee", pos=[-0.7, -1.0])
    v = npcs(env)
    assert v.shape == (6,) and v[0] == pytest.approx(0.3, abs=0.02) and v[4] == 0.0
    body_steps(env, 0.6)
    v = npcs(env)
    assert v[2] > 0.02 and v[4] == 1.0 and v[5] == 1.0


def test_speed_warning(asset_dir, tmp_path):
    fast = npc_env(tmp_path, name="fast", behavior="flee", pos=[0.0, 0.0], speed=0.16)
    res = check_parts(fast)
    assert res["ok"] and "speed 0.16" in res["warnings"][0]
    slow = npc_env(tmp_path, name="slow", behavior="flee", pos=[0.0, 0.0])
    assert "warnings" not in check_parts(slow)


@pytest.mark.parametrize("bot,demo,needle", [
    ({"behavior": "patrol", "path": [[0, 0], [0.2, 0]]}, ["herd bot b"], "cannot herd"),
    ({"behavior": "lure", "bait": "b"}, ["herd bot b"], "cannot herd"),     # bait cannot be grabbed
    ({"behavior": "flee"}, ["herd bot"], "herd <npc>"),
    ({"behavior": "flee"}, ["herd bot 1.0 x"], "two numbers"),
])
def test_bad_herd_rejected(tmp_path, bot, demo, needle):
    import yaml
    raw = {"name": "h", "size": [3.0, 3.0], "spawn": {"pos": [-1.0, -1.0]},
           "parts": [{"name": "bot", "type": "npc", "pos": [0.0, 0.0], **bot},
                     {"name": "b", "type": "button", "trigger": "npc", "pos": [0.5, 0.5]},
                     {"name": "goal", "type": "goal", "pos": [1.3, 1.3]}], "demo": demo}
    path = tmp_path / "h.yaml"
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(RoomError, match=needle):
        PuzzleEnv(room=path)


def test_npc_button_is_a_switch_for_the_planner(asset_dir):
    env = PuzzleEnv(room=HERD)
    assert env.parts["corner"].role == "switch"
    assert env.plan["ok"] and env.plan["order"] == ["corner", "door_a", "goal"]
    env.close()


@pytest.mark.parametrize("bot,sensor,role", [
    ({"behavior": "flee"}, {"type": "zone", "trigger": "npc"}, "switch"),
    ({"behavior": "flee"}, {"type": "zone", "objects": ["box"]}, "sensor"),   # robots never count
    ({"behavior": "lure", "bait": "box"}, {"type": "button", "trigger": "npc"}, "sensor"),   # not grabbable
    ({"behavior": "lure", "bait": "c"}, {"type": "button", "trigger": "npc"}, "switch"),
])
def test_which_npc_sensors_the_planner_presses(asset_dir, tmp_path, bot, sensor, role):
    box = {"name": "box", "type": "t_box", "pos": [1.0, -1.0]}
    cube = {"name": "c", "type": "cube", "pos": [1.0, -0.5]}
    env = npc_env(tmp_path, pos=[0.0, 0.0], extra=[box, cube, {"name": "z", "pos": [0.5, 0.5], **sensor}],
                  **bot)
    assert env.parts["z"].role == role


def seek_room(tmp_path, target, name):
    """Switch `a` sends a seek_button robot onto `plate`; the door behind it reads `plate`."""
    import yaml
    raw = {"name": name, "size": [2.6, 1.8], "wall": {"thickness": 0.06, "height": 0.25},
           "walls": [{"from": [0.4, -0.9], "to": [0.4, -0.28]}, {"from": [0.4, 0.28], "to": [0.4, 0.9]}],
           "spawn": {"pos": [-1.0, -0.5], "yaw": 0.0},
           "parts": [{"name": "a", "type": "button", "kind": "oneshot", "pos": [-0.4, -0.5]},
                     {"name": "plate", "pos": [-1.0, 0.6], **target},
                     {"name": "bot", "type": "npc", "behavior": "seek_button", "pos": [-0.3, 0.5],
                      "button": "plate", "in": ["a"], "mass": 0.3},
                     {"name": "door", "preset": "device/sliding_door", "pos": [0.4, 0.0], "in": ["plate"]},
                     {"name": "goal", "type": "goal", "pos": [1.0, 0.0], "radius": 0.15}]}
    path = tmp_path / f"{name}.yaml"
    path.write_text(yaml.safe_dump(raw))
    return PuzzleEnv(room=path)


@pytest.mark.parametrize("target", [
    {"type": "pressure", "trigger": "npc", "size": [0.3, 0.3], "min_mass": 0.25},
    {"type": "button", "trigger": "any"},          # the body could press it: still left to the robot
])
def test_seek_button_target_follows_the_robots_inputs(asset_dir, tmp_path, target):
    env = seek_room(tmp_path, target, "seek_" + target["type"])
    assert env.parts["plate"].role == "sensor"                       # the body is not sent onto it
    res = check_order(env)
    assert res["ok"] and res["order"] == ["a", "door", "goal"], res
    assert run_episode(env, 0, ScriptedSolver(env))["success"]
    env.close()


def test_herd_drives_the_robot_onto_the_button(asset_dir):
    env = PuzzleEnv(room=HERD)
    res = run_episode(env, 0, ScriptedSolver(env))
    assert res["success"] and res["demo_step"] == 1
    env.close()


@pytest.mark.slow
def test_herd_room_passes_check(asset_dir, tmp_path):
    res = check(HERD, snap_dir=tmp_path)
    assert res["ok"], res
    assert res["solver"]["successes"] >= 18


LURE_MAP = """\
########################
#...P............#.....#
#................#.....#
#................#.....#
#................#.....#
#......................#
#......................#
#......................#
#......................#
#................#.....#
#................#.....#
#................#.....#
#................#.....#
########################
"""


@pytest.mark.slow
def test_herd_leads_a_lure_robot_with_the_held_bait(asset_dir, tmp_path):
    """The bait waits on a stand (P); `grab bait`, then `herd` leads the robot onto the pad."""
    import yaml
    raw = {"name": "lure_herd", "cell": 0.1, "legend": {"P": {"type": "floor", "height": 0.18}},
           "map": LURE_MAP, "spawn": {"pos": [-0.3, 0.1], "yaw": 1.57},
           "parts": [{"name": "bait", "type": "cube", "pos": [-0.75, 0.55]},
                     {"name": "bot", "type": "npc", "behavior": "lure", "bait": "bait", "pos": [-0.6, -0.2],
                      "sense": 0.8, "keep": 0.25},
                     {"name": "pad", "type": "button", "kind": "oneshot", "trigger": "npc",
                      "pos": [-0.2, -0.2], "size": [0.2, 0.2]},
                     {"name": "door", "type": "door", "pos": [0.55, 0.0], "width": 0.4, "axis": "y",
                      "in": ["pad"]},
                     {"name": "goal", "type": "goal", "pos": [0.9, 0.0], "radius": 0.12}],
           "demo": ["grab bait", "herd bot pad"]}
    path = tmp_path / "lure_herd.yaml"
    path.write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(room=path)
    assert env.parts["pad"].role == "switch"
    for seed in range(3):
        res = run_episode(env, seed, ScriptedSolver(env))
        assert res["success"] and res["demo_step"] == 2, res
    env.close()
