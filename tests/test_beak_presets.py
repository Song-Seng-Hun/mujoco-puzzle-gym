"""R28 beak gimmick presets: the objects (cube_weight, laundry, key, handle_ball,
egg, stick), receivers (hook, basket, scale, wall_button_beak low / mid) (E3b-1),
and the devices lever and pull_cord (E3b-2), with the `look` sub-goal (R27).

Fast: egg breaking, key / socket marker match, stick press window, basket count
and mass, scale mass, hook hang and take-off, the `beak` obs kind, `demo:`
parsing, `mpg check` stage `actions` for the two wall buttons; lever cells,
detents and snapshots, the turret ballistic check per lever cell, the pull
cord's flexcomp, latch and snapshots, a held grab lever knob, `turn` / `pull`
/ `look` parsing, `actions` for `pull`, a `.seen` door room's order.
Slow: per preset, the scripted solution (`demo:`: grab, place, hang, drop,
press, turn, pull, look) turns its signal on.
"""

import math
import re

import mujoco
import numpy as np
import pytest
import yaml
from rooms import body_steps, tick
from test_grab import grab, make, tip, to_beak  # noqa: F401  (make: fixture)

from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.parts.beak.objects import BREAK_HEIGHT
from mujoco_puzzle_gym.parts.input.button import BEHIND, PRESS_DEPTH, THICK
from mujoco_puzzle_gym.room import RoomError
from mujoco_puzzle_gym.solver import ScriptedSolver


# ---- egg ---------------------------------------------------------------------------------
def drop_egg(env, height):
    e = env.parts["e"]
    e.put(env.data, [0.8, 0.8, e.height / 2 + height])
    mujoco.mj_forward(env.model, env.data)
    body_steps(env, 0.8)


@pytest.mark.parametrize("height, broken", [(0.02, 0), (0.035, 0), (0.08, 1), (0.15, 1)])
def test_egg_breaks_above_its_drop_height(make, height, broken):
    env = make([{"name": "e", "preset": "beak/egg", "pos": [0.8, 0.8], "on_break": "none"}])
    assert env.parts["e"].cfg["break_height"] == BREAK_HEIGHT == 0.05
    drop_egg(env, height)
    assert env.signals.get("e.broken") == broken
    assert not env.parts["e"].failed()


def test_broken_egg_fails_with_on_break_fail_and_reset_mends_it(make):
    env = make([{"name": "e", "preset": "beak/egg", "pos": [0.8, 0.8]}])        # preset: on_break fail
    drop_egg(env, 0.15)
    assert env.signals.get("e.broken") == 1 and env.parts["e"].failed()
    body_steps(env, 0.5)
    assert env.signals.get("e.broken") == 1                       # stays broken
    env.reset(seed=0)
    assert env.signals.get("e.broken") == 0 and not env.parts["e"].failed()


def test_egg_survives_being_carried(make):
    env = make([{"name": "e", "preset": "beak/egg", "pos": [0.8, 0.8]}])
    to_beak(env, "e")
    grab(env)
    assert env.signals.get("beak.holding") == 1
    body_steps(env, 3.0, cmd=np.array([0.3, 0.0, 0.0]))
    assert env.signals.get("e.broken") == 0 and env.signals.get("beak.holding") == 1
    grab(env, on=False, seconds=1.0)                              # let go onto the floor: breaks
    assert env.signals.get("e.broken") == 1


def test_restoring_a_resting_egg_while_it_falls_does_not_break_it(make):
    env = make([{"name": "e", "preset": "beak/egg", "pos": [0.8, 0.8]}])     # preset: on_break fail
    body_steps(env, 0.3)
    snap = env.save_snapshot()                                     # resting on the floor
    e = env.parts["e"]
    e.put(env.data, [0.8, 0.8, e.height / 2 + 0.3])
    mujoco.mj_forward(env.model, env.data)
    body_steps(env, 0.2)                                           # falling, not yet landed
    assert env.data.qvel[e._v + 2] < -1.0 and env.signals.get("e.broken") == 0
    env.restore_snapshot(snap)
    body_steps(env, 0.5)
    assert env.signals.get("e.broken") == 0 and not e.failed()


# ---- key and socket ----------------------------------------------------------------------
@pytest.mark.parametrize("key_id, ok", [(7, 1), (8, 0)])
def test_key_turns_on_only_the_socket_with_its_number(make, key_id, ok):
    env = make([{"name": "k", "preset": "beak/key", "pos": [0.8, 0.8], "marker": {"id": key_id}},
                {"name": "s", "preset": "input/socket", "pos": [0.5, -0.5], "accept": [7],
                 "marker": {"id": 7}, "angle_tolerance": 45.0}])
    assert env.parts["s"].marker.id == 7                         # the socket shows the number it takes
    env.parts["k"].put(env.data, [0.5, -0.5, 0.02])
    mujoco.mj_forward(env.model, env.data)
    body_steps(env, 0.6)
    assert env.signals.get("s.ok") == ok and env.signals.get("s.wrong") == 1 - ok


def test_two_unrelated_parts_still_cannot_share_a_marker(tmp_path, asset_dir):
    from rooms import open_room

    with pytest.raises(RoomError, match="marker id 7"):
        open_room(tmp_path, [{"name": "k", "preset": "beak/key", "pos": [0.8, 0.8], "marker": {"id": 7}},
                             {"name": "s", "preset": "input/socket", "pos": [0.5, -0.5], "accept": [8],
                              "marker": {"id": 7}}])


@pytest.mark.parametrize("second", [
    {"name": "d", "preset": "device/sliding_door", "pos": [0.0, 0.9], "marker": {"id": 7}},
    {"name": "k2", "preset": "beak/key", "pos": [-0.8, 0.8], "marker": {"id": 7}},
])
def test_a_socket_shares_its_number_with_one_object_only(tmp_path, asset_dir, second):
    from rooms import open_room

    with pytest.raises(RoomError, match="marker id 7"):
        open_room(tmp_path, [{"name": "s", "preset": "input/socket", "pos": [0.5, -0.5], "accept": [7],
                              "marker": {"id": 7}},
                             {"name": "k", "preset": "beak/key", "pos": [0.8, 0.8], "marker": {"id": 7}},
                             second])


# ---- stick -------------------------------------------------------------------------------
def held_stick(make):
    env = make([{"name": "s", "preset": "beak/stick", "pos": [0.8, 0.8]},
                {"name": "b", "preset": "beak/wall_button_beak_mid", "pos": [1.2, -1.2], "facing": 180,
                 "kind": "momentary"}])
    to_beak(env, "s")
    grab(env)
    assert env.signals.get("beak.holding") == 1
    return env


def put_plate(env, gap, dz=0.0):
    """Put wall button `b` (facing -x) so the stick's far end is `gap` m in front of its
    face (negative: behind it) and `dz` above its centre, then one bus step."""
    b = env.parts["b"]
    end = env.parts["s"].tip_point(env.data)
    b.place(env.model, end[:2] - b.normal * (gap + THICK))
    b.cfg["height"] = float(end[2] - dz)
    tick(env)
    return env.signals.get("b")


def test_stick_end_presses_a_wall_button_in_the_r4_window(make):
    env = held_stick(make)
    end = env.world.tool_tip
    assert np.linalg.norm(end - tip(env)) > 0.2                 # the end is well ahead of the beak
    for gap in (0.0, 0.03, PRESS_DEPTH - 0.005, -(THICK + BEHIND) + 0.003):
        assert put_plate(env, gap) == 1, gap
    assert put_plate(env, PRESS_DEPTH + 0.01) == 0               # too far in front
    assert put_plate(env, -(THICK + BEHIND) - 0.005) == 0        # behind the wall
    assert put_plate(env, 0.02, dz=0.04) == 0                    # above the plate (half height 3 cm)


def test_a_stick_not_held_presses_nothing(make):
    env = held_stick(make)
    assert put_plate(env, 0.02) == 1
    env.world.grab = -1.0
    tick(env)
    assert env.world.tool_tip is None and env.signals.get("b") == 0


# ---- basket, scale, hook -----------------------------------------------------------------
def test_basket_counts_and_weighs(make):
    env = make([{"name": "l1", "preset": "beak/laundry", "pos": [0.8, 0.8]},
                {"name": "l2", "preset": "beak/laundry", "pos": [0.8, 0.5], "mass": 0.03},
                {"name": "bk", "preset": "beak/basket", "pos": [-0.5, -0.5], "count": 2}])
    for n, dy in (("l1", -0.05), ("l2", 0.05)):
        env.parts[n].put(env.data, [-0.5, -0.5 + dy, 0.03 + dy])
    mujoco.mj_forward(env.model, env.data)
    body_steps(env, 0.3)
    env.parts["l1"].put(env.data, [-0.5, -0.55, 0.02])          # still falling? put them in again
    body_steps(env, 0.6)
    assert env.signals.get("bk.count") == 2 and env.signals.get("bk.ok") == 1
    assert env.signals.get("bk.mass") == pytest.approx(0.05, abs=1e-6)


def test_scale_gives_the_mass_as_a_number(make):
    env = make([{"name": "c", "preset": "beak/cube_weight", "pos": [0.8, 0.8]},
                {"name": "sc", "preset": "beak/scale", "pos": [0.0, -0.6]}])
    body_steps(env, 0.2)
    assert env.signals.get("sc") == 0 and env.signals.get("sc.mass") == 0.0
    env.parts["c"].put(env.data, [0.0, -0.6, 0.03])
    mujoco.mj_forward(env.model, env.data)
    body_steps(env, 0.6)
    assert env.signals.get("sc") == 1
    assert env.signals.get("sc.mass") == pytest.approx(0.04, abs=0.002)


def test_hook_hangs_a_let_go_object_and_gives_it_back(make):
    env = make([{"name": "l", "preset": "beak/laundry", "pos": [0.8, 0.8]},
                {"name": "h", "preset": "beak/hook", "pos": [-0.5, -0.5], "facing": 0}])
    h, lnd = env.parts["h"], env.parts["l"]
    peg = h.peg_tip()
    lnd.put(env.data, peg - lnd._local[lnd.handles.index("-x")] + [0.01, 0.0, 0.0])   # -x hem at the tip
    mujoco.mj_forward(env.model, env.data)
    z0 = lnd.pos(env.data)[2]
    body_steps(env, 1.0)
    assert env.signals.get("h.ok") == 1 and h.hung == ["l"]
    assert lnd.pos(env.data)[2] == pytest.approx(z0, abs=0.01)  # hangs: does not fall
    snap = env.save_snapshot()
    env.restore_snapshot(snap)
    body_steps(env, 0.5)
    assert env.signals.get("h.ok") == 1
    hem = lnd.handle_points(env.data)[lnd.handles.index("+x")]
    env.body.teleport(env.data, (hem[0] + 0.15, hem[1], math.pi))   # face the +x hem ...
    env.body.set_head(0.4)
    body_steps(env, 1.0)
    env.body.shift(env.data, [*(hem[:2] - tip(env)[:2]), 0.0])      # ... beak tip at it
    mujoco.mj_forward(env.model, env.data)
    assert np.linalg.norm(tip(env) - hem) < 0.03
    grab(env, seconds=0.2)                                       # the beak takes it off the hook
    assert env.signals.get("beak.holding") == 1 and h.hung == [] and env.signals.get("h.ok") == 0


def test_beak_obs_numbers_the_new_kinds(make):
    from mujoco_puzzle_gym.obs import GRAB_KINDS, beak

    env = make([{"name": "c", "type": "cube", "pos": [0.8, 0.8]},
                {"name": "l", "preset": "beak/laundry", "pos": [0.8, -0.8]}])
    assert GRAB_KINDS[0] == "cube" and beak(env).shape == (5,)
    to_beak(env, "l")
    grab(env)
    assert beak(env)[:2].tolist() == [1.0, float(GRAB_KINDS.index("laundry") + 1)]


def test_demo_grabs_any_object_and_drops(asset_dir, tmp_path):
    from mujoco_puzzle_gym.room import parse_demo

    parts = {"e": object(), "z": object()}
    parts["e"] = type("E", (), {"handle_points": None})()
    assert parse_demo(["grab e", "drop e z", "place e z"], parts) == [("grab", "e"), ("drop", "e", "z"),
                                                                      ("place", "e", "z")]
    with pytest.raises(RoomError, match="not a grabbable object"):
        parse_demo(["grab z"], parts)
    with pytest.raises(RoomError, match="drop <object> <part>"):
        parse_demo(["drop e"], parts)


# ---- `mpg check` stage `actions` for the two beak wall buttons ----------------------------
WALL_ROOM = """
####################
#..................#
#..................#
#..................#
#..................#
#..................#
#..................#
#..................#
#..................#
#..................#
####################
"""


@pytest.mark.parametrize("which, actions, ok", [("low", ["move"], False), ("low", ["move", "head"], True),
                                                ("mid", ["move"], True)])
def test_check_asks_for_head_only_for_the_low_button(asset_dir, tmp_path, which, actions, ok):
    from mujoco_puzzle_gym.check import check

    path = beak_room(tmp_path, [{"name": "btn", "preset": f"beak/wall_button_beak_{which}",
                                 "pos": [0.899, 0.0], "facing": 180}], ["press btn"], stand=False)
    res = check(path, task={"actions": actions}, static_only=True)
    assert res["actions"]["ok"] is ok and res["ok"] is ok, res


# ---- slow: each preset's scripted solution ------------------------------------------------
STAND_ROOM = """
####################
#..................#
#..................#
#....PP............#
#..................#
#..................#
#..................#
#..................#
#..................#
#..................#
#..................#
####################
"""


def beak_room(tmp_path, parts, demo, stand=True, name="beak"):
    """A 1.8 x 1.0 m letter-map room; with `stand` a 0.18 m stand (x -0.5..-0.3, y 0.2..0.3)."""
    raw = {"name": name, "cell": 0.1, "map": STAND_ROOM if stand else WALL_ROOM,
           "spawn": {"pos": [-0.75, -0.25], "yaw": 1.57, "random": [0.05, 0.05, 0.3]},
           "parts": [*parts, {"name": "goal", "type": "goal", "pos": [0.8, -0.35], "radius": 0.05}],
           "demo": demo}
    if stand:
        raw["legend"] = {"P": {"type": "floor", "height": 0.18}}
    else:
        raw["spawn"] = {"pos": [-0.5, -0.1], "yaw": 0.0, "random": [0.1, 0.1, 0.5]}
    path = tmp_path / f"{name}.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


def solve(path, signal, seed, seconds=40.0, every=None):
    """Run the scripted solver; returns (env, the time the signal first turned on)."""
    env = PuzzleEnv(task={"actions": ["move", "head", "grab"]}, room=path)
    sol = ScriptedSolver(env)
    env.reset(seed=seed)
    sol.reset()
    on = None
    for i in range(int(seconds * env.hz)):
        _, _, term, trunc, _ = env.step(sol.act())
        if every is not None:
            every(env)
        if on is None and env.signals.get(signal) > 0.5:
            on = i / env.hz
        if term or trunc or (on is not None and not sol.in_demo):
            break
    return env, on, sol


CASES = {   # preset under test: (parts, demo, signal)
    "cube_weight+scale": ([{"name": "obj", "preset": "beak/cube_weight", "pos": [-0.45, 0.22]},
                           {"name": "sc", "preset": "beak/scale", "pos": [0.2, -0.2]}],
                          ["grab obj", "place obj sc"], "sc"),
    "laundry+basket": ([{"name": "obj", "preset": "beak/laundry", "pos": [-0.44, 0.25]},
                        {"name": "bk", "preset": "beak/basket", "pos": [0.35, -0.1]}],
                       ["grab obj", "place obj bk"], "bk.ok"),
    "key+socket": ([{"name": "obj", "preset": "beak/key", "pos": [-0.48, 0.25], "marker": {"id": 7}},
                    {"name": "s", "preset": "input/socket", "pos": [0.2, -0.2], "accept": [7],
                     "marker": {"id": 7}, "angle_tolerance": 45.0}],
                   ["grab obj", "place obj s"], "s.ok"),
    "handle_ball+basket": ([{"name": "obj", "preset": "beak/handle_ball", "pos": [-0.47, 0.25]},
                            {"name": "bk", "preset": "beak/basket", "pos": [0.35, -0.1]}],
                           ["grab obj", "place obj bk"], "bk.ok"),
    "egg": ([{"name": "obj", "preset": "beak/egg", "pos": [-0.45, 0.22], "on_break": "none"},
             {"name": "spot", "type": "zone", "pos": [0.2, -0.2], "size": [0.2, 0.2]}],
            ["grab obj", "drop obj spot"], "obj.broken"),
    "laundry+hook": ([{"name": "obj", "preset": "beak/laundry", "pos": [-0.44, 0.25]},
                      {"name": "h", "preset": "beak/hook", "pos": [0.5, -0.1], "facing": 180}],
                     ["grab obj", "place obj h"], "h.ok"),
    "stick+wall_button": ([{"name": "obj", "preset": "beak/stick", "pos": [-0.4, 0.25]},
                           {"name": "btn", "preset": "beak/wall_button_beak_mid", "pos": [0.899, -0.1],
                            "facing": 180}],
                          ["grab obj", "press btn"], "btn"),
}


SEEDS, NEED = 5, 4         # like `mpg check` (18 of 20): most seeds, not every one


def run_case(tmp_path, case, seed) -> str | None:
    """None when the scripted solution turned the preset on (and, but for the egg, kept it
    on after walking off, without a fall); else what went wrong."""
    parts, demo, signal = CASES[case]
    path = beak_room(tmp_path, parts, demo, name=f"{case.replace('+', '_')}_{seed}")
    beak_off = []

    def watch(env):            # stick: how far the beak tip is in front of the plate when it turns on
        b = env.parts.get("btn")
        if b is not None and not beak_off and env.signals.get("btn") > 0.5:
            beak_off.append(float((env.data.site_xpos[env.world.beak_site][:2] - b.xy) @ b.normal))

    env, on, sol = solve(path, signal, seed, every=watch)
    if on is None:
        return f"{signal} never on (demo step {sol.step_i})"
    if case == "egg":
        return None
    if sol.in_demo or env.body_state.fallen or env.signals.get(signal) != 1:
        return f"on at {on:.1f} s, then: demo step {sol.step_i}, fell {env.body_state.fallen}"
    if case == "cube_weight+scale":       # the 40 g cube; less when it rests partly off the plate
        assert 0.02 < env.signals.get("sc.mass") <= 0.042
    if case == "laundry+basket":
        assert env.signals.get("bk.count") == 1 and env.signals.get("bk.mass") == pytest.approx(0.02)
    if case == "stick+wall_button":       # the stick's end pressed it, the beak was far off
        assert beak_off[0] > PRESS_DEPTH + 0.1
    return None


@pytest.mark.slow
@pytest.mark.parametrize("case", list(CASES))
def test_scripted_solution_turns_the_preset_on(asset_dir, tmp_path, case):
    why = {seed: run_case(tmp_path, case, seed) for seed in range(SEEDS)}
    bad = {k: v for k, v in why.items() if v is not None}
    assert SEEDS - len(bad) >= NEED, f"{case}: {bad}"


@pytest.mark.slow
@pytest.mark.parametrize("seed", [0, 1, 2])
@pytest.mark.parametrize("which", ["low", "mid"])
def test_beak_wall_buttons_are_pressed(asset_dir, tmp_path, which, seed):
    path = beak_room(tmp_path, [{"name": "btn", "preset": f"beak/wall_button_beak_{which}",
                                 "pos": [0.899, 0.0], "facing": 180}], ["press btn"], stand=False)
    heads = []
    env, on, sol = solve(path, "btn", seed, every=lambda e: heads.append(float(e.body.head[0])))
    assert on is not None and not env.body_state.fallen
    if which == "low":
        assert min(heads) < -0.3                                  # the head went down (neck pitch)


# ---- lever (E3b-2) -----------------------------------------------------------------------
LEVER = {"name": "lv", "preset": "beak/lever", "pos": [0.6, 0.6], "facing": 180}


def set_lever(env, deg):
    lv = env.parts["lv"]
    env.data.qpos[lv._qa] = math.radians(deg)
    env.data.qvel[lv._va] = 0.0
    mujoco.mj_forward(env.model, env.data)


def test_lever_cells_and_detents(make):
    env = make([LEVER, {**LEVER, "name": "lc", "pos": [-0.6, 0.6], "detents": 0}])
    lv, lc = env.parts["lv"], env.parts["lc"]
    assert lv.mode == "push" and lv.channel_cells("lv.angle").tolist() == [-1.0, 0.0, 1.0]
    assert lc.channel_cells("lc.angle") is None and lv.channel_cells("other") is None
    assert env.signals.get("lv.angle") == -1.0                    # push levers start leaning at the body
    set_lever(env, -12.0)                                         # push: value = -q / 35 deg, nearest cell 0
    body_steps(env, 1.0)
    assert env.signals.get("lv.angle") == 0.0
    assert abs(math.degrees(env.data.qpos[lv._qa])) < 1.0         # the detent pulled the arm to the cell
    env.data.qpos[lc._qa] = math.radians(-7.0)
    body_steps(env, 0.5)
    assert env.signals.get("lc.angle") == pytest.approx(7.0 / 35.0, abs=0.02)   # continuous: stays


def test_lever_detents_must_be_zero_or_two_and_up(tmp_path, asset_dir):
    from rooms import open_room

    with pytest.raises(RoomError, match="detents must be 0"):
        open_room(tmp_path, [{**LEVER, "detents": 1}])


def test_lever_snapshot_and_reset(make):
    env = make([LEVER])
    set_lever(env, -35.0)
    body_steps(env, 0.3)
    assert env.signals.get("lv.angle") == 1.0
    snap = env.save_snapshot()
    env.reset(seed=0)
    assert env.signals.get("lv.angle") == -1.0 and env.parts["lv"].value == -1.0
    env.restore_snapshot(snap)
    assert env.signals.get("lv.angle") == 1.0                     # no step needed, no other cell on the way
    body_steps(env, 0.5)
    assert env.signals.get("lv.angle") == 1.0


def test_the_beak_holds_a_grab_levers_knob(make):
    env = make([{**LEVER, "mode": "grab", "pos": [0.5, 0.0]}])
    lv = env.parts["lv"]
    assert lv.max == pytest.approx(math.radians(10.0)) and env.signals.get("lv.angle") == 0.0
    knob = lv.handle_points(env.data)[0]
    env.body.teleport(env.data, (knob[0] - 0.085, knob[1], 0.0))
    body_steps(env, 1.0)
    env.body.shift(env.data, [*(knob[:2] - tip(env)[:2]), 0.0])
    mujoco.mj_forward(env.model, env.data)
    grab(env, seconds=0.2)
    assert env.signals.get("beak.holding") == 1 and env.signals.get("lv.held") == 1
    grab(env, on=False)
    assert env.signals.get("beak.holding") == 0


TURRET = [{"name": "fire", "preset": "input/floor_momentary", "pos": [-1.0, 0.25]},
          {"name": "tur", "preset": "trap/cannon_turret", "pos": [-1.2, 0.75], "in": ["fire"], "aim_pitch": 17.0,
           "bind": {"aim_yaw": {"from": "lv.angle", "map": [-1, 1, -30, 30]}}}]


@pytest.mark.parametrize("aim, detents, ok", [(0.0, 3, True), (15.0, 3, False), (15.0, 0, True)])
def test_turret_check_aims_per_lever_cell(asset_dir, tmp_path, aim, detents, ok):
    """R16 ballistic check with a lever: a target 1.35 m off in the `aim` direction is hit by
    a lever cell (-30, 0, 30 deg) only straight ahead; a continuous lever reaches 15 deg."""
    from mujoco_puzzle_gym.check import check

    a = math.radians(aim)
    xy = [-1.2 + 1.35 * math.cos(a), 0.75 + 1.35 * math.sin(a)]
    raw = {"name": f"tur_{detents}_{aim:g}", "size": [3.0, 2.0], "spawn": {"pos": [-1.2, -0.6], "yaw": 0.0},
           "parts": [{**LEVER, "pos": [-0.5, -0.45], "detents": detents}, *TURRET,
                     {"name": "t", "preset": "input/target", "pos": xy, "yaw": aim - 180.0},
                     {"name": "goal", "type": "goal", "pos": [1.1, 0.0]}]}
    path = tmp_path / f"{raw['name']}.yaml"
    path.write_text(yaml.safe_dump(raw))
    res = check(path, static_only=True)
    assert res["parts"]["ok"] is ok, res
    if not ok:
        assert "aim_yaw -30..30" in res["parts"]["reason"]


# ---- pull cord (E3b-2) -------------------------------------------------------------------
CORD = {"name": "pc", "preset": "beak/pull_cord", "pos": [0.6, 0.6], "facing": 180}


def test_pull_cord_is_a_flexcomp_hanging_still(make):
    from mujoco_puzzle_gym.parts.beak.devices import CORD_NODES

    env = make([CORD])
    m = env.model
    assert m.nflex == 1 and m.flex_dim[0] == 1 and m.flex_vertnum[0] == CORD_NODES
    top = m.body("part/pc/cord_0").id
    assert m.body_jntnum[top] == 0                                  # pinned to the slider
    pc = env.parts["pc"]
    body_steps(env, 1.0)
    knob = pc.handle_points(env.data)[0]
    assert knob[:2] == pytest.approx([0.6, 0.6], abs=0.005) and knob[2] == pytest.approx(0.19, abs=0.01)
    assert abs(pc.drawn(env.data)) < 0.002 and env.signals.get("pc") == 0


def test_pull_cord_turns_on_latches_and_resets(make):
    env = make([CORD])
    pc = env.parts["pc"]
    env.data.qpos[pc._qa] = 0.065                                  # 6.5 cm drawn out
    tick(env)
    assert env.signals.get("pc") == 1
    body_steps(env, 1.0)                                           # the spring draws it back in
    assert pc.drawn(env.data) < 0.01 and env.signals.get("pc") == 1   # latched
    snap = env.save_snapshot()
    env.reset(seed=0)
    assert env.signals.get("pc") == 0
    env.restore_snapshot(snap)
    assert env.signals.get("pc") == 1


# ---- `demo:` turn / pull / look parsing, `actions` ----------------------------------------
@pytest.mark.parametrize("entry, needle", [
    ("turn lv 0.5", "one of its cells [-1.0, 0.0, 1.0]"),
    ("turn lv", "turn <lever> <value>"),
    ("turn lg 1", "levers with mode: push"),
    ("turn pc 1", "levers with mode: push"),
    ("pull lv", "pull a pull_cord"),
    ("look lv", "no part reads 'lv.seen'"),
])
def test_demo_turn_pull_look_errors(tmp_path, asset_dir, entry, needle):
    with pytest.raises(RoomError, match=re.escape(needle)):
        _demo_room(tmp_path, [entry])


def _demo_room(tmp_path, demo):
    raw = {"name": "dm", "size": [3.0, 3.0], "spawn": {"pos": [-1.0, -1.0], "yaw": 0.0},
           "parts": [LEVER, {**LEVER, "name": "lg", "pos": [-0.6, 0.6], "mode": "grab"},
                     {**CORD, "pos": [0.6, -0.6]}, {"name": "goal", "type": "goal", "pos": [1.3, 1.3]}],
           "demo": demo}
    path = tmp_path / "dm.yaml"
    path.write_text(yaml.safe_dump(raw))
    return PuzzleEnv(room=path)


def test_demo_parses_turn_pull_look(tmp_path, asset_dir):
    env = _demo_room(tmp_path, ["turn lv 0", "pull pc", "turn lv -1"])
    assert env.room.demo == [("turn", "lv", 0.0), ("pull", "pc"), ("turn", "lv", -1.0)]


@pytest.mark.parametrize("actions, ok", [(["move"], False), (["move", "grab"], True)])
def test_check_asks_for_grab_to_pull(asset_dir, tmp_path, actions, ok):
    from mujoco_puzzle_gym.check import check

    path = beak_room(tmp_path, [{**CORD, "pos": [0.3, 0.0]}], ["pull pc"], stand=False, name="pull")
    res = check(path, task={"actions": actions}, static_only=True)
    assert res["actions"]["ok"] is ok, res["actions"]


# ---- look (R27): a door that opens when looked at ----------------------------------------
def look_room(tmp_path, demo=None):
    raw = {"name": "look_door", "size": [2.0, 1.4], "wall": {"thickness": 0.06, "height": 0.25},
           "walls": [{"from": [0.0, -0.7], "to": [0.0, -0.28]}, {"from": [0.0, 0.28], "to": [0.0, 0.7]}],
           "spawn": {"pos": [-0.7, -0.3], "yaw": 0.0, "random": [0.08, 0.1, 1.0]},
           "parts": [{"name": "door", "preset": "device/sliding_door", "pos": [0.0, 0.0], "in": ["door.seen"],
                      "mode": "oneshot"},
                     {"name": "goal", "type": "goal", "pos": [0.7, 0.0], "radius": 0.15}]}
    if demo:
        raw["demo"] = demo
    path = tmp_path / "look_door.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


def test_a_seen_door_room_has_a_solution_order(asset_dir, tmp_path):
    from mujoco_puzzle_gym.check import check

    res = check(look_room(tmp_path, ["look door"]), static_only=True)
    assert res["ok"], res
    assert res["order"]["order"] == ["_look_door", "door", "goal"]


# ---- slow: device sub-goals and look -----------------------------------------------------
DEVICE_CASES = {    # (parts, demo, signal, value it must end at)
    "lever to 1": ([{**LEVER, "pos": [0.3, 0.0]}], ["turn lv 1"], "lv.angle", 1.0),
    "lever to 0": ([{**LEVER, "pos": [0.3, 0.0]}], ["turn lv 0", "wait 2"], "lv.angle", 0.0),
    "lever 1 then back to -1": ([{**LEVER, "pos": [0.1, 0.0]}], ["turn lv 1", "turn lv -1", "wait 2"],
                                "lv.angle", -1.0),
    "pull cord": ([{**CORD, "pos": [0.3, 0.0]}], ["pull pc"], "pc", 1.0),
}


@pytest.mark.slow
@pytest.mark.parametrize("case", list(DEVICE_CASES))
def test_scripted_solution_works_the_device(asset_dir, tmp_path, case):
    parts, demo, signal, value = DEVICE_CASES[case]
    bad = {}
    for seed in range(SEEDS):
        path = beak_room(tmp_path, parts, demo, stand=False, name=f"dev_{seed}")
        env = PuzzleEnv(task={"actions": ["move", "head", "grab"]}, room=path)
        sol = ScriptedSolver(env)
        env.reset(seed=seed)
        sol.reset()
        for _ in range(int(40 * env.hz)):
            env.step(sol.act())
            if not sol.in_demo or env.body_state.fallen:
                break
        if sol.in_demo or env.body_state.fallen or env.signals.get(signal) != value:
            bad[seed] = (sol.step_i, env.body_state.fallen, env.signals.get(signal))
        env.close()
    assert SEEDS - len(bad) >= NEED, f"{case}: {bad}"


@pytest.mark.slow
@pytest.mark.parametrize("demo", [None, ["look door"]])
def test_look_opens_a_seen_door(asset_dir, tmp_path, demo):
    from mujoco_puzzle_gym.check import check

    res = check(look_room(tmp_path, demo), seeds=SEEDS, need=NEED, snap_dir=tmp_path)
    assert res["ok"], res
