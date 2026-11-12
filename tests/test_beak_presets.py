"""R28 beak gimmick presets: the objects (cube_weight, laundry, key, handle_ball,
egg, stick), receivers (hook, basket, scale, wall_button_beak low / mid) (E3b-1),
and the devices lever, dial, pull_cord and drawer (E3b-2), with the `look` sub-goal (R27),
and the dispenser (E3b-3).

Fast: egg breaking, key / socket marker match, stick press window, basket count
and mass, scale mass, hook hang and take-off, the `beak` obs kind, `demo:`
parsing, `mpg check` stage `actions` for the two wall buttons; lever cells,
detents and snapshots, the turret ballistic check per lever cell, the pull
cord's flexcomp, latch and snapshots, a held grab lever knob, `turn` / `pull`
/ `look` parsing, `actions` for `pull`, a `.seen` door room's order and
target point, the turret target point and aim lever role, warping with a held knob;
dial cells, target, snapshots and aim role, drawer latch, strap off, contents depth,
`actions` for a drawer; dispenser rising edges, stock, cooldown, max_active, consume /
recycle / refill, fallen copies, shared marker, snapshots, parked copies out of obs and
reach, npc copies, config errors, `take` parsing, `actions` for a dispenser room.
Slow: per preset, the scripted solution (`demo:`: grab, place, hang, drop,
press, turn, pull, look) turns its signal on; the grid answer pulls a cord and lets go,
turns a dial and opens a drawer; `turn` sets a dial cell; a cube is grabbed from an opened drawer;
both dispenser presets: `take` / `grab` / `place` into a consuming basket, and the grid answer.
"""

import math
import re

import mujoco
import numpy as np
import pytest
import yaml
from rooms import body_steps, open_room, stand_at, tick
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


def turret_env(asset_dir):
    from pathlib import Path

    return PuzzleEnv(room=Path(__file__).parents[1] / "examples/rooms/turret.yaml")


def test_turret_target_points_at_the_fire_button_and_aim_is_no_switch(asset_dir):
    """The target board's target area is the fire button, so `target_pos` points there; the
    aim lever only feeds the turret's `bind`, so it is no switch and pays no `new_on`."""
    from mujoco_puzzle_gym import obs

    env = turret_env(asset_dir)
    env.reset(seed=0)
    t, fire, aim = env.current_target(), env.parts["fire"], env.parts["aim"]
    assert t.name == "board" and t.target_xy().tolist() == fire.xy.tolist()
    s = env.body_state
    d = fire.xy - s.pos[:2]
    want = [math.cos(s.yaw) * d[0] + math.sin(s.yaw) * d[1], -math.sin(s.yaw) * d[0] + math.cos(s.yaw) * d[1]]
    assert obs.target_pos(env) == pytest.approx(want)
    assert aim.role == "prop" and "aim" not in env.plan["order"]
    env.data.qpos[aim._qa] = -aim.max                              # push lever to 1 (value = -q / max)
    mujoco.mj_forward(env.model, env.data)
    env.step(np.zeros(env.action_space.shape, dtype=np.float32))
    assert env.signals.get("aim.angle") == 1.0 and env.events["new_on"] == 0


def test_a_lever_read_by_a_door_stays_a_switch(make):
    env = make([LEVER, {"name": "d", "preset": "device/sliding_door", "pos": [-0.6, -0.6], "in": ["lv.angle"]}])
    assert env.parts["lv"].role == "switch"


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


def test_warping_the_body_lets_a_held_cord_knob_go(make):
    from mujoco_puzzle_gym.grab import BEAK_PART

    env = make([CORD])
    beak, pc = env.parts[BEAK_PART], env.parts["pc"]
    beak._grab(env.model, env.data, beak.objects.index(pc))
    knob = pc.handle_points(env.data)[0].copy()
    env.warp("body", (-0.5, -0.5, 0.0))
    assert beak.held is None and env.signals.get("beak.holding") == 0 and env.signals.get("pc.held") == 0
    assert np.hypot(*(env.read_body().pos[:2] - [-0.5, -0.5])) < 0.05
    assert pc.handle_points(env.data)[0] == pytest.approx(knob, abs=1e-6)   # the cord stays put


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
def look_room(tmp_path, demo=None, door=0.0):
    raw = {"name": "look_door", "size": [2.0 + 2 * door, 1.4], "wall": {"thickness": 0.06, "height": 0.25},
           "walls": [{"from": [door, -0.7], "to": [door, -0.28]}, {"from": [door, 0.28], "to": [door, 0.7]}],
           "spawn": {"pos": [-0.7, -0.3], "yaw": 0.0, "random": [0.08, 0.1, 1.0]},
           "parts": [{"name": "door", "preset": "device/sliding_door", "pos": [door, 0.0], "in": ["door.seen"],
                      "mode": "oneshot"},
                     {"name": "goal", "type": "goal", "pos": [door + 0.7, 0.0], "radius": 0.15}]}
    if demo:
        raw["demo"] = demo
    path = tmp_path / "look_door.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


def test_look_target_is_the_watched_part_and_hidden_from_part_obs(asset_dir, tmp_path):
    from mujoco_puzzle_gym import obs

    env = PuzzleEnv(room=look_room(tmp_path, door=0.6))
    env.reset(seed=0)
    t = env.current_target()
    assert t.name == "_look_door" and t.xy.tolist() == [0.6, 0.0]
    s = env.body_state
    d = np.array([0.6, 0.0]) - s.pos[:2]
    want = [math.cos(s.yaw) * d[0] + math.sin(s.yaw) * d[1], -math.sin(s.yaw) * d[0] + math.cos(s.yaw) * d[1]]
    assert obs.target_pos(env) == pytest.approx(want)
    assert len(obs.part_pos(env)) == 2 * 3 and len(obs.part_state(env)) == 3   # door, goal, _gaze


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


@pytest.mark.slow
def test_grid_answer_pulls_the_cord_and_lets_go(asset_dir, tmp_path):
    """No `demo:`: the grid answer pulls the cord (next in the order), lets the knob go once
    it is on, and walks to the goal without a drop."""
    raw = {"name": "pull_door", "size": [2.4, 1.4], "wall": {"thickness": 0.06, "height": 0.25},
           "walls": [{"from": [0.3, -0.7], "to": [0.3, -0.28]}, {"from": [0.3, 0.28], "to": [0.3, 0.7]}],
           "spawn": {"pos": [-0.8, -0.3], "yaw": 0.0, "random": [0.08, 0.1, 0.8]},
           "parts": [{**CORD, "pos": [-0.3, 0.45], "facing": -90},
                     {"name": "d", "preset": "device/sliding_door", "pos": [0.3, 0.0], "in": ["pc"]},
                     {"name": "goal", "type": "goal", "pos": [0.9, 0.0], "radius": 0.15}]}
    path = tmp_path / "pull_door.yaml"
    path.write_text(yaml.safe_dump(raw))
    good = {}
    for seed in range(SEEDS):
        env = PuzzleEnv(task={"actions": ["move", "head", "grab"]}, room=path)
        sol = ScriptedSolver(env)
        env.reset(seed=seed)
        sol.reset()
        on = let_go = None
        dropped = False
        for i in range(int(40 * env.hz)):
            _, _, term, trunc, info = env.step(sol.act())
            dropped |= env.signals.get("pc.dropped") == 1
            if on is None and env.signals.get("pc") == 1:
                on = i
            if on is not None and let_go is None and env.signals.get("beak.holding") == 0:
                let_go = (i - on) / env.hz
            if term or trunc:
                break
        good[seed] = (info["success"], on is not None, let_go, dropped)
        env.close()
    ok = [k for k, v in good.items() if v[0] and v[1] and v[2] is not None and v[2] < 0.5 and not v[3]]
    assert len(ok) >= NEED, good


# ---- dial and drawer (E3b-2 redesign, spec R28 2026-11-12) -------------------------------
DIAL = {"name": "dl", "preset": "beak/dial", "pos": [0.6, 0.6], "facing": -90}
DRAWER = {"name": "dr", "preset": "beak/drawer", "pos": [-0.6, 0.6], "facing": -90}


def set_dial(env, rad):
    dl = env.parts["dl"]
    env.data.qpos[dl._qa] = rad
    env.data.qvel[dl._va] = 0.0
    mujoco.mj_forward(env.model, env.data)


def test_dial_cells_target_and_detent(make):
    env = make([{**DIAL, "target": [2, 3]}])
    dl = env.parts["dl"]
    assert dl.n == 4 and dl.channel_cells("dl.angle").tolist() == [0.0, 1.0, 2.0, 3.0]
    assert env.signals.get("dl.angle") == 0 and env.signals.get("dl") == 0
    set_dial(env, math.radians(170.0))                             # nearest cell 2 (180 deg)
    body_steps(env, 2.0)
    assert env.signals.get("dl.angle") == 2 and env.signals.get("dl") == 1
    assert abs(math.degrees(env.data.qpos[dl._qa]) - 180.0) < 5.0  # the detent pulled it to the cell
    set_dial(env, math.radians(360.0 + 95.0))                      # one way round and on: cell 1
    body_steps(env, 0.3)
    assert env.signals.get("dl.angle") == 1 and env.signals.get("dl") == 0


def test_dial_cells_out_of_range_are_rejected(tmp_path, asset_dir):
    from rooms import open_room

    with pytest.raises(RoomError, match=re.escape("cells are 0..3, got 4")):
        open_room(tmp_path, [{**DIAL, "target": [1, 4]}])


def test_dial_snapshot_and_reset(make):
    env = make([{**DIAL, "target": [3, 3]}])
    set_dial(env, 3 * math.pi / 2)
    body_steps(env, 0.3)
    assert env.signals.get("dl") == 1
    snap = env.save_snapshot()
    env.reset(seed=0)
    assert env.signals.get("dl.angle") == 0 and env.signals.get("dl") == 0
    env.restore_snapshot(snap)
    assert env.signals.get("dl.angle") == 3 and env.signals.get("dl") == 1   # no step, no cell between
    body_steps(env, 0.5)
    assert env.signals.get("dl.angle") == 3


def test_dial_turns_one_way_only(make):
    env = make([{**DIAL, "target": [2, 2]}])
    dl = env.parts["dl"]
    set_dial(env, math.radians(180.0))
    body_steps(env, 1.0)
    assert env.signals.get("dl.angle") == 2
    assert env.model.jnt_range[dl._jid][0] == pytest.approx(1.7 * math.pi / 2)   # the stop followed it
    for _ in range(int(2.0 * env.hz)):                              # a hard push the other way
        env.data.qfrc_applied[dl._va] = -2.0
        tick(env)
    env.data.qfrc_applied[dl._va] = 0.0
    body_steps(env, 0.5)
    assert env.signals.get("dl.angle") == 2 and env.signals.get("dl") == 1
    env.reset(seed=0)                                               # reset frees it again
    assert env.model.jnt_range[dl._jid][0] == pytest.approx(-0.3 * math.pi / 2)


def test_dial_blocks_planning_out_to_its_spokes(make):
    dl = make([DIAL]).parts["dl"]
    (seg,) = dl.blockers(False)
    assert seg.thickness == pytest.approx(2 * dl.cfg["radius"]) and seg.thickness > 0.25


def test_dial_target_from_above_to_is_rejected(tmp_path, asset_dir):
    from rooms import open_room

    with pytest.raises(RoomError, match=re.escape("needs from <= to, got [3, 0]")):
        open_room(tmp_path, [{**DIAL, "target": [3, 0]}])


def test_a_dial_only_a_turret_reads_is_no_switch(make):
    env = make([DIAL, {"name": "tur", "preset": "trap/cannon_turret", "pos": [-1.2, 0.75], "in": ["dl"],
                       "bind": {"aim_yaw": {"from": "dl.angle", "map": [0, 3, -30, 30]}}}])
    assert env.parts["dl"].role == "switch"                         # its `<name>` fires the turret
    env = make([DIAL, {"name": "fire", "preset": "input/floor_momentary", "pos": [-1.0, -0.5]},
                {"name": "tur", "preset": "trap/cannon_turret", "pos": [-1.2, 0.75], "in": ["fire"],
                 "bind": {"aim_yaw": {"from": "dl.angle", "map": [0, 3, -30, 30]}}}])
    assert env.parts["dl"].role == "prop"


def test_drawer_latches_open_and_its_strap_is_no_longer_grabbed(make):
    from mujoco_puzzle_gym.grab import BEAK_PART

    env = make([DRAWER, {"name": "c", "preset": "beak/cube_weight", "pos": [-0.6, 0.68]}])
    dr, beak, c = env.parts["dr"], env.parts[BEAK_PART], env.parts["c"]
    body_steps(env, 0.5)
    assert c.pos(env.data)[2] == pytest.approx(0.17 + 0.02, abs=0.005)   # resting on the tray
    beak._grab(env.model, env.data, beak.objects.index(dr))
    env.data.qpos[dr._qa] = 0.149                                  # pulled out all the way
    mujoco.mj_forward(env.model, env.data)
    body_steps(env, 0.1)
    assert env.signals.get("dr") == 1 and beak.held is None and env.signals.get("dr.held") == 0
    assert env.model.jnt_range[dr._jid][0] > 0.14                   # latched: it stays open
    assert all(beak.objects[i] is not dr for i in [beak.nearest(env.data)[0]])
    snap = env.save_snapshot()
    env.reset(seed=0)
    assert env.signals.get("dr") == 0 and env.model.jnt_range[dr._jid][0] == 0.0
    env.restore_snapshot(snap)
    assert env.signals.get("dr") == 1 and env.model.jnt_range[dr._jid][0] > 0.14


def test_drawer_contents_ride_with_a_jittered_drawer(make):
    env = make([{**DRAWER, "random": [0.3, 0.3]}, {"name": "c", "preset": "beak/cube_weight", "pos": [-0.6, 0.68]}])
    dr, c = env.parts["dr"], env.parts["c"]
    moved = []
    for seed in range(4):
        env.reset(seed=seed)
        moved.append(float(np.linalg.norm(dr.xy - dr.base_xy)))
        body_steps(env, 0.5)
        p = c.pos(env.data)
        assert np.allclose(p[:2] - dr.xy, [0.0, 0.08], atol=0.01), (seed, dr.xy, p)
        assert p[2] == pytest.approx(0.17 + 0.02, abs=0.005)       # on the tray, not the floor
    assert max(moved) > 0.1                                          # the jitter did move it


def test_beak_obs_skips_a_latched_drawers_strap(make):
    from mujoco_puzzle_gym.obs import OBS

    env = make([DRAWER])
    dr = env.parts["dr"]
    env.data.qpos[dr._qa] = 0.149
    mujoco.mj_forward(env.model, env.data)
    body_steps(env, 0.1)
    assert env.signals.get("dr") == 1
    assert OBS["beak"](env).tolist()[2:] == [0.0, 0.0, 0.0]         # no handle left to reach for


def test_drawer_contents_must_be_within_the_beaks_reach(asset_dir, tmp_path):
    from mujoco_puzzle_gym.check import check

    raw = {"name": "deep", "size": [3.0, 3.0], "spawn": {"pos": [-1.0, -1.0], "yaw": 0.0},
           "parts": [DRAWER, {"name": "c", "preset": "beak/cube_weight", "pos": [-0.6, 0.73]},
                     {"name": "goal", "type": "goal", "pos": [1.3, 1.3]}]}
    path = tmp_path / "deep.yaml"
    path.write_text(yaml.safe_dump(raw))
    res = check(path, static_only=True)
    assert not res["parts"]["ok"] and "sits 0.13 m deep in drawer 'dr'" in res["parts"]["reason"], res


def test_demo_parses_dial_and_drawer(tmp_path, asset_dir):
    raw = {"name": "dd", "size": [3.0, 3.0], "spawn": {"pos": [-1.0, -1.0], "yaw": 0.0},
           "parts": [DIAL, DRAWER, {"name": "goal", "type": "goal", "pos": [1.3, 1.3]}],
           "demo": ["turn dl 3", "pull dr"]}
    path = tmp_path / "dd.yaml"
    path.write_text(yaml.safe_dump(raw))
    assert PuzzleEnv(room=path).room.demo == [("turn", "dl", 3.0), ("pull", "dr")]
    raw["demo"] = ["turn dl 4"]
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(RoomError, match=re.escape("needs one of its cells 0..3")):
        PuzzleEnv(room=path)


@pytest.mark.parametrize("actions, ok", [(["move"], False), (["move", "grab"], True)])
def test_check_asks_for_grab_to_open_a_drawer(asset_dir, tmp_path, actions, ok):
    from mujoco_puzzle_gym.check import check

    path = beak_room(tmp_path, [{**DRAWER, "pos": [0.3, 0.3]}], ["pull dr"], stand=False, name="drw")
    res = check(path, task={"actions": actions}, static_only=True)
    assert res["actions"]["ok"] is ok, res["actions"]


def gate_room(tmp_path, device, demo=None, extra=()):
    """A divider with a door the device opens; the body starts on the device's side."""
    raw = {"name": f"gate_{device['name']}", "size": [3.0, 2.4], "wall": {"thickness": 0.06, "height": 0.25},
           "walls": [{"from": [0.7, -1.2], "to": [0.7, -0.28]}, {"from": [0.7, 0.28], "to": [0.7, 1.2]}],
           "spawn": {"pos": [-1.1, -0.7], "yaw": 0.0, "random": [0.08, 0.1, 0.8]},
           "parts": [device, *extra, {"name": "door", "preset": "device/sliding_door", "pos": [0.7, 0.0],
                                       "in": [device["name"]]},
                     {"name": "goal", "type": "goal", "pos": [1.2, 0.0], "radius": 0.15}]}
    if demo:
        raw["demo"] = demo
    path = tmp_path / f"{raw['name']}.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


LONG = {"actions": ["move", "head", "grab"], "env": {"episode_seconds": 120.0}}


@pytest.mark.slow
@pytest.mark.parametrize("device", [{**DIAL, "pos": [-0.2, 0.5], "target": [2, 2]},
                                    {**DRAWER, "pos": [-0.2, 0.6]}], ids=["dial", "drawer"])
def test_grid_answer_works_the_dial_and_the_drawer(asset_dir, tmp_path, device):
    from mujoco_puzzle_gym.check import check

    res = check(gate_room(tmp_path, device), task=LONG, seeds=SEEDS, need=NEED, snap_dir=tmp_path)
    assert res["ok"] and res["order"]["order"] == [device["name"], "door", "goal"], res


@pytest.mark.slow
def test_turn_sets_the_dial_to_a_chosen_cell(asset_dir, tmp_path):
    bad = {}
    for seed in range(SEEDS):
        path = gate_room(tmp_path, {**DIAL, "pos": [-0.2, 0.5], "target": [3, 3]}, demo=["turn dl 3"])
        env = PuzzleEnv(task=LONG, room=path)
        sol = ScriptedSolver(env)
        env.reset(seed=seed)
        sol.reset()
        for _ in range(int(90 * env.hz)):
            env.step(sol.act())
            if not sol.in_demo or env.body_state.fallen:
                break
        if sol.in_demo or env.body_state.fallen or env.signals.get("dl.angle") != 3:
            bad[seed] = (env.body_state.fallen, env.signals.get("dl.angle"))
        env.close()
    assert SEEDS - len(bad) >= NEED, bad


@pytest.mark.slow
def test_pull_opens_the_drawer_and_the_cube_inside_is_grabbed(asset_dir, tmp_path):
    bad = {}
    for seed in range(SEEDS):
        path = gate_room(tmp_path, {**DRAWER, "pos": [-0.2, 0.6]}, demo=["pull dr", "grab c"],
                         extra=[{"name": "c", "preset": "beak/cube_weight", "pos": [-0.2, 0.68]}])
        env = PuzzleEnv(task=LONG, room=path)
        sol = ScriptedSolver(env)
        env.reset(seed=seed)
        sol.reset()
        for _ in range(int(90 * env.hz)):
            env.step(sol.act())
            if not sol.in_demo or env.body_state.fallen:
                break
        if sol.in_demo or env.signals.get("dr") != 1 or env.signals.get("c.held") != 1:
            bad[seed] = (sol.step_i, env.signals.get("dr"), env.signals.get("c.held"))
        env.close()
    assert SEEDS - len(bad) >= NEED, bad


# ---- dispenser (E3b-3) -------------------------------------------------------------------
DISP = {"name": "d", "preset": "beak/dispenser", "pos": [-0.6, 0.6], "facing": 0.0}   # plate at (-0.25, 0.6)
BASKET = {"name": "bk", "preset": "beak/basket", "pos": [0.6, -0.6], "consume": True, "count": 3}


def step_on(env, seconds=0.05):
    """Stand on d's floor plate (a rising edge after `step_off`) and step `seconds`."""
    stand_at(env, *env.parts["d"].press_plate.xy)
    body_steps(env, seconds)


def step_off(env, seconds=0.05):
    stand_at(env, 0.0, 0.0)
    body_steps(env, seconds)


def out_copies(env):
    return [c.name for c in env.parts["d"].copies if not c.parked]


def test_dispenser_hands_out_one_copy_per_rising_edge(make):
    env = make([{**DISP, "stock": 2, "max_active": 2, "cooldown": 1.0}])
    d = env.parts["d"]
    assert [c.name for c in d.copies] == ["d/0", "d/1"] and out_copies(env) == []
    assert env.signals.get("d.stock") == 2 and env.signals.get("d.empty") == 0
    assert all(c.pos(env.data)[2] < -4 for c in d.copies)              # parked below the floor
    step_on(env, 0.2)
    assert out_copies(env) == ["d/0"] and env.signals.get("d.stock") == 1
    p = d.copies[0].pos(env.data)
    assert np.linalg.norm(p[:2] - d.exit_xy()) < 0.01 and p[2] == pytest.approx(0.17 + 0.02, abs=0.005)
    step_on(env, 0.2)                                                  # still on: no new edge
    assert out_copies(env) == ["d/0"]
    step_off(env)
    step_on(env)                                                       # cooldown 1 s not over
    assert out_copies(env) == ["d/0"]
    step_off(env, 1.0)
    step_on(env)
    assert out_copies(env) == ["d/0", "d/1"]
    assert env.signals.get("d.stock") == 0 and env.signals.get("d.empty") == 1
    step_off(env, 1.2)
    step_on(env)                                                       # empty: nothing more
    assert d.dispensed == 2


def test_dispenser_max_active_waits_for_the_copy_to_go(make):
    env = make([{**DISP, "cooldown": 0.0, "recycle": True}, BASKET])
    d = env.parts["d"]
    step_on(env)
    step_off(env)
    step_on(env)
    assert out_copies(env) == ["d/0"]                                  # max_active 1
    d.copies[0].put(env.data, [0.6, -0.6, 0.05])                       # into the consuming basket
    mujoco.mj_forward(env.model, env.data)
    step_off(env, 0.6)
    assert env.signals.get("bk.count") == 1 and env.parts["bk"].consumed == ["d/0"]
    assert out_copies(env) == [] and env.signals.get("d.stock") == 5   # recycle: in stock again
    step_on(env)
    assert out_copies(env) == ["d/0"]                                  # the same copy, handed out again


def test_consumed_copy_without_recycle_waits_for_refill(make):
    env = make([{**DISP, "refill": "rf"}, BASKET,
                {"name": "rf", "type": "button", "pos": [0.6, 0.6], "trigger": "body"}])
    d = env.parts["d"]
    step_on(env)
    d.copies[0].put(env.data, [0.6, -0.6, 0.05])
    mujoco.mj_forward(env.model, env.data)
    step_off(env, 0.6)
    assert env.signals.get("bk.count") == 1 and env.signals.get("d.stock") == 4 and d.spent == {0}
    stand_at(env, 0.6, 0.6)                                            # refill's rising edge
    body_steps(env, 0.05)
    assert env.signals.get("d.stock") == 5 and d.spent == set()


def test_a_basket_without_consume_keeps_the_copy(make):
    env = make([DISP, {**BASKET, "consume": False}])
    d = env.parts["d"]
    step_on(env)
    d.copies[0].put(env.data, [0.6, -0.6, 0.05])
    mujoco.mj_forward(env.model, env.data)
    step_off(env, 0.6)
    assert env.signals.get("bk.count") == 1 and out_copies(env) == ["d/0"]


@pytest.mark.parametrize("recycle, stock", [(True, 5), (False, 4)])
def test_a_copy_fallen_through_the_floor_goes_back(make, recycle, stock):
    env = make([{**DISP, "recycle": recycle, "cooldown": 0.0}])
    d = env.parts["d"]
    step_on(env)
    d.copies[0].put(env.data, [0.0, 0.8, -0.5])                        # under the floor (pit, cliff)
    mujoco.mj_forward(env.model, env.data)
    step_off(env)
    assert out_copies(env) == [] and env.signals.get("d.stock") == stock
    step_on(env)
    assert len(out_copies(env)) == 1                                   # max_active is free again


LEVEL_MAP = """
##########
#LLLL....#
#LLLL....#
#LLLL....#
#........#
#....HHHH#
#....HHHH#
##########
"""


@pytest.mark.parametrize("at, caught", [((0.25, -0.15), False), ((-0.25, 0.15), False),
                                         ((0.25, -0.15, -0.48), True)])
def test_fallen_copy_is_judged_by_the_catch_plane_not_the_dispenser_floor(asset_dir, tmp_path, at, caught):
    """The dispenser stands on a 0.3 m floor (H); a copy on the 0 m floor or the -0.2 m
    floor (L) is still out, one on the catch plane goes back."""
    raw = {"name": "lv", "cell": 0.1, "map": LEVEL_MAP, "spawn": {"pos": [-0.05, -0.05], "yaw": 0.0},
           "legend": {"H": {"type": "floor", "height": 0.3}, "L": {"type": "floor", "height": -0.2}},
           "parts": [{**DISP, "pos": [0.2, -0.15], "facing": 180, "button": "none", "in": ["sw"]},
                     {"name": "sw", "type": "switch", "pos": [-0.05, -0.05], "size": [0.2, 0.2]},
                     {"name": "goal", "type": "goal", "pos": [0.3, 0.2], "radius": 0.05}]}
    path = tmp_path / "lv.yaml"
    path.write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(room=path)
    env.reset(seed=0)
    d = env.parts["d"]
    assert d.floor_z == pytest.approx(0.3)
    body_steps(env, 0.2)
    assert out_copies(env) == ["d/0"]                                  # the body stands on sw
    c = d.copies[0]
    z = at[2] if len(at) == 3 else env.floor_height(at[:2])[0] + c._rest_z()
    c.put(env.data, [*at[:2], z])
    mujoco.mj_forward(env.model, env.data)
    body_steps(env, 0.3)
    assert (out_copies(env) == []) is caught
    env.close()


def test_a_recycled_broken_egg_comes_out_whole(make):
    env = make([{**DISP, "item": {"preset": "beak/egg", "on_break": "none"}, "stock": 1, "recycle": True,
                 "cooldown": 0.0}, BASKET])
    d = env.parts["d"]
    step_on(env)
    egg = d.copies[0]
    egg._set(env.model, True)                                          # broken on the way
    egg.put(env.data, [0.6, -0.6, 0.05])
    mujoco.mj_forward(env.model, env.data)
    step_off(env, 1.0)
    assert env.signals.get("bk.count") == 1 and egg.parked and env.signals.get("d/0.broken") == 1
    step_on(env)
    assert not egg.parked and not egg.broken and env.signals.get("d/0.broken") == 0


def test_solver_leaves_objects_that_hold_an_input_on(make):
    from mujoco_puzzle_gym.solver import ScriptedSolver

    env = make([{"name": "a", "preset": "beak/cube_weight", "pos": [0.8, 0.8]},
                {"name": "b", "preset": "beak/cube_weight", "pos": [0.8, 0.5]},
                {"name": "c", "preset": "beak/cube_weight", "pos": [0.8, -0.8]},
                {"name": "full", "preset": "beak/basket", "pos": [-0.6, -0.6]},
                {"name": "pl", "type": "pressure", "pos": [-0.6, 0.6], "size": [0.2, 0.2], "min_mass": 0.02}])
    env.parts["a"].put(env.data, [-0.6, -0.6, 0.03])
    env.parts["b"].put(env.data, [-0.6, 0.6, 0.03])
    mujoco.mj_forward(env.model, env.data)
    body_steps(env, 0.6)
    assert env.signals.get("full.ok") == 1 and env.signals.get("pl") == 1
    sol = ScriptedSolver(env)
    assert [sol._in_use(env.parts[n]) for n in "abc"] == [True, True, False]


def test_copies_share_one_marker_id(make):
    env = make([{**DISP, "item": {"preset": "beak/cube_weight", "marker": {"id": 5}}, "stock": 3}])
    d = env.parts["d"]
    assert [c.marker.id for c in d.copies] == [5, 5, 5]
    assert d.marker is not None and d.marker.id != 5                   # the plate has its own


def test_dispenser_snapshot_and_reset_round_trip(make):
    env = make([{**DISP, "max_active": 2, "cooldown": 0.0}])
    d = env.parts["d"]
    step_on(env, 0.5)
    snap = env.save_snapshot()
    p0 = d.copies[0].pos(env.data)
    step_off(env)
    step_on(env)
    assert out_copies(env) == ["d/0", "d/1"]
    env.restore_snapshot(snap)
    assert out_copies(env) == ["d/0"] and env.signals.get("d.stock") == 4
    assert np.allclose(d.copies[0].pos(env.data), p0) and d.copies[1].pos(env.data)[2] < -4
    assert env.model.geom_contype[env.model.body_geomadr[d.copies[1]._bid]] == 0
    step_off(env)
    step_on(env)
    assert out_copies(env) == ["d/0", "d/1"]                           # the trigger memory came back
    env.reset(seed=0)
    assert out_copies(env) == [] and env.signals.get("d.stock") == 5 and d.dispensed == 0


def test_parked_copies_are_out_of_obs_and_reach(make):
    from mujoco_puzzle_gym.obs import beak, part_pos, part_state

    env = make([DISP])
    d = env.parts["d"]
    names = [n for n in env.parts if not n.startswith("_look_")]
    rows = [names.index(c.name) for c in d.copies]
    assert not part_pos(env).reshape(-1, 2)[rows].any() and not part_state(env)[rows].any()
    to_beak(env, "d/0")                                                # still parked, its handle at the tip
    grab(env, seconds=0.02)
    assert env.signals.get("beak.holding") == 0 and beak(env)[0] == 0
    step_on(env)
    assert part_pos(env).reshape(-1, 2)[rows[0]].any() and not part_pos(env).reshape(-1, 2)[rows[1]].any()


def test_npc_item_parks_far_off_and_comes_out_on_the_floor(make):
    env = make([{**DISP, "item": "npc/patrol", "stock": 2}])
    d = env.parts["d"]
    r0 = d.copies[0]
    assert d.npc_item and r0.parked and r0.pos(env.data)[0] > 90
    step_on(env, 0.3)
    assert not r0.parked and np.linalg.norm(r0.pos(env.data)[:2] - d.exit_xy()) < 0.05


@pytest.mark.parametrize("cfg, needle", [({"item": "beak/basket"}, "grabbable object"),
                                         ({"button": "none"}, "needs `in`"),
                                         ({"stock": 21}, "stock")])
def test_dispenser_config_errors(make, cfg, needle):
    with pytest.raises(RoomError, match=needle):
        make([{**DISP, **cfg}])


def test_demo_take_parsing(asset_dir, tmp_path):
    from mujoco_puzzle_gym.room import parse_demo

    env = open_room(tmp_path, [DISP, BASKET, {**DISP, "name": "dn", "pos": [0.6, 0.6], "item": "npc/patrol"}])
    parts = env.parts
    assert parse_demo(["take d", "grab d", "place d bk"], parts) == [("take", "d"), ("grab", "d"),
                                                                     ("place", "d", "bk")]
    with pytest.raises(RoomError, match="take from a dispenser"):
        parse_demo(["take bk"], parts)
    with pytest.raises(RoomError, match="`take d` first"):
        parse_demo(["grab d"], parts)
    with pytest.raises(RoomError, match="robots, not grabbable"):
        parse_demo(["take dn", "grab dn"], parts)
    env.close()


# Slow: the scripted solution presses the dispenser, carries the copy into a consuming
# basket (demo `take` / `grab` / `place`), and the grid answer takes and delivers it.
DISP_PRESETS = ["dispenser", "dispenser_wall"]
DISP_AT, BASKET_AT = [-0.2, 0.33], [0.25, -0.1]       # measured (plan.md E3b-3 review) (plan.md E3b-3 review)


def disp_room(tmp_path, preset, demo, name):
    parts = [{"name": "d", "preset": f"beak/{preset}", "pos": DISP_AT, "facing": -90},
             {"name": "bk", "preset": "beak/basket", "pos": BASKET_AT, "consume": True}]
    raw = {"name": name, "cell": 0.1, "map": WALL_ROOM, "demo": demo,
           "spawn": {"pos": [-0.6, -0.25], "yaw": 0.0, "random": [0.05, 0.05, 0.3]},
           "parts": [*parts, {"name": "goal", "type": "goal", "pos": [0.8, -0.35], "radius": 0.05}]}
    path = tmp_path / f"{name}.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


@pytest.mark.slow
@pytest.mark.parametrize("preset", DISP_PRESETS)
def test_scripted_solution_takes_from_the_dispenser_and_delivers(asset_dir, tmp_path, preset):
    bad = {}
    for seed in range(SEEDS):
        path = disp_room(tmp_path, preset, ["take d", "grab d", "place d bk"], f"{preset}_{seed}")
        env, on, sol = solve(path, "bk.ok", seed, seconds=60.0)
        if on is None or sol.in_demo or env.body_state.fallen:
            bad[seed] = (on, sol.step_i, env.body_state.fallen)
        else:
            assert env.signals.get("bk.count") == 1 and env.parts["bk"].consumed == ["d/0"]
            d = env.parts["d"]                # walking off may cross the floor plate again
            assert env.signals.get("d.stock") == 5 - d.dispensed and d.copies[0].parked
        env.close()
    assert SEEDS - len(bad) >= NEED, f"{preset}: {bad}"


@pytest.mark.parametrize("preset, actions, ok", [("dispenser_wall", ["move", "grab"], False),
                                                 ("dispenser_wall", ["move", "head", "grab"], True),
                                                 ("dispenser", ["move", "grab"], True),
                                                 ("dispenser", ["move"], False)])
def test_check_actions_for_a_dispenser(asset_dir, tmp_path, preset, actions, ok):
    from mujoco_puzzle_gym.check import check

    path = disp_room(tmp_path, preset, ["take d", "grab d", "place d bk"], f"chk_{preset}")
    res = check(path, task={"actions": actions}, static_only=True)
    assert res["actions"]["ok"] is ok and res["ok"] is ok, res


def gate_disp_room(tmp_path, preset, name):
    """The basket (consume) opens the door to the goal; no `demo:`."""
    raw = {"name": name, "cell": 0.1, "map": GATE_MAP,
           "spawn": {"pos": [-0.6, -0.25], "yaw": 0.0, "random": [0.05, 0.05, 0.3]},
           "parts": [{"name": "d", "preset": f"beak/{preset}", "pos": DISP_AT, "facing": -90},
                     {"name": "bk", "preset": "beak/basket", "pos": [0.35, 0.15], "consume": True},
                     {"name": "door", "type": "door", "pos": [0.65, 0.0], "width": 0.3, "axis": "y",
                      "in": ["bk.ok"]},
                     {"name": "goal", "type": "goal", "pos": [0.85, 0.0], "radius": 0.08}]}
    path = tmp_path / f"{name}.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


@pytest.mark.parametrize("preset", DISP_PRESETS)
def test_gate_dispenser_room_passes_the_static_check(asset_dir, tmp_path, preset):
    from mujoco_puzzle_gym.check import check

    res = check(gate_disp_room(tmp_path, preset, f"gchk_{preset}"),
                task={"actions": ["move", "head", "grab"]}, static_only=True)
    assert res["ok"] and res["order"]["order"][:1] == ["bk"], res


GATE_MAP = """
####################
#...............#..#
#...............#..#
#...............#..#
#..................#
#..................#
#..................#
#...............#..#
#...............#..#
#...............#..#
####################
"""


@pytest.mark.slow
@pytest.mark.parametrize("preset", DISP_PRESETS)
def test_grid_answer_takes_a_copy_for_the_basket(asset_dir, tmp_path, preset):
    """No `demo:`: the basket opens the door to the goal, so the plan orders it first and
    the grid answer presses the dispenser and carries the copy in (until bk.ok)."""
    bad = {}
    for seed in range(SEEDS):
        path = gate_disp_room(tmp_path, preset, f"g_{preset}_{seed}")
        env, on, sol = solve(path, "bk.ok", seed, seconds=60.0)
        assert [t.name for t in env.targets] == ["bk", "goal"]
        if on is None or env.body_state.fallen:
            bad[seed] = (on, env.body_state.fallen)
        env.close()
    assert SEEDS - len(bad) >= NEED, f"{preset}: {bad}"


@pytest.mark.slow
@pytest.mark.parametrize("preset", DISP_PRESETS)
def test_take_twice_waits_out_the_cooldown(asset_dir, tmp_path, preset):
    """`take` again while on the plate (or before the cooldown ends): step off, press again."""
    path = disp_room(tmp_path, preset, ["take d", "take d"], f"twice_{preset}")
    raw = yaml.safe_load(path.read_text())
    raw["parts"][0].update(max_active=2, cooldown=5.0)
    path.write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(task={"actions": ["move", "head", "grab"]}, room=path)
    sol = ScriptedSolver(env)
    env.reset(seed=0)
    sol.reset()
    for _ in range(int(60 * env.hz)):
        env.step(sol.act())
        if not sol.in_demo:
            break
    assert not sol.in_demo and env.parts["d"].dispensed == 2 and not env.body_state.fallen
    env.close()
