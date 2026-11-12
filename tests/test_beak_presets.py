"""R28 beak gimmick presets (E3b-1): the objects (cube_weight, laundry, key,
handle_ball, egg, stick) and receivers (hook, basket, scale, wall_button_beak
low / mid).

Fast: egg breaking, key / socket marker match, stick press window, basket count
and mass, scale mass, hook hang and take-off, the `beak` obs kind, `demo:`
parsing, `mpg check` stage `actions` for the two wall buttons.
Slow: per preset, the scripted solution (`demo:`: grab, place, hang, drop,
press) turns its signal on.
"""

import math

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
