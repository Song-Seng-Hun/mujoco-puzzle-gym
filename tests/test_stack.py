"""R71: a part that reads a stack, which nothing in the package could do before.

Designed in M4 and built in M5. Stacking already worked - a `cube` can declare `rest_on`
another cube and one can be set on another - but every receiving part reads "went in" (a
box, a socket, a basket, a scale) and none reads a height, so a room could not ask for a
tower. `mpg parts` had no part taking a `height` at all.

The discrimination that matters is a stack against a row: three cubes side by side fill the
patch and count three, and must not read as built.
"""

import warnings

import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym.env import PuzzleEnv

CUBE = 0.03


def room(tmp_path, cubes, need=3, height=0.04, settle=0.1, accept=None):
    """A room with `cubes` [(x, y, rest_on or None), ...] and one `stack` reading them."""
    parts = [{"name": n, "type": "cube", "pos": [x, y], "size": CUBE, "mass": 0.02,
              **({"rest_on": on} if on else {})} for n, x, y, on in cubes]
    parts.append({"name": "tower", "type": "stack", "pos": [0.0, 0.0], "size": [0.2, 0.2],
                  "need": need, "height": height, "settle": settle,
                  **({"accept": accept} if accept else {})})
    parts.append({"name": "goal", "type": "goal", "pos": [0.45, 0.0], "radius": 0.06,
                  "enable": "tower"})
    path = tmp_path / "stack.yaml"
    path.write_text(yaml.safe_dump(
        {"name": "stackroom", "size": [1.4, 1.0], "wall": {"thickness": 0.02, "height": 0.1},
         "spawn": {"pos": [-0.5, 0.0], "yaw": 0.0}, "parts": parts}))
    return path


def settle(tmp_path, cubes, steps=12, **kw):
    """Build the room, let it stand, and report the stack part."""
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        env = PuzzleEnv(task={"room": str(room(tmp_path, cubes, **kw)), "body": "so101",
                              "env": {"hz": 10, "episode_seconds": 20}})
    try:
        env.reset(seed=0)
        for _ in range(steps):
            env.step(np.zeros(env.action_space.shape))
        tower = env.parts["tower"]
        return bool(tower.on), int(tower.count), float(tower.read("tower.count"))
    finally:
        env.close()


#: Two high, and two is the most a room can *declare*: `cube`'s `rest_on` refuses to name a
#: cube that is itself stacked ("must name another cube that is not stacked itself"), so a
#: three-high tower cannot be written in a room file. Found by this test file. A room that
#: wants a taller one has to have it built during the episode, which is what the arm's
#: stacking room is for; `need=3` is covered below by placing the third cube directly.
STACKED = [("c0", 0.0, 0.0, None), ("c1", 0.0, 0.0, "c0")]
ROW = [("c0", -0.06, 0.0, None), ("c1", 0.0, 0.0, None), ("c2", 0.06, 0.0, None)]


def test_a_declared_tower_reads_on(tmp_path, so101_asset_dir):
    on, count, channel = settle(tmp_path, STACKED, need=2)
    assert on and count == 2 and channel == 2


def test_three_in_a_row_does_not(tmp_path, so101_asset_dir):
    """The whole point of `height`: the patch holds three cubes and reaches nothing."""
    on, count, channel = settle(tmp_path, ROW)
    assert not on
    assert count == 3 and channel == 3, "they are all on the patch, they are just not stacked"


def test_one_short_does_not(tmp_path, so101_asset_dir):
    on, count, _ = settle(tmp_path, STACKED, need=3)
    assert not on and count == 2


def test_a_third_cube_set_on_top_meets_need_three(tmp_path, so101_asset_dir):
    """`need=3` with the third cube put straight onto the tower, since a room file cannot
    declare three high. This is also the state the arm's stacking room has to reach."""
    import mujoco

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        env = PuzzleEnv(task={"room": str(room(tmp_path, [*STACKED, ("c2", 0.3, 0.3, None)],
                                               need=3, height=0.07)),
                              "body": "so101", "env": {"hz": 10, "episode_seconds": 20}})
    try:
        env.reset(seed=0)
        bid = env.model.body(env.parts["c2"].body_name).id
        adr = env.model.jnt_qposadr[env.model.body_jntadr[bid]]
        env.data.qpos[adr:adr + 3] = [0.0, 0.0, 2.5 * CUBE]      # on top of the two
        mujoco.mj_forward(env.model, env.data)
        for _ in range(12):
            env.step(np.zeros(env.action_space.shape))
        assert env.parts["tower"].on
        assert env.parts["tower"].count == 3
    finally:
        env.close()


def test_it_does_not_read_on_while_they_are_still_moving(tmp_path, so101_asset_dir):
    """`settle` is why a tower being built - or collapsing - is not a built tower. One step
    in, the cubes are still dropping into contact, so the patch is full and the part is
    off."""
    on, count, _ = settle(tmp_path, STACKED, steps=1, settle=1.0, need=2)
    assert not on and count == 2


def test_what_it_does_not_accept_is_not_counted(tmp_path, so101_asset_dir):
    on, count, _ = settle(tmp_path, STACKED, need=2, accept=["c0"])
    assert not on, "the upper cube is not accepted, so the tower is one"
    assert count == 1


def test_the_goal_gated_on_it_is_not_satisfied_without_the_tower(tmp_path, so101_asset_dir):
    """A room wires a win to it the way the arm's rooms do (`enable:`), so the part has to
    be false at the start of an episode it has not yet been satisfied in."""
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        env = PuzzleEnv(task={"room": str(room(tmp_path, STACKED[:1])), "body": "so101",
                              "env": {"hz": 10, "episode_seconds": 20}})
    try:
        env.reset(seed=0)
        env.step(np.zeros(env.action_space.shape))
        assert not env.parts["tower"].on
        assert not env.parts["goal"].satisfied()
    finally:
        env.close()


def test_it_asks_the_body_for_nothing(so101_asset_dir):
    """It only looks. What puts the objects there is the room's business, and the capability
    gate cannot speak for a part that reads - which is why `needs_body` would be wrong here
    rather than merely unnecessary."""
    from mujoco_puzzle_gym.catalog import builtin_parts

    part = builtin_parts()["stack"]("t", {"pos": [0.0, 0.0]})
    assert part.requires_body == []


def test_a_height_the_room_cannot_reach_is_refused(tmp_path, so101_asset_dir):
    """A patch asking for more than its own objects can reach makes the room unsolvable,
    and `mpg check` would otherwise only say so after the solver had failed twenty times.
    The top of a tower of `need` cubes of edge `e` sits at `(need - 0.5) * e`."""
    from mujoco_puzzle_gym.check import check_parts

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        env = PuzzleEnv(task={"room": str(room(tmp_path, STACKED, need=2, height=0.5)),
                              "body": "so101", "env": {"hz": 10, "episode_seconds": 20}})
    try:
        res = check_parts(env)
    finally:
        env.close()
    assert not res["ok"]
    assert "0.5" in res["reason"] and "stacked" in res["reason"]


def test_a_height_it_can_reach_is_accepted(tmp_path, so101_asset_dir):
    from mujoco_puzzle_gym.check import check_parts

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        env = PuzzleEnv(task={"room": str(room(tmp_path, STACKED, need=2, height=0.04)),
                              "body": "so101", "env": {"hz": 10, "episode_seconds": 20}})
    try:
        assert check_parts(env)["ok"]
    finally:
        env.close()
