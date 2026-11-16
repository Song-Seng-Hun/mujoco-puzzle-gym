"""R76: a room written for one body is a claim about the other bodies, and this checks it.

Until this existed the claim could not be checked. A capability the gate names is refused
out loud - `part 'switch_a' needs body capabilities ['walk']` - but a body excluded by
geometry only produces "the solver did not reach the goal", which looks exactly like a room
that is merely hard. Every "only this body solves it" in a preset comment was my word for
it, and in one case my word was wrong: `puzzles/tabletop/reach_target` is solved by three
of the four bodies.

`check_all_bodies` names the stage that decided each cell, so the comment and the package
can be compared: `capability` (the gate refused by name), `unreachable` (outside a fixed
reach ring, R77), `static` (a part's own refusal), `unplannable` (no path on the grid),
`solver` (planned, not solved) and `pass`.
"""

import warnings

import pytest

from mujoco_puzzle_gym import catalog
from mujoco_puzzle_gym.bodies import BODIES
from mujoco_puzzle_gym.check import OUTCOMES, check_all_bodies

WIDE = catalog.PRESET_ROOT / "puzzles/wide/switch_door.yaml"
TABLE = catalog.PRESET_ROOT / "puzzles/tabletop/reach_target.yaml"


def matrix(room, **kw):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return check_all_bodies(room, seeds=kw.pop("seeds", 3), need=kw.pop("need", 2), **kw)


def test_every_body_gets_a_row_and_a_named_outcome(all_asset_dirs):
    res = matrix(WIDE, static_only=True)
    assert set(res["bodies"]) == set(BODIES)
    for name, row in res["bodies"].items():
        assert row["outcome"] in (*OUTCOMES, "unavailable"), (name, row)
        assert row["why"], name


def test_a_walking_room_refuses_the_arm_by_name(all_asset_dirs):
    """The one exclusion this environment could always report: the gate compares names."""
    res = matrix(WIDE, static_only=True)
    assert res["bodies"]["so101"]["outcome"] == "capability"
    assert "walk" in res["bodies"]["so101"]["why"]


def test_a_target_outside_a_fixed_reach_reads_as_unreachable(tmp_path, all_asset_dirs):
    """Distinct from `capability`: the arm has every name this room asks for and still
    cannot touch the goal (R77). Before that stage existed this cell would have said
    `solver`, which does not tell a room's author what is wrong with it."""
    import yaml

    path = tmp_path / "far.yaml"
    path.write_text(yaml.safe_dump(
        {"name": "far", "size": [1.6, 1.0], "wall": {"thickness": 0.02, "height": 0.1},
         "spawn": {"pos": [-0.3, 0.0], "yaw": 0.0},
         "parts": [{"name": "goal", "type": "goal", "pos": [0.3, 0.0], "radius": 0.06}],
         "demo": ["goto goal"]}))
    res = matrix(path, static_only=True)
    assert res["bodies"]["so101"]["outcome"] == "unreachable"
    assert "0.6 m" in res["bodies"]["so101"]["why"]


@pytest.mark.slow
def test_the_arms_first_room_is_not_exclusive_to_it(all_asset_dirs):
    """The finding this tool was built to make possible. `tabletop/reach_target` holds one
    `goal` and nothing that needs a capability, so three bodies solve it - the arm in 0.6 s,
    the first body in about 1.5 s, the humanoid in about 12 - and only the quadruped fails,
    on a table two of its body lengths across. A room is exclusive when the matrix says so.
    """
    res = matrix(TABLE)
    assert res["bodies"]["so101"]["outcome"] == "pass"
    assert not res["exclusive"]
    assert sorted(res["solved_by"]) == ["g1", "microduck", "so101"]
    assert res["bodies"]["go2"]["outcome"] == "solver"


@pytest.mark.slow
def test_the_large_body_rooms_are_solved_by_every_walking_body(all_asset_dirs):
    """Including the first body, slowly: 22.9 s against the quadruped's 25.4 and the
    humanoid's 20.2 on three seeds. `docs/bodies.md` says these dimensions are not wrong
    for a small body, only slow, and this is that sentence as a measurement."""
    res = matrix(WIDE)
    assert sorted(res["solved_by"]) == ["g1", "go2", "microduck"]
    assert res["bodies"]["so101"]["outcome"] == "capability"
