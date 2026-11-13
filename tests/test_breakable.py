"""R34: breaking a circuit part, the four causes, wires, relays and the destroyer.

Time is counted in body steps of 0.02 s (`rooms.body_steps`), as in test_logic.py.
"""

from __future__ import annotations

import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym import obs as obs_mod
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.parts.logic import Relay, Wire
from mujoco_puzzle_gym.room import RoomError
from mujoco_puzzle_gym.schema import SchemaError

from rooms import body_steps, open_room, stand_at, tick
from test_doors import Source  # noqa: F401  (registers t_signal)

SIG = {"name": "s", "type": "t_signal", "pos": [-1.2, 1.2]}


def room(tmp_path, parts, name="brk", spawn=(-1.0, -1.0)):
    return open_room(tmp_path, [SIG, *parts], spawn=spawn, name=name)


def box_part(name, pos, by, hp=1, **cfg):
    """A bodied signal source that can be broken (R33 body + R34 breakable)."""
    return {"name": name, "type": "source", "mode": "const", "value": 1.0, "body": True,
            "pos": list(pos), "breakable": True, "by": list(by), "hp": hp, **cfg}


# ---- the four causes ---------------------------------------------------------------
def test_a_destroyer_strike_costs_one_hit_and_other_causes_are_ignored(tmp_path):
    env = room(tmp_path, [box_part("b", [0.4, 0.4], ["destroyer"], hp=2)])
    b = env.parts["b"]
    assert not b.hit(env.model, "shell") and not b.hit(env.model, "crush")
    assert not b.hit(env.model, "peck")
    tick(env)
    assert b.hp_left == 2 and not b.broken
    assert b.hit(env.model, "destroyer")
    tick(env)
    assert b.hp_left == 1 and not b.broken
    b.hit(env.model, "destroyer")
    tick(env)
    assert b.hp_left == 0 and b.broken


def test_a_dropped_object_crushes_it_once_per_contact(tmp_path):
    """`crush` counts the contact, not every step: an object left lying on the part
    does not drain a second hit."""
    env = room(tmp_path, [box_part("b", [0.4, 0.4], ["crush"], hp=3),
                          {"name": "box", "type": "t_box", "pos": [0.4, 0.4],
                           "half": 0.03, "mass": 0.05}])
    b, box = env.parts["b"], env.parts["box"]
    box.put(env.data, [0.4, 0.4, b.height + 0.05])            # dropped from 5 cm up
    body_steps(env, 1.5)
    assert b.hp_left == 2, "one drop, one hit"
    body_steps(env, 1.5)
    assert b.hp_left == 2, "lying on it afterwards costs nothing"
    box.put(env.data, [0.4, 0.4, b.height + 0.05])            # dropped again
    body_steps(env, 1.5)
    assert b.hp_left == 1


def test_a_light_touch_does_not_crush_it(tmp_path):
    """0.5 N is the measured default: a 30 g object resting on it (0.3 N) is below it."""
    env = room(tmp_path, [box_part("b", [0.4, 0.4], ["crush"], hp=2),
                          {"name": "box", "type": "t_box", "pos": [0.4, 0.4],
                           "half": 0.03, "mass": 0.03}])
    b, box = env.parts["b"], env.parts["box"]
    box.put(env.data, [0.4, 0.4, b.height + 0.031])           # set down, not dropped
    body_steps(env, 2.0)
    assert b.hp_left == 2 and not b.broken


def test_a_shell_hit_counts_once_and_only_above_min_speed(tmp_path):
    env = room(tmp_path, [box_part("b", [0.5, 0.5], ["shell"], hp=3, min_speed=0.5),
                          {"name": "c", "type": "cannon", "pos": [0.0, 0.5], "yaw": 0.0,
                           "aim": "fixed", "aim_pitch": 0.0, "power": 3.0, "period": 5.0,
                           "in": ["s"]}])
    b = env.parts["b"]
    body_steps(env, 0.5)
    assert b.hp_left == 3                                     # the cannon is off
    env.parts["s"].value = 1.0
    body_steps(env, 2.0)
    assert b.hp_left == 2, "one shell, one hit, however long the contact lasts"


def test_a_slow_shell_is_below_min_speed(tmp_path):
    env = room(tmp_path, [box_part("b", [0.5, 0.5], ["shell"], hp=3, min_speed=20.0),
                          {"name": "c", "type": "cannon", "pos": [0.0, 0.5], "yaw": 0.0,
                           "aim": "fixed", "aim_pitch": 0.0, "power": 3.0, "period": 5.0,
                           "in": ["s"]}])
    env.parts["s"].value = 1.0
    body_steps(env, 2.0)
    assert env.parts["b"].hp_left == 3


def peck(env, part, near: bool, times: int = 1) -> None:
    """Grab on and off `times`, with the beak tip held near the part or away from it."""
    x, y = part.xy
    stand_at(env, x - 0.11 if near else x - 1.2, y)
    close = part.beak_distance(env.model, env.data) <= float(part.cfg["peck_dist"])
    assert close == near, f"rig: beak {part.beak_distance(env.model, env.data):.3f} m away"
    for _ in range(times):
        env.world.grab = 1.0
        tick(env)
        env.world.grab = -1.0
        tick(env)


def test_pecking_it_costs_a_hit_only_with_the_beak_close(tmp_path):
    """The box reaches 0.2 m, as a wall cover does: the beak tip is 0.21 m up with the
    head level, so a 0.12 m logic box is out of its reach whatever the duck does."""
    env = room(tmp_path, [box_part("b", [0.4, 0.0], ["peck"], hp=2, peck_dist=0.04, height=0.2)])
    b = env.parts["b"]
    peck(env, b, near=False)
    assert b.hp_left == 2, "a grab across the room is not a peck"
    peck(env, b, near=True)
    assert b.hp_left == 1
    peck(env, b, near=True)
    assert b.hp_left == 0 and b.broken


def test_holding_the_grab_closed_is_one_peck(tmp_path):
    """A peck is one on-and-off of the grab, not one per step."""
    env = room(tmp_path, [box_part("b", [0.4, 0.0], ["peck"], hp=5, peck_dist=0.04, height=0.2)])
    b = env.parts["b"]
    stand_at(env, 0.29, 0.0)
    env.world.grab = 1.0
    for _ in range(10):
        tick(env)
    assert b.hp_left == 5
    env.world.grab = -1.0
    tick(env)
    assert b.hp_left == 4


# ---- what breaking does ------------------------------------------------------------
def test_a_broken_part_freezes_at_its_broken_value(tmp_path):
    env = room(tmp_path, [box_part("b", [0.4, 0.4], ["destroyer"], broken_value=0.0, value=1.0)])
    b = env.parts["b"]
    tick(env)
    assert env.signals.get("b") == pytest.approx(1.0)
    assert env.signals.get("b.hp") == pytest.approx(1.0)
    b.hit(env.model, "destroyer")
    tick(env)
    assert env.signals.get("b") == pytest.approx(0.0)
    assert env.signals.get("b.broken") == pytest.approx(1.0)
    assert env.signals.get("b.hp") == pytest.approx(0.0)
    tick(env)
    assert env.signals.get("b") == pytest.approx(0.0), "it stays broken"


def wire_room(tmp_path, name="wires", **wire):
    """A source feeding two doors, one of them through a breakable wire."""
    parts = [{"name": "sig", "type": "source", "mode": "const", "value": 1.0,
              "body": True, "pos": [-1.0, 1.0]},
             {"name": "w", "type": "wire", "carries": "sig", "to": ["door"],
              "pos": [1.4, 0.0], "facing": 180.0, "breakable": True, "by": ["destroyer"],
              **wire},
             {"name": "door", "type": "door", "pos": [0.0, 0.0], "width": 0.5, "in": ["sig"]},
             {"name": "door2", "type": "door", "pos": [0.0, 1.0], "width": 0.5, "in": ["sig"]}]
    return room(tmp_path, parts, name=name)


def test_cutting_a_wire_stops_its_readers_only(tmp_path):
    env = wire_room(tmp_path)
    w, door, door2 = env.parts["w"], env.parts["door"], env.parts["door2"]
    body_steps(env, 1.5)
    assert door.state() > 0.9 and door2.state() > 0.9
    w.hit(env.model, "destroyer")
    body_steps(env, 1.5)
    assert w.broken and door.state() < 0.1, "the wire's reader lost the channel"
    assert door2.state() > 0.9, "the other reader of the same channel is untouched"
    assert env.signals.get("sig") == pytest.approx(1.0), "the channel itself is unchanged"


def test_a_cut_wire_stops_colliding_and_turns_dark(tmp_path):
    env = wire_room(tmp_path, name="cutshape")
    w = env.parts["w"]
    gid = env.model.geom("part/w/cover").id
    assert env.model.geom_contype[gid] == 1
    whole = tuple(env.model.geom_rgba[gid])
    w.hit(env.model, "destroyer")
    tick(env)
    assert env.model.geom_contype[gid] == 0 and env.model.geom_conaffinity[gid] == 0
    assert tuple(env.model.geom_rgba[gid]) != whole


def test_on_break_fail_ends_the_episode(tmp_path):
    env = room(tmp_path, [box_part("b", [0.4, 0.4], ["destroyer"], on_break="fail")])
    b = env.parts["b"]
    assert not b.failed()
    b.hit(env.model, "destroyer")
    tick(env)
    assert b.broken and b.failed()


# ---- repair -------------------------------------------------------------------------
def test_the_repair_channel_puts_it_back(tmp_path):
    env = room(tmp_path, [box_part("b", [0.4, 0.4], ["destroyer"], hp=2, repair="s")])
    b = env.parts["b"]
    b.hit(env.model, "destroyer")
    b.hit(env.model, "destroyer")
    tick(env)
    assert b.broken and b.hp_left == 0
    env.parts["s"].value = 1.0
    tick(env)
    assert not b.broken and b.hp_left == 2
    assert env.signals.get("b.hp") == pytest.approx(2.0)
    env.parts["s"].value = 0.0
    tick(env)
    assert not b.broken, "it stays whole once repaired"


def test_a_fuse_in_a_socket_repairs_it(tmp_path):
    """The fuse route is a receptacle (R5) writing the part's `repair` channel."""
    parts = [box_part("b", [0.4, 0.4], ["destroyer"], repair="socket.ok"),
             {"name": "socket", "type": "receptacle", "pos": [0.6, -0.4], "size": [0.2, 0.2],
              "names": ["fuse"]},
             {"name": "fuse", "type": "cube", "pos": [0.6, -0.8], "size": 0.04}]
    env = room(tmp_path, parts, name="fuse")
    b, fuse = env.parts["b"], env.parts["fuse"]
    b.hit(env.model, "destroyer")
    body_steps(env, 0.2)
    assert b.broken
    fuse.put(env.data, [0.6, -0.4, 0.02])                     # the fuse goes in the socket
    body_steps(env, 0.5)
    assert env.signals.get("socket.ok") > 0.5, "the socket took the fuse"
    assert not b.broken and b.hp_left == b.hp


# ---- the destroyer ------------------------------------------------------------------
def destroyer_room(tmp_path, name="press", **cfg):
    parts = [box_part("near", [0.4, 0.4], ["destroyer"], hp=2),
             box_part("far", [-1.2, -1.2], ["destroyer"], hp=2),
             {"name": "press", "type": "destroyer", "in": ["s"], "pos": [0.4, 0.4],
              "size": [0.3, 0.3], "cooldown": 0.0, **cfg},
             {"name": "box", "type": "t_box", "pos": [0.4, 0.4], "half": 0.03, "mass": 0.05}]
    return room(tmp_path, parts, name=name)


def test_the_destroyer_strikes_its_area_on_a_rising_edge(tmp_path):
    env = destroyer_room(tmp_path)
    near, far = env.parts["near"], env.parts["far"]
    tick(env)
    assert near.hp_left == 2
    env.parts["s"].value = 1.0
    tick(env)
    tick(env)                                                 # device -> logic: one step later
    assert near.hp_left == 1 and far.hp_left == 2, "only what stands in its area"
    tick(env)
    tick(env)
    assert near.hp_left == 1, "a held signal is one strike"
    env.parts["s"].value = 0.0
    tick(env)
    env.parts["s"].value = 1.0
    tick(env)
    tick(env)
    assert near.hp_left == 0 and near.broken


def test_the_destroyer_gives_no_force_to_the_duck_or_objects(tmp_path):
    parts = [box_part("near", [1.0, 1.0], ["destroyer"], hp=2),
             {"name": "press", "type": "destroyer", "in": ["s"], "pos": [0.0, -0.6],
              "size": [0.4, 0.4], "cooldown": 0.0, "targets": ["near"]},
             {"name": "box", "type": "t_box", "pos": [0.0, -0.6], "half": 0.03, "mass": 0.05}]
    env = room(tmp_path, parts, name="noforce")
    box = env.parts["box"]
    stand_at(env, 0.0, -0.6)                                  # the duck stands under it
    box.put(env.data, [0.1, -0.6, 0.03])
    tick(env)                                                 # mj_forward, so `before` is real
    before = box.pos(env.data).copy()
    env.parts["s"].value = 1.0
    fell = body_steps(env, 1.0)
    assert np.allclose(env.data.xfrc_applied, 0.0)
    assert np.allclose(box.pos(env.data)[:2], before[:2], atol=0.01)
    assert not fell
    assert env.parts["near"].hp_left == 1, "it did strike its target"


def test_charges_run_out(tmp_path):
    env = destroyer_room(tmp_path, name="charges", charges=1)
    near = env.parts["near"]
    for _ in range(3):
        env.parts["s"].value = 1.0
        tick(env)
        env.parts["s"].value = 0.0
        tick(env)
    assert near.hp_left == 1, "one charge, one strike"
    assert env.signals.get("press.charges") == pytest.approx(0.0)


# ---- relays ---------------------------------------------------------------------------
def test_a_relay_passes_its_input_through(tmp_path):
    env = room(tmp_path, [{"name": "r", "type": "relay", "in": ["s"], "pos": [0.4, 0.0],
                           "breakable": True, "by": ["destroyer"]}])
    r = env.parts["r"]
    tick(env)
    assert env.signals.get("r") == pytest.approx(0.0)
    env.parts["s"].value = 1.0
    tick(env)
    assert env.signals.get("r") == pytest.approx(1.0)
    r.hit(env.model, "destroyer")
    tick(env)
    assert env.signals.get("r") == pytest.approx(0.0) and r.broken


def test_a_movable_relay_is_a_free_box_that_can_be_pushed(tmp_path):
    env = room(tmp_path, [{"name": "r", "type": "relay", "in": ["s"], "pos": [0.4, 0.0],
                           "movable": True, "mass": 0.3, "hitch": True}])
    r = env.parts["r"]
    assert r.kind == "object" and r.blockers(False) == []
    start = r.pos(env.data).copy()
    env.data.qvel[r._v:r._v + 3] = [2.0, 0.0, 0.0]
    body_steps(env, 0.4)
    assert r.pos(env.data)[0] - start[0] > 0.05, "it slides when pushed"


def test_a_fixed_relay_blocks_walking(tmp_path):
    env = room(tmp_path, [{"name": "r", "type": "relay", "in": ["s"], "pos": [0.4, 0.0]}])
    assert len(env.parts["r"].blockers(False)) == 1


# ---- snapshots and reset ----------------------------------------------------------------
def test_reset_and_a_restore_put_the_broken_state_back(tmp_path):
    env = wire_room(tmp_path, name="snap")
    w, door = env.parts["w"], env.parts["door"]
    whole = w.save_state()
    w.hit(env.model, "destroyer")
    body_steps(env, 0.5)
    assert w.broken and door.state() < 0.1
    gid = env.model.geom("part/w/cover").id

    w.load_state(env.model, env.data, whole)                  # back to whole
    body_steps(env, 1.0)
    assert not w.broken and env.model.geom_contype[gid] == 1 and door.state() > 0.9

    w.hit(env.model, "destroyer")
    body_steps(env, 0.5)
    broken = w.save_state()
    env.reset(seed=0)
    assert not w.broken and env.model.geom_contype[gid] == 1
    w.load_state(env.model, env.data, broken)                 # broken again
    body_steps(env, 0.5)
    assert w.broken and env.model.geom_contype[gid] == 0 and door.state() < 0.1


def test_the_same_seed_gives_the_same_result(tmp_path):
    def run():
        env = wire_room(tmp_path, name="seeded")
        env.reset(seed=3)
        env.parts["w"].hit(env.model, "destroyer")
        body_steps(env, 0.6)
        return env.parts["w"].broken, round(env.parts["door"].state(), 6)

    assert run() == run()


# ---- observation --------------------------------------------------------------------
def test_the_circuit_observation_lists_the_breakable_parts(tmp_path):
    env = room(tmp_path, [box_part("b", [0.4, 0.4], ["destroyer"]),
                          {"name": "r", "type": "relay", "in": ["s"], "pos": [-0.4, 0.0]}])
    assert obs_mod.circuit(env).shape == (3,), "only the breakable part is listed"
    env.parts["b"].hit(env.model, "destroyer")
    tick(env)
    assert obs_mod.circuit(env)[2] == pytest.approx(1.0)


# ---- settings that are refused ---------------------------------------------------------
@pytest.mark.parametrize("cfg, bad", [
    ({"type": "gate", "op": "not", "in": ["s"], "breakable": True},
     "only a part with a body"),
    ({"type": "source", "mode": "const", "body": True, "breakable": True, "by": ["kick"]},
     "are not among"),
    ({"type": "source", "mode": "const", "body": True, "breakable": True, "by": []},
     "at least one cause"),
    ({"type": "source", "mode": "const", "body": True, "hp": 3},
     "only means something with `breakable: true`"),
])
def test_bad_breakable_settings_are_refused(tmp_path, cfg, bad):
    with pytest.raises((SchemaError, RoomError)) as exc:
        room(tmp_path, [{"name": "b", "pos": [0.4, 0.4], **cfg}], name="bad")
    assert bad in str(exc.value)


@pytest.mark.parametrize("cfg, bad", [
    ({"carries": "sig"}, "at least one reader"),
    ({"carries": "sig", "to": ["door.other"]}, "but this wire carries"),
    ({"carries": "sig", "to": ["door"], "mount": "part"}, "needs `on:"),
    ({"carries": "sig", "to": ["door"], "on": "door"}, "only means something with `mount: part`"),
])
def test_bad_wire_settings_are_refused(tmp_path, cfg, bad):
    parts = [{"name": "sig", "type": "source", "mode": "const", "value": 1.0, "pos": [-1.0, 1.0]},
             {"name": "door", "type": "door", "pos": [0.0, 0.0], "width": 0.5, "in": ["sig"]},
             {"name": "w", "type": "wire", "pos": [1.4, 0.0], **cfg}]
    with pytest.raises((SchemaError, RoomError)) as exc:
        room(tmp_path, parts, name="badwire")
    assert bad in str(exc.value)


def test_a_wire_to_a_movable_relay_is_refused_by_the_check(tmp_path):
    parts = [{"name": "sig", "type": "source", "mode": "const", "value": 1.0, "pos": [-1.0, 1.0]},
             {"name": "r", "type": "relay", "in": ["sig"], "pos": [0.4, 0.0], "movable": True},
             {"name": "w", "type": "wire", "carries": "sig", "to": ["r"], "pos": [1.4, 0.0]}]
    env = room(tmp_path, parts, name="movwire")
    problems = env.parts["w"].check_static(env)
    assert any("can be moved" in p for p in problems), problems


def test_a_wire_to_a_part_that_does_not_read_the_channel_is_refused(tmp_path):
    parts = [{"name": "sig", "type": "source", "mode": "const", "value": 1.0, "pos": [-1.0, 1.0]},
             {"name": "door", "type": "door", "pos": [0.0, 0.0], "width": 0.5},
             {"name": "w", "type": "wire", "carries": "sig", "to": ["door"], "pos": [1.4, 0.0]}]
    env = room(tmp_path, parts, name="deadwire")
    assert any("does not read" in p for p in env.parts["w"].check_static(env))


def test_a_destroyer_needs_breakable_targets(tmp_path):
    parts = [box_part("b", [0.4, 0.4], ["shell"]),
             {"name": "press", "type": "destroyer", "in": ["s"], "pos": [0.4, 0.4],
              "targets": ["b"]}]
    env = room(tmp_path, parts, name="badpress")
    assert any("does not list `destroyer`" in p for p in env.parts["press"].check_static(env))
