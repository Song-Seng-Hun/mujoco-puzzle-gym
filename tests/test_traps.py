"""R16 traps: wind, pusher, spinning bar, pendulum, pit, cannon (with its shell pool
and the R5 target), and the slippery / sticky floor presets.

For each trap: signal 0 leaves things near the body alone; signal 1 acts with
the strength `power` sets (force, speed, rate, amplitude, friction). Also: the
cannon's burst / edge / spread / hits, shells never pressing buttons, plates or
zones, the pool reusing and parking shells, snapshots with shells in flight,
the turret's ballistic check, and the pit's `on_fall`. Slow tests record the
shell strength that knocks the standing body over (plan step 12 table).
"""

from __future__ import annotations

import math

import mujoco
import numpy as np
import pytest
import yaml

from mujoco_puzzle_gym.check import check
from mujoco_puzzle_gym.env import PuzzleEnv

from rooms import body_steps, open_room, tick
from test_doors import Source  # noqa: F401  (registers t_signal)

SIG = {"name": "s", "type": "t_signal", "pos": [1.0, -1.0]}
BOX = {"name": "box", "type": "t_box", "pos": [1.2, 1.0], "half": 0.03, "mass": 0.05}


def env_with(tmp_path, part, name, extra=(), spawn=(-1.0, -1.0)):
    return open_room(tmp_path, [SIG, {**part, "in": ["s"]}, *extra], spawn=spawn, name=name)


def body_rid(env):
    return next(r for r, k in env.world.kinds.items() if k == "body")


# ---- wind ---------------------------------------------------------------------------
def test_wind_pushes_with_its_power_only_while_on(tmp_path):
    env = env_with(tmp_path, {"name": "w", "preset": "trap/crosswind", "pos": [-0.8, -0.8],
                              "size": [1.0, 1.0], "yaw": 0.0, "power": 2.0}, "wind", extra=[BOX])
    box = env.parts["box"]
    box.put(env.data, [-0.6, -0.6, 0.03])
    rid, brid = body_rid(env), box._bid
    body_steps(env, 0.5)
    assert np.allclose(env.data.xfrc_applied[[rid, brid]], 0.0)
    assert np.allclose(box.pos(env.data)[:2], [-0.6, -0.6], atol=0.002)
    env.parts["s"].value = 1.0
    body_steps(env, 0.02)
    for r in (rid, brid):
        assert np.allclose(env.data.xfrc_applied[r, :3], [2.0, 0.0, 0.0])
    x0 = box.pos(env.data)[0]
    body_steps(env, 0.3)
    assert box.pos(env.data)[0] - x0 > 0.02                  # 2 N beats 0.05 kg x g x mu 1
    box.put(env.data, [1.2, 1.0, 0.03])                      # out of the zone: force taken back
    body_steps(env, 0.04)
    assert np.allclose(env.data.xfrc_applied[brid], 0.0)
    env.parts["s"].value = 0.0
    body_steps(env, 0.04)
    assert np.allclose(env.data.xfrc_applied[rid], 0.0)


def test_overlapping_winds_add_up(tmp_path):
    a = {"name": "w1", "type": "wind", "pos": [-1.0, -1.0], "power": 1.0, "yaw": 0.0}
    b = {"name": "w2", "type": "wind", "pos": [-1.0, -1.0], "power": 0.5, "yaw": 90.0}
    env = open_room(tmp_path, [a, b], name="winds")
    body_steps(env, 0.04)
    assert np.allclose(env.data.xfrc_applied[body_rid(env), :3], [1.0, 0.5, 0.0])


# ---- pusher -------------------------------------------------------------------------------
def test_pusher_runs_out_at_its_power(tmp_path):
    env = env_with(tmp_path, {"name": "p", "preset": "trap/pusher", "pos": [0.0, 0.5], "stroke": 0.4,
                              "power": 0.25}, "push", extra=[BOX])
    p, box = env.parts["p"], env.parts["box"]
    box.put(env.data, [0.15, 0.5, 0.03])
    body_steps(env, 1.0)
    assert p.extension(env.data) == pytest.approx(0.0, abs=0.002)
    assert box.pos(env.data)[0] == pytest.approx(0.15, abs=0.002)
    env.parts["s"].value = 1.0
    body_steps(env, 0.2)
    e0 = p.extension(env.data)
    body_steps(env, 0.8)
    assert (p.extension(env.data) - e0) / 0.8 == pytest.approx(0.25, rel=0.05)
    assert box.pos(env.data)[0] > 0.2                         # shoved along +x
    body_steps(env, 1.4)                                      # at the end it comes back
    assert p.extension(env.data) < 0.35


# ---- spinning bar ---------------------------------------------------------------------------
def test_spinner_turns_at_its_power(tmp_path):
    env = env_with(tmp_path, {"name": "b", "preset": "trap/spinning_bar", "pos": [0.0, 0.5],
                              "power": 120.0}, "spin")
    b = env.parts["b"]
    body_steps(env, 1.0)
    assert abs(b.rate(env.data)) < 0.5
    env.parts["s"].value = 1.0
    body_steps(env, 0.5)
    a0 = b.angle(env.data)
    body_steps(env, 1.0)
    assert math.degrees(b.angle(env.data) - a0) == pytest.approx(120.0, rel=0.05)


# ---- pendulum ------------------------------------------------------------------------------
def test_pendulum_swings_with_its_amplitude(tmp_path):
    env = env_with(tmp_path, {"name": "pe", "preset": "trap/pendulum", "pos": [0.0, 0.5],
                              "power": 30.0, "period": 1.6}, "pend")
    pe = env.parts["pe"]
    body_steps(env, 1.0)
    assert abs(pe.angle(env.data)) < 0.5
    env.parts["s"].value = 1.0
    body_steps(env, 1.0)                                      # amplitude ramps at 60 deg/s
    seen = []
    body_steps(env, 1.6, every=lambda e: seen.append(pe.angle(e.data)))
    assert max(np.abs(seen)) == pytest.approx(30.0, rel=0.1)


# ---- pit -------------------------------------------------------------------------------------
def pit_env(tmp_path, name, **pit):
    raw = {"name": name, "size": [3.0, 3.0], "spawn": {"pos": [-1.0, -1.0], "yaw": 0.0},
           "parts": [SIG, {"name": "pit", "preset": "trap/pit", "pos": [0.0, 0.0], "size": [0.5, 0.5], **pit},
                     {"name": "goal", "type": "goal", "pos": [1.3, 1.3], "radius": 0.1}]}
    path = tmp_path / f"{name}.yaml"
    path.write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(room=path)
    env.reset(seed=0, options={"layout": env.sample_layout(None)})
    return env


def drop_in(env, steps=30):
    env._place_body((0.0, 0.0, 0.0))
    for _ in range(steps):
        out = env.step(np.zeros(3, np.float32))
        if out[2] or out[4]["respawned"]:
            return out
    return out


def test_pit_fail_ends_the_episode(tmp_path):
    env = pit_env(tmp_path, "pitfail")
    _, _, terminated, _, info = drop_in(env)
    assert terminated and info["failed"] == ["pit"]
    env.close()


def test_pit_reset_goes_back_to_the_start(tmp_path):
    env = pit_env(tmp_path, "pitreset", on_fall="reset")
    _, _, terminated, _, info = drop_in(env)
    assert info["respawned"] and not terminated
    assert np.allclose(env.read_body().pos[:2], [-1.0, -1.0], atol=0.02)
    env.close()


def test_pit_lid_opens_with_the_signal(tmp_path):
    env = pit_env(tmp_path, "pitlid", **{"in": ["s"]})
    out = drop_in(env, steps=20)                              # signal 0: the lid holds
    assert not out[2] and not env.read_body().fallen
    env.parts["s"].value = 1.0
    _, _, terminated, _, info = drop_in(env)
    assert terminated and info["failed"] == ["pit"]
    env.close()


def test_pit_without_in_cannot_be_switched(tmp_path):
    """No `in`: always open, so enable / enabled (value, setter, bind, difficulty step) are rejected."""
    from mujoco_puzzle_gym.room import RoomError
    from mujoco_puzzle_gym.task import TaskError
    pit = {"name": "p", "preset": "trap/pit", "pos": [0.5, 0.5]}
    bad = [[{**pit, "enabled": 0}], [SIG, {**pit, "enable": "s"}],
           [SIG, {**pit, "bind": {"enabled": {"from": "s", "map": [0, 1, 0, 1]}}}],
           [SIG, pit, {"name": "set", "type": "setter", "in": ["s"], "target": "p.enabled", "value": 0}]]
    for i, parts in enumerate(bad):
        with pytest.raises(RoomError, match="give the pit an `in`"):
            open_room(tmp_path, parts, name=f"pitsw{i}")
    with pytest.raises(TaskError, match="give the pit an `in`"):
        open_room(tmp_path, [pit], task={"difficulty": [{"param": "p.enabled", "from": 0, "to": 1, "steps": 2}]},
                  name="pitdiff")
    env = open_room(tmp_path, [SIG, {**pit, "in": ["s"]}],                     # with `in` it may be switched
                    task={"difficulty": [{"param": "p.enabled", "from": 0, "to": 1, "steps": 2}]}, name="pitok")
    env.reset(seed=0, options={"difficulty": 0})
    env.parts["s"].value = 1.0
    tick(env)
    assert not env.parts["p"].open


def test_pit_is_a_hole_for_planning(tmp_path):
    from mujoco_puzzle_gym.logic import free_mask
    env = pit_env(tmp_path, "pitplan")
    assert not free_mask(env.grid, env.walls, env.parts, set())[env.grid.cell((0.0, 0.0))]
    env.close()



# ---- pit depth: a flush box fills it (R16, R31 pit_bridge) ----------------------------------
DEEP = {"name": "pit", "preset": "trap/pit", "pos": [0.0, 0.0], "size": [0.31, 0.345], "depth": 0.3}
FILLER = {"name": "box", "type": "cube", "pos": [0.0, 0.0], "size": 0.3, "handles": ["-x"], "handle_size": 0.005,
          "mass": 0.08, "friction": 0.5, "hitch": True}
ROPE = {"name": "rope", "type": "rope", "pos": [0.4, 0.4], "b": "box", "length": 1.0}


def deep_pit_env(tmp_path, name, box=None, extra=()):
    """A deep pit with the box put in it (its handle clear of the rim) and settled on the plate."""
    env = open_room(tmp_path, [DEEP, {**FILLER, "pos": [0.8, 0.0], **(box or {})}, *extra], name=name)
    put_box(env, [0.0025, 0.0, -0.3 + env.parts["box"].size / 2])
    body_steps(env, 0.6)
    return env


def put_box(env, xyz, tilt_deg=0.0):
    box = env.parts["box"]
    a = env.model.jnt_qposadr[env.model.body_jntadr[box._bid]]
    t = math.radians(tilt_deg) / 2
    env.data.qpos[a:a + 7] = [*xyz, math.cos(t), math.sin(t), 0.0, 0.0]
    env.data.qvel[env.model.jnt_dofadr[env.model.body_jntadr[box._bid]]:][:6] = 0.0
    mujoco.mj_forward(env.model, env.data)


def test_deep_pit_with_a_flush_box_is_floor(tmp_path):
    from mujoco_puzzle_gym.logic import live_free
    env = deep_pit_env(tmp_path, "pitfill")
    pit = env.parts["pit"]
    assert pit.filler_top() == pytest.approx(0.0, abs=0.002)
    assert env.floor_height((0.0, 0.0))[0] == pytest.approx(0.0, abs=0.002)   # fall height from the box top
    assert pit.hole_now(env.grid) is None
    assert live_free(env.grid, env.walls, env.parts, 0.3)[env.grid.cell((0.0, 0.0))]
    env._place_body((0.0, 0.0, 0.0))                          # standing on the box: no fall, no pit fail
    assert not body_steps(env, 1.0) and not pit.failed()
    env.close()


def test_deep_pit_is_a_hole_again_when_the_box_leaves(tmp_path):
    env = deep_pit_env(tmp_path, "pitunfill")
    pit = env.parts["pit"]
    put_box(env, [0.8, 0.0, 0.15])                            # out on the floor
    assert pit.filler_top() is None and pit.hole_now(env.grid) is not None
    assert env.floor_height((0.0, 0.0))[0] is None
    put_box(env, [0.0025, 0.0, -0.15], tilt_deg=8.0)          # back in, tilted more than 5 deg
    assert pit.filler_top() is None
    put_box(env, [0.0025, 0.0, -0.15], tilt_deg=3.0)
    assert pit.filler_top() is not None
    put_box(env, [0.0025, 0.0, 0.05])                         # held up: top 0.2 m above the floor
    assert pit.filler_top() is None
    put_box(env, [0.8, 0.0, 0.15])
    _, _, terminated, _, info = drop_in(env)                  # the open deep pit still fails the episode
    assert terminated and info["failed"] == ["pit"]
    env.close()


def test_deep_pit_box_too_short_does_not_fill(tmp_path):
    env = deep_pit_env(tmp_path, "pitshort", box={"size": 0.2})
    assert env.parts["pit"].filler_top() is None and env.parts["pit"].hole_now(env.grid) is not None
    env.close()


def test_deep_pit_filled_state_follows_snapshots(tmp_path):
    env = deep_pit_env(tmp_path, "pitsnap")
    pit = env.parts["pit"]
    full = env.save_snapshot()
    put_box(env, [0.8, 0.0, 0.15])
    empty = env.save_snapshot()
    env.restore_snapshot(full)
    assert pit.filler_top() is not None and pit.hole_now(env.grid) is None
    env.restore_snapshot(empty)
    assert pit.filler_top() is None and pit.hole_now(env.grid) is not None
    env.close()


def test_pit_without_depth_is_unchanged(tmp_path):
    """No `depth`: no plate (the box falls to the catch plane), never filled, a hole for planning."""
    from mujoco_puzzle_gym.logic import free_mask
    raw_pit = {k: v for k, v in DEEP.items() if k != "depth"}
    env = open_room(tmp_path, [raw_pit, {**FILLER, "pos": [0.8, 0.0]}], name="pitnodepth")
    put_box(env, [0.0025, 0.0, -0.15])
    body_steps(env, 1.0)
    pit = env.parts["pit"]
    assert pit.cubes == [] and not pit.fillable and pit.filler_top() is None
    assert env.data.xpos[env.parts["box"]._bid][2] < -0.2           # below where a plate would hold it
    assert pit.hole_now(env.grid) is not None
    assert not free_mask(env.grid, env.walls, env.parts, set())[env.grid.cell((0.0, 0.0))]
    assert not any(env.model.geom(g).name.startswith("part/pit/plate") for g in range(env.model.ngeom))
    env.close()


@pytest.mark.parametrize("box, extra, fillable", [({}, (ROPE,), True),
                                                  ({}, (), False),                      # no rope can tie it
                                                  ({"hitch": False}, (), False),
                                                  ({"size": 0.25}, (ROPE,), False),     # not flush
                                                  ({"handle_size": 0.05}, (ROPE,), False)])   # wider than the opening
def test_deep_pit_is_fillable_for_the_static_order(tmp_path, box, extra, fillable):
    from mujoco_puzzle_gym.logic import free_mask
    env = open_room(tmp_path, [DEEP, {**FILLER, "pos": [0.8, 0.0], **box}, *extra], name="pitorder")
    assert env.parts["pit"].fillable == fillable
    assert free_mask(env.grid, env.walls, env.parts, set())[env.grid.cell((0.0, 0.0))] == fillable
    env.close()


@pytest.mark.parametrize("size, why", [([0.31, 0.345], None),
                                       ([0.30, 0.345], "must be 0-4 cm wider"),       # narrower than the box
                                       ([0.31, 0.35], "must be 0-4 cm wider"),        # 4.5 cm wider
                                       ([0.4, 0.4], "must be 0-4 cm wider")])
def test_pit_opening_fit(tmp_path, size, why):
    env = open_room(tmp_path, [{**DEEP, "size": size}, {**FILLER, "pos": [0.8, 0.0]}, ROPE], name="pitfit")
    got = env.parts["pit"].fits(env.parts["box"].extent())
    assert (got is None) if why is None else (why in got)
    env.close()

# ---- slippery / sticky floors ------------------------------------------------------------
@pytest.mark.parametrize("preset, slide", [("trap/slippery_floor", 0.2), ("trap/sticky_floor", 2.0)])
def test_floor_presets_set_friction(tmp_path, preset, slide):
    env = open_room(tmp_path, [{"name": "f", "preset": preset, "pos": [0.0, 0.5]}], name="fr")
    assert env.model.geom_friction[env.parts["f"]._gid][0] == pytest.approx(slide)


# ---- cannon ----------------------------------------------------------------------------------
def cannon_env(tmp_path, name, extra=(), spawn=(-1.0, -1.0), **cannon):
    c = {"name": "c", "type": "cannon", "pos": [0.0, 0.5], "yaw": 0.0, "aim": "fixed", "aim_pitch": 30.0,
         "power": 3.0, "period": 1.0, **cannon}
    return env_with(tmp_path, c, name, extra=extra, spawn=spawn)


def launches(env, seconds):
    """(time, launch velocity) of each shell launched in `seconds` of body steps."""
    c, seen, out = env.parts["c"], set(), []

    def look(e):
        for i, t in c.pool.out.items():
            if (i, t) not in seen:
                seen.add((i, t))
                out.append((t, c.pool.vel(e.data, i)))
    body_steps(env, seconds, every=look)
    return out


def test_cannon_fires_at_its_power_only_while_on(tmp_path):
    env = cannon_env(tmp_path, "cfire")
    assert launches(env, 2.0) == []
    env.parts["s"].value = 1.0
    shots = launches(env, 2.5)
    assert len(shots) == 3                                    # t = 0, 1, 2 s (period 1)
    assert np.diff([t for t, _ in shots]) == pytest.approx([1.0, 1.0], abs=0.03)
    v = shots[0][1]                                           # launch velocity (seen right after)
    assert np.linalg.norm(v) == pytest.approx(3.0) and v[2] == pytest.approx(3.0 * math.sin(math.radians(30)))
    assert abs(v[1]) < 1e-6


def test_cannon_fired_channel_and_burst(tmp_path):
    env = cannon_env(tmp_path, "cburst", burst=3, period=5.0)
    env.parts["s"].value = 1.0
    fired = []
    body_steps(env, 1.0, every=lambda e: fired.append(e.signals.get("c.fired")))
    assert sum(fired) == 3 and set(fired) == {0.0, 1.0}


def test_cannon_edge_fires_once_per_rising_edge(tmp_path):
    env = cannon_env(tmp_path, "cedge", fire="edge")
    env.parts["s"].value = 1.0
    assert len(launches(env, 3.0)) == 1                       # held: still one shot
    env.parts["s"].value = 0.0
    body_steps(env, 0.1)
    env.parts["s"].value = 1.0
    assert len(launches(env, 0.5)) == 1


def test_cannon_spread_stays_in_its_cone(tmp_path):
    env = cannon_env(tmp_path, "cspread", spread=10.0, period=0.2, aim_pitch=20.0, pool=20)
    env.parts["s"].value = 1.0
    shots = launches(env, 3.0)
    aim = np.array([math.cos(math.radians(20)), 0.0, math.sin(math.radians(20))])
    angles = []
    for _, v in shots:                                        # launch velocity (seen right after)
        angles.append(math.degrees(math.acos(np.clip(v @ aim / np.linalg.norm(v), -1, 1))))
    assert max(angles) <= 10.0 + 1e-6 and np.std(angles) > 1.0


def test_cannon_hits_count_the_body(tmp_path):
    env = cannon_env(tmp_path, "chits", spawn=(0.6, 0.5), aim="at_body", power=4.0, period=0.6,
                     lifetime=0.4)
    env.parts["s"].value = 1.0
    body_steps(env, 1.0)
    assert env.signals.get("c.hits") >= 1 and env.parts["c"].hits == env.signals.get("c.hits")


def test_shells_never_press_buttons_plates_or_zones(tmp_path):
    parts = [{"name": "plate", "type": "pressure", "pos": [0.6, 0.5], "trigger": "any", "min_mass": 0.01,
              "size": [0.6, 0.6]},
             {"name": "z", "type": "zone", "pos": [0.6, 0.5], "size": [0.6, 0.6], "trigger": "any"},
             {"name": "b", "type": "button", "pos": [0.6, 0.5], "size": [0.6, 0.6], "trigger": "any"}]
    env = cannon_env(tmp_path, "cnot", extra=parts, aim_pitch=45.0, power=2.4, period=0.3,
                     shell_mass=0.5, lifetime=3.0)
    env.parts["s"].value = 1.0
    c, on, over = env.parts["c"], [], []

    def look(e):
        on.append([e.signals.get(n) for n in ("plate", "z", "b")])
        over.extend(abs(p[0] - 0.6) < 0.3 and p[2] < 0.1 for p in (c.pool.pos(e.data, i) for i in c.pool.out))
    body_steps(env, 2.0, every=look)
    assert any(over)                                          # shells land and roll across them
    assert not np.any(on)


def test_pool_reuses_the_oldest_and_parks_after_lifetime(tmp_path):
    env = cannon_env(tmp_path, "cpool", pool=2, period=0.3, lifetime=2.0)
    c = env.parts["c"]
    env.parts["s"].value = 1.0
    body_steps(env, 0.35)
    assert sorted(c.pool.out) == [0, 1]
    first = dict(c.pool.out)
    body_steps(env, 0.3)
    assert len(c.pool.out) == 2 and c.pool.out[0] > first[0]          # oldest reused
    env.parts["s"].value = 0.0
    body_steps(env, 2.1)
    assert c.pool.out == {}
    for i in range(2):
        assert c.pool.pos(env.data, i)[2] == pytest.approx(-5.0)
        assert env.model.geom_contype[c.pool.gids[i]] == 0


def test_snapshot_restores_shells_in_flight(tmp_path):
    # fixed aim, body away: the walking body's controller memory is not in the snapshot (as before)
    env = cannon_env(tmp_path, "csnap", period=0.25, burst=2, pool=4, power=4.0)
    c = env.parts["c"]
    env.parts["s"].value = 1.0
    body_steps(env, 0.3)
    snap = env.save_snapshot()
    keep = (dict(c.pool.out), env.data.qpos.copy(), c.hits, env.model.geom_contype[c.pool.gids].copy(),
            env.model.body_gravcomp[c.pool.bids].copy())
    rec = []
    body_steps(env, 0.6, every=lambda e: rec.append((e.data.qpos.copy(), e.parts["c"].hits)))
    env.restore_snapshot(snap)
    assert c.pool.out == keep[0] and c.hits == keep[2]
    assert np.allclose(env.data.qpos, keep[1])
    assert np.array_equal(env.model.geom_contype[c.pool.gids], keep[3])
    assert np.array_equal(env.model.body_gravcomp[c.pool.bids], keep[4])
    again = []
    body_steps(env, 0.6, every=lambda e: again.append((e.data.qpos.copy(), e.parts["c"].hits)))
    shells = [q for i in range(c.pool.size) for q in range(c.pool.qadr[i], c.pool.qadr[i] + 3)]
    assert np.abs(again[-1][0][shells] - rec[-1][0][shells]).max() < 0.01
    assert again[-1][1] == rec[-1][1]
    env.reset(seed=0)
    assert c.pool.out == {} and c.hits == 0


def test_shell_mass_is_fixed_at_build(tmp_path):
    from mujoco_puzzle_gym.schema import SchemaError
    env = cannon_env(tmp_path, "cmass", shell_mass=0.3)
    c = env.parts["c"]
    assert env.model.body_mass[c.pool.bids].tolist() == pytest.approx([0.3] * c.pool.size)
    with pytest.raises(SchemaError, match="cannot change at episode start"):
        c.set_value(env.model, env.data, "shell_mass", 0.2, episode=True)


# ---- target (R5) ------------------------------------------------------------------------------
def target_env(tmp_path, name, **target):
    t = {"name": "t", "preset": "input/target", "pos": [0.8, 0.5], "yaw": 180.0, "height": 0.2,
         "size": [0.3, 0.3], **target}
    return cannon_env(tmp_path, name, extra=[t], aim_pitch=5.0, power=4.0, fire="edge", height=0.2)


def test_target_counts_shell_hits(tmp_path):
    env = target_env(tmp_path, "tgt")
    body_steps(env, 0.2)
    assert env.signals.get("t") == 0.0
    env.parts["s"].value = 1.0
    body_steps(env, 0.5)
    assert env.signals.get("t") == 1.0 and env.signals.get("t.hits") == 1.0


def test_target_ignores_slow_shells_and_the_body(tmp_path):
    env = target_env(tmp_path, "tslow", min_speed=10.0)
    env.parts["s"].value = 1.0
    body_steps(env, 0.5)
    assert env.signals.get("t") == 0.0 and env.signals.get("t.hits") == 0.0
    env2 = open_room(tmp_path, [{"name": "t", "preset": "input/target", "pos": [-0.75, -1.0], "yaw": 180.0,
                                 "height": 0.1}], spawn=(-1.0, -1.0), name="tbody")
    for _ in range(60):                                       # walk into the board
        env2.step(np.array([1.0, 0.0, 0.0], np.float32))
    assert env2.signals.get("t") == 0.0 and env2.signals.get("t.hits") == 0.0


def test_target_pulse_mode(tmp_path):
    env = target_env(tmp_path, "tpulse", mode="pulse")
    env.parts["s"].value = 1.0
    seen = []
    body_steps(env, 0.6, every=lambda e: seen.append(e.signals.get("t")))
    assert sum(seen) == 1.0


# ---- turret ballistic check (R16) ------------------------------------------------------------
def turret_room(tmp_path, target_pos, target_yaw, name, spread=0.0):
    raw = {"name": name, "size": [3.0, 3.0], "spawn": {"pos": [-1.0, -1.0], "yaw": 0.0},
           "parts": [SIG, {"name": "fire", "type": "t_signal", "pos": [1.0, 1.0]},
                     {"name": "tur", "preset": "trap/cannon_turret", "pos": [0.0, 0.0], "in": ["fire"],
                      "power": 4.0, "spread": spread,
                      "bind": {"aim_yaw": {"from": "s", "map": [0, 1, -30, 30]},
                               "aim_pitch": {"from": "s", "map": [0, 1, 0, 40]}}},
                     {"name": "t", "preset": "input/target", "pos": target_pos, "yaw": target_yaw},
                     {"name": "goal", "type": "goal", "pos": [1.3, -1.3], "radius": 0.1}]}
    path = tmp_path / f"{name}.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


def test_turret_check_passes_when_an_aim_hits(asset_dir, tmp_path):
    res = check(turret_room(tmp_path, [1.0, 0.3], 180.0, "tur_ok"), static_only=True)
    assert res["parts"]["ok"], res["parts"]


@pytest.mark.parametrize("pos, why", [([-1.0, 0.0], "behind"), ([2.5, 0.0], "too far for 4 m/s")])
def test_turret_check_fails_without_a_hitting_aim(asset_dir, tmp_path, pos, why):
    res = check(turret_room(tmp_path, pos, 0.0, "tur_bad"), static_only=True)
    assert not res["parts"]["ok"] and "parts" in res["failed"]
    assert "target 't'" in res["parts"]["reason"] and "aim_yaw -30..30" in res["parts"]["reason"]


def test_turret_check_needs_the_whole_cone(asset_dir, tmp_path):
    ok = check(turret_room(tmp_path, [1.0, 0.0], 180.0, "tur_cone0"), static_only=True)
    wide = check(turret_room(tmp_path, [1.0, 0.0], 180.0, "tur_cone", spread=15.0), static_only=True)
    assert ok["parts"]["ok"] and not wide["parts"]["ok"]
    assert "whole cone" in wide["parts"]["reason"]


def test_turret_check_follows_the_physics(tmp_path):
    """An aim the check calls a hit hits the board in the simulation."""
    env = open_room(tmp_path, [SIG, {"name": "fire", "type": "t_signal", "pos": [1.0, 1.0]},
                               {"name": "tur", "preset": "trap/cannon_turret", "pos": [0.0, 0.0],
                                "in": ["fire"], "power": 4.0, "aim_yaw": 17.0, "aim_pitch": 16.0},
                               {"name": "t", "preset": "input/target", "pos": [1.0, 0.3], "yaw": 180.0}],
                    name="tur_sim")
    tur, t = env.parts["tur"], env.parts["t"]
    assert tur.can_hit(t, env.parts)
    env.parts["fire"].value = 1.0
    body_steps(env, 0.6)
    assert env.signals.get("t.hits") == 1.0


# ---- record: what knocks the standing body over (slow) ----------------------------------------------
@pytest.mark.slow
@pytest.mark.parametrize("mass", [0.02, 0.05, 0.1, 0.2, 0.4])
def test_knockdown_record(tmp_path, mass):
    """Record only (plan step 12): one shell at the standing body from the side and the
    front (0.4 m), three firing moments each; falls / hits per launch speed."""
    rows = []
    for side, (pos, yaw) in {"side": ([0.0, 0.4], -90.0), "front": ([0.4, 0.0], 180.0)}.items():
        env = open_room(tmp_path, [SIG, {"name": "c", "type": "cannon", "pos": pos, "yaw": yaw,
                                         "aim": "at_body", "fire": "edge", "in": ["s"], "height": 0.15,
                                         "lifetime": 0.3, "shell_mass": mass}],
                        spawn=(0.0, 0.0), name=f"knock_{side}")
        c = env.parts["c"]
        for v in (3.0, 4.0, 5.0, 6.0, 8.0):
            falls = hits = 0
            for wait in (0.5, 0.7, 0.9):
                env.reset(seed=0, options={"layout": env.sample_layout(None)})
                c.set_value(env.model, env.data, "power", v)
                body_steps(env, wait)
                env.parts["s"].value = 1.0
                fell = body_steps(env, 2.5)
                env.parts["s"].value = 0.0
                hits += c.hits > 0
                falls += bool(fell and c.hits)
            rows.append(f"{side} {v:g} m/s: {falls}/{hits}")
        env.close()
    print(f"\nR16 record: shell {mass} kg, falls/hits: " + "; ".join(rows))


def test_lead_hits_a_moving_target_where_it_will_be(tmp_path):
    """`aim: lead` against a target moving at constant velocity: the shell comes down at the
    target's height within 3 cm of where the target is then (shell radius 2.5 cm plus a few mm
    of integration error; the aim is a fixed point over the flight time and the muzzle-tip
    launch point that converges well within that).
    `at_body` misses by about speed x flight time (~0.2 m)."""
    from types import SimpleNamespace
    misses = {}
    for aim in ("lead", "at_body"):
        env = cannon_env(tmp_path, f"lead_{aim}", aim=aim, power=4.0, pos=[0.0, 0.0])
        c = env.parts["c"]
        p0, v = np.array([1.5, 0.0, 0.15]), np.array([0.0, 0.5, 0.0])
        body = SimpleNamespace(pos=p0.copy(), yaw=0.0, lin_vel=v.copy())
        c._shoot(env.model, env.data, body)
        i, t0 = next(iter(c.pool.out.items()))
        prev = c.pool.pos(env.data, i)
        while True:
            mujoco.mj_step(env.model, env.data)
            q = c.pool.pos(env.data, i)
            if q[2] < p0[2] and c.pool.vel(env.data, i)[2] < 0:
                break
            prev = q
        a = (prev[2] - p0[2]) / (prev[2] - q[2])                     # interpolate to the target height
        land = prev + a * (q - prev)
        t = env.data.time - t0 - (1 - a) * env.model.opt.timestep
        misses[aim] = float(np.linalg.norm((land - (p0 + v * t))[:2]))
    assert misses["lead"] < 0.03
    assert misses["at_body"] > 0.1 and misses["at_body"] > 5 * misses["lead"]
