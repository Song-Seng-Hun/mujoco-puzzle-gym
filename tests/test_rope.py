"""R31 tow rope, hitch station and anchor.

- a room without a rope builds as before (no tendon, pairs, tow site or equalities added;
  a `hitch: true` object keeps its friction priority);
- slack inside `length` (no force, the sled stays), a pull beyond it (the sled follows,
  knots at most `length` + 1 cm apart);
- a tie farther than `length` fails with `attach_failed`;
- the hitch station, the channels and the beak each tie and untie; a tied knot sits
  on its hitch (within 1 cm, also on a random layout); off the pad or moving: nothing;
- a lever jump of `length` reels at `reel_speed` (tension bounded); a shorter bound
  length brings the sled to the anchor;
- warps take the towed sled along (same relative place); an object warp, a spring
  kick, a dispenser recycle and `set_addon` untie first;
- knots press no button; above `max_tension` the rope snaps (`on_snap: fail` fails);
- a snapshot keeps the ties; `reset()` ties the default ends again; same seed, same run;
- NPC leash: zero gains and no leash limits while tied, back once untied (inside);
- obs `tow`, `demo:` `tow` / `untow` and the solver steps, static checks;
- `demo:` `drag <rope> <part>`: parsed (errors for a non-rope, a part without a floor
  area, a wrong arity), the towed hitch (also down a chain of ropes), the aim beyond
  the part, done once the hitch is inside or comes loose; a short run drags the sled
  into a zone; a drag target needs no `grab` action; a drag leg gets the path warnings.
"""

import mujoco
import numpy as np
import pytest
import yaml
from rooms import body_steps, open_room, stand_at, tick

from mujoco_puzzle_gym.check import check_parts
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.parts.base import Part
from mujoco_puzzle_gym.parts.device.rope import KNOT_HZ, tow_path_warnings
from mujoco_puzzle_gym.registry import register_part
from mujoco_puzzle_gym.room import RoomError, parse_demo
from mujoco_puzzle_gym.solver import DRAG_PAST, STAND, ScriptedSolver

SLED = {"name": "sled", "type": "cube", "pos": [-1.30, 0.0], "size": 0.1, "mass": 0.3, "friction": 0.1,
        "hitch": True}
ROPE = {"name": "rope", "type": "rope", "pos": [0.5, -1.0], "a": "body", "b": "sled", "length": 0.4}
SPAWN = (-1.0, 0.0)


class Src(Part):
    """Test-only channel source: writes `value` every step."""
    stage = "input"
    emits = True
    has_body = False
    value = 0.0

    def update(self, model, data, body):
        self.write(self.value)


register_part("t_rsrc", Src, replace=True)


@pytest.fixture
def make(asset_dir, tmp_path):
    envs = []

    def _make(parts, spawn=SPAWN, **kw):
        env = open_room(tmp_path, parts, spawn=spawn, size=(4.0, 3.0), **kw)
        envs.append(env)
        return env

    yield _make
    for e in envs:
        e.close()


def rope_of(env, name="rope"):
    return env.parts[name]


def knot_gap(env, rope, end):
    """Distance [m] of end `end`'s knot from its hitch site."""
    h = rope.tied_hitch(end)
    return float(np.linalg.norm(env.data.site_xpos[rope.knots[end].sid] - env.data.site_xpos[h.sid]))


def knots_apart(env, rope):
    a, b = rope.knots
    return float(np.linalg.norm(env.data.site_xpos[a.sid] - env.data.site_xpos[b.sid]))


def walk(env, seconds, vx=0.4):
    """Body steps walking straight ahead (heading held at its start)."""
    y0 = float(env.body_state.pos[1])
    yaw0 = float(env.body_state.yaw)
    out = {"apart": []}

    def every(e):
        s = e.read_body()
        out["apart"].append(knots_apart(e, rope_of(e)))
        e._wz = float(np.clip(-2.0 * (s.yaw - yaw0) - 3.0 * (s.pos[1] - y0), -1, 1))

    env._wz = 0.0
    fell = False
    for _ in range(int(round(seconds / env.body.control_dt))):
        fell |= body_steps(env, env.body.control_dt, cmd=np.array([vx, 0.0, env._wz]), every=every)
    out["fell"] = fell
    return out


# ---- building -------------------------------------------------------------------------------
def test_room_without_rope_builds_as_before(make):
    plain = make([SLED])
    m = plain.model
    assert m.ntendon == 0 and m.npair == 0
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, plain.body.tow_site()) == -1
    assert m.neq == 1                                  # the beak weld of the cube only
    assert m.geom_priority[m.geom("part/sled/box").id] == 0
    env = make([SLED, ROPE])
    m2 = env.model
    hitches = len(rope_of(env).hitches)
    assert hitches == 2 and m2.ntendon == 1 and m2.neq == 1 + 2 + 2 * hitches   # + knot welds + ties
    assert m2.npair == 2 and m2.nbody == m.nbody + 2
    assert m2.geom_priority[m2.geom("part/sled/box").id] == 2
    assert set(env.parts) >= {"rope/a", "rope/b"} and list(env.parts)[0] == "_beak"


def test_bad_hitch_names_and_cap(make):
    with pytest.raises(RoomError, match="not a hitch"):
        make([{**ROPE, "b": "goal"}])
    with pytest.raises(RoomError, match="no hitch"):
        make([{**ROPE, "b": None, "tied": ["a", "b"]}])
    with pytest.raises(RoomError, match="max_hitches"):
        make([SLED, {**ROPE, "max_hitches": 1}])


# ---- slack and pull -------------------------------------------------------------------------
def test_slack_holds_nothing_and_a_pull_drags_the_sled(make):
    env = make([{**SLED, "pos": [-1.25, 0.0]}, ROPE])
    rope, sled = rope_of(env), env.parts["sled"]
    assert rope.tied == [0, 1] and knots_apart(env, rope) < 0.4 - 0.05
    x0 = sled.pos(env.data)[0]
    body_steps(env, 1.0)                                 # standing: slack, no force
    assert env.signals.get("rope.tension") == 0.0
    assert abs(sled.pos(env.data)[0] - x0) < 0.002
    run = walk(env, 4.0)
    assert not run["fell"]
    assert sled.pos(env.data)[0] - x0 > 0.3              # it follows the body
    assert max(run["apart"]) <= 0.4 + 0.01                # knots never stretch past length + 1 cm
    assert knot_gap(env, rope, 0) < 0.01 and knot_gap(env, rope, 1) < 0.01


# ---- tying ------------------------------------------------------------------------------------
def test_tie_beyond_length_fails(make):
    env = make([{"name": "src", "type": "t_rsrc", "pos": [0, 0]}, {**SLED, "pos": [-1.7, 0.0]},
                {**ROPE, "tied": ["a"], "attach_b": "src"}])
    rope = rope_of(env)
    assert rope.tied == [0, None]
    env.parts["src"].value = 1.0
    tick(env)
    assert rope.tied == [0, None]
    assert env.signals.get("rope.attach_failed") == 1.0 and env.signals.get("rope.attached_b") == 0.0
    tick(env)
    assert env.signals.get("rope.attach_failed") == 0.0   # a pulse


def test_channels_tie_and_untie(make):
    env = make([{"name": "src", "type": "t_rsrc", "pos": [0, 0]},
                {"name": "off", "type": "t_rsrc", "pos": [0, 0]},
                SLED, {**ROPE, "b": None, "tied": ["a"], "rest": [[-1.2, 0.3], [-1.3, 0.1]],
                       "attach_b": "src", "detach_b": "off"}])
    rope = rope_of(env)
    kb = rope.knots[1]
    assert rope.tied == [0, None] and abs(env.data.xpos[kb._bid][2] - KNOT_HZ) < 1e-6
    env.parts["src"].value = 1.0                          # no `b`: the nearest hitch near the knot
    tick(env)
    assert rope.tied_hitch(1).name == "sled" and env.signals.get("rope.attached_b") == 1.0
    mujoco.mj_forward(env.model, env.data)
    assert knot_gap(env, rope, 1) < 0.01
    env.parts["off"].value = 1.0
    tick(env)
    assert rope.tied[1] is None and env.signals.get("rope.attached_b") == 0.0
    assert env.data.qpos[kb._q + 2] == pytest.approx(KNOT_HZ)       # laid on the floor


def station_room(make, mode="attach", **st):
    return make([{**SLED, "pos": [-1.22, 0.0]}, {**ROPE, "a": None, "tied": ["b"], "rest": [[-0.9, -0.4], [-1.22, 0.0]]},
                 {"name": "pad", "type": "hitch_station", "pos": [-1.0, 0.0], "mode": mode, **st}])


def test_station_ties_on_the_pad_when_slow(make):
    env = station_room(make)
    rope = rope_of(env)
    stand_at(env, -1.0, 0.8)                              # off the pad
    assert rope.tied == [None, 1]
    stand_at(env, -1.0, 0.0)                              # on the pad, standing
    assert rope.on_body and rope.tied_hitch(0).name == "body" and rope.tied_hitch(1).name == "sled"
    mujoco.mj_forward(env.model, env.data)
    assert knot_gap(env, rope, 0) < 0.01
    tick(env)
    assert env.signals.get("pad.towing") == 1.0 and env.signals.get("body.towing") == 1.0


def test_station_ignores_a_moving_body_and_detaches(make):
    env = station_room(make, mode="toggle", cooldown=0.0)
    rope = rope_of(env)
    stand_at(env, -1.0, 0.8)
    env.body.teleport(env.data, (-1.0, 0.0, 0.0))
    env.body.kick(env.data, [0.3, 0.0, 0.0])              # on the pad at 0.3 m/s > max_speed
    tick(env)
    assert not rope.on_body
    env.data.qvel[:] = 0.0
    tick(env)                                             # slow now: it ties (toggle)
    assert rope.on_body
    stand_at(env, -1.0, 0.8)
    stand_at(env, -1.0, 0.0)                              # enter again: toggle unties the body's end
    assert not rope.on_body and rope.tied == [None, 1]


def test_station_ties_the_loose_end_of_a_rope_on_the_body(make):
    """`tow_yard`: the rope stays on the body after its post end comes loose; an attach
    (or toggle) station ties that loose end to the hitch next to the pad."""
    for mode in ("attach", "toggle"):
        env = make([{**SLED, "pos": [-1.0, 0.25]}, {**ROPE, "b": None, "tied": ["a"], "rest": [[-1.5, 0.5], [-1.6, -0.4]]},
                    {"name": "pad", "type": "hitch_station", "pos": [-1.0, 0.0], "mode": mode}])
        rope = rope_of(env)
        stand_at(env, -1.0, 0.8)
        assert rope.tied == [0, None] and rope.on_body
        stand_at(env, -1.0, 0.0)
        assert rope.tied_hitch(0).name == "body" and rope.tied_hitch(1).name == "sled"


def test_beak_ties_and_unties_a_knot(make):
    env = make([{**SLED, "pos": [0.5, 0.5]}, {**ROPE, "b": None, "tied": ["a"], "rest": [[0, 0], [-0.8, -0.2]]}])
    rope, sled = rope_of(env), env.parts["sled"]
    kb = rope.knots[1]
    beak = env.parts["_beak"]
    stand_at(env, -1.0, 0.0)
    tip = env.data.site_xpos[beak._site].copy()
    kb.put(env.data, tip - np.array([0.0, 0.0, kb.ring_z]))  # ring at the beak tip
    env.world.grab = 1.0
    tick(env)
    assert beak.held is not None and beak.objects[beak.held] is kb
    sled.put(env.data, env.data.site_xpos[kb.sid].copy())    # the sled's hitch at the held knot
    mujoco.mj_forward(env.model, env.data)
    env.world.grab = -1.0
    tick(env)                                             # let go within clip_radius: tied
    assert rope.tied_hitch(1) is not None and rope.tied_hitch(1).name == "sled"
    rid = int(env.model.body_rootid[kb._bid])
    env.world.held = {rid}                                # the beak holds the tied knot
    steps = int(round(rope.cfg["untie_time"] / env.body.control_dt))
    for _ in range(steps - 1):
        tick(env)
    assert rope.tied[1] is not None
    tick(env)
    assert rope.tied[1] is None                           # held untie_time: loose
    env.world.held = set()                                # let go right there, within clip_radius
    tick(env)
    tick(env)
    assert rope.tied[1] is None                           # it stays loose (no clip on this release)
    env.world.held = {rid}                                # a new hold, let go there: ties again
    tick(env)
    env.world.held = set()
    tick(env)
    assert rope.tied_hitch(1).name == "sled"


def test_tied_knots_sit_on_hitches_on_random_layouts(make):
    env = make([{**SLED, "random": [0.02, 0.05]}, ROPE])
    rope = rope_of(env)
    for seed in range(3):
        env.reset(seed=seed)
        assert rope.tied == [0, 1]
        assert knot_gap(env, rope, 0) < 0.01 and knot_gap(env, rope, 1) < 0.01


# ---- length -------------------------------------------------------------------------------------
REEL = [{"name": "lev", "type": "t_rsrc", "pos": [0, 0]},
        {"name": "post", "type": "anchor", "pos": [0.0, 0.0]},
        {"name": "sled", "type": "cube", "pos": [-0.43, 0.0], "size": 0.1, "mass": 0.3, "friction": 0.5,
         "hitch": True},
        {"name": "rope", "type": "rope", "pos": [0, -0.5], "a": "post", "b": "sled", "length": 0.4,
         "max_tension": 100.0, "bind": {"length": {"from": "lev", "map": [0, 1, 0.1, 0.4]}}}]


def _reel(env, seconds=9.0):
    rope = rope_of(env)
    env.parts["lev"].value = 1.0
    body_steps(env, 0.5)
    x0 = env.parts["sled"].pos(env.data)[0]
    peak = []
    env.parts["lev"].value = 0.0                          # the lever jumps: target 0.4 -> 0.1 at once
    body_steps(env, seconds, every=lambda e: peak.append(rope.tension))
    return x0, env.parts["sled"].pos(env.data)[0], max(peak), rope


def test_lever_jump_reels_slowly(make):
    env = make(REEL, spawn=(1.0, 1.0))
    x0, x1, peak, rope = _reel(env)
    assert rope.cur == pytest.approx(0.1)
    assert x1 - x0 > 0.25                                 # the sled came to the anchor
    weight = 0.5 * 0.3 * 9.81                             # sliding friction x weight 1.47 N
    assert peak < 1.5 * weight                            # measured 1.86 N peak at 0.05 m/s
    fast = make([*REEL[:3], {**REEL[3], "reel_speed": 2.0}], spawn=(1.0, 1.0))
    assert _reel(fast)[2] > 3 * weight                    # without reeling: 13.7 N


# ---- teleports ----------------------------------------------------------------------------------
def test_warp_takes_the_sled_along(make):
    env = make([SLED, ROPE])
    rope, sled = rope_of(env), env.parts["sled"]
    s = env.read_body()

    def rel():
        b = env.read_body()
        d = sled.pos(env.data)[:2] - b.pos[:2]
        c, sn = np.cos(b.yaw), np.sin(b.yaw)
        return np.array([c * d[0] + sn * d[1], -sn * d[0] + c * d[1]])

    before = rel()
    env.warp("body", (0.8, 0.5, np.pi / 2))
    assert rope.tied == [0, 1]
    assert np.linalg.norm(rel() - before) < 0.01
    assert knot_gap(env, rope, 0) < 0.01 and knot_gap(env, rope, 1) < 0.01
    assert s is not None


def test_warp_unties_an_anchor_end(make):
    env = make([{"name": "post", "type": "anchor", "pos": [-1.3, 0.0]},
                {**ROPE, "b": "post"}])
    rope = rope_of(env)
    env.warp("body", (0.8, 0.5, 0.0))
    assert rope.tied[0] == 0 and rope.tied[1] is None     # the rope goes along, off the post
    assert knot_gap(env, rope, 0) < 0.01


def test_object_warp_and_spring_kick_untie(make):
    env = make([SLED, ROPE])
    rope, sled = rope_of(env), env.parts["sled"]
    rid = int(env.model.body_rootid[sled._bid])
    env.warp(rid, (0.5, 0.5, 0.0))
    assert rope.tied == [0, None]
    env.reset(seed=0, options={"layout": env.sample_layout(None)})
    assert rope.tied == [0, 1]
    env.world.requests.append(("kick", sled, rid, np.array([1.0, 0.0, 0.0])))
    env._run_requests()
    assert rope.tied == [0, None]


def test_set_addon_unties_the_body_first(make):
    env = make([SLED, ROPE, {"name": "st", "preset": "device/addon_roller", "pos": [0.8, 0.8]}])
    rope = rope_of(env)
    assert rope.tied == [0, 1]
    env.world.requests.append(("addon", env.parts["st"], "roller"))
    env._run_requests()
    assert env.body.addon == "roller" and rope.tied == [None, 1]


def test_dispenser_recycle_unties_first(make):
    env = make([{"name": "src", "type": "t_rsrc", "pos": [0, 0]},
                {"name": "disp", "type": "dispenser", "pos": [0.5, 0.0], "button": "none", "in": ["src"],
                 "stock": 1, "recycle": True, "item": {"preset": "beak/cube_weight", "hitch": True}},
                {**ROPE, "b": None, "tied": ["a"], "attach_b": "src", "hitch_radius": 0.4,
                 "rest": [[0.0, 0.0], [0.6, 0.0]]}])
    rope, disp = rope_of(env), env.parts["disp"]
    stand_at(env, 0.8, 0.0)
    for k in range(4):                                    # edge 1 hands the copy out, edge 2 ties it
        env.parts["src"].value = float(k % 2 == 0)
        tick(env)
    assert rope.tied_hitch(1).name == "disp/0"
    disp.store(env.model, env.data, 0)
    assert rope.tied[1] is None


# ---- triggers, snapping ---------------------------------------------------------------------------
def test_knots_press_no_button(make):
    env = make([{"name": "btn", "preset": "input/floor_momentary", "pos": [-0.3, -0.6]},
                {**ROPE, "a": None, "b": None, "rest": [[-0.3, -0.6], [-0.35, -0.55]]}])
    body_steps(env, 0.5)
    rope = rope_of(env)
    assert np.linalg.norm(env.data.xpos[rope.knots[0]._bid][:2] - [-0.3, -0.6]) < 0.01
    assert env.signals.get("btn") == 0.0


def test_overload_snaps(make):
    env = make([{**SLED, "pos": [-1.25, 0.0], "mass": 0.5, "friction": 0.5},
                {**ROPE, "on_snap": "fail"}])
    rope = rope_of(env)
    walk(env, 2.0)
    assert rope.snapped and rope.tied == [None, None]
    assert env.signals.get("rope.snapped") == 1.0 and rope.failed()


# ---- snapshots, reset, determinism ---------------------------------------------------------------
def test_snapshot_restores_ties_and_reset_ties_defaults(make):
    env = make([{"name": "off", "type": "t_rsrc", "pos": [0, 0]}, SLED, {**ROPE, "detach_b": "off"}])
    rope = rope_of(env)
    body_steps(env, 0.3)
    snap = env.save_snapshot()
    env.parts["off"].value = 1.0
    body_steps(env, 0.1)
    assert rope.tied == [0, None]
    env.restore_snapshot(snap)
    assert rope.tied == [0, 1] and env.data.eq_active[rope._eq[1][1]] == 1
    env.parts["off"].value = 1.0
    body_steps(env, 0.1)
    env.parts["off"].value = 0.0
    env.reset(seed=3)
    assert rope.tied == [0, 1] and env.signals.get("rope.attached_b") == 1.0


def test_same_seed_same_run(make):
    env = make([SLED, ROPE])

    def run():
        env.reset(seed=5)
        out = []
        for _ in range(20):
            env.step(np.array([1.0, 0.0, 0.3]))
            out.append(env.parts["sled"].pos(env.data))
        return np.array(out)

    assert np.array_equal(run(), run())


# ---- NPC leash (R29) ---------------------------------------------------------------------------------
def test_npc_tied_is_dragged_with_zero_gains(make):
    npc = {"name": "bot", "preset": "npc/flee", "pos": [-1.3, 0.0], "leash": [-1.5, -0.3, -1.1, 0.3]}
    env = make([npc, {**ROPE, "b": "bot"}])
    rope, bot = rope_of(env), env.parts["bot"]
    m = env.model
    assert bot.towed and not m.actuator_gainprm[bot._act, 0].any()
    assert m.jnt_range[bot._j[0], 1] > 100
    walk(env, 3.0)
    assert bot.pos(env.data)[0] > -1.1                   # dragged out of its leash box
    env.world.untie(m, env.data, rope.tied_hitch(1).rid)
    assert not bot.towed and bot.returning and m.actuator_gainprm[bot._act, 0].all()
    body_steps(env, 6.0)
    assert -1.5 <= bot.pos(env.data)[0] <= -1.1 and not bot.returning
    assert m.jnt_range[bot._j[0], 1] < 1.0
    env.reset(seed=0, options={"layout": env.sample_layout(None)})
    assert bot.towed                                      # the default tie again


def test_towed_npc_stops_with_the_body(make):
    npc = {"name": "bot", "preset": "npc/flee", "pos": [-1.3, 0.0], "leash": [-1.5, -0.3, -1.1, 0.3]}
    env = make([npc, {**ROPE, "b": "bot"}])
    bot = env.parts["bot"]
    tension = []
    body_steps(env, 4.0, cmd=np.array([0.4, 0.0, 0.0]), every=lambda e: tension.append(rope_of(e).tension))
    assert np.mean(tension[100:]) < 0.3                   # measured 0.20 N mean at full walk (damping 1.0)
    assert np.linalg.norm(bot.vel(env.data)) > 0.1
    body_steps(env, 1.0)                                  # the body stops: no coasting on (measured 0.82 s)
    assert np.linalg.norm(bot.vel(env.data)) < 0.02
    env.world.untie(env.model, env.data, rope_of(env).tied_hitch(1).rid)
    assert env.model.dof_damping[bot._v[0]] == bot._damp0[0]


# ---- obs, demo, checks ------------------------------------------------------------------------------
def test_tow_obs(make):
    env = make([SLED, ROPE], task={"obs": ["tow"]})
    obs = env._obs()
    assert obs[0] == 1.0 and obs[2] < -0.2 and abs(obs[3]) < 0.05     # the sled behind
    env.world.untie(env.model, env.data, int(env.model.body_rootid[env.body.root_body_id(env.model)]))
    assert not env._obs().any()


def test_demo_tow_untow_parsed_and_solved(make):
    env = station_room(make, mode="toggle", cooldown=0.5)
    parts = env.parts
    assert parse_demo(["tow rope sled", "untow rope"], parts) == [("tow", "rope", "sled"), ("untow", "rope")]
    with pytest.raises(RoomError):
        parse_demo(["tow goal sled"], parts)
    with pytest.raises(RoomError):
        parse_demo(["tow rope goal"], parts)
    env.room.demo = [("tow", "rope", "sled"), ("untow", "rope")]
    solver = ScriptedSolver(env)
    env.reset(seed=0, options={"layout": env.sample_layout(None)})
    rope = rope_of(env)
    tied_at = None
    for k in range(200):                                  # measured: done after 83 steps (8.3 s)
        env.step(solver.act())
        if tied_at is None and rope.on_body:
            tied_at = k
        if tied_at is not None and not solver.in_demo:
            break
    assert tied_at is not None and not rope.on_body and not solver.in_demo


def wall_room(tmp_path, parts, walls=()):
    raw = {"name": "walls", "size": [3.0, 3.0], "spawn": {"pos": [-1.0, -0.8], "yaw": 0.0}, "walls": list(walls),
           "parts": [*parts, {"name": "goal", "type": "goal", "pos": [1.0, -0.8], "radius": 0.1}],
           "demo": ["tow rope sled", "goto goal"]}
    path = tmp_path / f"walls{len(list(tmp_path.iterdir()))}.yaml"
    path.write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(room=path)
    env.reset(seed=0)
    return env


def test_tow_path_warnings(asset_dir, tmp_path):
    sled = {**SLED, "pos": [-1.3, -0.8], "mass": 0.1}
    rope = {**ROPE, "pos": [0.0, 1.2]}
    gap = [{"from": [0.0, -1.5], "to": [0.0, -1.0]}, {"from": [0.0, -0.6], "to": [0.0, 1.5]}]
    door = [{"name": "btn", "preset": "input/floor_momentary", "pos": [-0.5, 1.0]},
            {"name": "door", "preset": "device/sliding_door", "pos": [0.0, -0.8], "in": ["btn"], "width": 0.4}]
    cases = [([sled, rope], (), None),                                       # straight: nothing
             ([sled, rope, *door], gap, None),                               # 0.1 m cube through 0.4 m door
             ([{"name": "sled", "type": "stick", "pos": [-1.4, -0.8], "length": 0.5, "hitch": True}, rope, *door],
              gap, "passes door 'door'"),                                    # 0.5 m stick: too wide
             ([sled, rope], [{"from": [0.0, -1.5], "to": [0.0, 0.6]}], "bends more than 90 deg")]  # hairpin
    for parts, walls, want in cases:
        env = wall_room(tmp_path, parts, walls)
        try:
            got = tow_path_warnings(env, env.parts["rope"])
        finally:
            env.close()
        assert (got == []) if want is None else (len(got) == 1 and want in got[0]), (want, got)


def test_static_checks(make):
    env = make([SLED, {**ROPE, "hitch_radius": 0.5}])
    why = env.parts["rope"].check_static(env)
    assert any("hitch_radius" in w for w in why)
    far = make([{**SLED, "pos": [-1.35, 0.0], "random": [0.05, 0.0]}, ROPE])
    assert any("apart" in w for w in far.parts["rope"].check_static(far))
    heavy = make([{**SLED, "mass": 0.3, "friction": 0.5}, ROPE])
    res = check_parts(heavy)
    assert res["ok"] and any("dragging" in w for w in res["warnings"])
    assert check_parts(make([SLED, ROPE]))["ok"]


# ---- drag (demo step) --------------------------------------------------------------------------
ZONE = {"name": "zone", "preset": "input/object_zone", "pos": [0.2, 0.0], "size": [0.4, 0.4], "objects": ["sled"]}


def put_obj(env, name, x, y):
    """Move free object `name` to (x, y) at its resting height, at rest."""
    p = env.parts[name]
    q = env.model.jnt_qposadr[env.model.body_jntadr[p._bid]]
    env.data.qpos[q:q + 2] = [x, y]
    mujoco.mj_forward(env.model, env.data)


def test_demo_drag_parsed(make):
    env = make([SLED, ROPE, ZONE])
    parts = env.parts
    assert parse_demo(["drag rope zone"], parts) == [("drag", "rope", "zone")]
    for bad, why in ((["drag rope"], "write `drag"), (["drag rope zone goal"], "write `drag"),
                     (["drag sled zone"], "not a rope"), (["drag rope goal"], "no floor area"),
                     (["drag rope sled"], "no floor area"), (["drag rope nope"], "no part named")):
        with pytest.raises(RoomError, match=why):
            parse_demo(bad, parts)


def test_drag_aims_beyond_the_part_and_ends_inside(make):
    env = make([SLED, ROPE, ZONE])
    rope, zone = rope_of(env), env.parts["zone"]
    env.room.demo = [("drag", "rope", "zone")]
    solver = ScriptedSolver(env)
    assert solver.towed_hitch(rope).name == "sled"
    act = solver.drag_act(rope, zone)
    aim = solver._drag["aim"]                             # on the line sled -> zone, past the zone
    sled_xy = env.parts["sled"].pos(env.data)[:2]
    gap = max(0.4, float(np.linalg.norm(env.body_state.pos[:2] - sled_xy)))
    assert aim[0] == pytest.approx(0.2 + gap + DRAG_PAST, abs=0.02) and abs(aim[1]) < 0.01
    assert act[0] == 1.0                                  # facing it: walks on
    put_obj(env, "sled", 0.2 - 0.2 + 0.03, 0.0)           # inside the zone, but within DRAG_MARGIN of its edge
    assert solver.drag_act(rope, zone) is not None
    put_obj(env, "sled", 0.1, 0.0)
    assert solver.drag_act(rope, zone) is None and solver._drag is None


def test_drag_without_a_tow_stands_and_a_loose_hitch_ends_it(make):
    env = make([SLED, {**ROPE, "tied": ["b"]}, ZONE])
    rope, zone = rope_of(env), env.parts["zone"]
    solver = ScriptedSolver(env)
    assert solver.towed_hitch(rope) is None
    assert np.array_equal(solver.drag_act(rope, zone), STAND) and solver._drag is None
    env.world.untie(env.model, env.data, rope.tied_hitch(1).rid)
    rope.tie(env.model, env.data, 0, rope._body_h, check=False)
    rope.tie(env.model, env.data, 1, rope._index["sled"], check=False)
    assert solver.drag_act(rope, zone) is not None and solver._drag is not None
    rope.untie(env.model, env.data, 1)                     # a detach channel let it go on the way
    assert solver.drag_act(rope, zone) is None and solver._drag is None


def test_towed_hitch_down_a_chain(make):
    c1 = {**SLED, "name": "c1", "pos": [-1.3, 0.0], "mass": 0.1}
    c2 = {**SLED, "name": "c2", "pos": [-1.6, 0.0], "mass": 0.1}
    env = make([c1, c2, {**ROPE, "name": "front", "b": "c1"},
                {**ROPE, "name": "rear", "pos": [0.5, -1.2], "a": "c2", "b": "c1"}])
    solver = ScriptedSolver(env)
    assert solver.towed_hitch(env.parts["front"]).name == "c1"
    assert solver.towed_hitch(env.parts["rear"]).name == "c2"      # either end order
    env.parts["front"].untie(env.model, env.data, 1)
    assert solver.towed_hitch(env.parts["rear"]) is None             # the chain is off the body


def test_drag_run_puts_the_sled_in_the_zone(make):
    env = make([{**SLED, "mass": 0.2}, ROPE, ZONE], spawn=(-1.0, 0.0))
    env.room.demo = [("drag", "rope", "zone")]
    solver = ScriptedSolver(env)
    env.reset(seed=0, options={"layout": env.sample_layout(None)})
    for k in range(150):                                  # measured: done after 7.2 s (72 steps)
        env.step(solver.act())
        if not solver.in_demo:
            break
    sled_xy = env.parts["sled"].pos(env.data)[:2]
    assert not solver.in_demo and env.signals.get("zone") == 1.0
    assert abs(sled_xy[0] - 0.2) < 0.2 and abs(sled_xy[1]) < 0.2 and not rope_of(env).snapped


def test_drag_target_needs_no_grab(asset_dir, tmp_path):
    raw = {"name": "dragroom", "size": [4.0, 1.5], "spawn": {"pos": [-1.0, 0.0], "yaw": 0.0},
           "parts": [{**SLED, "mass": 0.2}, ROPE, {k: v for k, v in ZONE.items() if k != "objects"},   # trigger object
                     {"name": "door", "preset": "device/oneshot_door", "pos": [1.0, 0.0], "in": ["zone"]},
                     {"name": "goal", "type": "goal", "pos": [1.5, 0.0], "radius": 0.1}],
           "walls": [{"from": [1.0, -0.75], "to": [1.0, -0.28]}, {"from": [1.0, 0.28], "to": [1.0, 0.75]}]}
    from mujoco_puzzle_gym.check import check

    for demo, ok in ((["drag rope zone", "goto goal"], True), (None, False)):
        path = tmp_path / f"drag_{ok}.yaml"
        path.write_text(yaml.safe_dump({**raw, **({"demo": demo} if demo else {})}))
        res = check(path, static_only=True)
        assert res["actions"]["ok"] == ok, res["actions"]
        if not ok:
            assert "'zone'" in res["actions"]["reason"] and "grab" in res["actions"]["reason"]


def test_drag_leg_path_warning(asset_dir, tmp_path):
    raw = {"name": "dragwall", "size": [3.0, 3.0], "spawn": {"pos": [-1.0, -0.8], "yaw": 0.0},
           "walls": [{"from": [0.0, -1.5], "to": [0.0, -1.0]}, {"from": [0.0, -0.6], "to": [0.0, 1.5]}],
           "parts": [{"name": "sled", "type": "stick", "pos": [-1.4, -0.8], "length": 0.5, "hitch": True},
                     {**ROPE, "pos": [0.0, 1.2]},
                     {"name": "btn", "preset": "input/floor_momentary", "pos": [-0.5, 1.0]},
                     {"name": "door", "preset": "device/sliding_door", "pos": [0.0, -0.8], "in": ["btn"], "width": 0.4},
                     {**ZONE, "pos": [0.9, -0.8]},
                     {"name": "goal", "type": "goal", "pos": [1.0, 1.0], "radius": 0.1}],
           "demo": ["drag rope zone"]}
    path = tmp_path / "dragwall.yaml"
    path.write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(room=path)
    env.reset(seed=0)
    try:
        got = tow_path_warnings(env, env.parts["rope"])
    finally:
        env.close()
    assert len(got) == 1 and "passes door 'door'" in got[0]
