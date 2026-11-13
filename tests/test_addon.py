"""R30 addon station and the roller mode of the Microduck.

- a room without a station builds the foot model as before; with one, the
  combined model (wheels hung under the ankles) starts dismounted;
- `set_addon` switches collision sets (mounted: the official roller model's 13
  colliding geoms), trunk height, wheel visibility and lower policy, and back;
- the station switches only with the body on its pad, slower than `max_speed`,
  when its signal comes, and not again within `cooldown`; outputs and obs follow;
- `reset()` starts on feet; a snapshot taken on wheels restores wheels;
- `crouch` is ignored on feet and when too slow;
- slow: zero command for 2 s after attaching and detaching, on the flat and on
  a raised step tile, without a fall; `roller_rink` passes `mpg check`.
"""

from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml
from rooms import body_steps, open_room, stand_at, tick

from mujoco_puzzle_gym import snapshot
from mujoco_puzzle_gym.bodies import flat_floor_spec, make_body
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.parts.base import Part
from mujoco_puzzle_gym.registry import register_part
from mujoco_puzzle_gym.room import RoomError, parse_demo
from mujoco_puzzle_gym.task import TaskError, load_task

ROOT = Path(__file__).resolve().parents[1]
STATION = {"name": "st", "preset": "device/addon_roller", "pos": [0.0, 0.0]}
WHEEL = "body/w_tire"


class Source(Part):
    """Test-only channel source: writes `value` every step."""
    stage = "input"
    emits = True
    has_body = False
    value = 0.0

    def update(self, model, data, body):
        self.write(self.value)


register_part("t_src", Source, replace=True)


@pytest.fixture
def make(asset_dir, tmp_path):
    envs = []

    def _make(parts, **kw):
        env = open_room(tmp_path, parts, **kw)
        envs.append(env)
        return env

    yield _make
    for e in envs:
        e.close()


def colliding(model, root):
    return {g for g in range(model.ngeom)
            if model.body_rootid[model.geom_bodyid[g]] == root
            and (model.geom_contype[g] or model.geom_conaffinity[g])}


def body_root(env):
    return int(env.model.body_rootid[env.body.root_body_id(env.model)])


def settle(env, x=0.0, y=0.0):
    stand_at(env, x, y)
    body_steps(env, 0.4)


# ---- building -------------------------------------------------------------------
def test_room_without_station_builds_the_foot_model(make, asset_dir):
    env = make([])
    plain = make_body()
    m = plain.spawn(flat_floor_spec()).compile()
    assert env.body.addons == frozenset()
    assert mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, WHEEL) == -1
    assert (env.model.nq, env.model.nv, env.model.nu) == (m.nq, m.nv, m.nu)
    assert len(colliding(env.model, body_root(env))) == len(colliding(m, int(m.body_rootid[plain.root_body_id(m)])))


def test_station_room_builds_wheels_dismounted(make):
    env = make([STATION])
    m = env.model
    plain = make_body().spawn(flat_floor_spec()).compile()
    assert env.body.addons == {"roller"} and env.body.addon == "none"
    assert m.nq == plain.nq + 4                               # four passive wheel hinges
    wheels = env.body._wheel_geoms
    assert len(wheels) == 12
    assert not (m.geom_contype[wheels].any() or m.geom_conaffinity[wheels].any())
    assert (m.geom_rgba[wheels, 3] == 0).all()
    assert colliding(m, body_root(env)) == set(env.body._foot_on)


# ---- set_addon ----------------------------------------------------------------------
def test_set_addon_round_trip(make):
    env = make([STATION])
    b, m, d = env.body, env.model, env.data
    settle(env, -1.0, 0.0)
    foot = colliding(m, body_root(env))
    z0 = d.qpos[b._root_qpos + 2]
    dz = b.set_addon(d, "roller")
    assert dz == pytest.approx(0.0236, abs=0.002)             # OQ7: +23.6 mm
    assert d.qpos[b._root_qpos + 2] == pytest.approx(z0 + dz)
    on = colliding(m, body_root(env))
    assert len(on) == 13 and set(b._contact["roller"]) <= on   # the official set, 4 tires in it
    assert (m.geom_rgba[b._wheel_geoms, 3] == 1).all()
    assert b.session is b._sessions["roller"] and b.action_scale == 0.8
    assert not d.qvel[b._root_qvel:b._root_qvel + 6].any()
    dz2 = b.set_addon(d, "none")
    assert -0.025 < dz2 < -0.012                              # OQ7: -17.7 .. -19.3 mm from a stand
    assert colliding(m, body_root(env)) == foot
    assert (m.geom_rgba[b._wheel_geoms, 3] == 0).all()
    assert b.session is b._sessions["velstand"] and b.action_scale == 1.0


def test_unbuilt_addon_is_refused(make):
    env = make([])
    with pytest.raises(ValueError):
        env.body.set_addon(env.data, "roller")


# ---- the station --------------------------------------------------------------------
def test_station_switches_on_the_pad_when_slow(make):
    env = make([STATION], task={"obs": ["addon"]})
    stand_at(env, 0.6, 0.0)                     # off the pad
    tick(env)
    assert env.body.addon == "none" and env.signals.get("st.attached") == 0.0
    stand_at(env, 0.0, 0.0)                     # on the pad, standing
    env._run_requests()
    assert env.body.addon == "roller"
    tick(env)
    assert env.signals.get("st.attached") == 1.0 and env.signals.get("body.addon") == 1.0
    assert list(env._obs()) == [0.0, 1.0]
    assert env.parts["st"].uses == 1


def test_station_ignores_a_moving_body(make):
    env = make([STATION])
    stand_at(env, 0.0, 0.0)
    env.world.requests.clear()
    env.parts["st"].reset(env.model, env.data)
    env.body.kick(env.data, [0.3, 0.0, 0.0])    # on the pad at 0.3 m/s > max_speed
    tick(env)
    assert not env.world.requests
    env.data.qvel[:] = 0.0
    tick(env)                                   # slow now: it switches
    assert env.world.requests and env.world.requests[0][0] == "addon"


def test_station_signal_and_cooldown(make):
    env = make([{"name": "src", "type": "t_src", "pos": [0.8, 0.8]},
                {**STATION, "mode": "toggle", "in": ["src"], "cooldown": 2.0}])
    src, st = env.parts["src"], env.parts["st"]
    stand_at(env, 0.0, 0.0)
    assert not env.world.requests               # on the pad, no signal yet
    src.value = 1.0
    tick(env)
    env._run_requests()
    assert env.body.addon == "roller"           # the signal turned on: toggled on
    tick(env)
    assert env.body.addon == "roller"           # held on: one switch per turn-on
    src.value = 0.0
    tick(env)
    src.value = 1.0
    tick(env)
    assert not env.world.requests               # turned on again within the cooldown: ignored
    env.data.time += 2.5
    src.value = 0.0
    tick(env)
    src.value = 1.0
    tick(env)
    env._run_requests()
    assert env.body.addon == "none" and st.uses == 2


def test_detach_station_and_enable(make):
    env = make([{"name": "src", "type": "t_src", "pos": [0.8, 0.8]},
                {**STATION, "mode": "detach", "enable": "src"}])
    env.body.set_addon(env.data, "roller")
    stand_at(env, 0.0, 0.0)                     # a warp keeps the wheels on
    assert env.body.addon == "roller"
    assert not env.world.requests               # power off: nothing
    env.parts["src"].value = 1.0
    stand_at(env, 0.6, 0.0)
    stand_at(env, 0.0, 0.0)                     # enter again with power
    env._run_requests()
    assert env.body.addon == "none"


# ---- reset and snapshots -----------------------------------------------------------
def test_reset_starts_on_feet(make):
    env = make([STATION])
    env.body.set_addon(env.data, "roller")
    env.reset(seed=1)
    b = env.body
    assert b.addon == "none" and b.session is b._sessions["velstand"]
    assert colliding(env.model, body_root(env)) == set(b._foot_on)


def test_snapshot_on_wheels_restores_wheels(make):
    env = make([STATION])
    settle(env, -1.0, 0.0)
    env.body.set_addon(env.data, "roller")
    body_steps(env, 0.5)
    snap = env.save_snapshot()
    qpos = env.data.qpos.copy()
    env.body.set_addon(env.data, "none")
    body_steps(env, 0.5)
    env.restore_snapshot(snap)
    b = env.body
    assert b.addon == "roller" and b.session is b._sessions["roller"]
    assert np.allclose(env.data.qpos, qpos)
    assert len(colliding(env.model, body_root(env))) == 13
    env.reset(seed=0)
    feet = snapshot.save(env.model, env.data, env.parts, env.signals, env.body)
    env.body.set_addon(env.data, "roller")
    snapshot.restore(feet, env.model, env.data, env.parts, env.signals, env.body)
    assert env.body.addon == "none" and colliding(env.model, body_root(env)) == set(env.body._foot_on)


# ---- crouch ------------------------------------------------------------------------
def test_crouch_ignored_unless_rolling_fast(make):
    env = make([STATION], task={"actions": ["move", "crouch"]})
    assert env.action_space.shape == (4,)
    settle(env, -1.0, 0.0)
    env.step(np.array([-1.0, 0.0, 0.0, 1.0]))
    assert not env.body.crouching               # on feet
    env.body.set_addon(env.data, "roller")
    env.step(np.array([-1.0, 0.0, 0.0, 1.0]))
    assert not env.body.crouching               # on wheels, standing
    env.data.qvel[env.body._root_qvel] = 0.3
    assert env.body.crouch(env.data) and env.body.crouching
    assert env.body.session is env.body._sessions["crouch"]


def test_crouch_action_order_and_speed_key():
    room = ROOT / "examples/rooms/roller_rink.yaml"
    assert load_task({"actions": ["move", "grab", "crouch"]}, room=room)
    with pytest.raises(TaskError):
        load_task({"actions": ["move", "crouch", "head"]}, room=room)
    with pytest.raises(TaskError):
        load_task({"env": {"crouch_min_speed": -1}}, room=room)


def test_demo_addon_parsed(make):
    env = make([STATION])
    assert parse_demo(["addon st", "goto goal"], env.parts) == [("addon", "st"), ("goto", "goal")]
    with pytest.raises(RoomError):
        parse_demo(["addon goal"], env.parts)


# ---- slow -------------------------------------------------------------------------
def _switch_falls(env, seeds):
    """Falls in 2 s of zero command after attaching and after detaching, per seed
    (the body stands off the station's pad, so only `set_addon` switches)."""
    falls = 0
    for s in range(seeds):
        env.reset(seed=s, options={"layout": env.sample_layout(None)})
        falls += body_steps(env, 2.0)
        assert env.body.addon == "none"
        env.body.set_addon(env.data, "roller")
        falls += body_steps(env, 2.0)
        env.body.set_addon(env.data, "none")
        falls += body_steps(env, 2.0)
    return falls


@pytest.mark.slow
def test_switch_stands_on_flat_floor(make):
    env = make([{**STATION, "pos": [0.5, 0.5]}])
    assert _switch_falls(env, 10) == 0


@pytest.mark.slow
def test_switch_stands_on_a_step_tile(asset_dir, tmp_path):
    raw = {"name": "step_station", "cell": 0.1, "step_height": 0.01,
           "map": "\n".join(["############"] + ["#..11111111#"] * 6 + ["############"]),
           "spawn": {"pos": [0.35, 0.0], "yaw": 0.0},
           "parts": [{**STATION, "pos": [0.0, 0.0]}, {"name": "goal", "type": "goal", "pos": [-0.45, 0.0]}]}
    path = tmp_path / "step_station.yaml"
    path.write_text(yaml.safe_dump(raw))
    env = PuzzleEnv(room=path)
    try:
        assert env.floor_height((0.35, 0.0))[0] == pytest.approx(0.01)
        assert _switch_falls(env, 10) == 0
    finally:
        env.close()


@pytest.mark.slow
def test_crouch_from_speed_lowers_the_trunk(make):
    env = make([{**STATION, "pos": [0.5, 0.5]}], size=(6.0, 3.0))
    settle(env, -2.5, 0.0)
    env.body.set_addon(env.data, "roller")
    body_steps(env, 3.0, cmd=np.array([0.6, 0.0, 0.0]))
    assert env.body.crouch(env.data)
    low = []
    fell = body_steps(env, 3.5, cmd=np.array([0.6, 0.0, 0.0]),
                      every=lambda e: low.append(e.data.qpos[e.body._root_qpos + 2]))
    assert not fell and min(low) < 0.08 and not env.body.crouching
    assert not body_steps(env, 2.0)


@pytest.mark.slow
def test_roller_rink_passes_check(asset_dir):
    from mujoco_puzzle_gym.check import check

    res = check(ROOT / "examples/rooms/roller_rink.yaml")
    assert res["ok"], res
    assert res["solver"]["successes"] >= 18
