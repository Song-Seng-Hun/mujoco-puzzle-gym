"""R6 checkpoint flags and snapshots; the body adapter's teleport and history
save/restore; the physics-step `substep` hook.

- flags turn on in order; out of turn: ignored, or the episode fails (skip: fail);
- after flag 2, a fall with `respawn: checkpoint` restarts at flag 2's snapshot
  (body position, part states, signals) and the body stands for 3 s after;
- snapshots use mjSTATE_INTEGRATION: an active weld (`eq_active`) and mocap
  poses come back; model values changed while running come back; reset()
  returns model values to their built state.
"""

import mujoco
import numpy as np
import pytest
from rooms import open_room, stand_at, tick

from mujoco_puzzle_gym import snapshot
from mujoco_puzzle_gym.parts.base import Part
from mujoco_puzzle_gym.registry import register_part

STAND = np.array([-1.0, 0.0, 0.0])          # action: stand still
FLAGS = [{"name": "f1", "type": "checkpoint", "order": 1, "pos": [-0.5, 0.0]},
         {"name": "f2", "type": "checkpoint", "order": 2, "pos": [0.3, 0.3]}]


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


def at(env, name):
    return env.parts[name].xy


def visit(env, name):
    """Teleport onto a flag and take one env step standing there."""
    env.body.teleport(env.data, (*at(env, name), 0.0))
    env.body_state = env.body.state(env.data)
    return env.step(STAND)


def push_over(env, max_steps=40):
    """Shove the trunk sideways until a step reports a fall or a respawn."""
    root = env.body.root_body_id(env.model)
    for _ in range(max_steps):
        env.data.xfrc_applied[root, :3] = [0.0, 30.0, 0.0]
        out = env.step(STAND)
        if out[2] or out[4]["respawned"]:
            return out
    raise AssertionError("the body did not fall")


# ---- flags --------------------------------------------------------------------------------
def test_flags_in_order(make):
    env = make(FLAGS)
    visit(env, "f2")                                    # out of turn: ignored
    assert env.signals.get("f2") == 0 and env.checkpoint_snap is None
    visit(env, "f1")
    assert env.signals.get("f1") == 1 and env.checkpoint_snap.meta["checkpoint"] == "f1"
    visit(env, "f2")
    assert env.signals.get("f2") == 1 and env.checkpoint_snap.meta["checkpoint"] == "f2"
    assert env.world.next_checkpoint() is None


def test_skip_fail(make):
    env = make([FLAGS[0], {**FLAGS[1], "skip": "fail"}])
    _, _, terminated, _, info = visit(env, "f2")
    assert terminated and info["failed"] == ["f2"]


def test_duplicate_order_rejected(asset_dir, tmp_path):
    from mujoco_puzzle_gym.room import RoomError
    with pytest.raises(RoomError, match="both have order 1"):
        open_room(tmp_path, [FLAGS[0], {**FLAGS[1], "order": 1}])


def test_fall_ends_episode_by_default(make):
    env = make(FLAGS)
    visit(env, "f1")
    _, _, terminated, _, info = push_over(env)
    assert terminated and info["fallen"] and not info["respawned"]


# ---- respawn ------------------------------------------------------------------------------
def test_respawn_at_flag_two(make):
    parts = FLAGS + [{"name": "btn", "type": "button", "kind": "toggle", "pos": [0.3, -0.5]},
                     {"name": "tap", "type": "button", "pos": [0.3, -0.5]},
                     {"name": "laps", "type": "counter", "up": "tap", "target": 5},
                     {"name": "keep", "type": "counter", "up": "tap", "persist": True}]
    env = make(parts, task={"env": {"respawn": "checkpoint"}})
    stand_at(env, *at(env, "btn"))                      # toggle on, counters 1
    visit(env, "f1")
    visit(env, "f2")
    snap_pos = env.body_state.pos.copy()
    states = {n: p.state() for n, p in env.parts.items()}
    signals = env.signals.snapshot()
    stand_at(env, *at(env, "btn"))                      # after the flag: toggle off, counters 2
    assert env.signals.get("btn") == 0 and env.signals.get("laps.count") == 2

    _, reward, terminated, _, info = push_over(env)
    assert info["respawned"] and not terminated and not info["fallen"]
    assert info["reward_terms"]["respawn_penalty"] == -1.0
    assert np.linalg.norm(env.body_state.pos[:2] - snap_pos[:2]) < 0.01
    assert env.data.xfrc_applied.any() == False        # noqa: E712  pushed after the save
    assert env.signals.get("btn") == 1 and env.signals.get("laps.count") == 1
    assert env.signals.get("keep.count") == 2           # persist: kept across the restore
    assert {n: p.state() for n, p in env.parts.items() if n != "keep"} == \
        {n: v for n, v in states.items() if n != "keep"}
    assert {k: v for k, v in env.signals.snapshot().items() if not k.startswith("keep")} == \
        {k: v for k, v in signals.items() if not k.startswith("keep")}

    for _ in range(int(3 * env.hz)):                    # stands for 3 s after the restore
        _, _, terminated, _, info = env.step(STAND)
        assert not info["fallen"] and not info["respawned"] and not terminated


def test_respawn_needs_a_flag(make):
    env = make(FLAGS, task={"env": {"respawn": "checkpoint"}})
    _, _, terminated, _, info = push_over(env)
    assert terminated and info["fallen"]


def test_bad_respawn_value(asset_dir, tmp_path):
    from mujoco_puzzle_gym.task import TaskError
    with pytest.raises(TaskError, match="respawn"):
        open_room(tmp_path, FLAGS, task={"env": {"respawn": "nowhere"}})


# ---- snapshot contents -----------------------------------------------------------------------
class Welded(Part):
    """A free box with an (inactive) weld to the world: stands in for a beak grab."""

    kind = "object"
    stage = "device"

    def build(self, spec):
        body = spec.worldbody.add_body(name=self.body_name, pos=[*self.base_xy, 0.03])
        body.add_freejoint()
        body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.03] * 3, mass=0.05)
        eq = spec.add_equality(name=self.body_name + "/weld", type=mujoco.mjtEq.mjEQ_WELD,
                               objtype=mujoco.mjtObj.mjOBJ_BODY, name1=self.body_name)
        eq.active = False
        return body

    def bind(self, model, data):
        super().bind(model, data)
        self.eq = model.equality(self.body_name + "/weld").id

    def place(self, model, xy):
        self.xy = np.asarray(xy, dtype=float)


register_part("t_welded", Welded, replace=True)


def test_snapshot_keeps_eq_active_mocap_and_model_values(make):
    env = make([{"name": "w", "type": "t_welded", "pos": [0.8, 0.0]},
                {"name": "rec", "type": "receptacle", "pos": [0.8, 0.8], "close_on_ok": True}])
    w, rec = env.parts["w"], env.parts["rec"]
    env.data.eq_active[w.eq] = 1
    rec.closed = True
    rec._lid(env.data, True)
    env.model.geom_friction[0, 0] = 0.3
    snap = env.save_snapshot()
    env.data.eq_active[w.eq] = 0
    rec.closed = False
    rec._lid(env.data, False)
    env.model.geom_friction[0, 0] = 1.7
    env.restore_snapshot(snap)
    assert env.data.eq_active[w.eq] == 1
    assert env.data.mocap_pos[rec._mocap][2] == pytest.approx(0.065)
    assert env.model.geom_friction[0, 0] == pytest.approx(0.3)
    env.reset(seed=1)
    assert env.data.eq_active[w.eq] == 0                # eq_active0 from the model
    assert env.model.geom_friction[0, 0] == pytest.approx(1.0)    # built value


def test_snapshot_uses_integration_state():
    assert snapshot.SIG == mujoco.mjtState.mjSTATE_INTEGRATION
    assert snapshot.SIG & mujoco.mjtState.mjSTATE_EQ_ACTIVE


# ---- body adapter: teleport and history -------------------------------------------------------
def test_teleport_places_and_stands(make):
    env = make([])
    env.body.teleport(env.data, (0.4, -0.3, 1.0))
    env.body_state = env.body.state(env.data)
    assert np.linalg.norm(env.body_state.pos[:2] - [0.4, -0.3]) < 0.01
    assert env.body_state.yaw == pytest.approx(1.0, abs=1e-3)
    for _ in range(int(3 * env.hz)):
        env.step(STAND)
    assert not env.body_state.fallen


def test_history_round_trip_follows_walk(make):
    # Restoring the controller memory replays a walk to within 1 cm over 1 s
    # (probe: 2 mm; with the memory left at its reset values: 4.3 cm). Not
    # exact: the motor model reads forces from the previous physics step, which
    # `restore` recomputes at the restored state with mj_forward.
    env = make([])
    walk = np.array([1.0, 0.0, 0.2])
    for _ in range(5):
        env.step(walk)
    snap = env.save_snapshot()
    for _ in range(10):
        env.step(walk)
    first = env.body_state.pos.copy()
    env.restore_snapshot(snap)
    for _ in range(10):
        env.step(walk)
    assert np.linalg.norm(env.body_state.pos - first) < 0.01


# ---- substep hook -----------------------------------------------------------------------------
class Counting(Part):
    stage, has_body = "device", False
    calls = 0

    def substep(self, model, data):
        self.calls += 1


register_part("t_counting", Counting, replace=True)


def test_substep_hook_runs_every_physics_step(make):
    env = make([{"name": "c", "type": "t_counting", "pos": [0, 0]}])
    env.parts["c"].calls = 0
    env.advance(np.zeros(3))
    per_body_step = round(env.body.control_dt / env.body.timestep)
    assert env.parts["c"].calls == env.body_steps * per_body_step
    tick(env)                                           # bus steps alone do not call it
    assert env.parts["c"].calls == env.body_steps * per_body_step


def test_m1_parts_restore(asset_dir, repo_root):
    from mujoco_puzzle_gym.env import PuzzleEnv
    env = PuzzleEnv(room=repo_root / "examples/switch_door.yaml")
    env.reset(seed=0)
    snap = env.save_snapshot()
    sw, goal = env.parts["switch_a"], env.parts["goal"]
    sw._set(env.model, True)
    goal.reached = True
    env.restore_snapshot(snap)
    assert not sw.on and not goal.reached and env.signals.get("switch_a") == 0
    assert not np.allclose(env.model.geom_rgba[sw._gid], (0.2, 0.8, 0.3, 1.0))   # off colour
    env.close()
