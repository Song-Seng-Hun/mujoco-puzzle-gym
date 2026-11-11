"""R21 staged training over checkpoint segments.

- with fake success rates, segments open in the set order and at the set time
  (forward and reverse);
- the replay share is within +-5 % of `replay`;
- a segment starts at its flag's snapshot (part states equal, body within 1 cm)
  and ends with success at its end flag; `to_goal` runs on to the goal;
- the `checkpoints` observation item; `curriculum.json` from the training
  wrapper driven by a fake env; task validation and missing snapshots.
"""

import json

import gymnasium as gym
import numpy as np
import pytest
import yaml
from flag_room import FORWARD, ROOM, STAND, flag_env, task_dict, write_room
from stable_baselines3.common.vec_env import DummyVecEnv

from mujoco_puzzle_gym.curriculum import Curriculum, CurriculumError, segments
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.task import TaskError, load_task
from mujoco_puzzle_gym.train import CurriculumVec, TaskError as TrainTaskError, train


@pytest.fixture
def flagged(asset_dir, tmp_path):
    env, snaps = flag_env(tmp_path)
    yield env, snaps
    env.close()


# ---- the controller with fake success rates -----------------------------------------------
def feed(cur, segment, results):
    """Record results for `segment`; the episode numbers (1-based) that opened a level."""
    return [i + 1 for i, ok in enumerate(results) if cur.record(segment, ok)]


def test_forward_opens_in_order_at_the_set_time():
    cur = Curriculum("forward", 3, advance_at=0.8, replay=0.2, window=10)
    assert cur.state()["open"] == ["0"] and cur.probs() == [1.0, 0.0, 0.0]
    assert feed(cur, 1, [True] * 20) == []                 # not open yet: not counted
    assert feed(cur, 0, [False] * 3 + [True] * 7) == []    # 0.7 < 0.8
    assert feed(cur, 0, [True]) == [1]                     # last 10: 8 of 10 -> opens 1
    assert cur.state()["open"] == ["0", "1"]
    assert cur.probs() == pytest.approx([0.2, 0.8, 0.0])
    assert feed(cur, 0, [True] * 30) == []                 # replays do not open anything
    assert feed(cur, 1, [False] * 5 + [True] * 7) == []    # window not 0.8 yet (7/10)
    assert feed(cur, 1, [True]) == [1]                     # 8/10
    assert cur.state()["open"] == ["0", "1", "2"]
    assert cur.probs() == pytest.approx([0.1, 0.1, 0.8])
    assert feed(cur, 2, [True] * 50) == []                 # the last level: nothing more to open
    assert cur.state()["rates"] == {"0": 1.0, "1": 0.8, "2": 1.0}


def test_reverse_starts_at_the_last_segment():
    cur = Curriculum("reverse", 3, advance_at=0.5, replay=0.3, window=4)
    assert cur.to_goal and cur.probs() == [0.0, 0.0, 1.0]
    assert feed(cur, 0, [True] * 4) == []
    assert feed(cur, 2, [True, False, True, False]) == [4]
    assert cur.probs() == pytest.approx([0.0, 0.7, 0.3])
    assert feed(cur, 1, [True, True, False, False]) == [4]
    assert cur.probs() == pytest.approx([0.7, 0.15, 0.15])


def test_none_is_m1():
    cur = Curriculum("none", 3)
    assert cur.to_goal and cur.probs() == [1.0, 0.0, 0.0] and not cur.record(0, True)


def test_replay_share_within_five_percent():
    cur = Curriculum("forward", 4, replay=0.2, window=1)
    for s in range(3):
        cur.record(s, True)
    assert cur.open == 4
    rng = np.random.default_rng(0)
    draws = np.array([cur.draw(rng) for _ in range(10_000)])
    assert abs(np.mean(draws != 3) - 0.2) < 0.05
    assert set(np.unique(draws)) == {0, 1, 2, 3}


def test_env_draws_from_the_open_segments(flagged):
    env, _ = flagged
    cur = Curriculum("forward", 3, replay=0.25, window=1)
    cur.record(0, True)                                    # open: 0, 1
    env.set_segment_draw(cur.probs(), cur.to_goal)
    seen = []
    for k in range(200):
        _, info = env.reset(seed=k)
        seen.append(env.segment["index"])
    seen = np.array(seen)
    assert set(seen) == {0, 1}
    assert abs(np.mean(seen == 0) - 0.25) < 0.05 + 3 * np.sqrt(0.25 * 0.75 / 200)
    env.set_segment_draw(None)
    env.reset(seed=0)
    assert env.segment is None


# ---- segment starts and ends -------------------------------------------------------------
def body_xy_in(env, snap):
    """Body position stored in the snapshot's MuJoCo state (time, then qpos)."""
    jnt = env.model.body_jntadr[env.body.root_body_id(env.model)]
    qa = int(env.model.jnt_qposadr[jnt])
    return snap.state[1 + qa:1 + qa + 2]


@pytest.mark.parametrize("flag,segment", [("f1", 1), ("f2", 2)])
def test_segment_starts_at_its_snapshot(flagged, flag, segment):
    env, snaps = flagged
    snap = snaps[flag]
    env.reset(seed=5)                                      # a different layout first
    env.reset(seed=7, options={"segment": segment})
    assert {n: p.save_state() for n, p in env.parts.items()} == snap.parts
    assert env.signals.snapshot() == snap.signals
    assert np.linalg.norm(env.body_state.pos[:2] - body_xy_in(env, snap)) < 0.01
    assert env.checkpoint_snap.meta["checkpoint"] == flag and env.steps == 0   # respawn point


def test_segment_zero_is_a_normal_start(flagged):
    env, _ = flagged
    obs, _ = env.reset(seed=3, options={"segment": 0})
    assert np.allclose(env.body_state.pos[:2], [-1.0, -1.0], atol=0.02)
    assert not any(env.parts[f].reached for f in ("f1", "f2"))


def test_segment_ends_with_success_at_its_end_flag(flagged):
    env, _ = flagged
    env.reset(seed=0, options={"segment": 1})
    for _ in range(40):
        _, r, terminated, truncated, info = env.step(FORWARD)
        if terminated or truncated:
            break
    assert terminated and info["segment"] == 1 and info["segment_success"]
    assert not info["success"]                             # the goal is not reached yet
    assert info["reward_terms"]["success"] == pytest.approx(10.0)


def test_to_goal_runs_past_the_segment_end(flagged):
    env, _ = flagged
    env.reset(seed=0, options={"segment": 2, "to_goal": True})
    env.step(STAND)
    assert env.segment["end"] == "goal"
    env.reset(seed=0, options={"segment": 1, "to_goal": True})
    for _ in range(40):
        _, _, terminated, _, info = env.step(FORWARD)
        if env.parts["f2"].reached:
            break
    assert env.parts["f2"].reached and not terminated and not info["segment_success"]


def test_bad_segment_rejected(flagged):
    env, _ = flagged
    with pytest.raises(CurriculumError, match="out of range"):
        env.reset(seed=0, options={"segment": 3})


def test_segments_of_the_room(flagged):
    env, _ = flagged
    assert [s["name"] for s in segments(env.parts)] == ["start->f1", "f1->f2", "f2->goal"]


def test_missing_snapshots_named(asset_dir, tmp_path):
    env = PuzzleEnv(task=task_dict(write_room(tmp_path), tmp_path / "nowhere"))
    with pytest.raises(CurriculumError, match=r"\['f1', 'f2'\].*mpg check"):
        env.reset(seed=0, options={"segment": 1})
    env.close()


# ---- the `checkpoints` observation -------------------------------------------------------
def test_checkpoints_obs(flagged):
    env, _ = flagged
    obs, _ = env.reset(seed=0, options={"segment": 0})
    assert list(obs[-5:]) == [1, 0, 0, 0, 0]               # next f1; none reached
    env.reset(seed=0, options={"segment": 1})
    assert list(env._obs()[-5:]) == [0, 1, 0, 1, 0]        # next f2; f1 reached
    env.reset(seed=0, options={"segment": 2})
    assert list(env._obs()[-5:]) == [0, 0, 1, 1, 1]        # the goal is next


# ---- training wrapper with a fake env ----------------------------------------------------
class FakeSegmentEnv(gym.Env):
    """One-step episodes at the drawn segment; success with a fixed rate per segment."""

    observation_space = gym.spaces.Box(-1, 1, (1,), np.float32)
    action_space = gym.spaces.Box(-1, 1, (3,), np.float32)
    RATES = (0.95, 0.0, 0.95)          # segment 1 never succeeds

    def __init__(self):
        self.draw = None
        self.started: list[int] = []

    def set_segment_draw(self, probs, to_goal=False):
        self.draw = (list(probs), to_goal)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.seg = int(self.np_random.choice(len(self.draw[0]), p=self.draw[0]))
        self.started.append(self.seg)
        return np.zeros(1, np.float32), {}

    def step(self, action):
        ok = bool(self.np_random.random() < self.RATES[self.seg])
        return np.zeros(1, np.float32), 0.0, True, False, {"segment": self.seg, "segment_success": ok}


def test_wrapper_opens_segments_and_logs(tmp_path):
    vec = DummyVecEnv([FakeSegmentEnv, FakeSegmentEnv])
    vec.seed(0)
    cur = Curriculum("forward", 3, advance_at=0.8, replay=0.2, window=20,
                     names=["start->f1", "f1->f2", "f2->goal"])
    log = tmp_path / "curriculum.json"
    wrapped = CurriculumVec(vec, cur, log)
    wrapped.reset()
    for _ in range(300):
        wrapped.step(np.zeros((2, 3), np.float32))
    wrapped.close()
    assert cur.state()["open"] == ["start->f1", "f1->f2"]          # stuck behind segment 1
    entries = json.loads(log.read_text())
    assert [e["event"] for e in entries][0] == "start" and entries[-1]["event"] == "end"
    opened = [e for e in entries if e["event"] == "open"]
    assert len(opened) == 1 and opened[0]["open"] == ["start->f1", "f1->f2"]
    assert {"t", "step", "open", "rates"} <= set(opened[0])
    assert entries[-1]["rates"]["f1->f2"] == 0.0
    started = [s for e in vec.envs for s in e.started]
    late = started[len(started) // 2:]                              # after the opening
    assert set(late) == {0, 1} and abs(np.mean(np.array(late) == 0) - 0.2) < 0.08


# ---- task and train wiring ---------------------------------------------------------------
def test_task_curriculum_validated(asset_dir, tmp_path):
    room = write_room(tmp_path)
    t = load_task({"room": str(room)})
    assert t["curriculum"]["mode"] == "none" and t["curriculum"]["advance_at"] == 0.8
    for bad in ({"mode": "sideways"}, {"replay": 1.0}, {"advance_at": 0}, {"window": 0}, {"speed": 1}):
        with pytest.raises(TaskError):
            load_task({"room": str(room), "curriculum": bad})


def test_train_needs_flags_and_snapshots(asset_dir, tmp_path):
    task = tmp_path / "task.yaml"
    task.write_text(json.dumps(task_dict(write_room(tmp_path), tmp_path / "nowhere")))
    with pytest.raises(TrainTaskError, match="mpg check"):
        train(task, minutes=0.1, curriculum="forward", runs_dir=tmp_path / "runs", run_id="x")


def test_train_forward_writes_curriculum_json(asset_dir, tmp_path):
    env, _ = flag_env(tmp_path)
    env.close()
    task = tmp_path / "task.yaml"
    raw = task_dict(tmp_path / "two_flags.yaml", "snaps",
                    algo_kwargs={"n_steps": 64, "batch_size": 64}, n_envs=1)
    task.write_text(json.dumps(raw))
    summary = train(task, minutes=1.0, curriculum="forward", replay=0.3,
                    runs_dir=tmp_path / "runs", run_id="cur", max_steps=128)
    run = tmp_path / "runs/cur"
    log = json.loads((run / "curriculum.json").read_text())
    assert log[0]["open"] == ["start->f1"] and log[-1]["event"] == "end"
    assert summary["curriculum"]["mode"] == "forward"
    resolved = load_task(run / "task.resolved.yaml")
    assert resolved["curriculum"]["replay"] == 0.3
    assert resolved["curriculum"]["snapshots"] == str((tmp_path / "snaps").resolve())
    eps = [json.loads(x) for x in (run / "episodes.jsonl").read_text().splitlines()]
    assert all(e["segment"] == 0 for e in eps)


def test_snapshots_of_another_room_rejected(flagged, tmp_path):
    room = tmp_path / "three_flags.yaml"
    room.write_text(yaml.safe_dump({**ROOM, "name": "three_flags", "parts": ROOM["parts"] + [
        {"name": "f3", "type": "checkpoint", "order": 3, "pos": [1.0, 1.0]}]}))
    (tmp_path / "snaps" / "f3.pkl").write_bytes((tmp_path / "snaps" / "f2.pkl").read_bytes())
    env = PuzzleEnv(task=task_dict(room, tmp_path / "snaps"))
    with pytest.raises(CurriculumError, match="another model"):
        env.reset(seed=0, options={"segment": 1})
    env.close()
