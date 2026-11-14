"""R22 `mpg eval --full --segments` with fake policies.

- a "walk forward" policy on the two-flag room: the full run and start->f1 fail,
  f1->f2 and f2->goal (started from their snapshots) succeed;
- the result JSON's full / per-segment / chain-gap fields from fake episodes;
- `--segments` on a room without flags, and the CLI flags.
"""

import json

import numpy as np
import pytest
from flag_room import FORWARD, STAND, flag_env, task_dict, write_room

from mujoco_puzzle_gym import cli, evaluate as ev
from mujoco_puzzle_gym.algos.base import Trainer
from mujoco_puzzle_gym.registry import register_algo
from mujoco_puzzle_gym.task import load_task, write_resolved
from mujoco_puzzle_gym.train import EVAL_SEED_BASE


class FakePolicy(Trainer):
    """Same action every step (no network)."""

    name = "t_fake"
    action = FORWARD

    def train(self, seconds, max_steps=None):
        return 0

    def save(self, path):
        return path

    @classmethod
    def load(cls, path, task, env=None):
        return cls(env, task)

    def predict(self, obs, deterministic=True):
        return self.action


class StandPolicy(FakePolicy):
    action = STAND


register_algo("t_fake", FakePolicy, replace=True)
register_algo("t_stand", StandPolicy, replace=True)


def fake_run(tmp_path, task, algo="t_fake"):
    run = tmp_path / "runs" / algo
    write_resolved(load_task(task), run)
    (run / "summary.json").write_text(json.dumps({"run_id": algo, "algo": algo, "checkpoint": "none"}))
    return run


@pytest.fixture
def flag_run(asset_dir, tmp_path):
    env, _ = flag_env(tmp_path)
    env.close()
    return fake_run(tmp_path, task_dict(tmp_path / "two_flags.yaml", tmp_path / "snaps"))


def test_full_and_segments_with_a_fake_policy(flag_run):
    res = ev.evaluate(flag_run, episodes=2, workers=1, full=True, segments=True)
    assert res["full_success_rate"] == res["success_rate"] == 0.0
    assert res["segment_success_rates"] == {"start->f1": 0.0, "f1->f2": 1.0, "f2->goal": 1.0}
    assert res["chain_gap_rate"] == 0.0
    assert res["seeds"] == [EVAL_SEED_BASE, EVAL_SEED_BASE + 1]


def test_plain_eval_has_no_segment_fields(flag_run):
    res = ev.evaluate(flag_run, episodes=1, workers=1)
    assert not {"full_success_rate", "segment_success_rates", "chain_gap_rate"} & set(res)


def test_segments_without_full_has_no_chain_gap(flag_run):
    res = ev.evaluate(flag_run, episodes=1, workers=1, segments=True)
    assert set(res["segment_success_rates"]) == {"start->f1", "f1->f2", "f2->goal"}
    assert "chain_gap_rate" not in res and "full_success_rate" not in res


def test_segment_episodes_start_from_snapshots(flag_run):
    jobs = [(EVAL_SEED_BASE, {"segment": 2})]
    ep, = ev.run_jobs(flag_run, jobs, workers=1)
    assert ep["segment"] == 2 and ep["segment_success"] and ep["success"]
    assert ep["length"] < 30                       # 0.4 m from f2 to the goal


def test_chain_gap_from_fake_episodes(monkeypatch, tmp_path):
    """10 seeds: segments all succeed on seeds 0-5, the full run on seeds 0-2 and 7."""
    n = 10
    full_ok = [k in (0, 1, 2, 7) for k in range(n)]
    seg_ok = [[True] * n, [k < 6 or k == 7 for k in range(n)], [k < 6 for k in range(n)]]

    def ep(k, ok, seg=None):
        e = {"seed": k, "success": ok, "fallen": False, "return": 0.0, "length": 1,
             "switches_on": 0, "seconds": 0.1, "terms": {}}
        if seg is not None:
            e.update(segment=seg, segment_success=ok, success=False)
        return e

    def fake_jobs(run_dir, jobs, policy, deterministic, workers):
        assert [o for _, o in jobs[:n]] == [{"randomize": "train"}] * n
        assert [o["segment"] for _, o in jobs[n:]] == [i for i in range(3) for _ in range(n)]
        out = [ep(k, full_ok[k]) for k in range(n)]
        return out + [ep(k, seg_ok[i][k], i) for i in range(3) for k in range(n)]

    run = fake_run(tmp_path, {"room": str(write_room(tmp_path))})
    monkeypatch.setattr(ev, "run_jobs", fake_jobs)
    monkeypatch.setattr(ev, "_segments", lambda run: [{"name": f"s{i}"} for i in range(3)])
    res = ev.evaluate(run, episodes=n, full=True, segments=True)
    assert res["full_success_rate"] == 0.4
    assert res["segment_success_rates"] == {"s0": 1.0, "s1": 0.7, "s2": 0.6}
    assert res["chain_gap_rate"] == 0.3            # seeds 3, 4, 5


def test_segments_need_flags(asset_dir, tmp_path):
    room = tmp_path / "flat.yaml"
    room.write_text(json.dumps({"name": "flat", "size": [2.0, 2.0],
                                "spawn": {"pos": [-0.5, 0.0], "yaw": 0.0},
                                "parts": [{"name": "goal", "type": "goal", "pos": [0.5, 0.0]}]}))
    run = fake_run(tmp_path, {"room": str(room)}, algo="t_stand")
    with pytest.raises(ValueError, match="checkpoint flags"):
        ev.evaluate(run, episodes=1, workers=1, segments=True)


def test_cli_flags(flag_run, capsys):
    code = cli.main(["eval", str(flag_run), "--episodes", "1", "--workers", "1",
                     "--full", "--segments", "--json"])
    out = json.loads(capsys.readouterr().out)
    assert code == 0 and out["ok"]
    assert {"full_success_rate", "segment_success_rates", "chain_gap_rate"} <= set(out)
    assert np.isclose(out["segment_success_rates"]["f2->goal"], 1.0)


# ---- R21 difficulty steps in evaluation ----------------------------------------------------
@pytest.fixture
def diff_run(asset_dir, tmp_path):
    room = tmp_path / "windroom.json"
    room.write_text(json.dumps({"name": "windroom", "size": [3.0, 3.0], "spawn": {"pos": [-1.0, -1.0]},
                                "parts": [{"name": "w", "preset": "trap/crosswind", "pos": [0.5, 0.5]},
                                          {"name": "goal", "type": "goal", "pos": [1.3, 1.3]}]}))
    task = {"room": str(room), "env": {"hz": 10, "episode_seconds": 0.5},
            "difficulty": [{"param": "w.power", "from": 1.0, "to": 3.0, "steps": 3}]}
    return fake_run(tmp_path, task, algo="t_stand")


def test_eval_runs_at_the_chosen_or_last_difficulty(diff_run, capsys):
    assert ev.evaluate(diff_run, episodes=1, workers=1)["difficulty"] == 2          # default: the last
    assert ev.evaluate(diff_run, episodes=1, workers=1, full=True)["difficulty"] == 2
    code = cli.main(["eval", str(diff_run), "--episodes", "1", "--workers", "1", "--difficulty", "0", "--json"])
    out = json.loads(capsys.readouterr().out)
    assert code == 0 and out["difficulty"] == 0
    code = cli.main(["eval", str(diff_run), "--episodes", "1", "--workers", "1", "--difficulty", "3", "--json"])
    assert code == 1 and "out of range" in json.loads(capsys.readouterr().out)["error"]


def test_eval_onnx_rejects_difficulty(tmp_path, capsys):
    code = cli.main(["eval", "--onnx", str(tmp_path / "policy.onnx"), "--task", str(tmp_path / "t.yaml"),
                     "--difficulty", "1", "--json"])
    assert code == 1 and "--difficulty" in json.loads(capsys.readouterr().out)["error"]
