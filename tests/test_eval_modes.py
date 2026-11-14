"""R35 part 4c-2: `mpg eval --randomize nominal | train | wide`, the episode record of
what was drawn, and a snapshot that keeps its own structure.

The draw mode is the evaluation's main knob: `train` draws the ranges the room
declares, `nominal` draws nothing, and `wide` draws the evaluation-only values the
training never saw. A room without such a value refuses `wide` before the workers
start, so a long evaluation does not fail halfway.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest
import yaml
from flag_room import STAND
from test_eval_full import fake_run

from mujoco_puzzle_gym import cli, evaluate as ev
from mujoco_puzzle_gym.algos.base import Trainer
from mujoco_puzzle_gym.env import PuzzleEnv
from mujoco_puzzle_gym.registry import register_algo

ROOT = Path(__file__).resolve().parents[1]
ROOM = {"name": "modes", "size": [3.0, 2.0], "wall": {"thickness": 0.06, "height": 0.25},
        "spawn": {"pos": [-1.2, 0.0], "yaw": 0.0},
        "parts": [{"name": "fan", "preset": "trap/crosswind", "pos": [-0.2, -0.7],
                   "randomize": {"power": {"range": 0.5, "holdout_scale": 2.0}}},
                  {"name": "goal", "type": "goal", "pos": [1.1, 0.0], "radius": 0.15}]}


class StillPolicy(Trainer):
    """Stands still: the numbers come from the draw, not from the policy."""

    name = "t_still"

    def train(self, seconds, max_steps=None):
        return 0

    def save(self, path):
        return path

    @classmethod
    def load(cls, path, task, env=None):
        return cls(env, task)

    def predict(self, obs, deterministic=True):
        return STAND


register_algo("t_still", StillPolicy, replace=True)


def write_room(tmp_path, **top) -> Path:
    raw = yaml.safe_load(yaml.safe_dump(ROOM))
    raw.update(top)
    path = tmp_path / "modes.yaml"
    path.write_text(yaml.safe_dump(raw))
    return path


def run_of(tmp_path, room: Path, **task) -> Path:
    return fake_run(tmp_path, {"room": str(room), "env": {"hz": 10, "episode_seconds": 0.3},
                               **task}, algo="t_still")


# ---- the draw mode reaches the episodes ---------------------------------------------
def test_the_mode_is_reported_and_passed_to_every_episode(asset_dir, tmp_path):
    run = run_of(tmp_path, write_room(tmp_path))
    for mode in ("nominal", "train", "wide"):
        res = ev.evaluate(run, episodes=2, workers=1, randomize=mode)
        assert res["randomize"] == mode

    jobs = [(3, {"randomize": "nominal"}), (3, {"randomize": "train"})]
    nominal, train = ev.run_jobs(run, jobs, workers=1)
    assert nominal["randomize"] == "nominal" and train["randomize"] == "train"


def test_nominal_draws_nothing_and_wide_draws_the_evaluation_only_band(asset_dir, tmp_path):
    """The same seed, three modes: the fan's power is the room value, inside the
    declared band, and in the evaluation-only band above it."""
    env = PuzzleEnv(task={"room": str(write_room(tmp_path))})
    try:
        nominal = env.sample_layout(None)
        powers = {}
        for mode in ("train", "wide"):
            layout = env.sample_layout(np.random.default_rng(0), mode)
            powers[mode] = float(layout["values"]["fan"]["power"])
        base = float(env.parts["fan"].room_value("power"))
        assert not nominal["values"]
        assert abs(powers["train"] - base) <= 0.5 + 1e-9
        assert 0.5 - 1e-9 <= abs(powers["wide"] - base) <= 1.0 + 1e-9
    finally:
        env.close()


def test_a_room_without_an_evaluation_only_value_refuses_wide(asset_dir, tmp_path):
    room = write_room(tmp_path, parts=[{"name": "goal", "type": "goal", "pos": [1.1, 0.0],
                                        "radius": 0.15, "random": [0.1, 0.3]}])
    run = run_of(tmp_path, room)
    with pytest.raises(ValueError, match="needs an evaluation-only value"):
        ev.evaluate(run, episodes=2, workers=1, randomize="wide")
    assert ev.evaluate(run, episodes=1, workers=1)["randomize"] == "train"


def test_an_unknown_mode_is_refused(asset_dir, tmp_path):
    run = run_of(tmp_path, write_room(tmp_path))
    with pytest.raises(ValueError, match="randomize mode must be one of"):
        ev.evaluate(run, episodes=1, workers=1, randomize="widee")


def test_the_cli_takes_the_mode(asset_dir, tmp_path, capsys):
    run = run_of(tmp_path, write_room(tmp_path))
    code = cli.main(["eval", str(run), "--episodes", "1", "--workers", "1",
                     "--randomize", "wide", "--json"])
    out = json.loads(capsys.readouterr().out)
    assert code == 0 and out["ok"] and out["randomize"] == "wide"
    code = cli.main(["eval", str(run), "--episodes", "1", "--workers", "1", "--json"])
    assert json.loads(capsys.readouterr().out)["randomize"] == "train"


def test_the_cli_refuses_a_mode_the_room_cannot_run(asset_dir, tmp_path, capsys):
    room = write_room(tmp_path, parts=[{"name": "goal", "type": "goal", "pos": [1.1, 0.0],
                                        "radius": 0.15}])
    run = run_of(tmp_path, room)
    code = cli.main(["eval", str(run), "--episodes", "1", "--workers", "1",
                     "--randomize", "wide", "--json"])
    out = json.loads(capsys.readouterr().out)
    assert code == 1 and "evaluation-only value" in out["error"]


# ---- the episode record -------------------------------------------------------------
def test_a_drawn_room_is_reported_per_room(asset_dir, tmp_path):
    """The evaluation of a room that draws its room says how each room did; `wide`
    measures the evaluation-only room alone."""
    run = run_of(tmp_path, ROOT / "examples/rooms/room_roulette.yaml",
                 obs=["part_pos", "part_state", "part_used"])
    res = ev.evaluate(run, episodes=6, workers=1)
    assert res["randomize"] == "train"
    assert set(res["success_by_room"]) <= {"room_roulette", "roulette_b"}
    res = ev.evaluate(run, episodes=2, workers=1, randomize="wide")
    assert set(res["success_by_room"]) == {"roulette_c"}


def test_the_episode_info_carries_the_draw(asset_dir, tmp_path):
    env = PuzzleEnv(task={"room": str(ROOT / "examples/rooms/shuffle_switches.yaml")})
    try:
        _, info = env.reset(seed=0)
        assert info["randomize"] == "train" and info["links"] == env.layout["links"]
        assert "room" not in info                      # this room does not draw its room
    finally:
        env.close()


# ---- a snapshot keeps its own structure ---------------------------------------------
def test_a_snapshot_restores_the_room_and_the_shelf_it_was_recorded_with(asset_dir, tmp_path):
    env = PuzzleEnv(task={"room": str(ROOT / "examples/rooms/room_roulette.yaml")})
    try:
        snaps = {}
        for s in range(12):
            env.reset(seed=s)
            snaps.setdefault(env.layout["zone"], (env.save_snapshot(),
                                                  {n: p.parked for n, p in env.parts.items()}))
            if len(snaps) == 2:
                break
        assert sorted(snaps) == [0, 1]
        for zone, (snap, parked) in snaps.items():
            other = next(z for z in snaps if z != zone)
            env.reset(seed=0, options={"layout": {**env.sample_layout(None), "zone": other}})
            env.restore_snapshot(snap)
            assert env.zone == zone
            assert {n: p.parked for n, p in env.parts.items()} == parked
    finally:
        env.close()


def test_a_snapshot_restores_the_group_members_it_was_recorded_with(asset_dir):
    env = PuzzleEnv(task=ROOT / "examples/task_mixed_stock.yaml")
    try:
        snaps = {}
        for s in range(20):
            env.reset(seed=s)
            key = tuple(sorted(env.layout["used"]["crate"]))
            snaps.setdefault(key, (env.save_snapshot(),
                                   {n: p.parked for n, p in env.parts.items()}))
            if len(snaps) >= 2:
                break
        assert len(snaps) >= 2
        for key, (snap, parked) in snaps.items():
            env.reset(seed=97)
            env.restore_snapshot(snap)
            assert {n: p.parked for n, p in env.parts.items()} == parked
            assert snap.meta["used"]["crate"] == list(key)
    finally:
        env.close()
