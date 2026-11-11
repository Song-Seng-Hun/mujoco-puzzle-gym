"""Community spec R5: export a run to ONNX and evaluate it without pickle."""

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from gymnasium import spaces

from mujoco_puzzle_gym.cli import main
from mujoco_puzzle_gym.onnx_policy import OnnxPolicy, OnnxPolicyError, export, fit_report
from mujoco_puzzle_gym.train import load_run, load_trainer, train

pytest.importorskip("onnx")


def _matches(run_dir: Path, out: Path) -> None:
    res = export(run_dir, out)
    meta = json.loads(Path(res["json"]).read_text())
    assert meta["obs_dim"] == sum(t["size"] for t in meta["obs_terms"])
    trainer = load_trainer(load_run(run_dir))
    onnx = OnnxPolicy(res["onnx"])
    obs = np.random.default_rng(0).normal(0.0, 1.0, (100, meta["obs_dim"])).astype(np.float32)
    for o in obs:
        a = np.asarray(trainer.predict(o, deterministic=True))
        b = onnx.predict(o)
        np.testing.assert_allclose(b, a, atol=1e-5)


def test_ppo_export_matches_sb3(quick_run, tmp_path):
    _matches(quick_run, tmp_path / "onnx")


def test_sac_export_matches_sb3(asset_dir, repo_root, tmp_path):
    s = train(repo_root / "examples/task_ppo.yaml", minutes=1.0, algo="sac", n_envs=1,
              runs_dir=tmp_path, run_id="sac", max_steps=300)
    _matches(Path(s["run_dir"]), tmp_path / "onnx")


def test_eval_onnx_cli(quick_run, repo_root, tmp_path, capsys):
    export(quick_run, tmp_path)
    assert main(["eval", "--onnx", str(tmp_path / "policy.onnx"), "--task",
                 str(repo_root / "examples/task_ppo.yaml"), "--episodes", "2",
                 "--workers", "1", "--json"]) == 0
    out = json.loads(capsys.readouterr().out)
    assert out["ok"] and out["episodes"] == 2 and out["policy_room"] == "switch_door"
    assert "warnings" not in out


def test_eval_onnx_needs_task(tmp_path, capsys):
    assert main(["eval", "--onnx", str(tmp_path / "policy.onnx"), "--json"]) == 1
    assert "--task" in json.loads(capsys.readouterr().out)["error"]


def test_fit_rejects_camera_and_size():
    meta = {"obs_dim": 10, "action_dim": 3, "obs_terms": [{"name": "x", "size": 10}]}
    cam = SimpleNamespace(observation_space=spaces.Dict(
        {"image": spaces.Box(0, 255, (8, 8, 3), np.uint8)}))
    with pytest.raises(OnnxPolicyError, match="camera"):
        fit_report(meta, cam)
    small = SimpleNamespace(observation_space=spaces.Box(-1, 1, (7,)),
                            action_space=spaces.Box(-1, 1, (3,)))
    with pytest.raises(OnnxPolicyError, match="expects 10"):
        fit_report(meta, small)
