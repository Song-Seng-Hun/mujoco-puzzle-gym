"""Share a trained policy without pickle (community spec R5).

`mpg export-onnx runs/<id>` writes `policy.onnx` (the deterministic actor: PPO and
A2C's Gaussian mean, SAC's squashed mean) and `policy.json` (what it expects:
observation terms and sizes, action size and range, the task's SHA256, the room).
`mpg eval --onnx policy.onnx --task task.yaml` runs it with onnxruntime: no SB3
checkpoint, no pickle. Vector (Box) observations only; camera (Dict) policies
are not exported.

The post-processing matches SB3's `predict`: actions are clipped to the action
range, or (SAC, squashed output) mapped from [-1, 1] onto it.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np

from .task import RESOLVED_NAME

FORMAT = "mpg-onnx-policy/1"


class OnnxPolicyError(ValueError):
    """The run cannot be exported, or the ONNX policy does not fit the task."""


def file_sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def obs_terms(env) -> list[dict[str, Any]]:
    """Name and size of each vector observation term, in the order they are concatenated."""
    env.reset(seed=0)
    return [{"name": it.name, "size": int(np.asarray(it.fn(env)).size)}
            for it in env.obs_items if not it.image]


def export(run_dir: str | Path, out_dir: str | Path | None = None) -> dict[str, Any]:
    import torch
    from gymnasium import spaces
    from stable_baselines3.common.base_class import BaseAlgorithm

    from .env import PuzzleEnv
    from .train import load_run, load_trainer

    run = load_run(run_dir)            # checks the run folder is trusted (R6)
    trainer = load_trainer(run)
    model = getattr(trainer, "model", None)
    if not isinstance(model, BaseAlgorithm):
        raise OnnxPolicyError(f"algo {run['summary']['algo']!r} is not an SB3 model; "
                              "only ppo, a2c and sac runs can be exported")
    if not isinstance(model.observation_space, spaces.Box):
        raise OnnxPolicyError("camera (Dict) observation policies cannot be exported; "
                              "vector (Box) observations only")
    policy = model.policy.to("cpu").eval()
    squash = bool(getattr(policy, "squash_output", False))
    sac = hasattr(policy, "actor")

    class Actor(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.policy = policy

        def forward(self, obs):
            if sac:
                return self.policy.actor(obs, deterministic=True)
            return self.policy(obs, deterministic=True)[0]

    obs_dim = int(model.observation_space.shape[0])
    out = Path(out_dir) if out_dir else Path(run["dir"]) / "onnx"
    out.mkdir(parents=True, exist_ok=True)
    onnx_path = out / "policy.onnx"
    import warnings

    with torch.no_grad(), warnings.catch_warnings():
        # SB3's Gaussian checks its arguments in Python; the trace keeps the mean path only
        warnings.simplefilter("ignore", torch.jit.TracerWarning)
        torch.onnx.export(Actor(), (torch.zeros(1, obs_dim),), str(onnx_path), dynamo=False,
                          input_names=["obs"], output_names=["action"])
    env = PuzzleEnv(task=run["task"])
    try:
        terms = obs_terms(env)
    finally:
        env.close()
    act = model.action_space
    meta = {
        "format": FORMAT,
        "algo": run["summary"]["algo"],
        "run_id": run["summary"]["run_id"],
        "room": Path(run["task"]["room"]).stem,
        "task_sha256": file_sha256(Path(run["dir"]) / RESOLVED_NAME),
        "obs_dim": obs_dim,
        "obs_terms": terms,
        "action_dim": int(act.shape[0]),
        "action_low": [float(x) for x in act.low],
        "action_high": [float(x) for x in act.high],
        "squashed": squash,
    }
    json_path = out / "policy.json"
    json_path.write_text(json.dumps(meta, indent=2) + "\n")
    return {"onnx": str(onnx_path), "json": str(json_path), "obs_dim": obs_dim,
            "action_dim": meta["action_dim"], "algo": meta["algo"]}


class OnnxPolicy:
    """`predict(obs)` like a Trainer, from policy.onnx + policy.json via onnxruntime."""

    def __init__(self, path: str | Path):
        import onnxruntime as ort

        path = Path(path)
        meta_path = path.with_suffix(".json")
        if not meta_path.exists():
            raise OnnxPolicyError(f"{meta_path} missing: `mpg export-onnx` writes it next to "
                                  f"{path.name}")
        self.meta = json.loads(meta_path.read_text())
        if self.meta.get("format") != FORMAT:
            raise OnnxPolicyError(f"{meta_path}: format {self.meta.get('format')!r}, "
                                  f"expected {FORMAT!r}")
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 1
        self.session = ort.InferenceSession(str(path), opts, providers=["CPUExecutionProvider"])
        self.low = np.asarray(self.meta["action_low"], dtype=np.float32)
        self.high = np.asarray(self.meta["action_high"], dtype=np.float32)

    def predict(self, obs, deterministic: bool = True) -> np.ndarray:
        x = np.asarray(obs, dtype=np.float32).reshape(1, -1)
        a = self.session.run(None, {"obs": x})[0][0]
        if self.meta["squashed"]:
            return self.low + 0.5 * (a + 1.0) * (self.high - self.low)
        return np.clip(a, self.low, self.high)


def fit_report(meta: dict[str, Any], env) -> list[str]:
    """Raises when the policy cannot drive `env` (sizes differ); returns warnings
    (same sizes, different observation terms). `task_sha256` in policy.json is a
    record of the training task, not compared: a task file and the resolved
    task the run saved never hash alike."""
    from gymnasium import spaces

    if not isinstance(env.observation_space, spaces.Box):
        raise OnnxPolicyError("the task has camera (Dict) observations; ONNX policies take "
                              "vector observations only")
    got = int(env.observation_space.shape[0])
    if got != meta["obs_dim"]:
        names = [t["name"] for t in meta["obs_terms"]]
        raise OnnxPolicyError(f"the task gives {got} observation values, the policy expects "
                              f"{meta['obs_dim']} (terms {names})")
    if int(env.action_space.shape[0]) != meta["action_dim"]:
        raise OnnxPolicyError(f"the task takes {env.action_space.shape[0]} action values, the "
                              f"policy gives {meta['action_dim']}")
    terms = obs_terms(env)
    if terms != meta["obs_terms"]:
        return [f"observation terms differ: task {terms}, policy {meta['obs_terms']}"]
    return []
