"""R8: a user algorithm registered through the `mujoco_puzzle_gym.algos` entry point
group trains and evaluates end to end through the real CLI.

The fake package is installed the way pip would leave it: a module plus a
`*.dist-info` directory with `entry_points.txt`, found through PYTHONPATH.
"""

import json
import os
import subprocess
import sys
import textwrap

FAKE_MODULE = textwrap.dedent('''
    import json
    from pathlib import Path

    import numpy as np

    from mujoco_puzzle_gym.algos.base import Deadline, Trainer


    class ConstantTrainer(Trainer):
        """Learns nothing: always walks straight ahead."""

        def __init__(self, env, task, seed=0, action=(1.0, 0.0, 0.0)):
            super().__init__(env, task, seed)
            self.action = np.asarray(action, dtype=np.float32)
            self.steps = 0

        def train(self, seconds, max_steps=None):
            deadline = Deadline(seconds, max_steps)
            self.env.reset()
            while not deadline.done(self.steps):
                self.env.step(np.tile(self.action, (self.env.num_envs, 1)))
                self.steps += self.env.num_envs
            return self.steps

        def save(self, path):
            path = Path(path).with_suffix(".json")
            path.write_text(json.dumps({"action": self.action.tolist(), "steps": self.steps}))
            return path

        @classmethod
        def load(cls, path, task, env=None):
            return cls(env, task, action=json.loads(Path(path).read_text())["action"])

        def predict(self, obs, deterministic=True):
            return self.action
''')


def mpg(env, *args):
    out = subprocess.run([sys.executable, "-m", "mujoco_puzzle_gym.cli", *args, "--json"],
                         capture_output=True, text=True, env=env)
    assert out.returncode == 0, (args, out.stdout, out.stderr)
    return json.loads(out.stdout)


def test_entry_point_algo_trains_and_evaluates(asset_dir, repo_root, tmp_path):
    site = tmp_path / "site"
    (site / "fake_mpg_algo-0.1.dist-info").mkdir(parents=True)
    (site / "fake_mpg_algo.py").write_text(FAKE_MODULE)
    (site / "fake_mpg_algo-0.1.dist-info" / "METADATA").write_text(
        "Metadata-Version: 2.1\nName: fake-mpg-algo\nVersion: 0.1\n")
    (site / "fake_mpg_algo-0.1.dist-info" / "entry_points.txt").write_text(
        "[mujoco_puzzle_gym.algos]\nconstant = fake_mpg_algo:ConstantTrainer\n")
    env = {**os.environ, "PYTHONPATH": os.pathsep.join(filter(None, [str(site),
                                                                     os.environ.get("PYTHONPATH")]))}

    summary = mpg(env, "train", str(repo_root / "examples/task_ppo.yaml"), "--algo", "constant",
                  "--minutes", "1", "--max-steps", "1600", "--n-envs", "2",
                  "--runs-dir", str(tmp_path / "runs"), "--id", "fake")
    run = tmp_path / "runs" / "fake"
    assert summary["algo"] == "constant" and summary["checkpoint"] == "checkpoint.json"
    assert summary["env_steps"] >= 1600 and summary["episodes"] >= 2
    assert json.loads((run / "checkpoint.json").read_text())["action"] == [1.0, 0.0, 0.0]
    assert (run / "curves.png").exists() and (run / "task.resolved.yaml").exists()

    result = mpg(env, "eval", str(run), "--episodes", "2", "--workers", "2")
    assert result["episodes"] == 2 and 0.0 <= result["success_rate"] <= 1.0


def test_unknown_algo_fails_with_reason(asset_dir, repo_root, tmp_path):
    out = subprocess.run([sys.executable, "-m", "mujoco_puzzle_gym.cli", "train",
                          str(repo_root / "examples/task_ppo.yaml"), "--algo", "nope",
                          "--minutes", "0.1", "--runs-dir", str(tmp_path), "--json"],
                         capture_output=True, text=True)
    assert out.returncode != 0
    res = json.loads(out.stdout)
    assert res["ok"] is False and "unknown algo 'nope'" in res["error"] and "ppo" in res["error"]
    assert not any(tmp_path.iterdir()), "no run directory for a failed start"
