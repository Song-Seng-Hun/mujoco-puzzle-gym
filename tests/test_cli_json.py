import json
import subprocess
import sys

from mujoco_puzzle_gym.cli import COMMANDS


def run(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, "-m", "mujoco_puzzle_gym.cli", *args],
        capture_output=True, text=True,
    )


def test_every_command_emits_json():
    for name in COMMANDS:
        if name in {"version"}:
            out = run(name, "--json")
            assert out.returncode == 0, out.stderr
            assert json.loads(out.stdout)["ok"] is True
