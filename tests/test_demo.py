"""`mpg demo` renders the bundled trained policy solving the example room."""

import json
import subprocess
import sys


def test_demo_renders_success(tmp_path):
    out = tmp_path / "demo.mp4"
    res = subprocess.run([sys.executable, "-m", "mujoco_puzzle_gym.cli", "demo", "--json", "--out", str(out)],
                         capture_output=True, text=True, timeout=300)
    assert res.returncode == 0, res.stderr
    data = json.loads(res.stdout)
    assert data["success"] is True
    assert out.exists() and out.stat().st_size > 10_000
