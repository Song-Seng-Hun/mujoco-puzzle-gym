import json
import subprocess
import sys

from mujoco_puzzle_gym.cli import COMMANDS

# Fast invocations that must print one JSON object (assets come from the cache).
FAST_ARGS = {
    "version": [],
    "fetch-assets": [],
    "bench": ["--seconds", "0.3", "--procs", "2"],
}


def run(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, "-m", "mujoco_puzzle_gym.cli", *args],
        capture_output=True, text=True,
    )


def test_every_command_emits_json(asset_dir):
    assert set(COMMANDS) == set(FAST_ARGS), "add a fast --json invocation for new commands"
    for name, extra in FAST_ARGS.items():
        out = run(name, "--json", *extra)
        assert out.returncode == 0, (name, out.stdout, out.stderr)
        result = json.loads(out.stdout)
        assert result["ok"] is True, result


def test_fetch_assets_json_fields(asset_dir):
    result = json.loads(run("fetch-assets", "--json").stdout)
    assert result["verified"] is True and result["downloaded"] == 0


def test_bench_json_fields(asset_dir):
    result = json.loads(run("bench", "--json", "--seconds", "0.3", "--procs", "2").stdout)
    assert result["physics_steps_per_body_step"] == 4
    assert result["body_steps_per_high_level_step"] == 5
    assert result["single"]["high_level_steps_per_s"] > 0
    assert [p["procs"] for p in result["parallel"]] == [2]


def test_bad_arguments_fail_nonzero_with_reason():
    out = run("bench", "--json", "--procs", "x")
    assert out.returncode != 0
    result = json.loads(out.stdout)
    assert result["ok"] is False and "--procs" in result["error"]
