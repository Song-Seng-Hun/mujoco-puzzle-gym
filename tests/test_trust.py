"""R6 (community spec): pickle-bearing paths load only from ./runs/ unless --trust."""

import json
from types import SimpleNamespace

import pytest

from mujoco_puzzle_gym import curriculum, trust
from mujoco_puzzle_gym.cli import main
from mujoco_puzzle_gym.train import load_run
from mujoco_puzzle_gym.trust import UntrustedPathError, require_trusted


@pytest.fixture
def untrusted(monkeypatch, tmp_path):
    monkeypatch.delenv(trust.TRUST_ENV, raising=False)
    work = tmp_path / "work"
    (work / "runs" / "mine").mkdir(parents=True)
    monkeypatch.chdir(work)
    outside = tmp_path / "downloaded"
    outside.mkdir()
    return SimpleNamespace(work=work, outside=outside)


def test_outside_rejected_with_reason(untrusted):
    with pytest.raises(UntrustedPathError, match="Pickle files can run code"):
        require_trusted(untrusted.outside, "run folder")


def test_dotdot_escape_rejected(untrusted):
    with pytest.raises(UntrustedPathError):
        require_trusted("runs/../../downloaded", "run folder")


def test_under_runs_and_bundled_demo_pass(untrusted):
    assert require_trusted("runs/mine", "run folder") == untrusted.work / "runs" / "mine"
    require_trusted(trust.BUNDLED / "switch_door", "run folder")


def test_trust_env_passes(untrusted, monkeypatch):
    monkeypatch.setenv(trust.TRUST_ENV, "1")
    assert require_trusted(untrusted.outside, "run folder") == untrusted.outside


def test_load_run_checks_before_reading(untrusted):
    # rejected before summary.json or task.resolved.yaml (plugin imports) are read
    with pytest.raises(UntrustedPathError):
        load_run(untrusted.outside)


def test_snapshots_outside_rejected(untrusted):
    env = SimpleNamespace(task={"curriculum": {"snapshots": str(untrusted.outside)}, "room": "r"},
                          room=SimpleNamespace(name="r"), parts={})
    with pytest.raises(UntrustedPathError):
        curriculum.load_snapshots(env)


def test_cli_eval_rejects_then_trust_passes(untrusted, capsys):
    assert main(["eval", str(untrusted.outside), "--json"]) == 1
    out = json.loads(capsys.readouterr().out)
    assert out["ok"] is False and "Pickle" in out["error"]
    assert main(["eval", str(untrusted.outside), "--trust", "--json"]) == 1
    out = json.loads(capsys.readouterr().out)
    assert "summary.json" in out["error"]       # got past the trust check
