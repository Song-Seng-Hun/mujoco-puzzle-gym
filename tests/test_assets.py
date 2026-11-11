import hashlib
import json
import os
import re
import subprocess
import sys

import pytest

from mujoco_puzzle_gym import assets

COMMIT = "0daab160d3fb1c2cbaba16a4a19052f36709dd9e"
HF_REV = "283771bcb3690a0a94a288c108365d9b630d21e4"


def _one_file_lock(tmp_path, content: bytes):
    lock = assets.load_lock()
    lock["files"] = [{"path": "policies/x.bin", "source": "huggingface", "remote": "x.bin",
                      "size": len(content), "sha256": hashlib.sha256(content).hexdigest()}]
    path = tmp_path / "lock.json"
    path.write_text(json.dumps(lock))
    return path


def test_lock_pins_commit_revision_and_hashes():
    lock = assets.load_lock()
    assert lock["sources"]["github"]["commit"] == COMMIT
    assert lock["sources"]["huggingface"]["revision"] == HF_REV
    paths = [e["path"] for e in lock["files"]]
    assert len(paths) == len(set(paths))
    assert {"LICENSE-MODELS.md", "policies/manifest.json"} <= set(paths)
    assert any(p.endswith(".onnx") for p in paths) and any(p.endswith(".xml") for p in paths)
    for e in lock["files"]:
        assert re.fullmatch(r"[0-9a-f]{64}", e["sha256"]), e
        assert e["source"] in {"github", "huggingface"}
    assert "CC BY-NC-SA 4.0" in lock["license"]["meshes"]


def test_second_run_downloads_nothing(tmp_path):
    lock_path = _one_file_lock(tmp_path, b"ok")
    target = assets.cache_dir(assets.load_lock(lock_path), tmp_path) / "policies/x.bin"
    target.parent.mkdir(parents=True)
    target.write_bytes(b"ok")
    result = assets.fetch(lock_path, root=tmp_path)
    assert result["verified"] is True and result["downloaded"] == 0


def test_corrupted_cache_file_is_renamed_bad_and_fails(tmp_path):
    lock_path = _one_file_lock(tmp_path, b"expected")
    target = assets.cache_dir(assets.load_lock(lock_path), tmp_path) / "policies/x.bin"
    target.parent.mkdir(parents=True)
    target.write_bytes(b"corrupted")
    with pytest.raises(assets.AssetError, match="mismatch"):
        assets.fetch(lock_path, root=tmp_path)
    assert not target.exists()
    assert target.with_name("x.bin.bad").read_bytes() == b"corrupted"


def test_corrupted_download_is_saved_bad_and_fails(tmp_path, monkeypatch):
    lock_path = _one_file_lock(tmp_path, b"expected")
    monkeypatch.setattr(assets, "_download", lambda lock, entries: {"policies/x.bin": b"tampered"})
    with pytest.raises(assets.AssetError, match="mismatch"):
        assets.fetch(lock_path, root=tmp_path)
    target = assets.cache_dir(assets.load_lock(lock_path), tmp_path) / "policies/x.bin"
    assert not target.exists()
    assert target.with_name("x.bin.bad").read_bytes() == b"tampered"


def test_cli_corrupted_real_lock_fails_nonzero(tmp_path):
    """Real lock, one corrupted file: fails before any download (no network)."""
    lock = assets.load_lock()
    entry = next(e for e in lock["files"] if e["path"] == "policies/manifest.json")
    target = tmp_path / COMMIT / entry["path"]
    target.parent.mkdir(parents=True)
    target.write_text("{}")
    out = subprocess.run(
        [sys.executable, "-m", "mujoco_puzzle_gym.cli", "fetch-assets", "--json"],
        capture_output=True, text=True, env={**os.environ, "MPG_CACHE_DIR": str(tmp_path)},
    )
    assert out.returncode != 0
    result = json.loads(out.stdout)
    assert result["ok"] is False and ".bad" in result["error"]
    assert target.with_name("manifest.json.bad").exists()


@pytest.mark.network
def test_fetch_real_assets_twice():
    first = assets.fetch()
    assert first["verified"] is True and first["files"] == len(assets.load_lock()["files"])
    second = assets.fetch()
    assert second["downloaded"] == 0
    assert "CC BY-NC-SA 4.0" in json.dumps(second["license"])
    assert COMMIT in second["source"]["github"] and HF_REV in second["source"]["huggingface"]
