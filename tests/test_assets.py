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


# ---- R40: the Go2 lock, its own file, and a paired ONNX -------------------

GO2_COMMIT = "370c43965323c0944b433b202aac5058b5f907a9"
GO2_HF_REV = "9a723b7ab0784cd86abb942836aa1f208ddde891"
POLICY_PAIR = ("policies/policy.onnx", "policies/policy.onnx.data")


def _go2_lock():
    from mujoco_puzzle_gym.bodies.go2 import LOCK

    return LOCK, assets.load_lock(LOCK)


def test_go2_lock_is_separate_and_pinned():
    """The Go2 assets live in their own lock: `cache_dir` keys the cache on one GitHub
    commit, so two bodies from two repos cannot share one lock file."""
    path, lock = _go2_lock()
    assert path.name == "upstream.go2.lock.json"
    assert lock["name"] == "go2"
    assert lock["sources"]["github"]["commit"] == GO2_COMMIT
    assert lock["sources"]["huggingface"]["revision"] == GO2_HF_REV
    assert "url_template" in lock["sources"]["github"], "Menagerie is fetched per file"
    paths = [e["path"] for e in lock["files"]]
    assert len(paths) == len(set(paths))
    assert set(POLICY_PAIR) <= set(paths), "the external-data file must be pinned too"
    assert {"robot/go2.xml", "robot/scene.xml", "robot/LICENSE"} <= set(paths)
    for e in lock["files"]:
        assert re.fullmatch(r"[0-9a-f]{64}", e["sha256"]), e
        assert e["source"] in {"github", "huggingface"}
    assert assets.cache_dir(lock).name == GO2_COMMIT
    assert assets.cache_dir(lock) != assets.cache_dir(assets.load_lock())


def test_go2_license_names_both_sources_and_denies_officialness():
    """R51 groundwork: the policy is a community checkpoint, and the lock says so."""
    _, lock = _go2_lock()
    text = json.dumps(lock["license"])
    assert "BSD-3-Clause" in lock["license"]["mjcf_and_meshes"]
    assert "BSD-3-Clause" in lock["license"]["policies"]
    assert "mujoco_menagerie" in text and "diasAiMaster" in text
    assert "NOT an official Unitree release" in text


def _pair_lock(tmp_path, go2_cache, corrupt: str):
    """A two-file lock holding just the ONNX pair, cached locally with one file broken."""
    path, lock = _go2_lock()
    from pathlib import Path

    lock["files"] = [e for e in lock["files"] if e["path"] in POLICY_PAIR]
    assert len(lock["files"]) == 2
    lock_path = tmp_path / "go2-pair.json"
    lock_path.write_text(json.dumps(lock))
    base = assets.cache_dir(lock, tmp_path)
    for e in lock["files"]:
        dst = base / e["path"]
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes((Path(go2_cache) / e["path"]).read_bytes())
    broken = base / corrupt
    broken.write_bytes(broken.read_bytes()[:-1] + b"\x00")
    return lock_path, base


@pytest.mark.parametrize("corrupt", POLICY_PAIR)
def test_one_broken_half_of_the_onnx_pair_fails_closed(tmp_path, go2_asset_dir, corrupt):
    """Either half corrupted must stop the fetch, so no half-model is ever loaded.

    `fetch` raises before anything reads the cache, which is the property that matters;
    the broken file is quarantined and the intact one is left alone for the retry.
    """
    lock_path, base = _pair_lock(tmp_path, go2_asset_dir, corrupt)
    with pytest.raises(assets.AssetError, match="mismatch"):
        assets.fetch(lock_path, root=tmp_path)
    assert not (base / corrupt).exists()
    assert (base / corrupt).with_name(os.path.basename(corrupt) + ".bad").exists()
    intact = [p for p in POLICY_PAIR if p != corrupt][0]
    assert (base / intact).exists(), "the good half stays; the raise is what protects us"


def test_cli_fetch_assets_body_go2(go2_asset_dir):
    """R40 Check: the command names the body and reports that body's lock."""
    out = subprocess.run(
        [sys.executable, "-m", "mujoco_puzzle_gym.cli", "fetch-assets", "--body", "go2", "--json"],
        capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
    result = json.loads(out.stdout)
    assert result["ok"] is True and result["verified"] is True
    assert result["body"] == "go2"
    assert result["files"] == len(_go2_lock()[1]["files"])
    assert GO2_COMMIT in result["cache_dir"]


def test_cli_fetch_assets_rejects_an_unknown_body():
    out = subprocess.run(
        [sys.executable, "-m", "mujoco_puzzle_gym.cli", "fetch-assets", "--body", "spot", "--json"],
        capture_output=True, text=True)
    assert out.returncode != 0
    assert "invalid choice" in out.stderr and "go2" in out.stderr


# ---- one upstream commit, two bodies (M4 R64) ----------------------------------------

def test_no_two_locks_claim_the_same_cache_path_for_different_content():
    """Two bodies pinned from the same upstream commit must not share a local path.

    `cache_dir` keys the cache on the commit, so bodies from one commit land in one
    directory, and a path like `robot/scene.xml` means a different file for each of them.
    It happened: adding the arm, whose model is in the same Menagerie commit as the
    quadruped's, overwrote the quadruped's `robot/scene.xml` and `robot/LICENSE` and then
    renamed them `.bad` for failing the arm's hashes. The arm's paths are under `arm/` now,
    and this is the check that keeps the next body from repeating it.
    """
    from pathlib import Path

    from mujoco_puzzle_gym.bodies import BODIES, lock_for

    claims: dict[tuple[str, str], tuple[str, str]] = {}     # (cache key, path) -> (body, sha)
    for body in BODIES:
        path = lock_for(body)
        lock = json.loads(Path(path).read_text()) if path else json.loads(
            (Path(assets.__file__).parent / "upstream.lock.json").read_text())
        key = lock["sources"]["github"]["commit"]
        for entry in lock["files"]:
            seen = claims.get((key, entry["path"]))
            assert seen is None or seen[1] == entry["sha256"], (
                f"{body} and {seen[0]} both write {entry['path']!r} into the cache for "
                f"commit {key[:7]}, with different content")
            claims[(key, entry["path"])] = (body, entry["sha256"])


def test_a_lock_may_pin_a_model_without_a_policy():
    """`fetch` used to read both a github and a huggingface source unconditionally, which
    is the shape of a body that pins a model *and* a policy. A body whose control needs no
    policy pins a model alone, and the report built at the end raised a KeyError after
    every file had already been verified."""
    from mujoco_puzzle_gym.bodies.so101 import LOCK

    lock = json.loads(LOCK.read_text())
    assert set(lock["sources"]) == {"github"}
    result = assets.fetch(LOCK)
    assert result["verified"] is True
    assert set(result["source"]) == {"github"}


def test_a_dropped_connection_is_retried(monkeypatch):
    """CI's fast suite failed on `ConnectionResetError: [Errno 104] Connection reset by peer`
    while fixtures fetched two bodies' models - five fixture setups and one test with it,
    where the same shard passed locally. One dropped connection should not fail a run.

    Nothing about the verification is relaxed: the caller hashes what arrives, so this only
    decides how many times the bytes are asked for.
    """
    from mujoco_puzzle_gym import assets

    calls = []

    class Resp:
        def read(self):
            return b"payload"

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

    def flaky(req, timeout=None):
        calls.append(timeout)
        if len(calls) == 1:
            raise ConnectionResetError(104, "Connection reset by peer")
        return Resp()

    monkeypatch.setattr(assets.urllib.request, "urlopen", flaky)
    monkeypatch.setattr(assets.time, "sleep", lambda s: None)
    assert assets._http_get("https://example.invalid/x") == b"payload"
    assert len(calls) == 2, calls


def test_a_missing_file_is_not_retried(monkeypatch):
    """The other side: a 404 is an answer - the lock names something that is not there - and
    asking twice more only delays the error that has to be read."""
    import urllib.error

    from mujoco_puzzle_gym import assets

    calls = []

    def missing(req, timeout=None):
        calls.append(timeout)
        raise urllib.error.HTTPError("https://example.invalid/x", 404, "Not Found", {}, None)

    monkeypatch.setattr(assets.urllib.request, "urlopen", missing)
    monkeypatch.setattr(assets.time, "sleep", lambda s: None)
    with pytest.raises(urllib.error.HTTPError):
        assets._http_get("https://example.invalid/x")
    assert len(calls) == 1, calls
