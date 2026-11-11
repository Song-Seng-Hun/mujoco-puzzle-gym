"""Fetch and verify pinned upstream assets (R1).

The pin list lives in `upstream.lock.json`: a GitHub commit, a Hugging Face
revision, and every needed file with its SHA256. Files land in
`~/.cache/mujoco-puzzle-gym/<commit>/<path>` (root overridable with
`MPG_CACHE_DIR`). A file whose hash does not match is never deleted: it is
renamed to `<name>.bad` and the fetch fails.
"""

from __future__ import annotations

import hashlib
import io
import json
import os
import tarfile
import urllib.request
from pathlib import Path
from typing import Any

LOCK_PATH = Path(__file__).with_name("upstream.lock.json")
USER_AGENT = "mujoco-puzzle-gym-fetch/0.1"


class AssetError(Exception):
    pass


def load_lock(lock_path: Path | None = None) -> dict[str, Any]:
    return json.loads(Path(lock_path or LOCK_PATH).read_text())


def cache_root() -> Path:
    env = os.environ.get("MPG_CACHE_DIR")
    return Path(env).expanduser() if env else Path.home() / ".cache" / "mujoco-puzzle-gym"


def cache_dir(lock: dict[str, Any] | None = None, root: Path | None = None) -> Path:
    lock = lock or load_lock()
    return Path(root or cache_root()) / lock["sources"]["github"]["commit"]


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _quarantine(path: Path) -> Path:
    bad = path.with_name(path.name + ".bad")
    if bad.exists():
        bad = path.with_name(f"{path.name}.{os.getpid()}.bad")
    path.rename(bad)
    return bad


def _http_get(url: str) -> bytes:
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=120) as resp:
        return resp.read()


def _download(lock: dict[str, Any], entries: list[dict[str, Any]]) -> dict[str, bytes]:
    """Return {path: bytes} for the given lock entries."""
    out: dict[str, bytes] = {}
    gh = [e for e in entries if e["source"] == "github"]
    if gh:
        src = lock["sources"]["github"]
        archive = _http_get(src["archive_url"])
        wanted = {e["remote"]: e["path"] for e in gh}
        with tarfile.open(fileobj=io.BytesIO(archive), mode="r:gz") as tar:
            for member in tar:
                if not member.isfile():
                    continue
                # GitHub archives have one top-level folder: <repo>-<commit>/
                rel = member.name.split("/", 1)[1] if "/" in member.name else member.name
                if rel in wanted:
                    out[wanted[rel]] = tar.extractfile(member).read()
        missing = sorted(set(wanted.values()) - set(out))
        if missing:
            raise AssetError(f"not in GitHub archive: {missing}")
    for e in entries:
        if e["source"] == "huggingface":
            tmpl = lock["sources"]["huggingface"]["url_template"]
            out[e["path"]] = _http_get(tmpl.format(remote=e["remote"]))
        elif e["source"] != "github":
            raise AssetError(f"unknown source {e['source']!r} for {e['path']}")
    return out


def fetch(lock_path: Path | None = None, root: Path | None = None) -> dict[str, Any]:
    """Make the cache match the lock. Raises AssetError on any hash mismatch."""
    lock = load_lock(lock_path)
    base = cache_dir(lock, root)
    entries = lock["files"]

    # 1. Verify what is already there (no network).
    bad, missing = [], []
    for e in entries:
        p = base / e["path"]
        if not p.exists():
            missing.append(e)
        elif sha256_file(p) != e["sha256"]:
            bad.append(str(_quarantine(p)))
    if bad:
        raise AssetError(f"SHA256 mismatch, renamed to .bad: {bad}")

    # 2. Download what is missing, verify before writing under the real name.
    if missing:
        blobs = _download(lock, missing)
        for e in missing:
            p = base / e["path"]
            p.parent.mkdir(parents=True, exist_ok=True)
            data = blobs[e["path"]]
            if hashlib.sha256(data).hexdigest() != e["sha256"]:
                badp = p.with_name(p.name + ".bad")
                badp.write_bytes(data)
                bad.append(str(badp))
                continue
            tmp = p.with_name(p.name + ".part")
            tmp.write_bytes(data)
            tmp.replace(p)
        if bad:
            raise AssetError(f"downloaded file SHA256 mismatch, saved as .bad: {bad}")

    gh, hf = lock["sources"]["github"], lock["sources"]["huggingface"]
    return {
        "verified": True,
        "downloaded": len(missing),
        "files": len(entries),
        "bytes": sum(e["size"] for e in entries),
        "cache_dir": str(base),
        "source": {
            "github": f"https://github.com/{gh['repo']}/tree/{gh['commit']}",
            "huggingface": f"https://huggingface.co/{hf['repo']}/tree/{hf['revision']}",
        },
        "license": lock["license"],
    }


def asset_path(rel: str, lock_path: Path | None = None) -> Path:
    return cache_dir(load_lock(lock_path)) / rel
