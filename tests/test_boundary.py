"""R2: outside bodies/, source must not name a concrete body, its policy or its joints."""

import re
from pathlib import Path

import mujoco_puzzle_gym

PKG = Path(mujoco_puzzle_gym.__file__).parent
FORBIDDEN = [
    "microduck", "velstand", "trunk_base",
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
]
# Pinned asset manifest: data that says which upstream files to fetch, not code.
ALLOWLIST = {"upstream.lock.json"}
PATTERN = re.compile("|".join(map(re.escape, FORBIDDEN)), re.IGNORECASE)


def scan(root: Path, skip_bodies: bool = True) -> list[str]:
    hits = []
    for path in sorted(root.rglob("*")):
        rel = path.relative_to(root)
        if not path.is_file() or "__pycache__" in rel.parts:
            continue
        if skip_bodies and rel.parts[0] == "bodies":
            continue
        if str(rel) in ALLOWLIST:
            continue
        for n, line in enumerate(path.read_text(errors="ignore").splitlines(), 1):
            if PATTERN.search(line):
                hits.append(f"{rel}:{n}: {line.strip()}")
    return hits


def test_no_body_names_outside_bodies():
    assert scan(PKG) == []


def test_scanner_sees_names_inside_bodies():
    assert any(h.startswith("bodies/") for h in scan(PKG, skip_bodies=False))
