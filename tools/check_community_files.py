"""Files a PR adds under community/ (community spec R1). Standard library only.

    python tools/check_community_files.py community/rooms/alice/room.yaml ...

Accepted: `community/rooms/<user>/<name>.yaml` and `community/rooms/<user>/README.md`,
each at most 64 KB. Prints one line per problem and exits 1 when there is any.
"""

from __future__ import annotations

import sys
from pathlib import Path

MAX_BYTES = 64 * 1024


def problems(paths: list[str], root: Path = Path(".")) -> list[str]:
    out = []
    for raw in paths:
        p = Path(raw)
        parts = p.parts
        if len(parts) != 4 or parts[:2] != ("community", "rooms"):
            out.append(f"{raw}: files go in community/rooms/<GitHub user>/ (no subfolders)")
            continue
        if not (p.suffix == ".yaml" or p.name == "README.md"):
            out.append(f"{raw}: only room files (*.yaml) and README.md are accepted")
            continue
        f = root / p
        if f.is_symlink():
            out.append(f"{raw}: symbolic links are not accepted")
        elif f.exists() and f.stat().st_size > MAX_BYTES:
            out.append(f"{raw}: {f.stat().st_size} bytes; the limit is {MAX_BYTES}")
    return out


def main(argv: list[str]) -> int:
    found = problems(argv)
    for line in found:
        print(line)
    return 1 if found else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
