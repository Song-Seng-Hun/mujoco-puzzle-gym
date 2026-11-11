"""Files a PR adds under community/ (community spec R1). Standard library only.

    python tools/check_community_files.py --base <sha> [--author <login>] [--rooms-out rooms.txt]
    python tools/check_community_files.py community/rooms/alice/room.yaml ...

Accepted: `community/rooms/<user>/<name>.yaml` and `community/rooms/<user>/README.md`,
each at most 64 KB, names of letters, digits, `.`, `_` and `-` only (no spaces:
the CI loop splits the room list on lines). With `--author`, `<user>` must be the
PR author. With `--base`, the paths come from `git diff -z` against that commit,
and a PR that changes community/ must change nothing else: CI runs the PR's own
checker and `mpg`, so a room PR that also edits them could pass itself.
Prints one line per problem and exits 1 when there is any; `--rooms-out` gets the
accepted room files, one per line.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

MAX_BYTES = 64 * 1024
SAFE = re.compile(r"[A-Za-z0-9_-][A-Za-z0-9._-]*")


def _diff(base: str, *spec: str, filt: str = "ACMRT") -> list[str]:
    out = subprocess.run(["git", "diff", "-z", "--name-only", f"--diff-filter={filt}",
                          f"{base}...HEAD", "--", *spec],
                         check=True, capture_output=True).stdout
    return [p.decode("utf-8", "surrogateescape") for p in out.split(b"\0") if p]


def changed(base: str) -> list[str]:
    return _diff(base, "community/")


def outside(base: str) -> list[str]:
    """Files the PR changes (deletions included) outside community/."""
    return _diff(base, ".", ":(exclude)community/", filt="ACDMRT")


def problems(paths: list[str], root: Path = Path("."), author: str | None = None) -> list[str]:
    out = []
    for raw in paths:
        p = Path(raw)
        parts = p.parts
        if len(parts) != 4 or parts[:2] != ("community", "rooms"):
            out.append(f"{raw!r}: files go in community/rooms/<GitHub user>/ (no subfolders)")
            continue
        if not all(SAFE.fullmatch(s) for s in parts[2:]):
            out.append(f"{raw!r}: use letters, digits, '.', '_' and '-' in folder and file names")
            continue
        if not (p.suffix == ".yaml" or p.name == "README.md"):
            out.append(f"{raw}: only room files (*.yaml) and README.md are accepted")
            continue
        if author is not None and parts[2].lower() != author.lower():
            out.append(f"{raw}: PRs add files only in their author's folder "
                       f"community/rooms/{author}/")
        f = root / p
        if f.is_symlink():
            out.append(f"{raw}: symbolic links are not accepted")
        elif not f.is_file():
            out.append(f"{raw}: listed as changed but not a file")
        elif f.stat().st_size > MAX_BYTES:
            out.append(f"{raw}: {f.stat().st_size} bytes; the limit is {MAX_BYTES}")
    return out


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="*")
    ap.add_argument("--base", default=None, help="diff against this commit (CI)")
    ap.add_argument("--author", default=None, help="PR author's GitHub login")
    ap.add_argument("--rooms-out", default=None, help="write accepted room files here")
    args = ap.parse_args(argv)
    paths = changed(args.base) if args.base else args.paths
    for p in paths:
        print(f"changed: {p!r}")
    found = problems(paths, author=args.author)
    if args.base and paths:
        # the checks below run this PR's own code; a room PR must not change it
        other = outside(args.base)
        if other:
            found.append(f"this PR changes community/ and {len(other)} file(s) outside it "
                         f"(e.g. {other[0]!r}); send rooms in a PR of their own")
    for line in found:
        print(line)
    if args.rooms_out:
        rooms = [] if found else [p for p in paths if p.endswith(".yaml")]
        Path(args.rooms_out).write_text("".join(f"{p}\n" for p in rooms))
    return 1 if found else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
