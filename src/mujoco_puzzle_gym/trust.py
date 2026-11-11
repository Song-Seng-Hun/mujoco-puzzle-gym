"""Where pickle files may be read from (community spec R6).

SB3 checkpoints (zip with pickles inside), flag snapshots (`*.pkl`) and a run's
`task.resolved.yaml` (its `plugins:` are imported) can all run code when opened.
They are read only from `./runs/` or the demo run bundled in the repo, unless
the user passes `--trust`, which sets `MPG_TRUST=1` so training and evaluation
worker processes inherit it.
"""

from __future__ import annotations

import os
from pathlib import Path

TRUST_ENV = "MPG_TRUST"
BUNDLED = Path(__file__).resolve().parents[2] / "examples" / "pretrained"


class UntrustedPathError(ValueError):
    """A pickle-bearing path outside the trusted folders, without --trust."""


def trusted_roots() -> list[Path]:
    return [(Path.cwd() / "runs").resolve(), BUNDLED]


def set_trust(on: bool) -> None:
    if on:
        os.environ[TRUST_ENV] = "1"


def require_trusted(path: str | Path, what: str) -> Path:
    """`path` resolved; raises UntrustedPathError when it lies outside the trusted roots."""
    p = Path(path).resolve()
    if os.environ.get(TRUST_ENV) == "1":
        return p
    if any(p == root or p.is_relative_to(root) for root in trusted_roots()):
        return p
    raise UntrustedPathError(
        f"{what} {p} is outside ./runs/. Pickle files can run code when opened, so open only "
        "files you made yourself. If you trust it, add --trust.")
