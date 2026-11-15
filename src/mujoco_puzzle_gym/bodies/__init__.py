"""Walking bodies. Code outside this package gets a body only through `make_body`."""

from __future__ import annotations

from .base import (ADDON_MODES, BodyAdapter, BodyState, Capabilities, Pose, SubstepFn,
                   UNDECLARED_LIMIT, declared_limit, flat_floor_spec)

DEFAULT_BODY = "microduck"


#: Every body `make_body` knows, for error messages and the CLI's `--body` choices.
BODIES = ("microduck", "go2")


def lock_for(name: str = DEFAULT_BODY):
    """Path of the pinned-asset lock a body fetches from, or None for the default lock.

    Each body brings its own upstream repo and policy, and `assets.cache_dir` keys the
    cache on one GitHub commit, so a second body needs a second lock file (R40). Callers
    outside `bodies/` ask here instead of naming a body's files (R2).
    """
    if name == "microduck":
        return None                      # the package's default upstream.lock.json
    if name == "go2":
        from .go2 import LOCK
        return LOCK
    raise ValueError(f"unknown body {name!r}; known bodies: {', '.join(BODIES)}")


def make_body(name: str = DEFAULT_BODY, **kwargs) -> BodyAdapter:
    if name == "microduck":
        from .microduck import MicroduckAdapter
        return MicroduckAdapter(**kwargs)
    if name == "go2":
        from .go2 import Go2Adapter
        return Go2Adapter(**kwargs)
    raise ValueError(f"unknown body {name!r}; known bodies: {', '.join(BODIES)}")


__all__ = ["ADDON_MODES", "BODIES", "UNDECLARED_LIMIT", "declared_limit", "lock_for", "BodyAdapter", "BodyState", "Capabilities", "Pose", "SubstepFn", "flat_floor_spec", "make_body", "DEFAULT_BODY"]
