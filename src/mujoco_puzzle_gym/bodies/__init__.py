"""Walking bodies. Code outside this package gets a body only through `make_body`."""

from __future__ import annotations

from .base import (ADDON_MODES, BodyAdapter, BodyState, Capabilities, Pose, SubstepFn,
                   UNDECLARED_LIMIT, declared_limit, flat_floor_spec)

DEFAULT_BODY = "microduck"


#: Every body `make_body` knows, for error messages and the CLI's `--body` choices.
BODIES = ("microduck", "go2", "so101")


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
    if name == "so101":
        from .so101 import LOCK
        return LOCK
    raise ValueError(f"unknown body {name!r}; known bodies: {', '.join(BODIES)}")


def resolve(name: str | None) -> str:
    """The body a task or command line means: `default` and nothing are the package default.

    One place decides what "default" is, so `--body` (R49), `task.yaml` `body:` and the
    env all agree, and a run record can name the body it actually used instead of the
    word `default`.
    """
    return DEFAULT_BODY if not name or name == "default" else str(name)


def make_body(name: str = DEFAULT_BODY, **kwargs) -> BodyAdapter:
    name = resolve(name)
    if name == "microduck":
        from .microduck import MicroduckAdapter
        return MicroduckAdapter(**kwargs)
    if name == "go2":
        from .go2 import Go2Adapter
        return Go2Adapter(**kwargs)
    if name == "so101":
        from .so101 import So101Adapter
        return So101Adapter(**kwargs)
    raise ValueError(f"unknown body {name!r}; known bodies: {', '.join(BODIES)}")


__all__ = ["ADDON_MODES", "BODIES", "UNDECLARED_LIMIT", "declared_limit", "lock_for", "resolve", "BodyAdapter", "BodyState", "Capabilities", "Pose", "SubstepFn", "flat_floor_spec", "make_body", "DEFAULT_BODY"]
