"""Walking bodies. Code outside this package gets a body only through `make_body`."""

from __future__ import annotations

from .base import BodyAdapter, BodyState, Capabilities, Pose, SubstepFn, flat_floor_spec

DEFAULT_BODY = "microduck"


def make_body(name: str = DEFAULT_BODY, **kwargs) -> BodyAdapter:
    if name == "microduck":
        from .microduck import MicroduckAdapter
        return MicroduckAdapter(**kwargs)
    raise ValueError(f"unknown body {name!r}")


__all__ = ["BodyAdapter", "BodyState", "Capabilities", "Pose", "SubstepFn", "flat_floor_spec", "make_body", "DEFAULT_BODY"]
