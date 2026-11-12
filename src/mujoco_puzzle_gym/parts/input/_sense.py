"""Shared helpers for input parts: who is where (R4, R5)."""

from __future__ import annotations

from typing import Iterator

import numpy as np

from ..base import TRIGGERS, Part

BODY_KEY = "body"


def candidates(part: Part, data, body, trigger: str,
               names: list[str] | None = None) -> Iterator[tuple[object, np.ndarray]]:
    """(key, position) of everything `trigger` counts: the walking body (key
    "body", trunk position) and free bodies of matching kind (key = root body
    id, body position), skipping bodies the walking body holds and parked ones
    (a dispenser's stored copies, R28). `names`, when
    given, keeps only object parts with those names."""
    kinds = TRIGGERS[trigger]
    if "body" in kinds and body is not None and not names:
        yield BODY_KEY, np.asarray(body.pos, dtype=float)
    world = part.world
    if world is None:
        return
    for rid, owner in world.objects.items():
        if world.kinds.get(rid) not in kinds or rid in world.held or owner.parked:
            continue
        if names and owner.name not in names:
            continue
        yield rid, np.array(data.xpos[rid])


def in_rect(xy, centre, half, margin: float = 0.0) -> bool:
    d = np.abs(np.asarray(xy[:2], dtype=float) - centre)
    return bool((d <= np.asarray(half) + margin).all())
