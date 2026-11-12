"""Shared by the traps (R16): they act only while their signal is on.

A trap acts while its `in` channels are on (`require` all / any; no `in`:
always), `enable` is on and its run-time value `enabled` is 1 (a setter or a
difficulty step, R21, can switch it). A pit is the exception: without `in`
it is always open and rejects `enable` / `enabled` (pit.py). Its strength is
`power`, whose unit each trap's schema states.
"""

from __future__ import annotations

import math

import numpy as np

from ...schema import Field
from ..base import Part


class Trap(Part):
    stage = "device"
    fields = (
        Field("enabled", "int", 1, min=0, max=1, runtime=True,
              doc="0 turns the trap off (setters, difficulty steps)"),
    )

    def active(self) -> bool:
        """Whether the trap acts this body step."""
        return (self.enabled() and int(self.live["enabled"]) == 1
                and (self.inputs_on() if self.inputs else True))


def heading(yaw_deg: float) -> np.ndarray:
    a = math.radians(yaw_deg)
    return np.array([math.cos(a), math.sin(a)])


def yaw_quat(yaw_deg: float) -> list[float]:
    a = math.radians(yaw_deg) / 2
    return [math.cos(a), 0.0, 0.0, math.sin(a)]


def swept_half(yaw_deg: float, along: tuple[float, float], across: float) -> tuple[np.ndarray, np.ndarray]:
    """Axis-aligned (centre offset, half size) of a rectangle from `along[0]` to
    `along[1]` along heading `yaw_deg` and `across` wide (planning masks)."""
    d = heading(yaw_deg)
    n = np.array([-d[1], d[0]])
    pts = [d * a + n * s * across / 2 for a in along for s in (-1, 1)]
    pts = np.asarray(pts)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    return (lo + hi) / 2, (hi - lo) / 2
