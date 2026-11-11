"""Puzzle parts (R4). Built-ins register here through `register_part` (R17)."""

from ..registry import register_part
from .base import Part
from .door import Door
from .goal import Goal
from .switch import Switch

register_part("switch", Switch, replace=True)
register_part("door", Door, replace=True)
register_part("goal", Goal, replace=True)

__all__ = ["Part", "Switch", "Door", "Goal"]
