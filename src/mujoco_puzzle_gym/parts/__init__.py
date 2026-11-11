"""Puzzle parts (R4). Built-ins register here through `register_part` (R17)."""

from ..registry import register_part
from .base import Part
from .device import Door, SwingDoor, Trapdoor, Warp
from .floor import FloorTile
from .goal import Goal
from .input import Button, Checkpoint, Laser, PressurePlate, Receptacle, Zone
from .logic import Counter, Gate, Receiver, Transmitter
from .switch import Switch

register_part("switch", Switch, replace=True)
register_part("door", Door, replace=True)
register_part("goal", Goal, replace=True)
register_part("gate", Gate, replace=True)
register_part("counter", Counter, replace=True)
register_part("transmitter", Transmitter, replace=True)
register_part("receiver", Receiver, replace=True)
register_part("button", Button, replace=True)
register_part("zone", Zone, replace=True)
register_part("laser", Laser, replace=True)
register_part("pressure", PressurePlate, replace=True)
register_part("receptacle", Receptacle, replace=True)
register_part("checkpoint", Checkpoint, replace=True)
register_part("swing_door", SwingDoor, replace=True)
register_part("trapdoor", Trapdoor, replace=True)
register_part("warp", Warp, replace=True)
register_part("floor_tile", FloorTile, replace=True)

__all__ = ["Part", "Switch", "Door", "Goal", "Gate", "Counter", "Transmitter", "Receiver",
           "Button", "Zone", "Laser", "PressurePlate", "Receptacle", "Checkpoint",
           "SwingDoor", "Trapdoor", "Warp", "FloorTile"]
