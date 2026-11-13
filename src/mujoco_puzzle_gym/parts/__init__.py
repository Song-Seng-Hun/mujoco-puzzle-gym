"""Puzzle parts (R4). Built-ins register here through `register_part` (R17)."""

from ..registry import register_part
from .base import Part
from .beak import Dial, Dispenser, Drawer, Egg, HandleBall, Key, Laundry, Lever, PullCord, Stick
from .device import (AddonStation, Conveyor, Door, GoalNet, Pillar, Platform, Setter, Spring, SwingDoor,
                     Trapdoor, Turntable, Warp)
from .floor import FloorTile
from .floor.coin import Coin, CoinGroup, CoinLock
from .goal import Goal
from .input import Button, Checkpoint, Laser, PressurePlate, Receptacle, Target, Zone
from .logic import Counter, Gate, Receiver, Transmitter
from .npc import Npc
from .object import Ball, Cube, Pin
from .switch import Switch
from .trap import Cannon, Pendulum, Pit, Pusher, Spinner, Wind

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
register_part("coin", Coin, replace=True)
register_part("coin_group", CoinGroup, replace=True)
register_part("coin_lock", CoinLock, replace=True)
register_part("ball", Ball, replace=True)
register_part("pin", Pin, replace=True)
register_part("cube", Cube, replace=True)
register_part("goal_net", GoalNet, replace=True)
register_part("platform", Platform, replace=True)
register_part("pillar", Pillar, replace=True)
register_part("turntable", Turntable, replace=True)
register_part("conveyor", Conveyor, replace=True)
register_part("spring", Spring, replace=True)
register_part("setter", Setter, replace=True)
register_part("target", Target, replace=True)
register_part("wind", Wind, replace=True)
register_part("pusher", Pusher, replace=True)
register_part("spinner", Spinner, replace=True)
register_part("pendulum", Pendulum, replace=True)
register_part("pit", Pit, replace=True)
register_part("cannon", Cannon, replace=True)
register_part("npc", Npc, replace=True)
register_part("laundry", Laundry, replace=True)
register_part("key", Key, replace=True)
register_part("handle_ball", HandleBall, replace=True)
register_part("egg", Egg, replace=True)
register_part("stick", Stick, replace=True)
register_part("lever", Lever, replace=True)
register_part("pull_cord", PullCord, replace=True)
register_part("dial", Dial, replace=True)
register_part("drawer", Drawer, replace=True)
register_part("dispenser", Dispenser, replace=True)
register_part("addon_station", AddonStation, replace=True)

__all__ = ["Part", "Switch", "Door", "Goal", "Gate", "Counter", "Transmitter", "Receiver",
           "Button", "Zone", "Laser", "PressurePlate", "Receptacle", "Checkpoint",
           "SwingDoor", "Trapdoor", "Warp", "FloorTile", "Coin", "CoinGroup", "CoinLock",
           "Ball", "Pin", "Cube", "GoalNet", "Platform", "Pillar", "Turntable", "Conveyor", "Spring",
           "Setter", "Target", "Wind", "Pusher", "Spinner", "Pendulum", "Pit", "Cannon", "Npc",
           "Laundry", "Key", "HandleBall", "Egg", "Stick", "Lever", "PullCord", "Dial", "Drawer", "Dispenser",
           "AddonStation"]
