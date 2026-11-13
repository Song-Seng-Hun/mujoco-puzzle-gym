"""Traps (R16): wind, pusher, spinning bar, pendulum, pit, cannon; the brake strip (R30). Slippery and
sticky floors are `floor_tile` presets (`trap/slippery_floor`, `trap/sticky_floor`)."""

from .brake import BrakeStrip
from .cannon import Cannon
from .pendulum import Pendulum
from .pit import Pit
from .pusher import Pusher
from .spinner import Spinner
from .wind import Wind

__all__ = ["BrakeStrip", "Cannon", "Pendulum", "Pit", "Pusher", "Spinner", "Wind"]
