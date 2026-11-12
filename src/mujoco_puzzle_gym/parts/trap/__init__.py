"""Traps (R16): wind, pusher, spinning bar, pendulum, pit, cannon. Slippery and
sticky floors are `floor_tile` presets (`trap/slippery_floor`, `trap/sticky_floor`)."""

from .cannon import Cannon
from .pendulum import Pendulum
from .pit import Pit
from .pusher import Pusher
from .spinner import Spinner
from .wind import Wind

__all__ = ["Cannon", "Pendulum", "Pit", "Pusher", "Spinner", "Wind"]
