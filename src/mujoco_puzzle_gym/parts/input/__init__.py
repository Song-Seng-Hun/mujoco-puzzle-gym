"""Input parts (R4-R6): floor buttons, sensors, receptacles, checkpoint flags,
targets."""

from .beam import Beam
from .button import Button
from .checkpoint import Checkpoint
from .laser import Laser
from .light import Emitter, Photocell
from .pressure import PressurePlate
from .receptacle import Receptacle
from .stack import Stack
from .target import Target
from .zone import Zone

__all__ = ["Beam", "Button", "Checkpoint", "Emitter", "Laser", "Photocell", "PressurePlate", "Receptacle", "Stack", "Target", "Zone"]
