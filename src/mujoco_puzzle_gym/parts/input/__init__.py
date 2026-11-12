"""Input parts (R4-R6): floor buttons, sensors, receptacles, checkpoint flags,
targets."""

from .button import Button
from .checkpoint import Checkpoint
from .laser import Laser
from .pressure import PressurePlate
from .receptacle import Receptacle
from .target import Target
from .zone import Zone

__all__ = ["Button", "Checkpoint", "Laser", "PressurePlate", "Receptacle", "Target", "Zone"]
