"""Input parts (R4-R6): floor buttons, sensors, receptacles, checkpoint flags."""

from .button import Button
from .checkpoint import Checkpoint
from .laser import Laser
from .pressure import PressurePlate
from .receptacle import Receptacle
from .zone import Zone

__all__ = ["Button", "Checkpoint", "Laser", "PressurePlate", "Receptacle", "Zone"]
