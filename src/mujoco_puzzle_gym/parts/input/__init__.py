"""Input parts (R4-R6): floor buttons, sensors, receptacles, checkpoint flags (checkpoint.py)."""

from .button import Button
from .laser import Laser
from .pressure import PressurePlate
from .receptacle import Receptacle
from .zone import Zone

__all__ = ["Button", "Laser", "PressurePlate", "Receptacle", "Zone"]
