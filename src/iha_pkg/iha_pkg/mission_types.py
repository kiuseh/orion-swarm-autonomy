"""Mission state and role enums shared by UAV control modules."""

from enum import Enum


class DroneState(Enum):
    IDLE = 0
    TAKE_OFF = 1
    MOVE = 2
    ROTATION = 3
    INITIAL_POSITIONING = 4
    GOTO_AREA = 5
    LAND_TO_AREA = 6
    RETURN_TO_HOME = 7


class DroneRole(Enum):
    LEADER = 0
    SOL = 1
    SAG = 2
