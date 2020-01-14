from enum import Enum

class Direction(Enum):
    STRONG_RIGHT = 2
    LIGHT_RIGHT = 1
    FORWARD = 0
    LIGHT_LEFT = -1
    STRONG_LEFT = -2
