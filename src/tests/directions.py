from enum import Enum

HALF_PI = 1.5708

class Direction(Enum):
    STRONG_RIGHT = 2
    LIGHT_RIGHT = 1
    FORWARD = 0
    LIGHT_LEFT = -1
    STRONG_LEFT = -2
    
    @staticmethod
    def get_angle_from_index(ind, max_angle):
        # turn left
        if ind == 0:
            return -max_angle
        elif ind == 1:
            return -max_angle / 2
        elif ind == 2:
            return 0
        elif ind == 3:
            return max_angle / 2
        elif ind == 4:
            return max_angle
