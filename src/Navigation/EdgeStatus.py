from enum import Enum

class EdgeStatus(int, Enum):
    UNKNOWN = 140
    OBSTRUCTED = 130
    FREE = 100
    POTENTIALLY_OBSTRUCTED = 120
    POTENTIALLY_FREE = 110