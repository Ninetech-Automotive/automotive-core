from enum import Enum

class EdgeStatus(int, Enum):
    UNKNOWN = 140
    OBSTRUCTED = 130
    FREE = 100
    MISSING = 9999999
    # POTENTIALLY means that the status was set based on information from the object detection (camera) which is not 100% reliable
    POTENTIALLY_OBSTRUCTED = 120
    POTENTIALLY_FREE = 110
    POTENTIALLY_MISSING = 9999998