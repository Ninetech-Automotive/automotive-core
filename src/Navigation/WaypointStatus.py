from enum import Enum

class WaypointStatus(str, Enum):
    UNKNOWN = "UNKNOWN"
    BLOCKED = "BLOCKED"
    FREE = "FREE"
    # POTENTIALLY means that the status was set based on information from the object detection (camera) which is not 100% reliable
    POTENTIALLY_BLOCKED = "POTENTIALLY_BLOCKED"
    POTENTIALLY_FREE = "POTENTIALLY_FREE"