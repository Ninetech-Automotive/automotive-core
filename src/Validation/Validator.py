from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus

class Validator:
    """
    Contains static methods for recurring validation tasks.
    """

    @staticmethod
    def validate_waypoint_status(waypoint_status):
        if waypoint_status not in WaypointStatus:
            raise ValueError(f"Waypoint status {waypoint_status} is not valid")

    @staticmethod
    def validate_edge_status(edge_status):
        if edge_status not in EdgeStatus:
            raise ValueError(f"Edge status {edge_status} is not valid")

    @staticmethod
    def validate_angle_value(angle_value):
        if not isinstance(angle_value, float):
            raise ValueError(f"Angle value {angle_value} is not a float")
        if angle_value < 0 or angle_value >= 360:
            raise ValueError(f"Angle value {angle_value} is not in the range of [0, 360)")
        
    @staticmethod
    def validate_configuration(configuration):
        if "angles" not in configuration:
            raise ValueError("The configuration file does not contain the 'angles' object")
        for key, value in configuration["angles"].items():
            if not isinstance(value, dict):
                raise ValueError(f"The value of the '{key}' key is not a dictionary")
            for sub_key, sub_value in value.items():
                if not isinstance(sub_value, float):
                    raise ValueError(f"The value of the '{sub_key}' key is not a float")
        if "communication" not in configuration:
            raise ValueError("The configuration file does not contain the 'communication' object")
        if "device" not in configuration["communication"]:
            raise ValueError("The configuration file does not contain the 'device' key in the 'communication' object")
        if "baud" not in configuration["communication"]:
            raise ValueError("The configuration file does not contain the 'baud' key in the 'communication' object")
                
    @staticmethod
    def validate_waypoint_id_format(waypoint_id):
        if not isinstance(waypoint_id, str):
            raise ValueError(f"Waypoint ID {waypoint_id} is not a string")
        if len(waypoint_id) != 1:
            raise ValueError(f"Waypoint ID {waypoint_id} is not of length 1")
        if not waypoint_id.isalpha():
            raise ValueError(f"Waypoint ID {waypoint_id} is not a letter")
        if not waypoint_id.isupper():
            raise ValueError(f"Waypoint ID {waypoint_id} is not uppercase")