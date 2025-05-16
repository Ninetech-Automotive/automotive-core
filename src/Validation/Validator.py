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
        # Validate communication section
        if "communication" not in configuration:
            raise ValueError("The configuration file does not contain the 'communication' object")
        if "device" not in configuration["communication"]:
            raise ValueError("The configuration file does not contain the 'device' key in the 'communication' object")
        if "baud" not in configuration["communication"]:
            raise ValueError("The configuration file does not contain the 'baud' key in the 'communication' object")

        # Validate tolerances section
        if "tolerances" not in configuration:
            raise ValueError("The configuration file does not contain the 'tolerances' object")
        for key in ["waypoint", "obstacle", "edge_x", "edge_y"]:
            if key not in configuration["tolerances"]:
                raise ValueError(f"The configuration file does not contain the '{key}' key in the 'tolerances' object")
            if not isinstance(configuration["tolerances"][key], int):
                raise ValueError(f"The value of '{key}' in 'tolerances' must be an integer")

        # Validate waypoints section
        if "waypoints" not in configuration:
            raise ValueError("The configuration file does not contain the 'waypoints' object")
        for waypoint_id, waypoint_data in configuration["waypoints"].items():
            if not isinstance(waypoint_data, dict):
                raise ValueError(f"The data for waypoint '{waypoint_id}' must be a dictionary")
            if "x" not in waypoint_data or "y" not in waypoint_data:
                raise ValueError(f"Waypoint '{waypoint_id}' must contain 'x' and 'y' coordinates")
            if not isinstance(waypoint_data["x"], int) or not isinstance(waypoint_data["y"], int):
                raise ValueError(f"The 'x' and 'y' coordinates of waypoint '{waypoint_id}' must be integers")
            if "edges" not in waypoint_data:
                raise ValueError(f"Waypoint '{waypoint_id}' must contain an 'edges' object")
            for edge_id, edge_data in waypoint_data["edges"].items():
                if not isinstance(edge_data, dict):
                    raise ValueError(f"The data for edge '{edge_id}' in waypoint '{waypoint_id}' must be a dictionary")
                if "angle" not in edge_data or not isinstance(edge_data["angle"], float):
                    raise ValueError(f"Edge '{edge_id}' in waypoint '{waypoint_id}' must contain a float 'angle'")
                if "obstacle_coords" not in edge_data:
                    raise ValueError(f"Edge '{edge_id}' in waypoint '{waypoint_id}' must contain 'obstacle_coords'")
                if not isinstance(edge_data["obstacle_coords"], dict):
                    raise ValueError(f"'obstacle_coords' in edge '{edge_id}' of waypoint '{waypoint_id}' must be a dictionary")
                for coord in ["x", "y"]:
                    if coord not in edge_data["obstacle_coords"] or not isinstance(edge_data["obstacle_coords"][coord], int):
                        raise ValueError(f"'obstacle_coords.{coord}' in edge '{edge_id}' of waypoint '{waypoint_id}' must be an integer")
                if "bounding_box_corners" not in edge_data:
                    raise ValueError(f"Edge '{edge_id}' in waypoint '{waypoint_id}' must contain 'bounding_box_corners'")
                if not isinstance(edge_data["bounding_box_corners"], dict):
                    raise ValueError(f"'bounding_box_corners' in edge '{edge_id}' of waypoint '{waypoint_id}' must be a dictionary")
                for corner in ["from", "to"]:
                    if corner not in edge_data["bounding_box_corners"] or not isinstance(edge_data["bounding_box_corners"][corner], str):
                        raise ValueError(f"'bounding_box_corners.{corner}' in edge '{edge_id}' of waypoint '{waypoint_id}' must be a string")
                
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