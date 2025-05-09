import pytest
from Validation.Validator import Validator
from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus
from pathlib import Path
import json


def test_validate_waypoint_status_valid():
    Validator.validate_waypoint_status(WaypointStatus.FREE)


def test_validate_waypoint_status_invalid():
    with pytest.raises(ValueError, match="Waypoint status INVALID is not valid"):
        Validator.validate_waypoint_status("INVALID")


def test_validate_edge_status_valid():
    Validator.validate_edge_status(EdgeStatus.FREE)


def test_validate_edge_status_invalid():
    with pytest.raises(ValueError, match="Edge status INVALID is not valid"):
        Validator.validate_edge_status("INVALID")


def test_validate_angle_value_valid():
    Validator.validate_angle_value(45.0)


def test_validate_angle_value_invalid_type():
    with pytest.raises(ValueError, match="Angle value 45 is not a float"):
        Validator.validate_angle_value(45)


def test_validate_angle_value_invalid_range():
    with pytest.raises(
        ValueError, match="Angle value 400.0 is not in the range of \[0, 360\)"
    ):
        Validator.validate_angle_value(400.0)


def test_validate_configuration_valid():
    mock_config_path = Path(__file__).resolve().parent / "mock_config.json"
    with open(mock_config_path, 'r') as file:
        config = json.load(file)
    Validator.validate_configuration(config)


def test_validate_configuration_missing_communication():
    config = {}
    with pytest.raises(ValueError, match="The configuration file does not contain the 'communication' object"):
        Validator.validate_configuration(config)

def test_validate_configuration_missing_device():
    config = {"communication": {}}
    with pytest.raises(ValueError, match="The configuration file does not contain the 'device' key in the 'communication' object"):
        Validator.validate_configuration(config)

def test_validate_configuration_missing_baud():
    config = {"communication": {"device": "device_name"}}
    with pytest.raises(ValueError, match="The configuration file does not contain the 'baud' key in the 'communication' object"):
        Validator.validate_configuration(config)

def test_validate_configuration_missing_tolerances():
    config = {"communication": {"device": "device_name", "baud": 9600}}
    with pytest.raises(ValueError, match="The configuration file does not contain the 'tolerances' object"):
        Validator.validate_configuration(config)

def test_validate_configuration_invalid_tolerance_type():
    config = {
        "communication": {"device": "device_name", "baud": 9600},
        "tolerances": {"waypoint": "invalid", "obstacle": 10, "edge_x": 5, "edge_y": 5}
    }
    with pytest.raises(ValueError, match="The value of 'waypoint' in 'tolerances' must be an integer"):
        Validator.validate_configuration(config)

def test_validate_configuration_missing_waypoints():
    config = {
        "communication": {"device": "device_name", "baud": 9600},
        "tolerances": {"waypoint": 10, "obstacle": 10, "edge_x": 5, "edge_y": 5}
    }
    with pytest.raises(ValueError, match="The configuration file does not contain the 'waypoints' object"):
        Validator.validate_configuration(config)

def test_validate_configuration_invalid_waypoint_data():
    config = {
        "communication": {"device": "device_name", "baud": 9600},
        "tolerances": {"waypoint": 10, "obstacle": 10, "edge_x": 5, "edge_y": 5},
        "waypoints": {"wp1": "invalid"}
    }
    with pytest.raises(ValueError, match="The data for waypoint 'wp1' must be a dictionary"):
        Validator.validate_configuration(config)

def test_validate_configuration_missing_waypoint_coordinates():
    config = {
        "communication": {"device": "device_name", "baud": 9600},
        "tolerances": {"waypoint": 10, "obstacle": 10, "edge_x": 5, "edge_y": 5},
        "waypoints": {"wp1": {"edges": {}}}
    }
    with pytest.raises(ValueError, match="Waypoint 'wp1' must contain 'x' and 'y' coordinates"):
        Validator.validate_configuration(config)

def test_validate_configuration_invalid_edge_data():
    config = {
        "communication": {"device": "device_name", "baud": 9600},
        "tolerances": {"waypoint": 10, "obstacle": 10, "edge_x": 5, "edge_y": 5},
        "waypoints": {
            "wp1": {
                "x": 0, "y": 0,
                "edges": {"edge1": "invalid"}
            }
        }
    }
    with pytest.raises(ValueError, match="The data for edge 'edge1' in waypoint 'wp1' must be a dictionary"):
        Validator.validate_configuration(config)

def test_validate_configuration_valid():
    config = {
        "communication": {"device": "device_name", "baud": 9600},
        "tolerances": {"waypoint": 10, "obstacle": 10, "edge_x": 5, "edge_y": 5},
        "waypoints": {
            "wp1": {
                "x": 0, "y": 0,
                "edges": {
                    "edge1": {
                        "angle": 45.0,
                        "obstacle_coords": {"x": 1, "y": 2},
                        "bounding_box_corners": {"from": "A", "to": "B"}
                    }
                }
            }
        }
    }
    Validator.validate_configuration(config)

