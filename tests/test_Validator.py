import pytest
from Validation.Validator import Validator
from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus

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
    with pytest.raises(ValueError, match="Angle value 400.0 is not in the range of \[0, 360\)"):
        Validator.validate_angle_value(400.0)

def test_validate_configuration_valid():
    config = {
        "angles": {
            "angle1": {
                "sub_angle1": 45.0
            }
        }
    }
    Validator.validate_configuration(config)

def test_validate_configuration_missing_angles():
    config = {}
    with pytest.raises(ValueError, match="The configuration file does not contain the 'angles' object"):
        Validator.validate_configuration(config)

def test_validate_configuration_invalid_value_type():
    config = {
        "angles": {
            "angle1": "not_a_dict"
        }
    }
    with pytest.raises(ValueError, match="The value of the 'angle1' key is not a dictionary"):
        Validator.validate_configuration(config)

def test_validate_configuration_invalid_sub_value_type():
    config = {
        "angles": {
            "angle1": {
                "sub_angle1": "not_a_float"
            }
        }
    }
    with pytest.raises(ValueError, match="The value of the 'sub_angle1' key is not a float"):
        Validator.validate_configuration(config)

def test_validate_waypoint_id_format_valid():
    Validator.validate_waypoint_id_format("A")

def test_validate_waypoint_id_format_invalid_type():
    with pytest.raises(ValueError, match="Waypoint ID 1 is not a string"):
        Validator.validate_waypoint_id_format(1)

def test_validate_waypoint_id_format_invalid_length():
    with pytest.raises(ValueError, match="Waypoint ID AB is not of length 1"):
        Validator.validate_waypoint_id_format("AB")

def test_validate_waypoint_id_format_invalid_character():
    with pytest.raises(ValueError, match="Waypoint ID 1 is not a letter"):
        Validator.validate_waypoint_id_format("1")

def test_validate_waypoint_id_format_not_uppercase():
    with pytest.raises(ValueError, match="Waypoint ID a is not uppercase"):
        Validator.validate_waypoint_id_format("a")