import pytest
from unittest.mock import Mock
from ObjectDetection.ColorDetector import ColorDetector
from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus

@pytest.fixture
def mock_camera():
    camera = Mock()
    camera.get_width.return_value = 640
    camera.get_height.return_value = 480
    camera.get_image_array.return_value = [
        [[228, 162, 55], [228, 162, 55], [255, 0, 0], [255, 0, 0]],
        [[228, 162, 55], [228, 162, 55], [255, 0, 0], [255, 0, 0]],
        [[228, 162, 55], [228, 162, 55], [255, 0, 0], [255, 0, 0]],
        [[228, 162, 55], [228, 162, 55], [255, 0, 0], [255, 0, 0]],
    ]
    return camera

@pytest.fixture
def color_detector(mock_camera):
    return ColorDetector(mock_camera)

def test_detect_potentially_free(color_detector, mock_camera):
    mock_camera.get_image_array.return_value = [
        [[0, 0, 0] for _ in range(640)] for _ in range(480)
    ]
    waypoint_status, edge_status = color_detector.detect()
    assert waypoint_status == WaypointStatus.POTENTIALLY_FREE
    assert edge_status == EdgeStatus.POTENTIALLY_FREE

def test_detect_potentially_blocked(color_detector, mock_camera):
    mock_camera.get_image_array.return_value = [
        [[228, 162, 55] for _ in range(640)] for _ in range(480)
    ]
    waypoint_status, edge_status = color_detector.detect()
    assert waypoint_status == WaypointStatus.POTENTIALLY_BLOCKED
    assert edge_status == EdgeStatus.POTENTIALLY_FREE

def test_detect_potentially_obstructed(color_detector, mock_camera):
    mock_camera.get_image_array.return_value = [
        [[255, 0, 0] for _ in range(640)] for _ in range(480)
    ]
    waypoint_status, edge_status = color_detector.detect()
    assert waypoint_status == WaypointStatus.POTENTIALLY_FREE
    assert edge_status == EdgeStatus.POTENTIALLY_OBSTRUCTED

def test_detect_blocked_and_obstructed(color_detector, mock_camera):
    mock_camera.get_image_array.return_value = [
        [[228, 162, 55] if x % 2 == 0 else [255, 0, 0] for x in range(640)] for _ in range(480)
    ]
    waypoint_status, edge_status = color_detector.detect()
    assert waypoint_status == WaypointStatus.POTENTIALLY_BLOCKED
    assert edge_status == EdgeStatus.POTENTIALLY_OBSTRUCTED