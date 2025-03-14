import pytest
from pathlib import Path
from ObjectDetection.YOLODetector import YOLODetector
from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus
from CameraStub import CameraStub

IMAGES_PATH = Path(__file__).resolve().parent / "images"

def test_detect_white_obstacle_with_cone():
    camera = CameraStub(IMAGES_PATH / "1.JPG")
    yolo_detector = YOLODetector(camera)
    waypoint_status, edge_status = yolo_detector.detect()
    assert waypoint_status == WaypointStatus.POTENTIALLY_BLOCKED
    assert edge_status == EdgeStatus.POTENTIALLY_OBSTRUCTED

def test_detect_red_obstacle_no_cone():
    camera = CameraStub(IMAGES_PATH / "2.JPG")
    yolo_detector = YOLODetector(camera)
    waypoint_status, edge_status = yolo_detector.detect()
    assert waypoint_status == WaypointStatus.POTENTIALLY_FREE
    assert edge_status == EdgeStatus.POTENTIALLY_OBSTRUCTED

def test_detect_red_obstacle_with_cone():
    camera = CameraStub(IMAGES_PATH / "3.JPG")
    yolo_detector = YOLODetector(camera)
    waypoint_status, edge_status = yolo_detector.detect()
    assert waypoint_status == WaypointStatus.POTENTIALLY_BLOCKED
    assert edge_status == EdgeStatus.POTENTIALLY_OBSTRUCTED

def test_detect_white_obstacle_no_cone():
    camera = CameraStub(IMAGES_PATH / "4.JPG")
    yolo_detector = YOLODetector(camera)
    waypoint_status, edge_status = yolo_detector.detect()
    assert waypoint_status == WaypointStatus.POTENTIALLY_FREE
    assert edge_status == EdgeStatus.POTENTIALLY_OBSTRUCTED

def test_detect_no_obstacle_with_cone():
    camera = CameraStub(IMAGES_PATH / "5.JPG")
    yolo_detector = YOLODetector(camera)
    waypoint_status, edge_status = yolo_detector.detect()
    assert waypoint_status == WaypointStatus.POTENTIALLY_BLOCKED
    assert edge_status == EdgeStatus.POTENTIALLY_FREE

def test_detect_no_obstacle_no_cone():
    camera = CameraStub(IMAGES_PATH / "6.JPG")
    yolo_detector = YOLODetector(camera)
    waypoint_status, edge_status = yolo_detector.detect()
    assert waypoint_status == WaypointStatus.POTENTIALLY_FREE
    assert edge_status == EdgeStatus.POTENTIALLY_FREE

def test_detect_red_obstacle_with_cone2():
    camera = CameraStub(IMAGES_PATH / "7.JPG")
    yolo_detector = YOLODetector(camera)
    waypoint_status, edge_status = yolo_detector.detect()
    assert waypoint_status == WaypointStatus.POTENTIALLY_BLOCKED
    assert edge_status == EdgeStatus.POTENTIALLY_OBSTRUCTED

def test_detect_white_obstacle_with_cone2():
    camera = CameraStub(IMAGES_PATH / "8.JPG")
    yolo_detector = YOLODetector(camera)
    waypoint_status, edge_status = yolo_detector.detect()
    assert waypoint_status == WaypointStatus.POTENTIALLY_FREE
    assert edge_status == EdgeStatus.POTENTIALLY_FREE