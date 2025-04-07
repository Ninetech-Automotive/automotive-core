import pytest
from pathlib import Path
from ObjectDetection.YOLODetector import YOLODetector
from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus
from Navigation.Graph import Graph
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

def test_start_up_process_detect():
    camera = CameraStub(IMAGES_PATH / "11.JPG")
    graph = Graph()
    yolo_detector = YOLODetector(camera, graph)
    map_status = yolo_detector.start_up_process_detect()
    waypoints_with_cone = ["G", "B"]
    edges_with_obstacle = ["S_to_G", "G_to_S", "S_to_H", "H_to_S", "H_to_G", "G_to_H"]
    for waypoint in map_status["Waypoint_Statuses"]:
        if waypoint in waypoints_with_cone:
            assert map_status["Waypoint_Statuses"][waypoint] == WaypointStatus.POTENTIALLY_BLOCKED
        else:
            assert map_status["Waypoint_Statuses"][waypoint] == WaypointStatus.POTENTIALLY_FREE
    for edge in map_status["Edge_Statuses"]:
        if edge in edges_with_obstacle:
            assert map_status["Edge_Statuses"][edge] == EdgeStatus.POTENTIALLY_OBSTRUCTED
        else:
            assert map_status["Edge_Statuses"][edge] == EdgeStatus.UNKNOWN
    