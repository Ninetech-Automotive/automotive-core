import pytest
from unittest.mock import Mock, patch
from Navigation.NavigationController import NavigationController
from Communication.Emitter import Emitter
from ObjectDetection.ObjectDetector import ObjectDetector
from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus
from Validation.Validator import Validator
from Navigation.Graph import Graph
from Configuration.Configurator import Configurator
import json


class TestNavigationController:

    @pytest.fixture
    def emitter(self):
        return Mock(spec=Emitter)

    @pytest.fixture
    def object_detector(self):
        return Mock(spec=ObjectDetector)

    @pytest.fixture(scope="class", autouse=True)
    def setup_configurator(self, tmp_path_factory):
        config_data = {
            "angles": {
                "X": {"S": 0.0},
                "S": {"G": 30.0, "F": 60.0, "X": 180.0, "H": 300.0},
                "H": {"A": 0.0, "I": 60.0, "G": 90.0, "S": 118.0},
                "G": {"C": 30.0, "F": 90.0, "S": 210.0, "H": 270.0, "I": 330.0},
                "F": {"C": 0.0, "S": 240.0, "G": 270.0},
                "I": {"B": 0.0, "C": 60.0, "G": 150.0, "H": 240.0, "A": 300.0},
                "A": {"B": 60.0, "I": 120.0, "H": 180.0},
                "C": {"F": 180.0, "G": 210.0, "I": 240.0, "B": 300.0},
                "B": {"C": 120.0, "I": 180.0, "A": 240.0},
            }
        }
        config_file = tmp_path_factory.mktemp("config") / "config.json"
        with config_file.open("w") as f:
            json.dump(config_data, f)
        Configurator.initialize(str(config_file))

    @pytest.fixture
    def controller(self, emitter, object_detector):
        return NavigationController(emitter, object_detector)

    def test_start(self, controller):
        controller.start("A")
        assert controller.graph.target_waypoint.id == "A"
        assert "S" in controller.outgoing_waypoint_ids
        controller.emitter.emit.assert_called_once_with("ping")

    def test_on_pong(self, controller):
        with patch.object(controller, "_NavigationController__next") as mock_next:
            controller.on_pong()
            mock_next.assert_called_once()

    def test_on_waypoint(self, controller):
        controller.graph.set_target_waypoint("A")
        controller.graph.current_waypoint = controller.graph.waypoints[1]
        controller.graph.previous_waypoint = controller.graph.waypoints[0]
        controller.on_waypoint()
        assert controller.graph.current_waypoint.status == WaypointStatus.FREE
        assert (
            controller.graph.previous_waypoint.angles[0].edge.status == EdgeStatus.FREE
        )
        controller.emitter.emit.assert_called_once_with("scan_point")

    def test_on_angle(self, controller):
        controller.graph.set_target_waypoint("A")
        controller.graph.go_to_next_best_waypoint()
        with patch.object(
            controller.object_detector,
            "detect",
            return_value=(
                WaypointStatus.POTENTIALLY_FREE,
                EdgeStatus.POTENTIALLY_OBSTRUCTED,
            ),
        ):
            controller.on_angle(30.0)
            assert (
                controller.graph.current_waypoint.angles[0].outgoing_waypoint.status
                == WaypointStatus.POTENTIALLY_FREE
            )
            assert (
                controller.graph.current_waypoint.angles[0].edge.status
                == EdgeStatus.POTENTIALLY_OBSTRUCTED
            )

    def test_on_turned_to_target_line(self, controller):
        controller.on_turned_to_target_line()
        controller.emitter.emit.assert_called_once_with("follow_line")