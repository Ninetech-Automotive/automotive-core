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
from pathlib import Path
import json


class TestNavigationController:

    @pytest.fixture
    def emitter(self):
        return Mock(spec=Emitter)

    @pytest.fixture
    def object_detector(self):
        return Mock(spec=ObjectDetector)

    @pytest.fixture(scope="class", autouse=True)
    def setup_configurator(self):
        mock_config_path = Path(__file__).resolve().parent / "mock_config.json"
        Configurator.initialize(str(mock_config_path))

    @pytest.fixture
    def controller(self, emitter, object_detector):
        controller = NavigationController(emitter, object_detector)
        controller.graph = Graph()
        return controller

    def test_start(self, controller):
        controller._start("A")
        assert controller.graph.target_waypoint.id == "A"
        controller.emitter.emit.assert_called_once_with("ping")

    def test_on_pong(self, controller):
        with patch.object(controller, "_NavigationController__go_to_next_waypoint_by_ideal_path") as mock_next:
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
