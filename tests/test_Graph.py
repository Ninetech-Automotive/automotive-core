import pytest
from unittest.mock import patch
from Navigation.Graph import Graph
from Navigation.Waypoint import Waypoint
from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus
from Configuration.Configurator import Configurator
import json


class TestGraph:

    @pytest.fixture(scope="class", autouse=True)
    def setup_configurator(self, tmp_path_factory):
        config_data = {
            "communication": {"device": "/dev/ttyAMA1", "baud": 9600},
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
            },
        }
        config_file = tmp_path_factory.mktemp("config") / "config.json"
        with config_file.open("w") as f:
            json.dump(config_data, f)
        Configurator.initialize(str(config_file))

    @pytest.fixture
    def graph(self):
        return Graph()

    def test_initialize_waypoints(self, graph):
        assert len(graph.waypoints) == 9
        assert graph.current_waypoint.get_id() == "X"
        assert graph.current_waypoint.get_dijkstra_visited() is True
        assert graph.current_waypoint.get_status() == WaypointStatus.FREE

    def test_set_target_waypoint(self, graph):
        graph.set_target_waypoint("A")
        assert graph.target_waypoint.get_id() == "A"

    def test_set_target_waypoint_invalid(self, graph):
        with pytest.raises(ValueError):
            graph.set_target_waypoint("Z")

    def test_go_to_next_best_waypoint(self, graph):
        graph.set_target_waypoint("A")
        next_best_waypoint_id = graph.go_to_next_best_waypoint()
        assert next_best_waypoint_id == "S"

    def test_go_back_to_previous_waypoint(self, graph):
        graph.set_target_waypoint("A")
        graph.go_to_next_best_waypoint()
        graph.go_back_to_previous_waypoint()
        assert graph.current_waypoint.get_id() == "X"

    def test_has_reached_target_waypoint(self, graph):
        graph.set_target_waypoint("X")
        assert graph.has_reached_target_waypoint() is True

    def test_update_waypoint_status(self, graph):
        graph.update_waypoint_status(WaypointStatus.BLOCKED)
        assert graph.current_waypoint.get_status() == WaypointStatus.BLOCKED

    def test_update_previous_edge_status(self, graph):
        graph.set_target_waypoint("A")
        graph.go_to_next_best_waypoint()
        graph.update_previous_edge_status()
        assert (
            graph.previous_waypoint.get_edge_to_waypoint(
                graph.current_waypoint.get_id()
            ).get_status()
            == EdgeStatus.FREE
        )
        assert (
            graph.current_waypoint.get_edge_to_waypoint(
                graph.previous_waypoint.get_id()
            ).get_status()
            == EdgeStatus.FREE
        )

    def test_update_previous_edge_status_obstructed(self, graph):
        graph.set_target_waypoint("A")
        graph.go_to_next_best_waypoint()
        graph.obstacle_detected()
        graph.update_previous_edge_status()
        assert (
            graph.previous_waypoint.get_edge_to_waypoint(
                graph.current_waypoint.get_id()
            ).get_status()
            == EdgeStatus.OBSTRUCTED
        )
        assert (
            graph.current_waypoint.get_edge_to_waypoint(
                graph.previous_waypoint.get_id()
            ).get_status()
            == EdgeStatus.OBSTRUCTED
        )

    def test_update_waypoint_from_angle(self, graph):
        graph.set_target_waypoint("A")
        graph.go_to_next_best_waypoint()
        # coming from X to S, waypoint G is at 30 degrees
        angle = graph.update_waypoint_from_angle(
            30.0, WaypointStatus.FREE, EdgeStatus.FREE
        )
        assert angle.get_waypoint().get_id() == "G"
        assert angle.get_edge().get_status() == EdgeStatus.FREE
        assert angle.get_waypoint().get_status() == WaypointStatus.FREE

    def test_remove_missing_angles(self, graph):
        graph.current_waypoint.get_angles()[0].get_edge().set_status(EdgeStatus.UNKNOWN)
        graph.remove_missing_angles()
        assert len(graph.current_waypoint.get_angles()) == 0

    def test_cone_detected(self, graph):
        graph.cone_detected()
        assert graph.current_waypoint.get_status() == WaypointStatus.BLOCKED

    def test_obstacle_detected(self, graph):
        graph.set_target_waypoint("A")
        graph.go_to_next_best_waypoint()
        graph.obstacle_detected()
        assert (
            graph.previous_waypoint.get_edge_to_waypoint(
                graph.current_waypoint.get_id()
            ).get_status()
            == EdgeStatus.OBSTRUCTED
        )
        assert (
            graph.current_waypoint.get_edge_to_waypoint(
                graph.previous_waypoint.get_id()
            ).get_status()
            == EdgeStatus.OBSTRUCTED
        )
