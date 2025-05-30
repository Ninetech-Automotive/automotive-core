import pytest
from unittest.mock import patch
from Navigation.Graph import Graph
from Navigation.Waypoint import Waypoint
from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus
from Configuration.Configurator import Configurator
from Exceptions.NoPathLeftError import NoPathLeftError
from pathlib import Path
import json


class TestGraph:

    @pytest.fixture(scope="class", autouse=True)
    def setup_configurator(self):
        mock_config_path = Path(__file__).resolve().parent / "mock_config.json"
        Configurator.initialize(str(mock_config_path))

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

    def test_update_missing_angles(self, graph):
        graph.current_waypoint.get_angles()[0].get_edge().set_status(EdgeStatus.UNKNOWN)
        graph.update_missing_angles()
        assert graph.current_waypoint.get_angles()[0].get_edge().get_status() == EdgeStatus.MISSING

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

    def test_reset_object_detection_data_waypoints(self, graph):
        graph.set_target_waypoint("A")
        graph._get_waypoint_by_id("H").set_status(WaypointStatus.POTENTIALLY_BLOCKED)
        graph._get_waypoint_by_id("G").set_status(WaypointStatus.POTENTIALLY_BLOCKED)
        graph._get_waypoint_by_id("F").set_status(WaypointStatus.POTENTIALLY_BLOCKED)

        graph.go_to_next_best_waypoint()
        assert graph._get_waypoint_by_id("H").get_status() == WaypointStatus.UNKNOWN
        assert graph._get_waypoint_by_id("G").get_status() == WaypointStatus.UNKNOWN
        assert graph._get_waypoint_by_id("F").get_status() == WaypointStatus.UNKNOWN

    def test_no_paths_left_waypoints(self, graph):
        graph.set_target_waypoint("A")
        graph.go_to_next_best_waypoint()
        graph._get_waypoint_by_id("H").set_status(WaypointStatus.BLOCKED)
        graph._get_waypoint_by_id("G").set_status(WaypointStatus.BLOCKED)
        graph._get_waypoint_by_id("F").set_status(WaypointStatus.BLOCKED)
        with pytest.raises(NoPathLeftError):
            graph.go_to_next_best_waypoint()

    def test_reset_object_detection_data_edges(self, graph):
        graph.set_target_waypoint("A")
        edge1 = graph._get_waypoint_by_id("S").get_edge_to_waypoint("H")
        edge2 = graph._get_waypoint_by_id("S").get_edge_to_waypoint("G")
        edge3 = graph._get_waypoint_by_id("S").get_edge_to_waypoint("F")
        edge1.set_status(EdgeStatus.POTENTIALLY_MISSING)
        edge2.set_status(EdgeStatus.POTENTIALLY_MISSING)
        edge3.set_status(EdgeStatus.POTENTIALLY_MISSING)
        graph.go_to_next_best_waypoint()
        assert edge1.get_status() == EdgeStatus.UNKNOWN
        assert edge2.get_status() == EdgeStatus.UNKNOWN
        assert edge3.get_status() == EdgeStatus.UNKNOWN

    def test_no_paths_left_edges(self, graph):
        graph.set_target_waypoint("A")
        edge1 = graph._get_waypoint_by_id("S").get_edge_to_waypoint("H")
        edge2 = graph._get_waypoint_by_id("S").get_edge_to_waypoint("G")
        edge3 = graph._get_waypoint_by_id("S").get_edge_to_waypoint("F")
        edge1.set_status(EdgeStatus.MISSING)
        edge2.set_status(EdgeStatus.MISSING)
        edge3.set_status(EdgeStatus.MISSING)
        with pytest.raises(NoPathLeftError):
            graph.go_to_next_best_waypoint()

    def test_reset_object_detection_data_mixed(self, graph):
        graph.set_target_waypoint("A")
        edge1 = graph._get_waypoint_by_id("S").get_edge_to_waypoint("H")
        edge2 = graph._get_waypoint_by_id("S").get_edge_to_waypoint("G")
        edge1.set_status(EdgeStatus.POTENTIALLY_MISSING)
        edge2.set_status(EdgeStatus.POTENTIALLY_MISSING)
        graph._get_waypoint_by_id("F").set_status(WaypointStatus.POTENTIALLY_BLOCKED)
        graph.go_to_next_best_waypoint()
        assert edge1.get_status() == EdgeStatus.UNKNOWN
        assert edge2.get_status() == EdgeStatus.UNKNOWN
        assert graph._get_waypoint_by_id("F").get_status() == WaypointStatus.UNKNOWN

    def test_no_paths_left_mixed(self, graph):
        graph.set_target_waypoint("A")
        edge1 = graph._get_waypoint_by_id("S").get_edge_to_waypoint("H")
        edge2 = graph._get_waypoint_by_id("S").get_edge_to_waypoint("G")
        edge1.set_status(EdgeStatus.MISSING)
        edge2.set_status(EdgeStatus.MISSING)
        graph._get_waypoint_by_id("F").set_status(WaypointStatus.BLOCKED)
        with pytest.raises(NoPathLeftError):
            graph.go_to_next_best_waypoint()

    