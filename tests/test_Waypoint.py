import pytest
from Navigation.Waypoint import Waypoint
from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus
from Navigation.Angle import Angle
from Navigation.Edge import Edge

class TestWaypoint:

    @pytest.fixture
    def waypoint(self):
        b = Waypoint("B")
        edge = Edge()
        angle = Angle(b, 60.0, edge)
        a = Waypoint("A")
        a.set_angles([angle])
        return a

    def test_set_incoming_angle_by_id(self, waypoint):
        waypoint.set_incoming_angle_by_id("B")
        assert waypoint.incoming_angle == 60.0

    def test_remove_angle_to_waypoint(self, waypoint):
        waypoint.remove_angle_to_waypoint("B")
        assert len(waypoint.angles) == 0

    def test_get_edge_to_waypoint(self, waypoint):
        edge = waypoint.get_edge_to_waypoint("B")
        assert edge is not None

    def test_update_angle_status_free(self, waypoint):
        angle = waypoint.angles[0]
        angle.get_waypoint().set_status(WaypointStatus.UNKNOWN)
        angle.get_edge().set_status(EdgeStatus.UNKNOWN)

        updated_angle = waypoint.update_angle(60.0, WaypointStatus.FREE, EdgeStatus.FREE)

        assert updated_angle.get_waypoint().get_status() == WaypointStatus.FREE
        assert updated_angle.get_edge().get_status() == EdgeStatus.FREE

    def test_update_angle_status_no_change(self, waypoint):
        angle = waypoint.angles[0]
        angle.get_waypoint().set_status(WaypointStatus.BLOCKED)
        angle.get_edge().set_status(EdgeStatus.OBSTRUCTED)

        updated_angle = waypoint.update_angle(60.0, WaypointStatus.POTENTIALLY_FREE, EdgeStatus.POTENTIALLY_OBSTRUCTED)

        assert updated_angle.get_waypoint().get_status() == WaypointStatus.BLOCKED
        assert updated_angle.get_edge().get_status() == EdgeStatus.OBSTRUCTED