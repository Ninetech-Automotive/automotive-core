import sys
from Navigation.Waypoint import Waypoint
from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus
from Navigation.Angle import Angle
from Navigation.Edge import Edge
from Configuration.Configurator import Configurator
from Validation.Validator import Validator
from Exceptions.NoPathLeftError import NoPathLeftError


class Graph:
    """
    Logical representation of the waypoint network. This graph is updated regularly based on the information provided by the sensors.
    On each waypoint, the fastet path to the target waypoint is calculated using Dijkstra's algorithm.
    """

    def __init__(self):
        self.current_waypoint: Waypoint = None
        self.target_waypoint: Waypoint = None
        self.previous_waypoint: Waypoint = None
        self.waypoints = []
        self.__initialize_waypoints()
        self.shortest_path_to_target = []
        # this is used to prevent the object detection data from being reset multiple times
        self.is_object_detection_data_reset = False

    def __initialize_waypoints(self):
        # waypoint X is the starting position which is not a physical waypoint
        x = Waypoint("X")
        s = Waypoint("S")
        h = Waypoint("H")
        g = Waypoint("G")
        f = Waypoint("F")
        i = Waypoint("I")
        a = Waypoint("A")
        c = Waypoint("C")
        b = Waypoint("B")
        self.waypoints = [x, s, h, g, f, i, a, c, b]
        self.current_waypoint = x
        self.current_waypoint.set_dijkstra_visited(True)
        self.current_waypoint.set_status(WaypointStatus.FREE)
        self.__load_configuration_angles()

    def __load_configuration_angles(self):
        for waypoint_id, angle_values in Configurator().get_angles().items():
            waypoint = self._get_waypoint_by_id(waypoint_id)
            angles = []
            for outgoing_waypoint_id, angle in angle_values.items():
                edge = Edge()
                outgoing_waypoint = self._get_waypoint_by_id(outgoing_waypoint_id)
                angle = Angle(outgoing_waypoint, angle, edge)
                angles.append(angle)
            waypoint.set_angles(angles)

    def _get_waypoint_by_id(self, id):
        return [w for w in self.waypoints if w.get_id() == id][0]

    def __validate_if_waypoint_exists(self, waypoint_id):
        if not any(w.get_id() == waypoint_id for w in self.waypoints):
            raise ValueError(f"Waypoint with id {waypoint_id} does not exist")

    def set_target_waypoint(self, target_waypoint_id: str):
        self.__validate_if_waypoint_exists(target_waypoint_id)
        self.target_waypoint = self._get_waypoint_by_id(target_waypoint_id)

    def go_to_next_best_waypoint(self):
        next_best_waypoint = self.__get_next_best_waypoint()
        next_best_waypoint.set_incoming_angle_by_id(self.current_waypoint.get_id())
        self.previous_waypoint = self.current_waypoint
        self.current_waypoint = next_best_waypoint
        return next_best_waypoint.get_id()

    def __get_next_best_waypoint(self):
        self.__calculate_shortest_path()
        self.__store_shortest_path()
        return self.shortest_path_to_target[0]

    def __store_shortest_path(self):
        node = self.target_waypoint
        try:
            while node.get_id() != self.current_waypoint.get_id():
                self.shortest_path_to_target.insert(0, node)
                node = node.get_previous_node_to_this_waypoint()
            print(
                "[pi    ] shortest path: ",
                list(map(lambda n: n.get_id(), self.shortest_path_to_target)),
            )
            self.is_object_detection_data_reset = False
        except AttributeError:
            # if the target waypoint is not reachable, the previous node is None and leads to an AttributeError
            self.__handle_no_path_left()

    def __handle_no_path_left(self):
        # prevents infinite loops
        if self.is_object_detection_data_reset:
            raise NoPathLeftError()
        print(
            "[pi    ] no path to target waypoint left, resetting objecet detection data"
        )
        self.__reset_object_detection_data()
        self.is_object_detection_data_reset = True
        self.__get_next_best_waypoint()

    def __reset_object_detection_data(self):
        for waypoint in self.waypoints:
            if waypoint.get_status() in [
                WaypointStatus.POTENTIALLY_BLOCKED,
                WaypointStatus.POTENTIALLY_FREE,
            ]:
                waypoint.set_status(WaypointStatus.UNKNOWN)
            for angle in waypoint.get_angles():
                if angle.get_edge().get_status() in [
                    EdgeStatus.POTENTIALLY_OBSTRUCTED,
                    EdgeStatus.POTENTIALLY_FREE,
                    EdgeStatus.POTENTIALLY_MISSING,
                ]:
                    angle.get_edge().set_status(EdgeStatus.UNKNOWN)

    def go_back_to_previous_waypoint(self):
        temp_current_waypoint = self.current_waypoint
        self.current_waypoint = self.previous_waypoint
        self.current_waypoint.set_incoming_angle_by_id(temp_current_waypoint.get_id())
        self.previous_waypoint = temp_current_waypoint

    def has_reached_target_waypoint(self):
        return self.current_waypoint == self.target_waypoint

    def update_waypoint_status(self, waypoint_status: WaypointStatus):
        Validator.validate_waypoint_status(waypoint_status)
        self.current_waypoint.set_status(waypoint_status)

    def update_previous_edge_status(self):
        self.current_waypoint.update_edge_to_waypoint(self.previous_waypoint.get_id())
        self.previous_waypoint.update_edge_to_waypoint(self.current_waypoint.get_id())

    def update_waypoint_from_angle(
        self,
        angle_value: float,
        waypoint_status: WaypointStatus,
        edge_status: EdgeStatus,
    ):
        Validator.validate_angle_value(angle_value)
        Validator.validate_waypoint_status(waypoint_status)
        Validator.validate_edge_status(edge_status)
        return self.current_waypoint.update_angle(
            angle_value, waypoint_status, edge_status
        )

    def update_missing_angles(self):
        """
        After point scanning, when an edge still has the status UNKNOWN, then this edge does not exist and the status is set to MISSING.
        """
        for angle in self.current_waypoint.get_angles():
            if angle.get_edge().get_status() == EdgeStatus.UNKNOWN:
                angle.get_waypoint().set_angle_to_waypoint_as_missing(
                    self.current_waypoint.get_id()
                )
                self.current_waypoint.set_angle_to_waypoint_as_missing(
                    angle.get_waypoint().get_id()
                )

    def __calculate_shortest_path(self):
        # dijkstra
        self.__reset_dijkstra()
        self.current_waypoint.set_weight_to_target(0)
        while self.__has_next_unvisited_node():
            current_node = self.__get_next_unvisited_node()
            for angle in current_node.get_possible_angles():
                outgoing_node = angle.get_waypoint()
                outgoing_edge = angle.get_edge()
                calculated_weight_to_target_on_outgoing_node = (
                    current_node.get_weight_to_target() + outgoing_edge.get_weight()
                )
                current_weight_to_target_on_outgoing_node = (
                    outgoing_node.get_weight_to_target()
                )
                if (
                    calculated_weight_to_target_on_outgoing_node
                    < current_weight_to_target_on_outgoing_node
                    and not outgoing_node.get_dijkstra_visited()
                ):
                    outgoing_node.set_weight_to_target(
                        calculated_weight_to_target_on_outgoing_node
                    )
                    outgoing_node.set_previous_node_to_this_waypoint(current_node)
            current_node.set_dijkstra_visited(True)

    def __reset_dijkstra(self):
        self.shortest_path_to_target.clear()
        for waypoint in self.waypoints:
            waypoint.set_dijkstra_visited(False)
            waypoint.set_previous_node_to_this_waypoint(None)
            waypoint.set_weight_to_target(sys.maxsize)
            waypoint.set_previous_node_to_this_waypoint(None)

    def __get_next_unvisited_node(self):
        unvisited_nodes = self.__get_unvisited_nodes()
        return min(unvisited_nodes, key=lambda n: n.get_weight_to_target())

    def __has_next_unvisited_node(self):
        return len(self.__get_unvisited_nodes()) > 0

    def __get_unvisited_nodes(self):
        return [
            w
            for w in self.waypoints
            if w.get_dijkstra_visited() == False
            and w.get_status()
            not in [WaypointStatus.BLOCKED, WaypointStatus.POTENTIALLY_BLOCKED]
        ]

    def cone_detected(self):
        self.current_waypoint.status = WaypointStatus.BLOCKED

    def obstacle_detected(self):
        edge_from_previous = self.previous_waypoint.get_edge_to_waypoint(
            self.current_waypoint.get_id()
        )
        edge_from_previous.set_status(EdgeStatus.OBSTRUCTED)
        edge_to_previous = self.current_waypoint.get_edge_to_waypoint(
            self.previous_waypoint.get_id()
        )
        edge_to_previous.set_status(EdgeStatus.OBSTRUCTED)

    def __str__(self):
        return f"""
        Graph[
            Current_Waypoint:{self.current_waypoint}
            Target_Waypoint:{self.target_waypoint};
            Previous_Waypoint:{self.previous_waypoint};
            Waypoints:{self.waypoints};
            Shortest_Path_To_Target:{self.shortest_path_to_target}
        ]
        """
