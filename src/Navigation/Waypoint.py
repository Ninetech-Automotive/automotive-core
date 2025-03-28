import sys
from typing import List
from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus
from Navigation.Angle import Angle
from Validation.Validator import Validator

class Waypoint:
    def __init__(self, id: str):
        self.status = WaypointStatus.UNKNOWN
        self.id = id
        self.angles: List[Angle] = []
        self.incoming_angle: float = 0.0
        self.weight_to_target: int = sys.maxsize
        self.dijkstra_visied = False
        # The previous node to this waypoint in the shortest path from the current waypoint to the target waypoint.
        self.previous_node_to_this_waypoint = None

    def get_id(self):
        return self.id
    
    def get_status(self):
        return self.status

    def set_angles(self, angles: List[Angle]):
        self.angles = angles

    def set_status(self, status: WaypointStatus):
        self.status = status

    def set_previous_node_to_this_waypoint(self, waypoint):
        self.previous_node_to_this_waypoint = waypoint

    def get_previous_node_to_this_waypoint(self):
        return self.previous_node_to_this_waypoint

    def set_incoming_angle_by_id(self, waypoint_id: str):
        Validator.validate_waypoint_id_format(waypoint_id)
        angle = [a for a in self.angles if a.get_waypoint().get_id() == waypoint_id][0]
        self.incoming_angle = angle.get_value()

    def set_weight_to_target(self, weight: int):
        self.weight_to_target = weight

    def set_dijkstra_visited(self, visited: bool):
        self.dijkstra_visied = visited

    def get_dijkstra_visited(self):
        return self.dijkstra_visied

    def get_weight_to_target(self):
        return self.weight_to_target

    def get_angles(self):
        return self.angles

    def get_possible_angles(self):
        unblocked_angles = [a for a in self.angles if a.get_waypoint().get_status() not in [WaypointStatus.BLOCKED, WaypointStatus.POTENTIALLY_BLOCKED]]
        return [a for a in unblocked_angles if a.get_edge().get_status() not in [EdgeStatus.MISSING, EdgeStatus.POTENTIALLY_MISSING]]
    
    def set_angle_to_waypoint_as_missing(self, waypoint_id: str):
        Validator.validate_waypoint_id_format(waypoint_id)
        angle = self.__get_angle_to_waypoint(waypoint_id)
        angle.get_edge().set_status(EdgeStatus.MISSING)
        
    
    def __get_angle_to_waypoint(self, waypoint_id: str):
        return [a for a in self.angles if a.get_waypoint().get_id() == waypoint_id][0]
    
    def get_edge_to_waypoint(self, waypoint_id: str):
        Validator.validate_waypoint_id_format(waypoint_id)
        angle = self.__get_angle_to_waypoint(waypoint_id)
        return angle.get_edge()
    
    def update_angle(self, value: float, waypoint_status: WaypointStatus, edge_status: EdgeStatus):
        Validator.validate_angle_value(value)
        Validator.validate_waypoint_status(waypoint_status)
        Validator.validate_edge_status(edge_status)
        angle = self.__get_angle_from_value(value)
        if not angle.get_waypoint().get_status() in [WaypointStatus.BLOCKED, WaypointStatus.FREE]:
            angle.get_waypoint().set_status(waypoint_status)
        if not angle.get_edge().get_status() in [EdgeStatus.OBSTRUCTED, EdgeStatus.FREE]:
            angle.get_edge().set_status(edge_status)
        return angle
    
    def update_edge_to_waypoint(self, waypoint_id: str):
        Validator.validate_waypoint_id_format(waypoint_id)
        edge = self.get_edge_to_waypoint(waypoint_id)
        if edge.get_status() is not EdgeStatus.OBSTRUCTED:
            edge.set_status(EdgeStatus.FREE)

    def __get_angle_from_value(self, value):
        calculated_angle = self.__calculate_angle_from_value(value)
        # returns the angle with the predefined value that is closest to the calculated angle
        return min(self.angles, key=lambda a: self.__modulo_360_difference(a.get_value(),calculated_angle))
    
    def __modulo_360_difference(self, a, b):
        diff = (a - b) % 360
        if diff > 180:
            diff -= 360
        return abs(diff)

    def __calculate_angle_from_value(self, value: float):
        return (self.incoming_angle - 180.0 + value) % 360
    
    def __str__(self):
        return f"Waypoint[Status:{self.status};ID:{self.id};Angles:{self.angles};Incoming_Angle:{self.incoming_angle};Weight_To_Target:{self.weight_to_target};Dijkstra_Visited:{self.dijkstra_visied};previous_node_to_this_waypoint:{self.previous_node_to_this_waypoint}]"