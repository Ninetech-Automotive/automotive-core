import sys
from typing import List
from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus
from Navigation.Angle import Angle

class Waypoint:
    def __init__(self, id: str):
        self.status = WaypointStatus.UNKNOWN
        self.id = id
        self.angles: List[Angle] = []
        self.incoming_angle: float = 0.0
        self.weight_to_target: int = sys.maxsize
        self.dijkstra_visied = False
        self.previous_node_to_current_waypoint = None

    def get_id(self):
        return self.id
    
    def get_status(self):
        return self.status

    def set_angles(self, angles):
        self.angles = angles

    def set_status(self, status):
        self.status = status

    def set_previous_node_to_current_waypoint(self, waypoint):
        self.previous_node_to_current_waypoint = waypoint

    def get_previous_node_to_current_waypoint(self):
        return self.previous_node_to_current_waypoint

    def set_incoming_angle_by_id(self, waypoint_id):
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

    def get_unblocked_angles(self):
        return [a for a in self.angles if a.get_waypoint().get_status() not in [WaypointStatus.BLOCKED, WaypointStatus.POTENTIALLY_BLOCKED]]
    
    def remove_angle_to_waypoint(self, waypoint_id):
        self.angles = [a for a in self.angles if a.get_waypoint().get_id() != waypoint_id]
    
    def get_angle_to_waypoint(self, waypoint_id):
        return [a for a in self.angles if a.get_waypoint().get_id() == waypoint_id][0]
    
    def get_edge_to_waypoint(self, waypoint_id):
        angle = self.get_angle_to_waypoint(waypoint_id)
        return angle.get_edge()
    
    def update_angle(self, value: float, waypoint_status: WaypointStatus, edge_status: EdgeStatus):
        angle = self.get_angle_from_value(value)
        if not angle.get_waypoint().get_status() in [WaypointStatus.BLOCKED, WaypointStatus.FREE]:
            angle.get_waypoint().set_status(waypoint_status)
        if not angle.get_edge().get_status() in [EdgeStatus.OBSTRUCTED, EdgeStatus.FREE]:
            angle.get_edge().set_status(edge_status)
        return angle
    
    def update_edge_to_waypoint(self, waypoint_id):
        edge = self.get_edge_to_waypoint(waypoint_id)
        if edge.get_status() is not EdgeStatus.OBSTRUCTED:
            edge.set_status(EdgeStatus.FREE)

    def get_angle_from_value(self, value: float):
        calculated_angle = self.calculate_angle_from_value(value)
        return min(self.angles, key=lambda a: self.modulo_360_difference(a.get_value(),calculated_angle))
    
    def modulo_360_difference(self, a, b):
        diff = (a - b) % 360
        if diff > 180:
            diff -= 360
        return abs(diff)

    def calculate_angle_from_value(self, value: float):
        return (self.incoming_angle - 180.0 + value) % 360
    
    def __str__(self):
        return f"Waypoint[Status:{self.status};ID:{self.id};Angles:{self.angles};Incoming_Angle:{self.incoming_angle};Weight_To_Target:{self.weight_to_target};Dijkstra_Visited:{self.dijkstra_visied};Previous_Node_To_Current_Waypoint:{self.previous_node_to_current_waypoint}]"