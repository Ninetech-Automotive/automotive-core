import sys
import json
from Navigation.Waypoint import Waypoint
from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus
from Navigation.Angle import Angle
from Navigation.Edge import Edge

class Graph:
    def __init__(self):
        self.current_waypoint: Waypoint = None
        self.target_waypoint: Waypoint = None
        self.previous_waypoint: Waypoint = None
        self.waypoints = []
        self.initialize_waypoints()
        self.shortest_path_to_target = []

    def initialize_waypoints(self):
        x = Waypoint("X")
        s = Waypoint("S")
        h = Waypoint("H")
        g = Waypoint("G")
        f = Waypoint("F")
        i = Waypoint("I")
        a = Waypoint("A")
        c = Waypoint("C")
        b = Waypoint("B")
        self.waypoints = [x,s,h,g,f,i,a,c,b]
        self.current_waypoint = x
        self.current_waypoint.set_dijkstra_visited(True)
        self.current_waypoint.set_status(WaypointStatus.FREE)

        with open('./Navigation/angles.json', 'r') as file:
            data = json.load(file)
        
        for waypoint_id, angle_values in data.items():
            waypoint = self.get_waypoint_by_id(waypoint_id)
            angles = []
            for outgoing_waypoint_id, angle in angle_values.items():
                edge = Edge()
                outgoing_waypoint = self.get_waypoint_by_id(outgoing_waypoint_id)
                angle = Angle(outgoing_waypoint, angle, edge)
                angles.append(angle)
            waypoint.set_angles(angles)
                
    def get_waypoint_by_id(self, id):
        return [w for w in self.waypoints if w.get_id() == id][0]

    def set_target_waypoint(self, target_waypoint_id):
        self.target_waypoint = self.get_waypoint_by_id(target_waypoint_id)

    def go_to_next_best_waypoint(self):
        next_best_waypoint = self.get_next_best_waypoint()
        next_best_waypoint.set_incoming_angle_by_id(self.current_waypoint.get_id())
        self.previous_waypoint = self.current_waypoint
        self.current_waypoint = next_best_waypoint
        return next_best_waypoint.get_id()
    
    def get_next_best_waypoint(self):
        self.calculate_shortest_path()
        self.store_shortest_path()
        return self.shortest_path_to_target[0]

    def store_shortest_path(self):
        node = self.target_waypoint
        while (node.get_id() != self.current_waypoint.get_id()):
            self.shortest_path_to_target.insert(0, node)
            node = node.get_previous_node_to_current_waypoint()
        print('[pi    ] shortest path: ', list(map(lambda n: n.get_id(), self.shortest_path_to_target)))
 
    def go_back_to_previous_waypoint(self):
        temp_current_waypoint = self.current_waypoint
        self.current_waypoint = self.previous_waypoint
        self.current_waypoint.set_incoming_angle_by_id(temp_current_waypoint.get_id())
        self.previous_waypoint = temp_current_waypoint

    def has_reached_target_waypoint(self):
        return self.current_waypoint == self.target_waypoint
    
    def update_waypoint_status(self, waypoint_status):
        self.current_waypoint.set_status(waypoint_status)

    def update_edge_status(self):
        self.current_waypoint.update_edge_to_waypoint(self.previous_waypoint.get_id())

    def update_waypoint_from_angle(self, angle_value, waypoint_status, edge_status):
        return self.current_waypoint.update_angle(angle_value, waypoint_status, edge_status)

    def remove_missing_angles(self):
        for angle in self.current_waypoint.get_angles():
            if angle.get_edge().get_status() == EdgeStatus.UNKNOWN:
                angle.get_waypoint().remove_angle_to_waypoint(self.current_waypoint.get_id())
                self.current_waypoint.remove_angle_to_waypoint(angle.get_waypoint().get_id())

    def calculate_shortest_path(self):
        # dijkstra
        self.reset_dijkstra()
        self.current_waypoint.set_weight_to_target(0)
        while(self.has_next_unvisited_node()):
            current_node = self.get_next_unvisited_node()
            for angle in current_node.get_unblocked_angles():
                waypoint = angle.get_waypoint()
                edge = angle.get_edge()
                weight_to_target = current_node.get_weight_to_target() + edge.get_weight()
                if (weight_to_target < waypoint.get_weight_to_target() and not waypoint.get_dijkstra_visited()):
                    waypoint.set_weight_to_target(weight_to_target)
                    waypoint.set_previous_node_to_current_waypoint(current_node)
            current_node.set_dijkstra_visited(True)

    def reset_dijkstra(self):
        self.shortest_path_to_target.clear()
        for waypoint in self.waypoints:
            waypoint.set_dijkstra_visited(False)
            waypoint.set_previous_node_to_current_waypoint(None)
            waypoint.set_weight_to_target(sys.maxsize)
            waypoint.set_previous_node_to_current_waypoint(None)

    def get_next_unvisited_node(self):
        unvisited_nodes = self.get_unvisited_nodes()
        return min(unvisited_nodes, key=lambda n: n.get_weight_to_target())
    
    def has_next_unvisited_node(self):
        return len(self.get_unvisited_nodes()) > 0
    
    def get_unvisited_nodes(self):
        return [w for w in self.waypoints if w.get_dijkstra_visited() == False and w.get_status() not in [WaypointStatus.BLOCKED, WaypointStatus.POTENTIALLY_BLOCKED]]
    
    def cone_detected(self):
        self.current_waypoint.status = WaypointStatus.BLOCKED

    def obstacle_detected(self):
        edge = self.previous_waypoint.get_edge_to_waypoint(self.current_waypoint.get_id())
        edge.set_status(EdgeStatus.OBSTRUCTED)

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
