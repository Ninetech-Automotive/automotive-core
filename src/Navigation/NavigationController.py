from Navigation.Graph import Graph
from Communication.Emitter import Emitter
from ObjectDetection.ObjectDetector import ObjectDetector
from Navigation.WaypointStatus import WaypointStatus

class NavigationController():
    def __init__(self, emitter: Emitter, object_detector: ObjectDetector):
        self.graph = Graph()
        self.emitter = emitter
        self.object_detector = object_detector
        # keeps track of the outgoing waypoints and their indexes for the current waypoint
        self.outgoing_waypoint_ids = []

    def startup_procedure(self):
        # TODO
        pass

    def start(self, target_waypoint: str):
        self.graph.set_target_waypoint(target_waypoint)
        self.outgoing_waypoint_ids.append("S")
        print("[pi    ] target set to ", target_waypoint)
        self.emitter.emit("ping")
        # controller should now receive a 'pong' message and continue with the on_pong method

    def next(self):
        next_best_waypoint_id = self.graph.go_to_next_best_waypoint()
        next_best_waypoint_index = self.outgoing_waypoint_ids.index(next_best_waypoint_id)
        self.outgoing_waypoint_ids.clear()
        self.emitter.emit(f"target_line:{next_best_waypoint_index}")

    def on_pong(self):
        # continue with the next waypoint if communication test was successful
        self.next()
    
    def on_waypoint(self):
        self.graph.update_waypoint_status(WaypointStatus.FREE)
        self.graph.update_previous_edge_status()
        if (not self.graph.has_reached_target_waypoint()):
            self.emitter.emit("scan_point")
        else:
            print('[pi    ] target reached')

    def on_angle(self, angle_value):
        waypoint_status, edge_status = self.object_detector.detect()
        print(f"[pi    ] waypoint_status: {waypoint_status}, edge_status: {edge_status.name}")
        angle = self.graph.update_waypoint_from_angle(angle_value, waypoint_status, edge_status)
        self.outgoing_waypoint_ids.append(angle.get_waypoint().get_id())
        
    def on_point_scanning_finished(self):
        self.graph.remove_missing_angles()
        self.next()

    def on_turned_to_target_line(self):
        self.emitter.emit("follow_line")

    def on_cone_detected(self):
        self.graph.cone_detected()
        self.graph.go_back_to_previous_waypoint()

    def on_obstacle_detected(self):
        self.graph.obstacle_detected()
