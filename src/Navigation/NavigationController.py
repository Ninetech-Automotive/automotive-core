from Navigation.Graph import Graph
from Communication.Communicator import Communicator
from ObjectDetection.ObjectDetector import ObjectDetector
from Communication.CommunicationReceiver import CommunicationReceiver
from Navigation.WaypointStatus import WaypointStatus

class NavigationController(CommunicationReceiver):
    def __init__(self, communicator: Communicator, object_detector: ObjectDetector):
        self.graph = Graph()
        self.communicator = communicator
        self.object_detector = object_detector
        self.angle_ids = []

    def startup_procedure(self):
        # TODO
        pass

    def start(self, target_waypoint: str):
        print("[pi    ] target set to ", target_waypoint)
        self.graph.set_target_waypoint(target_waypoint)
        self.angle_ids.append("S")
        self.next()

    def next(self):
        next_best_waypoint_id = self.graph.go_to_next_best_waypoint()
        next_best_waypoint_index = self.angle_ids.index(next_best_waypoint_id)
        self.angle_ids.clear()
        self.communicator.emit(f"target_line:{next_best_waypoint_index}")
    
    def on_waypoint(self):
        self.graph.update_waypoint_status(WaypointStatus.FREE)
        self.graph.update_edge_status()
        if (not self.graph.has_reached_target_waypoint()):
            self.communicator.emit("scan_point")
        else:
            print('[pi    ] target reached')

    def on_angle(self, angle_value):
        waypoint_status, edge_status = self.object_detector.detect()
        print(f"[pi    ] waypoint_status: {waypoint_status}, edge_status: {edge_status.name}")
        angle = self.graph.update_waypoint_from_angle(angle_value, waypoint_status, edge_status)
        self.angle_ids.append(angle.get_waypoint().get_id())
        
    def on_point_scanning_finished(self):
        self.graph.remove_missing_angles()
        self.next()

    def on_turned_to_target_line(self):
        self.communicator.emit("follow_line")

    def on_cone_detected(self):
        self.graph.cone_detected()
        self.graph.go_back_to_previous_waypoint()

    def on_obstacle_detected(self):
        self.graph.obstacle_detected()
