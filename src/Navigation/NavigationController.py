from Navigation.Graph import Graph
from Communication.Emitter import Emitter
from ObjectDetection.ObjectDetector import ObjectDetector
from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus
from Validation.Validator import Validator
import sys
import time

class NavigationController():
    def __init__(self, emitter: Emitter, object_detector: ObjectDetector):
        self.emitter = emitter
        self.object_detector = object_detector
        self.communication_available = False
        # keeps track of the outgoing waypoints and their indexes for the current waypoint
        self.outgoing_waypoint_ids = []
        self.is_on_ideal_path = True
        self.currently_turned_angle = 0.0

    def start(self):
        self.check_communication()

    def check_communication(self):
        if not self.communication_available:
            self.__test_communication()
        else:
            pass

    def __test_communication(self):
        self.emitter.emit("ping")
        time.sleep(5)

    def __go_to_next_waypoint_after_portscanning(self):
        next_best_waypoint = self.graph.get_next_best_waypoint()
        next_best_waypoint_index = self.outgoing_waypoint_ids.index(next_best_waypoint.get_id())
        self.outgoing_waypoint_ids.clear()
        self.emitter.emit(f"target_line:{next_best_waypoint_index}")

    def __go_to_next_waypoint_by_ideal_path(self):
        current_waypoint = self.graph.get_current_waypoint()
        next_best_waypoint = self.graph.get_next_best_waypoint()
        angle_value = current_waypoint.get_value_from_angle_to_waypoint(next_best_waypoint.get_id())
        angle_value = angle_value + self.currently_turned_angle
        angle_value = self.__optimize_angle_direction(angle_value)
        self.currently_turned_angle = angle_value
        self.emitter.emit(f"target_line_angle:{angle_value}")

    def __optimize_angle_direction(self, angle_value: float):
        if angle_value > 180:
            return angle_value - 360
        else:
            return angle_value
        
    def use_pointscanning(self):
        self.is_on_ideal_path = False

    def on_pong(self):
        self.communication_available = True
    
    def on_waypoint(self):
        self.graph.update_waypoint_status(WaypointStatus.FREE)
        self.graph.update_previous_edge_status(EdgeStatus.FREE)
        if self.graph.has_reached_target_waypoint():
            print('[pi    ] target reached')
            self.emitter.emit("target_reached")
        elif self.is_on_ideal_path:
            self.__go_to_next_waypoint_by_ideal_path()
        else:
            self.emitter.emit("scan_point")

    def on_angle(self, angle_value: float):
        Validator.validate_angle_value(angle_value)
        angle_value = angle_value + self.currently_turned_angle
        waypoint_status, edge_status = self.object_detector.detect()
        print(f"[pi    ] waypoint_status: {waypoint_status}, edge_status: {edge_status.name}")
        angle = self.graph.update_waypoint_from_angle(angle_value, waypoint_status, edge_status)
        self.outgoing_waypoint_ids.append(angle.get_waypoint().get_id())
        
    def on_point_scanning_finished(self):
        if self.currently_turned_angle != 0.0:
            self.currently_turned_angle = 0.0
        self.graph.update_missing_angles()
        self.__go_to_next_waypoint_after_portscanning()

    def on_line_missing(self):
        # self.is_on_ideal_path = False
        # self.emitter.emit("scan_point")
        intended_waypoint = self.graph.get_shortest_path_to_target()[0]
        self.graph.update_missing_line(intended_waypoint.get_id())
        self.__go_to_next_waypoint_by_ideal_path()

    def on_turned_to_target_line(self):
        self.graph.go_to_next_best_waypoint()
        self.currently_turned_angle = 0.0
        self.emitter.emit("follow_line")

    def on_cone_detected(self):
        # self.is_on_ideal_path = False
        self.graph.cone_detected()
        self.graph.go_back_to_previous_waypoint()

    def on_obstacle_detected(self):
        self.graph.obstacle_detected()

    def on_set_target(self, target_waypoint_id: str):
        Validator.validate_waypoint_id_format(target_waypoint_id)
        # startup procedure
        self.graph = self.object_detector.start_up_process_detect()
        # setup graph
        self.graph.set_target_waypoint(target_waypoint_id)
        self.outgoing_waypoint_ids.append("S")
        print("[pi    ] target set to ", target_waypoint_id)
        # start navigation
        if self.is_on_ideal_path:
            self.__go_to_next_waypoint_by_ideal_path()
        else:
            self.__go_to_next_waypoint_after_portscanning()

    def on_stop(self):
        sys.exit()