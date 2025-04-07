import cv2
import json
from pathlib import Path
from ultralytics import YOLO
from Navigation.WaypointStatus import WaypointStatus
from Navigation.Waypoint import Waypoint
from Navigation.EdgeStatus import EdgeStatus
from Navigation.Angle import Angle
from ObjectDetection.ObjectDetector import ObjectDetector


class YOLODetector(ObjectDetector):
    def __init__(self, camera, graph=None, center_stripe_percentage=0.5):
        self.camera = camera
        self.graph = graph
        path_to_model = Path(__file__).resolve().parent / "best.pt"
        self.model = YOLO(path_to_model)
        # percentage of the image width that is considered the center stripe and is checked for obstacles
        self.center_stripe_percentage = center_stripe_percentage
        self.waypoints_config = self.__load_config("waypoints_config.json")
        self.waypoints_tolerance = self.__load_config("waypoints_tolerance.json")
        self.obstacles_config = self.__load_config("obstacles_config.json")
        self.obstacles_tolerance = self.__load_config("obstacles_tolerance.json")

    def __load_config(self, filename):
        config_path = Path(__file__).resolve().parent.parent / "Configuration" / filename
        with open(config_path, "r") as file:
            return json.load(file)

    def detect(self):
        frame = self.camera.get_image_array()
        results = self.model.predict(frame, imgsz=640)
        objects = self.__parse_results(results)
        return self.__get_object_status(objects)
    
    def start_up_process_detect(self):
        if self.graph is None:
            raise ValueError("Graph is not set. Please provide a graph instance.")
        frame = self.camera.get_image_array()
        results = self.model.predict(frame, imgsz=640)
        objects = self.__parse_results(results)
        #self.__visualize_results(results)
        return self.__get_map_status(objects)
        #self.__print_object_coordinates(objects)

    def __get_object_status(self, objects):
        waypoint_status = WaypointStatus.POTENTIALLY_FREE if not self.__check_for_label_in_center_stripe(objects, "cone") else WaypointStatus.POTENTIALLY_BLOCKED
        edge_status = EdgeStatus.POTENTIALLY_FREE if not self.__check_for_label_in_center_stripe(objects, "obstacle") else EdgeStatus.POTENTIALLY_OBSTRUCTED
        return waypoint_status, edge_status

    def __check_for_label_in_center_stripe(self, objects, label):
        center_stripe_width = self.camera.get_width() * self.center_stripe_percentage
        center = self.camera.get_width() / 2
        center_stripe_left_bound = center - center_stripe_width / 2
        center_stripe_right_bound = center + center_stripe_width / 2
        for obj in objects:
            if obj["label"] == label:
                center = obj["bounding_box"]["x_min"] + obj["bounding_box"]["width"] / 2
                if center_stripe_left_bound < center < center_stripe_right_bound:
                    return True
        return False
    
    def __get_map_status(self, objects):
        return {"Waypoint_Statuses": self.__assign_cones_to_waypoints(objects, "cone"),
                "Edge_Statuses": self.__assign_obstacles_to_edges(objects, "obstacle")}
            
    
    def __assign_cones_to_waypoints(self, objects, label):
        waypoint_statuses = {}
        tolerance = self.waypoints_tolerance.get("Tolerance", 0)
        for waypoint_name, coords in self.waypoints_config.items():
            waypoint = self.graph._get_waypoint_by_id(waypoint_name)
            for obj in objects:
                if obj["label"] == label:
                    if (coords["x_min"] - tolerance) <= obj["bounding_box"]["x_min"] <= (coords["x_min"] + tolerance) and \
                       (coords["y_min"] - tolerance) <= obj["bounding_box"]["y_min"] <= (coords["y_min"] + tolerance):
                        waypoint.set_status(WaypointStatus.POTENTIALLY_BLOCKED) #Set status to POTENTIALLY_BLOCKED if cone is detected
                        break #check next waypoint
            if waypoint.get_status() != WaypointStatus.POTENTIALLY_BLOCKED:
                waypoint.set_status(WaypointStatus.POTENTIALLY_FREE)
            waypoint_statuses[waypoint_name] = waypoint.get_status()
        return waypoint_statuses
    
    def __assign_obstacles_to_edges(self, objects, label):
        edge_statuses = {}
        tolerance = self.obstacles_tolerance.get("Tolerance", 0)
        for waypoint_name, outgoing_waypoint_names in self.obstacles_config.items():
            waypoint = self.graph._get_waypoint_by_id(waypoint_name)
            angles = waypoint.get_angles()
            for angle in angles:
                outgoing_waypoint_id = angle.get_waypoint().get_id()
                edge = angle.get_edge()
                for obj in objects:
                    if obj["label"] == label:
                        if (outgoing_waypoint_names[outgoing_waypoint_id]["x_min"] - tolerance) <= obj["bounding_box"]["x_min"] <= (outgoing_waypoint_names[outgoing_waypoint_id]["x_min"] + tolerance) and \
                            (outgoing_waypoint_names[outgoing_waypoint_id]["y_min"] - tolerance) <= obj["bounding_box"]["y_min"] <= (outgoing_waypoint_names[outgoing_waypoint_id]["y_min"] + tolerance):
                            edge.set_status(EdgeStatus.POTENTIALLY_OBSTRUCTED)
                            break #check next edge
                edge_statuses[f"{waypoint_name}_to_{outgoing_waypoint_id}"] = edge.get_status()
        return edge_statuses
                

    def __print_object_coordinates(self, objects):
        """Can be used as a help for setting up the config files"""
        for obj in objects:
            x_min = obj["bounding_box"]["x_min"]
            y_min = obj["bounding_box"]["y_min"]
            width = obj["bounding_box"]["width"]
            height = obj["bounding_box"]["height"]
            label = obj["label"]
            confidence = obj["confidence"]
            print(f"Detected {label} with confidence {confidence:.2f} at ({x_min}, {y_min}) with size ({width}, {height})")

    def __visualize_results(self, results):
        annotated_frame = results[0].plot()
        cv2.imshow("Captured Image", annotated_frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def __parse_results(self, results):
        objects = []
        for result in results[0].boxes:
            x_min, y_min, x_max, y_max = map(int, result.xyxy[0])
            width = x_max - x_min
            height = y_max - y_min
            label = self.model.names[int(result.cls[0])]
            confidence = result.conf[0].item()
            objects.append(
                {
                    "label": label,
                    "confidence": confidence,
                    "bounding_box": {
                        "x_min": x_min,
                        "y_min": y_min,
                        "width": width,
                        "height": height,
                    },
                }
            )
        return objects
