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
        path_to_line_model = Path(__file__).resolve().parent / "line_model.pt"
        self.model = YOLO(path_to_model)
        self.line_model = YOLO(path_to_line_model)
        # percentage of the image width that is considered the center stripe and is checked for obstacles
        self.center_stripe_percentage = center_stripe_percentage
        self.waypoints_config = self.__load_config("waypoints_config.json")
        self.tolerances = self.__load_config("waypoints_tolerance.json")
        self.obstacles_config = self.__load_config("obstacles_config.json")
        self.lines_config = self.__load_config("lines_config.json")

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
        line_results = self.line_model.predict(frame, imgsz=640)
        objects = self.__parse_results(results)
        line_objects = self.__parse_results(line_results, line_model=True)
        self.__print_object_coordinates(line_objects)
        self.__visualize_results(line_results)
        return self.__get_map_status(objects, line_objects)

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
    
    def __get_map_status(self, objects, line_objects):
        return {"Waypoint_Statuses": self.__update_waypoints(objects, "cone"),
                "Edge_Statuses": self.__update_edges(objects, line_objects, ["obstacle", "edge"]),}
            
    
    def __update_waypoints(self, objects, label):
        waypoint_statuses = {}
        tolerance = self.tolerances.get("waypoint_tolerance", 0)
        for waypoint_name, coords in self.waypoints_config.items():
            waypoint = self.graph._get_waypoint_by_id(waypoint_name)
            for obj in objects:
                if obj["label"] == label:
                    if (coords["x"] - tolerance) <= obj["bounding_box"]["x_min"] <= (coords["x"] + tolerance) and \
                       (coords["y"] - tolerance) <= obj["bounding_box"]["y_min"] <= (coords["y"] + tolerance):
                        waypoint.set_status(WaypointStatus.POTENTIALLY_BLOCKED) #Set status to POTENTIALLY_BLOCKED if cone is detected
                        break #check next waypoint

            if waypoint.get_status() != WaypointStatus.POTENTIALLY_BLOCKED:
                waypoint.set_status(WaypointStatus.POTENTIALLY_FREE)
            waypoint_statuses[waypoint_name] = waypoint.get_status()
        return waypoint_statuses
    
    def __update_edges(self, objects, line_objects, labels):
        edge_statuses = {}
        obstacle_tolerance = self.obstacles_tolerance.get("obstacle_tolerance", 0)
        edge_tolerance_x = self.obstacles_tolerance.get("edge_tolerance_x", 0)
        edge_tolerance_y = self.obstacles_tolerance.get("edge_tolerance_y", 0)
        waypoints_config = self.waypoints_config
        lines_config = self.lines_config
        for waypoint_id, outgoing_waypoint_names in self.obstacles_config.items():
            waypoint = self.graph._get_waypoint_by_id(waypoint_id)
            angles = waypoint.get_angles()
            # iterate over all angles of the waypoint
            for angle in angles:
                outgoing_waypoint_id = angle.get_waypoint().get_id()
                if outgoing_waypoint_id == "X":
                    continue
                edge = angle.get_edge()
                edge_waypoint_coords, edge_outgoing_waypoint_coords = self.__get_relevant_edge_coords(lines_config, waypoint_id, outgoing_waypoint_id) #which coordinates to use for the edge
                waypoint_x_coords, waypoint_y_coords = waypoints_config[waypoint_id]["x"], waypoints_config[waypoint_id]["y"]#get config coordinates of the waypoint
                outgoing_waypoint_x_coords, outgoing_waypoint_y_coords = waypoints_config[outgoing_waypoint_id]["x"], waypoints_config[outgoing_waypoint_id]["y"] #get config coordinates of the outgoing waypoint
                # Check for obstucted edges
                for obj in objects:
                    if obj["label"] == labels[0]:
                        if (outgoing_waypoint_names[outgoing_waypoint_id]["x_min"] - obstacle_tolerance) <= obj["bounding_box"]["x_min"] <= (outgoing_waypoint_names[outgoing_waypoint_id]["x_min"] + obstacle_tolerance) and \
                            (outgoing_waypoint_names[outgoing_waypoint_id]["y_min"] - obstacle_tolerance) <= obj["bounding_box"]["y_min"] <= (outgoing_waypoint_names[outgoing_waypoint_id]["y_min"] + obstacle_tolerance):
                            edge.set_status(EdgeStatus.POTENTIALLY_OBSTRUCTED)
                            continue #check next edge

                # Check for free edges
                for line_obj in line_objects:
                    if line_obj["label"] == labels[1]:
                        bbox = line_obj["bounding_box"]
                        # Get relevant bounding box coordinates
                        x1 = bbox[edge_waypoint_coords[0]]
                        y1 = bbox[edge_waypoint_coords[1]]
                        x2 = bbox[edge_outgoing_waypoint_coords[0]]
                        y2 = bbox[edge_outgoing_waypoint_coords[1]]
                        # Check the conditions
                        within_x1 = (waypoint_x_coords - edge_tolerance_x) <= x1 <= (waypoint_x_coords + edge_tolerance_x)
                        within_y1 = (waypoint_y_coords - edge_tolerance_y) <= y1 <= (waypoint_y_coords + edge_tolerance_y)
                        within_x2 = (outgoing_waypoint_x_coords - edge_tolerance_x) <= x2 <= (outgoing_waypoint_x_coords + edge_tolerance_x)
                        within_y2 = (outgoing_waypoint_y_coords - edge_tolerance_y) <= y2 <= (outgoing_waypoint_y_coords + edge_tolerance_y)
                        # Special case for waypoint_id == "S"
                        if waypoint_id == "S":
                            condition_met = x1 < 4032 and within_y1 and within_x2 and within_y2
                        if outgoing_waypoint_id == "S":
                            condition_met = within_x1 and within_y1 and x2 < 4032 and within_y2
                        else:
                            condition_met = within_x1 and within_y1 and within_x2 and within_y2
                        # Update edge status if conditions are met
                        if condition_met:
                            if edge.get_status() != EdgeStatus.POTENTIALLY_OBSTRUCTED:
                                edge.set_status(EdgeStatus.POTENTIALLY_FREE)
                                break  # check next edge
                        
                edge_statuses[f"{waypoint_id}_to_{outgoing_waypoint_id}"] = edge.get_status()
        return edge_statuses
    
    def __get_relevant_edge_coords(self,lines_config, waypoint_name, outgoing_waypoint_id):
        for key, value in lines_config.items():
            if key in (waypoint_name, outgoing_waypoint_id):
                for sub_key, sub_value in value.items():
                    if sub_key in (waypoint_name, outgoing_waypoint_id) and key != sub_key:
                        if key == waypoint_name:
                            return sub_value[0:2], sub_value[2:4]
                        else:
                            return sub_value[2:4], sub_value[0:2]
        return None, None  # Falls nichts gefunden wird
    

    def __print_object_coordinates(self, objects):
        """Can be used as a help for setting up the config files"""
        for obj in objects:
            x_min = obj["bounding_box"]["x_min"]
            x_max = obj["bounding_box"]["x_max"]
            y_min = obj["bounding_box"]["y_min"]
            y_max = obj["bounding_box"]["y_max"]
            width = obj["bounding_box"]["width"]
            height = obj["bounding_box"]["height"]
            label = obj["label"]
            confidence = obj["confidence"]
            print(f"Detected {label} with confidence {confidence:.2f} at x_min: {x_min}, x_max: {x_max}, y_min: {y_min}, y_max: {y_max} with size ({width}, {height})")

    def __visualize_results(self, results):
        annotated_frame = results[0].plot()
        cv2.imshow("Captured Image", annotated_frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def __parse_results(self, results, line_model=False):
        objects = []
        for result in results[0].boxes:
            x_min, y_min, x_max, y_max = map(int, result.xyxy[0])
            width = x_max - x_min
            height = y_max - y_min
            label = self.model.names[int(result.cls[0])] if not line_model else self.line_model.names[int(result.cls[0])]
            confidence = result.conf[0].item()
            objects.append(
                {
                    "label": label,
                    "confidence": confidence,
                    "bounding_box": {
                        "x_min": x_min,
                        "x_max": x_max,
                        "y_min": y_min,
                        "y_max": y_max,
                        "width": width,
                        "height": height,
                    },
                }
            )
        return objects
