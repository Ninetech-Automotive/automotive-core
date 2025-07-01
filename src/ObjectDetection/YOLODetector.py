from datetime import datetime
import cv2
import json
import os
from pathlib import Path
from ultralytics import YOLO
from Navigation.WaypointStatus import WaypointStatus
from Navigation.Waypoint import Waypoint
from Navigation.EdgeStatus import EdgeStatus
from Navigation.Angle import Angle
from ObjectDetection.ObjectDetector import ObjectDetector
from Navigation.Graph import Graph
from Configuration.Configurator import Configurator


class YOLODetector(ObjectDetector):
    def __init__(self, top_camera, bottom_camera, center_stripe_percentage=0.5):
        self.top_camera = top_camera
        self.bottom_camera = bottom_camera
        path_to_object_model = Path(__file__).resolve().parent / "small_object_model.pt"
        path_to_line_model = Path(__file__).resolve().parent / "small_line_model.pt"
        self.object_model = YOLO(path_to_object_model)
        self.line_model = YOLO(path_to_line_model)
        # percentage of the image width that is considered the center stripe and is checked for obstacles
        self.center_stripe_percentage = center_stripe_percentage

    def detect(self):
        frame = self.bottom_camera.get_image_array()
        results = self.object_model.predict(frame, imgsz=640)
        objects = self.__parse_results(results)
        return self.__get_object_status(objects)
    
    def start_up_process_detect(self):
        graph = Graph()
        frame = self.top_camera.get_image_array()
        object_results = self.object_model.predict(frame, imgsz=640)
        line_results = self.line_model.predict(frame, imgsz=640)
        objects = self.__parse_results(object_results)
        # self.__print_object_coordinates(objects)
        # self.__visualize_results(line_results)

        # save results to files
        output_dir = "test_images"
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        object_detection_file_path = os.path.join(output_dir, f"{timestamp}_objects.jpg")
        line_detection_file_path = os.path.join(output_dir, f"{timestamp}_lines.jpg")
        self.__save_results_to_file(object_results, object_detection_file_path)
        self.__save_results_to_file(line_results, line_detection_file_path)
        line_objects = self.__parse_results(line_results, line_model=True)

        self.__update_waypoints(graph, objects, "cone")
        self.__update_edges(graph, objects, line_objects, ["obstacle", "edge"])
        return graph

    def __get_object_status(self, objects):
        waypoint_status = WaypointStatus.POTENTIALLY_FREE if not self.__check_for_label_in_center_stripe(objects, "cone") else WaypointStatus.POTENTIALLY_BLOCKED
        edge_status = EdgeStatus.POTENTIALLY_FREE if not self.__check_for_label_in_center_stripe(objects, "obstacle") else EdgeStatus.POTENTIALLY_OBSTRUCTED
        return waypoint_status, edge_status

    def __check_for_label_in_center_stripe(self, objects, label):
        center_stripe_width = self.bottom_camera.get_width() * self.center_stripe_percentage
        center = self.bottom_camera.get_width() / 2
        center_stripe_left_bound = center - center_stripe_width / 2
        center_stripe_right_bound = center + center_stripe_width / 2
        for obj in objects:
            if obj["label"] == label:
                center = obj["bounding_box"]["x_min"] + obj["bounding_box"]["width"] / 2
                if center_stripe_left_bound < center < center_stripe_right_bound:
                    return True
        return False
    

    def __update_waypoints(self, graph, objects, label):
        waypoint_statuses = {}
        tolerance = Configurator().get_tolerances()["waypoint"]
        waypoints = Configurator().get_waypoints()
        for waypoint_name, waypoint_data in waypoints.items():
            x = waypoint_data["x"]
            y = waypoint_data["y"]
            waypoint = graph._get_waypoint_by_id(waypoint_name)
            for obj in objects:
                if obj["label"] == label:
                    if (x - tolerance) <= obj["bounding_box"]["x_min"] + (obj["bounding_box"]["width"] / 2) <= (x + tolerance) and \
                       (y - tolerance) <= obj["bounding_box"]["y_max"] <= (y + tolerance):
                        #Set status to POTENTIALLY_BLOCKED if cone is detected
                        waypoint.set_status(WaypointStatus.POTENTIALLY_BLOCKED)
                        # check next waypoint
                        break

            if waypoint.get_status() != WaypointStatus.POTENTIALLY_BLOCKED:
                waypoint.set_status(WaypointStatus.POTENTIALLY_FREE)
            waypoint_statuses[waypoint_name] = waypoint.get_status()
            print(waypoint_statuses)
        return waypoint_statuses
    
    def __update_edges(self, graph, objects, line_objects, labels):
        edge_statuses = {}
        obstacle_tolerance = Configurator().get_tolerances()["obstacle"]
        edge_tolerance_x = Configurator().get_tolerances()["edge_x"]
        edge_tolerance_y = Configurator().get_tolerances()["edge_y"]
        waypoints = Configurator().get_waypoints()
        for waypoint_id, waypoint_data in waypoints.items():
            if waypoint_id == "X":
                continue
            x = waypoint_data["x"]
            y = waypoint_data["y"]
            waypoint = graph._get_waypoint_by_id(waypoint_id)
            angles = waypoint.get_angles()
            # iterate over all angles of the waypoint
            for angle in angles:
                outgoing_waypoint_id = angle.get_waypoint().get_id()
                if outgoing_waypoint_id == "X":
                    continue
                edge_data = waypoint_data["edges"][outgoing_waypoint_id]
                bounding_box_corners = edge_data["bounding_box_corners"]
                edge = angle.get_edge()
                outgoing_x = waypoints[outgoing_waypoint_id]["x"]
                outgoing_y = waypoints[outgoing_waypoint_id]["y"]
                edge_waypoint_coords, edge_outgoing_waypoint_coords = self.__get_relevant_edge_coords(bounding_box_corners) #which coordinates to use for the edge
                # Check for obstucted edges
                for obj in objects:
                    if obj["label"] == labels[0]:
                        obstacle_x = edge_data["obstacle_coords"]["x"]
                        obstacle_y = edge_data["obstacle_coords"]["y"]
                        if (obstacle_x - obstacle_tolerance) <= obj["bounding_box"]["x_min"] <= (obstacle_x + obstacle_tolerance) and \
                            (obstacle_y - obstacle_tolerance) <= obj["bounding_box"]["y_max"] <= (obstacle_y + obstacle_tolerance):
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
                        within_x1 = (x - edge_tolerance_x) <= x1 <= (x + edge_tolerance_x)
                        within_y1 = (y - edge_tolerance_y) <= y1 <= (y + edge_tolerance_y)
                        within_x2 = (outgoing_x - edge_tolerance_x) <= x2 <= (outgoing_x + edge_tolerance_x)
                        within_y2 = (outgoing_y - edge_tolerance_y) <= y2 <= (outgoing_y + edge_tolerance_y)
                        # Special case for waypoint_id == "S"
                        if waypoint_id == "S":
                            condition_met = y1 > 1900 and within_x2 and within_y2
                        elif outgoing_waypoint_id == "S":
                            condition_met = within_x1 and within_y1 and y2 > 1900
                        else:
                            condition_met = within_x1 and within_y1 and within_x2 and within_y2
                        # Update edge status if conditions are met
                        if condition_met:
                            if edge.get_status() != EdgeStatus.POTENTIALLY_OBSTRUCTED:
                                edge.set_status(EdgeStatus.POTENTIALLY_FREE)
                                break  # check next edge    
                edge_statuses[f"{waypoint_id}_to_{outgoing_waypoint_id}"] = edge.get_status()

                edge_statuses[f"{waypoint_id}_to_{outgoing_waypoint_id}"] = edge.get_status()

        print(edge_statuses)
        return edge_statuses
    
    def __get_relevant_edge_coords(self, bounding_box_corners):
        coords = {
            "UPPER_LEFT": ["x_min", "y_min"],
            "UPPER_RIGHT": ["x_max", "y_min"],
            "LOWER_LEFT": ["x_min", "y_max"],
            "LOWER_RIGHT": ["x_max", "y_max"],
        }
        coords_from = coords[bounding_box_corners["from"]]
        coords_to = coords[bounding_box_corners["to"]]
        return coords_from, coords_to
    

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
        def mouse_callback(event, x, y, flags, param):
            nonlocal annotated_frame
            if event == cv2.EVENT_LBUTTONDOWN:  # Left mouse button click
                # Clear the top-right corner
                cv2.rectangle(annotated_frame, (annotated_frame.shape[1] - 200, 0), (annotated_frame.shape[1], 50), (0, 0, 0), -1)
                # Write the coordinates
                text = f"x: {x}, y: {y}"
                cv2.putText(annotated_frame, text, (annotated_frame.shape[1] - 190, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                cv2.imshow("Captured Image", annotated_frame)  # Update the displayed frame

        annotated_frame = results[0].orig_img.copy()
        for result in results[0].boxes:
            x_min, y_min, x_max, y_max = map(int, result.xyxy[0])
            confidence = result.conf[0].item()
            # Draw thicker bounding boxes
            cv2.rectangle(annotated_frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            text = f"({confidence:.2f})"
            cv2.putText(annotated_frame, text, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)

        waypoints = Configurator().get_waypoints()
        for waypoint_id, waypoint_data in waypoints.items():
            x = waypoint_data["x"]
            y = waypoint_data["y"]
            self.__write_text(annotated_frame, waypoint_id, x, y)
            for outgoint_waypoint_id, outgoint_waypoint_data in waypoint_data["edges"].items():
                obstacle_x = outgoint_waypoint_data["obstacle_coords"]["x"]
                obstacle_y = outgoint_waypoint_data["obstacle_coords"]["y"]
                self.__write_text(annotated_frame, f"{waypoint_id}-{outgoint_waypoint_id}", obstacle_x, obstacle_y)

        cv2.namedWindow("Captured Image")
        cv2.setMouseCallback("Captured Image", mouse_callback)
        cv2.imshow("Captured Image", annotated_frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def __save_results_to_file(self, results, save_path):
        annotated_frame = results[0].orig_img.copy()
        for result in results[0].boxes:
            x_min, y_min, x_max, y_max = map(int, result.xyxy[0])
            confidence = result.conf[0].item()
            # Draw thicker bounding boxes
            cv2.rectangle(annotated_frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 3)
            text = f"({confidence:.2f})"
            cv2.putText(annotated_frame, text, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)

        waypoints = Configurator().get_waypoints()
        for waypoint_id, waypoint_data in waypoints.items():
            x = waypoint_data["x"]
            y = waypoint_data["y"]
            self.__write_text(annotated_frame, waypoint_id, x, y)
            for outgoint_waypoint_id, outgoint_waypoint_data in waypoint_data["edges"].items():
                obstacle_x = outgoint_waypoint_data["obstacle_coords"]["x"]
                obstacle_y = outgoint_waypoint_data["obstacle_coords"]["y"]
                self.__write_text(annotated_frame, f"{waypoint_id}-{outgoint_waypoint_id}", obstacle_x, obstacle_y)

        # Draw a 100px ruler at the top-left corner
        start_point = (10, 30)
        end_point = (110, 30)
        cv2.line(annotated_frame, start_point, end_point, (255, 0, 0), 4)
        cv2.putText(annotated_frame, "100px", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        cv2.imwrite(save_path, annotated_frame)

    def __write_text(self, annotated_frame, text, x, y):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 2  # Reduced font scale for smaller text
        thickness = 3  # Reduced thickness for smaller text
        text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
        background_top_left = (x - 5, y - text_size[1] - 5)
        background_bottom_right = (x + text_size[0] + 5, y + 5)
        cv2.rectangle(annotated_frame, background_top_left, background_bottom_right, (0, 0, 0), -1)
        cv2.putText(annotated_frame, text, (x, y), font, font_scale, (255, 255, 255), thickness)

    def __parse_results(self, results, line_model=False):
        objects = []
        for result in results[0].boxes:
            x_min, y_min, x_max, y_max = map(int, result.xyxy[0])
            width = x_max - x_min
            height = y_max - y_min
            label = self.object_model.names[int(result.cls[0])] if not line_model else self.line_model.names[int(result.cls[0])]
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
