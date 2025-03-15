import cv2
from pathlib import Path
from ultralytics import YOLO
from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus
from ObjectDetection.ObjectDetector import ObjectDetector


class YOLODetector(ObjectDetector):
    def __init__(self, camera, center_stripe_percentage=0.5):
        self.camera = camera
        path_to_model = Path(__file__).resolve().parent / "best.pt"
        self.model = YOLO(path_to_model)
        # percentage of the image width that is considered the center stripe and is checked for obstacles
        self.center_stripe_percentage = center_stripe_percentage

    def detect(self):
        frame = self.camera.get_image_array()
        results = self.model.predict(frame, imgsz=640)
        objects = self.__parse_results(results)
        return self.__get_object_status(objects)

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
