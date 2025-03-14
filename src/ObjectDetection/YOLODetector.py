from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus
from ObjectDetection.ObjectDetector import ObjectDetector

class YOLODetector(ObjectDetector):

    def detect(self):
        # dummy implementation
        return WaypointStatus.POTENTIALLY_FREE, EdgeStatus.POTENTIALLY_FREE
