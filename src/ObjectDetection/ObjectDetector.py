from abc import ABC, abstractmethod
from typing import Tuple
from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus

class ObjectDetector(ABC):
    @abstractmethod
    def detect(self) -> Tuple[WaypointStatus, EdgeStatus]:
        pass
