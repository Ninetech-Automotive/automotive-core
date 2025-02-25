from abc import ABC, abstractmethod

class CommunicationReceiver(ABC):
    @abstractmethod
    def on_waypoint(self):
        pass

    @abstractmethod
    def on_angle(self, angle: str):
        pass
    
    @abstractmethod
    def on_point_scanning_finished(self):
        pass
    
    @abstractmethod
    def on_turned_to_target_line(self):
        pass

    @abstractmethod
    def on_obstacle_detected(self):
        pass

    @abstractmethod
    def on_cone_detected(self):
        pass