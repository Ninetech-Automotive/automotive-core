from abc import ABC, abstractmethod

class Camera(ABC):
    @abstractmethod
    def enable(self):
        pass

    @abstractmethod
    def disable(self):
        pass

    @abstractmethod
    def get_width(self) -> int:
        pass

    @abstractmethod
    def get_height(self) -> int:
        pass

    @abstractmethod
    def get_image_array(self) -> list[list[list[3]]]:
        pass