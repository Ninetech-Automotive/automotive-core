from abc import ABC, abstractmethod

class Emitter(ABC):
    @abstractmethod
    def emit(self, message):
        pass