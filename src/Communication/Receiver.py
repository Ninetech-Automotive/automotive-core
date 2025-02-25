from abc import ABC, abstractmethod

class Receiver(ABC):
    @abstractmethod
    def receive(self, message):
        pass