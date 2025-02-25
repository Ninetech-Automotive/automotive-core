from Communication.CommunicationReceiver import CommunicationReceiver
from abc import ABC, abstractmethod

class Communicator(ABC):
    @abstractmethod
    def set_communication_receiver(self, communication_receiver: CommunicationReceiver):
        pass

    @abstractmethod
    def emit(self, message):
        pass

    @abstractmethod
    def receive(self):
        pass
            
    @abstractmethod
    def ping(self):
        pass

    @abstractmethod
    def handle_pong(self):
        pass
