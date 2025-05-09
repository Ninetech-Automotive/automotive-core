import json
from Validation.Validator import Validator

class Configurator:
    """
    Singleton class which manages static configurations.
    """

    _instance = None
    _configuration_path = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(Configurator, cls).__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self):
        if not hasattr(self, 'initialized'):
            with open(self._configuration_path, 'r') as file:
                self.configuration = json.load(file)
            Validator.validate_configuration(self.configuration)
            self.initialized = True

    @classmethod
    def initialize(cls, configuration_path):
        cls._configuration_path = configuration_path
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def get_waypoints(self):
        return self.configuration["waypoints"]
    
    def get_communication(self):
        return self.configuration["communication"]
    
    def get_tolerances(self):
        return self.configuration["tolerances"]

