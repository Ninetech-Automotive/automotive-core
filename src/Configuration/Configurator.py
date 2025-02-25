import json

# singleton
class Configurator:
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
            self.initialized = True

    @classmethod
    def initialize(cls, configuration_path):
        cls._configuration_path = configuration_path
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def get_angles(self):
        return self.configuration["angles"]

