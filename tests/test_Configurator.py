from pathlib import Path
from Configuration.Configurator import Configurator

mock_config_path = Path(__file__).resolve().parent / "mock_config.json"

def test_singleton_instance():
    Configurator.initialize(str(mock_config_path))
    instance1 = Configurator()
    instance2 = Configurator()
    assert instance1 is instance2


def test_initialize():
    Configurator.initialize(str(mock_config_path))
    instance = Configurator()
    assert instance is not None


def test_get_waypoints():
    Configurator.initialize(str(mock_config_path))
    instance = Configurator()
    waypoints = instance.get_waypoints()
    assert len(waypoints) == 9


def test_get_communication():
    Configurator.initialize(str(mock_config_path))
    instance = Configurator()
    communication = instance.get_communication()
    assert communication is not None
    assert communication["device"] == "/dev/ttyAMA1"


def test_get_tolerances():
    Configurator.initialize(str(mock_config_path))
    instance = Configurator()
    tolerances = instance.get_tolerances()
    assert tolerances is not None
    assert tolerances["waypoint"] == 200
