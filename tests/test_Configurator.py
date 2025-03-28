import pytest
import json
from Configuration.Configurator import Configurator

mock_configuration = {
    "communication": {"device": "/dev/ttyAMA1", "baud": 9600},
    "angles": {"X": {"S": 0.0}, "S": {"G": 30.0, "F": 60.0, "X": 180.0, "H": 300.0}},
}


@pytest.fixture
def mock_config_file(tmp_path):
    config_file = tmp_path / "config.json"
    with config_file.open("w") as f:
        json.dump(mock_configuration, f)
    return config_file


def test_singleton_instance(mock_config_file):
    Configurator.initialize(str(mock_config_file))
    instance1 = Configurator()
    instance2 = Configurator()
    assert instance1 is instance2


def test_initialize(mock_config_file):
    Configurator.initialize(str(mock_config_file))
    instance = Configurator()
    assert instance.configuration == mock_configuration


def test_get_angles(mock_config_file):
    Configurator.initialize(str(mock_config_file))
    instance = Configurator()
    angles = instance.get_angles()
    assert angles == mock_configuration["angles"]
    

def test_get_communication(mock_config_file):
    Configurator.initialize(str(mock_config_file))
    instance = Configurator()
    communication = instance.get_communication()
    assert communication == mock_configuration["communication"]