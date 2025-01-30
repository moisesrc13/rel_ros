from rel_ros_master_control.config import load_status_device_config
from rel_ros_master_control.models.status_device_m import TowerControlStates, TowerStatus


def test_config():
    config = load_status_device_config()
    assert isinstance(config, TowerStatus)
    assert isinstance(config.start_address, int)
    assert isinstance(config.total_registers, int)
    assert isinstance(config.states, TowerControlStates)
