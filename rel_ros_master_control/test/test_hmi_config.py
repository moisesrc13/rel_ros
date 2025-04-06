from rel_ros_master_control.config import load_hmi_config
from rel_ros_master_control.models.hmi_m import ModbusHMIConfig


def test_dummy():
    config = load_hmi_config()
    assert isinstance(config, ModbusHMIConfig)
