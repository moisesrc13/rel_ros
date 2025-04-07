from rel_ros_master_control.config import load_modbus_config
from rel_ros_master_control.models.modbus_m import ModbusConfig


def test_master_config():
    config = load_modbus_config()
    assert isinstance(config, ModbusConfig)
    assert isinstance(config.iolinks, list)
