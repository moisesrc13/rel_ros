from rel_ros_hmi.config import load_modbus_config
from rel_ros_hmi.models.modbus_m import ModbusConfig


def test_config():
    config = load_modbus_config()
    assert isinstance(config, ModbusConfig)
