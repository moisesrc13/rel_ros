from rel_ros_master_control.config import load_modbus_config
from rel_ros_master_control.models.modbus_m import ModbusConfig, SlaveTCP


def test_config():
    config = load_modbus_config()
    assert isinstance(config, ModbusConfig)
    assert isinstance(config.slaves.master_io_link, SlaveTCP)
