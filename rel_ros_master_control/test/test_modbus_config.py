from rel_ros_master_control.config import load_modbus_config
from rel_ros_master_control.models.modbus_m import ModbusConfig, ModbusIOLinkSlaves


def test_config():
    config = load_modbus_config()
    assert isinstance(config, ModbusConfig)
    assert isinstance(config.slaves, list)
    print(type(config.slaves[0]))
    assert isinstance(config.slaves[0], ModbusIOLinkSlaves)
