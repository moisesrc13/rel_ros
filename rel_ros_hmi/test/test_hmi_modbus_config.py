import pytest

from rel_ros_hmi.config import load_modbus_config
from rel_ros_hmi.models.modbus_m import HoldingRegisters, ModbusConfig, get_register_by_address


@pytest.fixture
def config() -> ModbusConfig:
    return load_modbus_config()


def test_config(config):
    assert isinstance(config, ModbusConfig)
    assert isinstance(config.modbus.holding_registers, HoldingRegisters)


def test_get_register_by_address(config):
    r = get_register_by_address(config.modbus.holding_registers.parameters, 1000)
    assert 2 == 2
