import pytest

from rel_ros_hmi.config import load_modbus_config
from rel_ros_hmi.models.modbus_m import ModbusConfig, Register, get_register_by_address


@pytest.fixture
def config() -> ModbusConfig:
    return load_modbus_config()


def test_config(config):
    assert isinstance(config, ModbusConfig)
    assert isinstance(config.modbus.holding_registers, list)


def test_get_register_by_address():
    test_registers = [
        Register(name="test1", address=1000, data_type="uint16", value=30),
        Register(name="test2", address=2000, data_type="uint16", value=40),
    ]
    r, idx = get_register_by_address(test_registers, 40001)
    assert not r
    assert not idx
    r, idx = get_register_by_address(test_registers, 1000)
    assert r.name == "test1"
    assert idx == 0
    r, idx = get_register_by_address(test_registers, 2000)
    assert r.name == "test2"
    assert idx == 1
