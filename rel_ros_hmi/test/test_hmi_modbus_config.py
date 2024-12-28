import pytest

from rel_ros_hmi.config import load_modbus_config
from rel_ros_hmi.models.modbus_m import (
    ModbusConfig,
    Register,
    SlaveTCP,
    get_hr_addresses,
    get_register_by_address,
)


@pytest.fixture
def config() -> ModbusConfig:
    return load_modbus_config()


def test_get_registers_addrresses():
    addresses = get_hr_addresses(
        [
            Register(address=5004, value=100, name="test4"),
            Register(address=5001, value=100, name="test1"),
            Register(address=5002, value=100, name="test2"),
            Register(address=5003, value=100, name="test3"),
            Register(address=5005, value=100, name="test5"),
        ]
    )
    assert addresses == [5001, 5002, 5003, 5004, 5005]


def test_config(config):
    assert isinstance(config, ModbusConfig)
    assert isinstance(config.holding_registers, list)
    assert isinstance(config.slaves, list)
    assert isinstance(config.slaves[0], SlaveTCP)


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
