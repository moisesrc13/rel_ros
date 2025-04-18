import pytest

from rel_ros_hmi.config import load_modbus_config
from rel_ros_hmi.models.modbus_m import (
    HRegister,
    ModbusConfig,
    SlaveHMI,
    SlaveTCP,
    get_hr_addresses,
    get_register_by_address,
    get_register_by_name,
)


@pytest.fixture
def config() -> ModbusConfig:
    return load_modbus_config()


def test_get_registers_name():
    register = get_register_by_name(
        [
            HRegister(address=5004, value=100, name="test4"),
            HRegister(address=5001, value=100, name="test1"),
            HRegister(address=5002, value=100, name="test2"),
            HRegister(address=5003, value=100, name="test3"),
            HRegister(address=5005, value=100, name="test5"),
        ],
        "test2",
    )
    assert isinstance(register, HRegister)
    assert register.address == 5002


def test_get_registers_addrresses():
    addresses = get_hr_addresses(
        [
            HRegister(address=5004, value=100, name="test4"),
            HRegister(address=5001, value=100, name="test1"),
            HRegister(address=5002, value=100, name="test2"),
            HRegister(address=5003, value=100, name="test3"),
            HRegister(address=5005, value=100, name="test5"),
        ]
    )
    assert addresses == [5001, 5002, 5003, 5004, 5005]


def test_config(config):
    assert isinstance(config, ModbusConfig)
    assert isinstance(config.holding_registers, list)
    assert isinstance(config.hmis, list)
    assert isinstance(config.hmis[0], SlaveHMI)
    hmi_slave = config.hmis[0]
    assert isinstance(hmi_slave.slave_tcp, SlaveTCP)


def test_get_register_by_address():
    test_registers = [
        HRegister(name="test1", address=1000, data_type="uint16", value=30),
        HRegister(name="test2", address=2000, data_type="uint16", value=40),
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
