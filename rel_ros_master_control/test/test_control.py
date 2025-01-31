from unittest.mock import MagicMock

import pytest

from rel_ros_master_control.config import load_modbus_config
from rel_ros_master_control.control import (
    ModbusStatus,
    RelControl,
    RelModbusMaster,
    SlaveTCP,
    get_builder,
    get_decoder_from_rr,
    get_value,
)
from rel_ros_master_control.models.modbus_m import (
    DigitalHydValve,
    HRegister,
    IOLinkHR,
    RegisterDataType,
    get_register_by_name,
)
from rel_ros_master_control.models.status_device_m import TowerState, TowerStatusDevice
from rel_ros_master_control.util import is_bit_on


@pytest.fixture
def rel_control(monkeypatch):
    config = load_modbus_config()
    slave_tcp = SlaveTCP(
        host="0.0.0.0",
        port=9090,
    )
    monkeypatch.setattr(RelModbusMaster, "do_connect", MagicMock(return_value=None))
    return RelControl(slave=slave_tcp, hr=config.holding_registers)


@pytest.mark.parametrize(
    "test_value",
    [
        (100),
        (110),
        (14),
    ],
)
def test_get_data(rel_control: RelControl, test_value):
    rel_control.read_holding_register = MagicMock(
        return_value=ModbusStatus(
            status="ok",
            value=test_value,
        )
    )
    registers = rel_control.get_data()
    for register in registers:
        assert register.value == test_value


def test_control_hyd_valve(rel_control: RelControl):
    digital_valve = DigitalHydValve()
    slave_conn_mock = MagicMock()
    write_register = MagicMock(return_value=True)
    slave_conn_mock.write_register = write_register
    rel_control.master_io_link.slave_conn = slave_conn_mock
    hr: HRegister = get_register_by_name(rel_control.hr, IOLinkHR.DIGITAL_OUT_HYD_VALVE.value)
    print()
    for _, value in digital_valve.model_dump().items():
        rel_control.apply_hyd_valve_state(value)
        write_register.assert_called_with(
            hr.address,
            value,
        )


def test_control_instance(rel_control: RelControl):
    assert isinstance(rel_control.tower_devive, TowerStatusDevice)
    slave_conn_mock = MagicMock()
    write_registers = MagicMock(return_value=True)
    slave_conn_mock.write_registers = write_registers
    rel_control.master_io_link.slave_conn = slave_conn_mock

    # apply all tower states
    for _, state in enumerate(TowerState):
        state_addresses = getattr(rel_control.tower_devive.tower_status.states, state.value)
        rel_control.apply_tower_state(state)
        write_registers.assert_called_with(
            rel_control.tower_devive.tower_status.start_address,
            state_addresses,
        )


@pytest.mark.parametrize(
    "number, test_bit, expected_bit",
    [
        (10, 0, False),  # where 10 = 1010
        (10, 1, True),
        (10, 2, False),
        (10, 3, True),
        (255, 3, True),  # this is all FF to 1 so any position is True
        (16, 4, True),  # 10000
    ],
)
def test_bit(number, test_bit, expected_bit):
    assert is_bit_on(number, test_bit) is expected_bit


@pytest.mark.parametrize(
    "number, register",
    [
        (
            1234,
            HRegister(
                address=100,
                data_type=RegisterDataType.uint32,
                words=1,
            ),
        ),
        (
            123444,
            HRegister(
                address=100,
                data_type=RegisterDataType.uint32,
                words=1,
            ),
        ),
    ],
)
def test_get_uint32_value(number, register):
    builder = get_builder()
    builder.add_32bit_uint(number)
    rr = builder.to_registers()
    assert get_value(get_decoder_from_rr(rr), register.data_type) == number


@pytest.mark.parametrize(
    "number, register",
    [
        (
            1234.44,
            HRegister(
                address=100,
                data_type=RegisterDataType.float32,
                words=1,
            ),
        ),
        (
            12.3444,
            HRegister(
                address=100,
                data_type=RegisterDataType.float32,
                words=1,
            ),
        ),
    ],
)
def test_get_float32_value(number, register):
    builder = get_builder()
    builder.add_32bit_float(number)
    rr = builder.to_registers()
    assert round(get_value(get_decoder_from_rr(rr), register.data_type)) == round(number)
