from unittest.mock import MagicMock

import pytest

from rel_ros_master_control.config import load_hmi_config, load_modbus_config
from rel_ros_master_control.constants import (
    DigitalHydValve,
    DigitalOutput,
    HMIWriteAction,
    ManifoldActions,
)
from rel_ros_master_control.control import (
    ModbusStatus,
    RelControl,
    RelModbusMaster,
    SlaveIOLink,
    get_builder,
    get_decoder_from_rr,
    get_value,
)
from rel_ros_master_control.models.hmi_m import SlaveHMI
from rel_ros_master_control.models.modbus_m import (
    HRegister,
    RegisterDataType,
    SlaveTCP,
    get_register_by_name,
)
from rel_ros_master_control.models.status_device_m import TowerState, TowerStatusDevice
from rel_ros_master_control.util import is_bit_on


class ModbusMockResponse:
    registers: list[int]

    def __init__(self, registers: list[int]):
        self.registers = registers

    def isError(self):
        return False


def writer_register_mock(rel_control: RelControl) -> MagicMock:
    slave_conn_mock = MagicMock()
    write_register = MagicMock(return_value=True)
    slave_conn_mock.write_register = write_register
    rel_control.master_io_link.slave_conn = slave_conn_mock
    return write_register


def writer_registers_mock(rel_control: RelControl) -> MagicMock:
    slave_conn_mock = MagicMock()
    write_registers = MagicMock(return_value=True)
    slave_conn_mock.write_registers = write_registers
    rel_control.master_io_link.slave_conn = slave_conn_mock
    return write_registers


@pytest.fixture
def rel_control(monkeypatch):
    config = load_modbus_config()
    slave_tcp = SlaveTCP(
        host="0.0.0.0",
        port=9090,
    )
    iolink_slave = SlaveIOLink(slave_tcp=slave_tcp)
    hmi_slave = SlaveIOLink(slave_tcp=slave_tcp)
    hmi_config = load_hmi_config()
    monkeypatch.setattr(RelModbusMaster, "do_connect", MagicMock(return_value=None))
    return RelControl(
        iolink_slave=iolink_slave,
        iolink_hr=config.holding_registers,
        hmi_slave=hmi_slave,
        hmi_hr=hmi_config.holding_registers,
        hmi_cr=hmi_config.coil_registers,
    )


@pytest.mark.parametrize(
    "test_value",
    [
        (100),
        (110),
        (14),
    ],
)
def test_get_data(rel_control: RelControl, test_value):
    rel_control.read_iolink_hregister = MagicMock(
        return_value=ModbusStatus(
            status="ok",
            value=test_value,
        )
    )
    registers = rel_control.get_iolink_data()
    for register in registers:
        assert register.value == test_value


@pytest.mark.parametrize(
    "test_address, modbus_value, count",
    [
        (100, [10], 1),
        (110, [20], 1),
        (14, [30], 1),
    ],
)
def test_read_holding_register(rel_control: RelControl, test_address, modbus_value, count):
    slave_conn_mock = MagicMock()
    read_register = MagicMock(return_value=ModbusMockResponse(modbus_value))
    slave_conn_mock.read_holding_registers = read_register
    rel_control.master_io_link.slave_conn = slave_conn_mock
    offset = rel_control.master_io_link.slave.slave_tcp.offset
    called_address = test_address - offset
    rel_control.read_iolink_hregister(test_address)
    read_register.assert_called_with(address=called_address, count=count)


def test_manifold_actions(rel_control: RelControl):
    manifold = ManifoldActions()
    write_register = writer_register_mock(rel_control)
    hr: HRegister = get_register_by_name(
        rel_control.iolink_hr, HMIWriteAction.ACTION_MANIFOLD.value
    )
    for _, value in manifold.model_dump().items():
        rel_control.apply_manifold_state(value)
        write_register.assert_called_with(
            hr.address,
            value,
        )


def test_control_hyd_valve(rel_control: RelControl):
    digital_valve = DigitalHydValve()
    write_register = writer_register_mock(rel_control)
    hr: HRegister = get_register_by_name(
        rel_control.iolink_hr, DigitalOutput.DIGITAL_OUT_HYD_VALVE.value
    )
    for _, value in digital_valve.model_dump().items():
        rel_control.apply_hyd_valve_state(value)
        write_register.assert_called_with(
            hr.address,
            value,
        )


def test_control_instance(rel_control: RelControl):
    assert isinstance(rel_control.tower_devive, TowerStatusDevice)
    write_registers = writer_registers_mock(rel_control)
    # apply all tower states
    for _, state in enumerate(TowerState):
        state_addresses = getattr(rel_control.tower_devive.tower_status.states, state.value)
        rel_control.apply_tower_state(state)
        if state == TowerState.ACOSTIC_ALARM_ON or state_addresses == TowerState.ACOSTIC_ALARM_OFF:
            write_registers.assert_called_with(
                rel_control.tower_devive.tower_status.alarm_address,
                state_addresses,
            )
        else:
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
