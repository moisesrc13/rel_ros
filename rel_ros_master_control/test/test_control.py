from unittest.mock import MagicMock

import pytest

from rel_ros_master_control.control import (
    RelControl,
    RelModbusMaster,
    SlaveTCP,
    get_builder,
    get_decoder_from_rr,
    get_value,
)
from rel_ros_master_control.models.modbus_m import HRegister, RegisterDataType
from rel_ros_master_control.models.status_device_m import TowerState, TowerStatusDevice
from rel_ros_master_control.util import is_bit_on


@pytest.fixture
def slave_tcp():
    return SlaveTCP(
        host="0.0.0.0",
        port=9090,
    )


@pytest.fixture
def rel_control(monkeypatch):
    hr = [HRegister(address=100, value=0)]
    slave_tcp = SlaveTCP(
        host="0.0.0.0",
        port=9090,
    )
    monkeypatch.setattr(RelModbusMaster, "do_connect", MagicMock(return_value=None))
    return RelControl(slave=slave_tcp, hr=hr)


def test_control_hyd_valve(rel_control):
    slave_conn_mock = MagicMock()
    write_registers = MagicMock(return_value=True)
    slave_conn_mock.write_registers = write_registers
    rel_control.master_io_link.slave_conn = slave_conn_mock
    assert 1 == 1


def test_control_instance(monkeypatch, slave_tcp: SlaveTCP):
    hr = [HRegister(address=100, value=0)]
    monkeypatch.setattr(RelModbusMaster, "do_connect", MagicMock(return_value=None))
    rel_control = rel_control = RelControl(slave=slave_tcp, hr=hr)
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
