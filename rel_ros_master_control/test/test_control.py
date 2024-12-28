import pytest

from rel_ros_master_control.control import get_builder, get_decoder_from_rr, get_value
from rel_ros_master_control.models.modbus_m import Register, RegisterDataType
from rel_ros_master_control.util import is_bit_on


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
            Register(
                address=100,
                data_type=RegisterDataType.uint32,
                words=1,
            ),
        ),
        (
            123444,
            Register(
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
            Register(
                address=100,
                data_type=RegisterDataType.float32,
                words=1,
            ),
        ),
        (
            12.3444,
            Register(
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
