from rel_ros_master_control.modbus_master import RelModbusMaster
from rel_ros_master_control.models.modbus_m import ModbusConfig, SlaveTCP


def is_bit_on(num: int, bit_position: int) -> bool:
    """
    Checks if the bit at the specified position is set (1) or not (0).

    Args:
      num: The integer to check.
      bit_position: The position of the bit to check (starting from 0 for the least significant bit).

    Returns:
      True if the bit is set, False otherwise.
    """

    mask = 1 << bit_position
    return (num & mask) != 0


def set_bit(value, bit, bit_value):
    """
    Sets the bit at the specified position in the given integer value.

    Args:
      value: The integer value.
      bit: The position of the bit to set (0-indexed).
      bit_value: The value to set the bit to (0 or 1).

    Returns:
      The integer value with the bit at the specified position set accordingly.
    """

    # Create a mask with the bit at the specified position set to 1
    mask = 1 << bit

    # Clear the bit at the specified position
    value &= ~mask

    # Set the bit at the specified position if 'bit' is 1
    if bit_value:
        value |= mask

    return value
