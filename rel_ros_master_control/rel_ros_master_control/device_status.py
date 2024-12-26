from enum import Enum

from rel_ros_master_control.util import is_bit_on


class StatusRegister(Enum):
    CONNECTED = 0
    DATA_VALID = 1
    EVENT_PENDING = 2
    ISDU_READY = 3


class DeviceStatus:
    def __init__(self) -> None:
        pass

    @staticmethod
    def connected(reg_value: int) -> bool:
        return is_bit_on(reg_value, StatusRegister.CONNECTED)

    @staticmethod
    def valid(reg_value: int) -> bool:
        return is_bit_on(reg_value, StatusRegister.DATA_VALID)

    @staticmethod
    def event_pending(reg_value: int) -> bool:
        return is_bit_on(reg_value, StatusRegister.EVENT_PENDING)

    @staticmethod
    def ready(reg_value: int) -> bool:
        return is_bit_on(reg_value, StatusRegister.ISDU_READY)
