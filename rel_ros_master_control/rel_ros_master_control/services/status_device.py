from rel_ros_master_control.config import load_status_device_config
from rel_ros_master_control.models.status_device_m import (
    BasicActions,
    StatusDeviceConfig,
    StatusSegment,
)
from rel_ros_master_control.util import set_bit


def parse_bits(action: BasicActions, bits: list[int]) -> dict:
    if action == BasicActions.AUDIBLE or action == BasicActions.ANIMATION:
        return {bits[0]: 1, bits[1]: 1}
    elif action == BasicActions.TURN_OFF:
        return {bits[0]: 0, bits[1]: 0}
    elif action == BasicActions.TURN_ON:
        return {bits[0]: 0, bits[1]: 1}
    elif action == BasicActions.FLASH:
        return {bits[0]: 1, bits[1]: 0}


class StatusDeviceService:
    def __init__(self, data_output: int):
        config: StatusDeviceConfig = load_status_device_config()
        self.basic = config.basic
        self.data_output = data_output  # data output start address number

    def _get_address_for_segment(self, segment_name: str) -> int:
        segment: StatusSegment = getattr(self.basic.segments, segment_name)
        if segment.octect in [0, 1]:  # start register address
            return self.data_output
        if segment.octect in [2, 3]:
            return self.data_output + 1
        if segment.octect in [4, 5]:
            return self.data_output + 2

    def _get_updated_value(self, segment_name: str, action: BasicActions, current_value: int):
        """
        need to call this function with the current value of the given segment in order to update it
        """
        segment: StatusSegment = getattr(self.basic.segments, segment_name)
        parsed_bits = parse_bits(action, segment.bits)
        for bit, bit_value in parsed_bits.items():
            current_value = set_bit(current_value, bit, bit_value)
        return current_value
