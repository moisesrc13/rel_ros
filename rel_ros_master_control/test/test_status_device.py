import pytest

from rel_ros_master_control.models.status_device_m import BasicActions
from rel_ros_master_control.services.status_device import StatusDeviceService

service = StatusDeviceService(5001)


@pytest.mark.parametrize(
    "segment, current_value, action, expected_number",
    [
        ("segment_10", 0, BasicActions.TURN_ON, 4),
        ("segment_10", 0, BasicActions.ANIMATION, 12),
        ("segment_9", 0, BasicActions.FLASH, 2),
    ],
)
def test_set_bit(segment, current_value, action, expected_number):
    updated_value = service._get_updated_value(segment, action, current_value)
    assert expected_number == updated_value
