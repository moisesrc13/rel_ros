import pytest

from rel_ros_master_control.config import load_status_device_config
from rel_ros_master_control.models.status_device_m import TowerStatusDevice


@pytest.fixture
def TowerDevice():
    config = load_status_device_config()
    return TowerStatusDevice(config)


def test_status_device(TowerDevice: TowerStatusDevice):
    assert TowerDevice.tower_status.total_registers == 8
    assert TowerDevice.tower_status.start_address == 6060
