import pytest

from rel_ros_master_control.models.status_device_m import TowerStatusDevice


@pytest.fixture
def TowerDevice():
    return TowerStatusDevice()


def test_status_device(TowerDevice):
    assert TowerDevice.status_start_address == 6060
    assert TowerDevice.status_total_registers == 8
