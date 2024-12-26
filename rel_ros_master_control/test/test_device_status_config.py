from rel_ros_master_control.config import load_status_device_config
from rel_ros_master_control.models.status_device_m import StatusDeviceConfig, StatusDevice


def test_config():
    config = load_status_device_config()
    assert isinstance(config, StatusDeviceConfig)
    assert isinstance(config.basic, StatusDevice)
    assert config.basic.segments.audible is not None
