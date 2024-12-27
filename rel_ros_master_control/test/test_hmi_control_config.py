from rel_ros_master_control.config import load_hmi_control_config
from rel_ros_master_control.models.hmi_m import HMI, HMIConfig


def test_config():
    config = load_hmi_control_config()
    assert isinstance(config, HMIConfig)
    assert isinstance(config.hmi, HMI)
