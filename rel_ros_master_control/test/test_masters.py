import pytest

from rel_ros_master_control.config import load_modbus_config
from rel_ros_master_control.modbus_master import RelModbusMaster
from rel_ros_master_control.util import get_master


@pytest.fixture
def test_config():
    return load_modbus_config()


def test_masters(test_config):
    masters = get_master(test_config.slaves, "master_io_link")
    assert isinstance(masters, RelModbusMaster)
