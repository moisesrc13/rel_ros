from rel_ros_hmi.models.modbus_m import HRegister


def test_dummy():
    register = HRegister(name="test", address=40001, value=10)
    assert register.value == 10
