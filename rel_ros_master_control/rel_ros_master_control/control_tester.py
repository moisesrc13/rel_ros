import argparse
from enum import Enum

from rel_ros_master_control.config import load_hmi_config, load_iolink_config
from rel_ros_master_control.constants import Params, Sensors
from rel_ros_master_control.control import RelControl
from rel_ros_master_control.logger import new_logger

logger = new_logger(__name__)


class TestCase(Enum):
    A = "a"  # A) Sensor laser d<Z
    B = "b"  # B) Sensor laser d>Z && d<=Y
    C = "c"  # C) Sensor laser d>Y && d<=X
    D = "d"  # D) Sensor laser d>X && d<W
    E = "e"  # E) Sensor laser d>W && d<= ∞


if __name__ == "__main__":
    logger.info("starting control tester for node id 0 🤠")
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t",
        "--testcase",
        help="test case",
        dest="testcase",
        default="a",
        type=str,
    )
    args = parser.parse_args()
    test_case = TestCase(args.testcase)
    logger.info("running test case %s", test_case)
    iolink_config = load_iolink_config()
    hmi_config = load_hmi_config()
    control = RelControl(
        iolink_slave=iolink_config.iolinks[0],
        iolink_hr=iolink_config.holding_registers,
        hmi_slave=hmi_config.hmis[0],
        hmi_hr=hmi_config.holding_registers,
        hmi_cr=hmi_config.coil_registers,
    )
    match test_case:
        case TestCase.A:
            control.write_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE, 90)
            control.write_hmi_hregister_by_name(Params.PARAM_VACUUM_DISTANCE, 100)

    logger.info("done ...")
