import importlib
from dataclasses import dataclass
from typing import Any, Optional

from hamilton import base, driver, lifecycle, node, telemetry

from rel_ros_master_control.config import load_modbus_config
from rel_ros_master_control.constants import Constants
from rel_ros_master_control.control import RelControl
from rel_ros_master_control.logger import new_logger

logger = new_logger(__name__)

telemetry.disable_telemetry()


@dataclass
class FlowControlInputs:
    hmi_action_publisher: Any
    control: RelControl


class LoggingPostNodeExecute(lifecycle.api.BasePostNodeExecute):
    def __init__(self) -> None:
        super().__init__()
        self.task_count = 0
        self.completed_tasks = []

    def post_node_execute(
        self,
        *,
        run_id: str,
        node_: "node.Node",
        kwargs: dict[str, Any],
        success: bool,
        error: Optional[Exception],
        result: Optional[Any],
        task_id: Optional[str] = None,
    ):
        if not success:
            logger.error("❌ error %s, running task %s", error, node_._name)
        else:
            logger.info("✅ task %s completed ✔️", node_._name)


class LoggingPreNodeExecute(lifecycle.api.BasePreNodeExecute):
    def __init__(self) -> None:
        super().__init__()

    def pre_node_execute(
        self,
        *,
        run_id: str,
        node_: "node.Node",
        kwargs: dict[str, Any],
        task_id: Optional[str] = None,
    ):
        logger.info("🚀 running 📋 %s", node_._name)


def run(flow_inputs: FlowControlInputs, visualize: bool = False):
    router_module = importlib.import_module("rel_ros_master_control.pipeline")
    default_adapter = base.DefaultAdapter(base.DictResult())
    inputs = {
        "hmi_action_publisher": flow_inputs.hmi_action_publisher,
        "control": flow_inputs.control,
    }
    dr = (
        driver.Builder()
        .with_modules(router_module)
        .with_config(inputs)
        .with_adapters(
            default_adapter,
            LoggingPreNodeExecute(),
            LoggingPostNodeExecute(),
        )
        .build()
    )
    if visualize:
        dr.display_all_functions()
        return
    try:
        logger.info("running control flow")
        dr.execute(Constants.flow_tasks)
    except Exception as err:
        logger.error("❌ error running flow - %s", err)


if __name__ == "__main__":
    """this method is for manual testing"""

    class TestPubliser:
        def publish(self, msg: str):
            print(f"publish {msg}")

    config = load_modbus_config()
    control = RelControl(iolink_slave=config.iolinks[0], iolink_hr=config.holding_registers)
    logger.info("visualize ...")
    flow_inputs = FlowControlInputs(
        control_hmi_data={
            "param_vacuum_limit_high": 100,
            "param_pre_vacuum_limit_high": 80,
            "param_vacuum_distance": 10,
        },
        control_iolink_data={
            "sensor_laser_distance": 100,
        },
        hmi_action_publisher=TestPubliser(),
        control=control,
    )
    run(flow_inputs=flow_inputs, visualize=True)
