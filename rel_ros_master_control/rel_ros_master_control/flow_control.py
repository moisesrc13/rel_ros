import importlib
from dataclasses import dataclass
from typing import Any, Optional

from hamilton import base, driver, lifecycle, node, telemetry

from rel_ros_master_control.control import RelControl
from rel_ros_master_control.logger import new_logger

logger = new_logger(__name__)

telemetry.disable_telemetry()


class Constants:
    tasks: list[str] = [
        "bucket_distance",
        "sensor_distance_params",
        "bucket_state",
        "check_distance_sensor_for_electrovales",
        "sensor_distance_state",
        "sensor_laser_on",
    ]


@dataclass
class FlowControlInputs:
    hmi_status_publisher: Any
    master_control: RelControl
    control_iolink_data: dict
    control_hmi_data: dict


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


def run(flow_inputs: FlowControlInputs):
    router_module = importlib.import_module("rel_ros_master_control.pipeline")
    default_adapter = base.DefaultAdapter(base.DictResult())
    inputs = {
        "hmi_status_publisher": flow_inputs.hmi_status_publisher,
        "control_hmi_data": flow_inputs.control_hmi_data,
        "control_iolink_data": flow_inputs.control_iolink_data,
        "master_control": flow_inputs.master_control,
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
    try:
        logger.info("running control flow")
        dr.execute(Constants.tasks)
    except Exception as err:
        logger.error("❌ error running flow - %s", err)


if __name__ == "__main__":
    logger.info("running ...")
