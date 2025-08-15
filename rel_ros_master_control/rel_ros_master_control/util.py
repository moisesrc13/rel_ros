import importlib
from typing import Any, Optional

from hamilton import base, driver, lifecycle, node, telemetry

from rel_ros_master_control.constants import FlowTask
from rel_ros_master_control.logger import new_logger

logger = new_logger(__name__)

telemetry.disable_telemetry()


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
            logger.info("✅ task %s completed ✨", node_._name)


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


def run_flow(inputs: dict, flow_task: FlowTask) -> dict:
    router_module = importlib.import_module("rel_ros_master_control.pipeline")
    default_adapter = base.DefaultAdapter(base.DictResult())
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
        logger.info("✨ running control flow with tasks %s", flow_task.tasks)
        r = dr.execute(flow_task.tasks)
        outputs = {key: value for key, value in r.items() if key in flow_task.outputs}
        logger.info("🗳 ==> next inputs %s", outputs)
        return outputs
    except Exception as err:
        logger.error("❌ error running flow tasks: %s - %s", flow_task.tasks, err)
        raise err


def is_bit_on(num: int, bit_position: int) -> bool:
    """
    Checks if the bit at the specified position is set (1) or not (0).

    Args:
      num: The integer to check.
      bit_position: The position of the bit to check (starting from 0 for the least significant bit).

    Returns:
      True if the bit is set, False otherwise.
    """

    mask = 1 << bit_position
    return (num & mask) != 0


def set_bit(value, bit, bit_value):
    """
    Sets the bit at the specified position in the given integer value.

    Args:
      value: The integer value.
      bit: The position of the bit to set (0-indexed).
      bit_value: The value to set the bit to (0 or 1).

    Returns:
      The integer value with the bit at the specified position set accordingly.
    """

    # Create a mask with the bit at the specified position set to 1
    mask = 1 << bit

    # Clear the bit at the specified position
    value &= ~mask

    # Set the bit at the specified position if 'bit' is 1
    if bit_value:
        value |= mask

    return value
