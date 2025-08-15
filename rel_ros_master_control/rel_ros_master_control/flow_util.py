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
