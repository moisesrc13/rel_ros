import importlib
from typing import Any, Optional

from hamilton import base, driver, lifecycle, node, telemetry

from rel_ros_master_control.config import load_hmi_config, load_iolink_config
from rel_ros_master_control.constants import Constants, FlowStateAction
from rel_ros_master_control.control import RelControl
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


def run(control: RelControl, tasks: list[str], node_to_validate: str = "init_flow_state"):
    router_module = importlib.import_module("rel_ros_master_control.pipeline")
    default_adapter = base.DefaultAdapter(base.DictResult())
    inputs = {
        "control": control,
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
    init_flow_state = FlowStateAction.UNKNOWN
    try:
        logger.info("✨ running control flow with tasks %s", tasks)
        r = dr.execute(tasks)
        init_flow_state = FlowStateAction(int(r[node_to_validate].iloc[-1]))
    except Exception as err:
        logger.error("❌ error running flow - %s", err)

    match init_flow_state:
        case FlowStateAction.TO_RECYCLE_PROCESS:
            run(control, Constants.flow_tasks_recycle, "validate_recycle")
        case _:
            logger.info("completing flow ...")


if __name__ == "__main__":
    #  this method is for manual testing
    iolink_config = load_iolink_config()
    hmi_config = load_hmi_config()
    control = RelControl(
        iolink_slave=iolink_config.iolinks[0],
        iolink_hr=iolink_config.holding_registers,
        hmi_slave=hmi_config.hmis[0],
        hmi_hr=hmi_config.holding_registers,
        hmi_cr=hmi_config.coil_registers,
    )
    logger.info("visualize ...")
    run(control, Constants.flow_tasks_init_state)
