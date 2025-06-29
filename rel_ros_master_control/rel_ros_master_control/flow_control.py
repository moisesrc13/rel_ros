import importlib
from queue import Queue
from typing import Any, Optional

from hamilton import base, driver, lifecycle, node, telemetry

from rel_ros_master_control.config import load_hmi_config, load_iolink_config
from rel_ros_master_control.constants import (
    Constants,
    FlowStateAction,
    FlowTask,
    SensorDistanceStateName,
)
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
        logger.error("❌ error running flow tasks: %s - %s", flow_task, err)
        raise err


def run_control(control: RelControl, flow_task: FlowTask, queue: Queue = None, debug=False):
    inputs = {"control": control}
    while True:
        try:
            logger.info("💡 running flow: %s \n inputs: \n %s", flow_task.name, inputs)
            if debug:
                input("😴 continue? ...")
            inputs = run_flow(inputs, flow_task)
            inputs["control"] = control
            final_output_name = flow_task.tasks[-1]
            final_value = inputs[final_output_name]
            logger.info("📤 latest flow output: %s", final_value)
            if isinstance(final_value, SensorDistanceStateName):
                flow_task = Constants.flow_tasks_init_state
            elif isinstance(final_value, FlowStateAction):
                match final_value:
                    case FlowStateAction.TO_RECYCLE_PROCESS:
                        flow_task = Constants.flow_tasks_recycle
                    case FlowStateAction.TO_PWM:
                        flow_task = Constants.flow_tasks_pwm
                    case FlowStateAction.PRESSURE_NOT_ON_TARGET_BARES:
                        flow_task = Constants.flow_tasks_pwm
                    case FlowStateAction.WAITING_FOR_BUCKET:
                        flow_task = Constants.flow_tasks_bucket_change
                    case FlowStateAction.COMPLETE:
                        logger.info("🤘 🎮 completing flow with state DONE")
                        flow_task = Constants.flow_calculate_distance_sensor_case
                        inputs = {"control": control}
                    case _:
                        logger.warning("❓ completing flow with state %s", final_value)
                        flow_task = Constants.flow_calculate_distance_sensor_case
                        inputs = {"control": control}
        except Exception as err:
            logger.error("❌ error running flow - %s", err)
            if queue is not None and (item := queue.get()):
                logger.warning("mark %s as done", item)
                queue.task_done()


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
    run_flow(control, Constants.flow_tasks_init_state)
