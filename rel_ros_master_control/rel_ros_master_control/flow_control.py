from queue import Queue

from rel_ros_master_control.config import load_hmi_config, load_iolink_config
from rel_ros_master_control.constants import (
    Constants,
    FlowStateAction,
    FlowTask,
    ManualTasks,
    SensorDistanceStateName,
)
from rel_ros_master_control.control import RelControl
from rel_ros_master_control.flow_util import run_flow
from rel_ros_master_control.logger import new_logger

logger = new_logger(__name__)


def run_control(control: RelControl, flow_task: FlowTask, queue: Queue = None, debug=False):
    inputs = {"control": control}
    while True:
        try:
            if control.control_state.is_manual:
                continue
            logger.info("💡 running flow: %s \n inputs: \n %s", flow_task.name, inputs)
            if debug:
                input("😴 debug & continue? ...")
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
                    case FlowStateAction.BUCKET_CHANGE_OVER_W:
                        flow_task = Constants.flow_tasks_bucket_change_frame
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
