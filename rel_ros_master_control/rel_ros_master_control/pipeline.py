import time
from enum import Enum
from typing import Any

from hamilton.function_modifiers import config
from pydantic import BaseModel

from rel_interfaces.msg import HMIStatus
from rel_ros_master_control.constants import HMIWriteAction
from rel_ros_master_control.control import RelControl
from rel_ros_master_control.logger import new_logger
from rel_ros_master_control.models.status_device_m import TowerState

logger = new_logger(__name__)


class SensorDistanceParams(BaseModel):
    vacuum_distance: int  # Z
    bucket_distance: int  # W
    high_pre_vacuum_limit: int  # X
    high_vacuum_limit: int  # Y


class SensorDistanceState(Enum):
    """_summary_

    W = distancia de tamaño de cubeta
    X = Límite superior de prevacío
    Y = Límite superior de vacio
    Z = Distancia para vacío
    """

    LT_VACUUM_DISTANCE = "lt_vacuum_distance"  # less than vaccum distance
    GT_VACUUM_DISTANCE_AND_LE_HIGH_VACUUM_LIMIT = "gt_vacuum_distance_and_le_high_vaccum_limit"
    GT_HIGH_VACUUM_LIMIT_AND_LE_HIGH_PRE_VACUUM_LIMIT = (
        "gt_high_vacuum_limit_and_le_high_pre_vacuum_limit"
    )
    GT_HIGH_VACUUM_LIMIT_AND_LT_BUCKET_SIZE_DISTANCE = (
        "gt_high_vacuum_limit_and_lt_bucket_size_distance"
    )
    GT_BUCKET_SIZE_DISTANCE_AND_LE_INFINITY = "gt_bucket_size_distance_and_le_infinity"


class SensorDistanceStateName(Enum):
    A = SensorDistanceState.LT_VACUUM_DISTANCE
    B = SensorDistanceState.GT_VACUUM_DISTANCE_AND_LE_HIGH_VACUUM_LIMIT
    C = SensorDistanceState.GT_HIGH_VACUUM_LIMIT_AND_LE_HIGH_PRE_VACUUM_LIMIT
    D = SensorDistanceState.GT_HIGH_VACUUM_LIMIT_AND_LT_BUCKET_SIZE_DISTANCE
    E = SensorDistanceState.GT_BUCKET_SIZE_DISTANCE_AND_LE_INFINITY


def bucket_distance(param_bucket_size_selection: int, control_hmi_data: dict) -> int:
    distance = control_hmi_data.get("param_distance_bucket_1")  # default
    match param_bucket_size_selection:
        case 1:
            return distance
        case 2:
            distance = control_hmi_data.get("param_distance_bucket_2")
        case 3:
            distance = control_hmi_data.get("param_distance_bucket_3")
    return distance


def sensor_distance_params(bucket_distance: int, control_hmi_data: dict) -> SensorDistanceParams:
    return SensorDistanceParams(
        bucket_distance=bucket_distance,
        high_pre_vacuum_limit=control_hmi_data.get("param_pre_vacuum_limit_high"),
        high_vacuum_limit=control_hmi_data.get("param_vacuum_limit_high"),
        vacuum_distance=control_hmi_data.get("param_vacuum_distance"),
    )


def sensor_distance_state(
    control_iolink_data: dict, control_hmi_data: dict, sensor_distance_params: SensorDistanceParams
) -> SensorDistanceState:
    sensor_distance = control_iolink_data.get("sensor_laser_distance")
    if sensor_distance < control_hmi_data.get("param_vacuum_distance"):
        return SensorDistanceStateName.A
    elif (
        sensor_distance > sensor_distance_params.vacuum_distance
        and sensor_distance <= sensor_distance_params.high_vacuum_limit
    ):
        return SensorDistanceStateName.B
    if (
        sensor_distance > sensor_distance_params.high_vacuum_limit
        and sensor_distance <= sensor_distance_params.high_pre_vacuum_limit
    ):
        return SensorDistanceStateName.C
    if (
        sensor_distance > sensor_distance_params.high_vacuum_limit
        and sensor_distance < sensor_distance_params.bucket_distance
    ):
        return SensorDistanceStateName.D
    # return default for now
    return SensorDistanceStateName.E


def bucket_state(bucket_distance: int) -> TowerState:
    if bucket_distance >= 80 and TowerState <= 100:
        return TowerState.FULL
    if bucket_distance >= 50 and TowerState <= 79:
        return TowerState.MEDIUM_HIGH
    if bucket_distance >= 10 and TowerState <= 20:
        return TowerState.PRE_VACUUM
    return TowerState.BUCKET_CHANGE


def check_distance_sensor_for_electrovales(
    control: RelControl, control_iolink_data: dict, control_hmi_data: dict
):
    sensor_distance = control_iolink_data.get("sensor_laser_distance")
    if sensor_distance < control_hmi_data.get("param_vacuum_distance"):
        control.eletrovalve_off()
        control.apply_tower_state(TowerState.VACUUM)
        control.apply_tower_state(TowerState.ACOSTIC_ALARM_ON)


@config.when(sensor_distance_state=SensorDistanceStateName.A)
def sensor_laser_on__a(sensor_distance_state: SensorDistanceState, control: RelControl):
    control.apply_tower_state(TowerState.VACUUM)
    control.apply_tower_state(TowerState.ACOSTIC_ALARM_ON)
    control.apply_tower_state(TowerState.BUCKET_CHANGE)


@config.when(sensor_distance_state=SensorDistanceStateName.B)
def sensor_laser_on__b(
    hmi_action_publisher: Any,
    control: RelControl,
    sensor_distance_state: SensorDistanceState,
    bucket_state: TowerState,
):
    control.apply_tower_state(bucket_state)
    msg = HMIStatus()
    msg.hmi_id = control.hmi_id
    msg.action_value = 1
    if bucket_state == TowerState.PRE_VACUUM:
        msg.action_name = HMIWriteAction.STATUS_ALARM_PRE_VACUUM.value
    elif bucket_state == TowerState.VACUUM:
        msg.action_name = HMIWriteAction.STATUS_ALARM.value
    hmi_action_publisher.publish(msg)
    msg = HMIStatus()
    msg.action_name = HMIWriteAction.ACTION_PULL_DOWN_PISTONS_BUCKET.value
    hmi_action_publisher.publish(msg)


@config.when(sensor_distance_state=SensorDistanceStateName.C)
def sensor_laser_on__c(hmi_action_publisher: Any, control: RelControl, bucket_state: TowerState):
    control.apply_tower_state(bucket_state)
    msg = HMIStatus()
    msg.hmi_id = control.hmi_id
    msg.action_name = HMIWriteAction.STATUS_ALARM_PRE_VACUUM.value
    msg.action_value = 1
    hmi_action_publisher.publish(msg)


@config.when(sensor_distance_state=SensorDistanceStateName.D)
def sensor_laser_on__d(sensor_distance_state: SensorDistanceState):
    pass


@config.when(sensor_distance_state=SensorDistanceStateName.E)
def sensor_laser_on__e(sensor_distance_state: SensorDistanceState):
    pass
