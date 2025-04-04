import time
from enum import Enum

from hamilton.function_modifiers import config
from pydantic import BaseModel

from rel_ros_master_control.control import RelControl
from rel_ros_master_control.logger import new_logger
from rel_ros_master_control.models.status_device_m import TowerState

logger = new_logger(__name__)


class SensorDistanceParams(BaseModel):
    vacuum_distance: int  # Z
    bucket_distance: int  # W
    high_pre_vacuum_limit: int  # X
    high_vacuum_limit: int  # Y


class Constants:
    laser_infinit: int = 1000


class SensorDistanceState(Enum):
    """_summary_

    W=distancia de tamaño de cubeta
    X=Límite superior de prevacío
    Y=Límite superior de vacio
    Z=Distancia para vacío
    """

    LT_VACUUM_DISTANCE = "lt_vacuum_distance"  # less than vaccum distance
    GT_VACUUM_DISTANCE_AND_LE_HIGH_VACUUM_LIMIT = "gt_vacuum_distance_and_le_high_vaccum_limit"
    GT_HIGH_VACUUM_LIMIT_AND_LE_HIGH_PRE_VACUUM_LIMIT = (
        "gt_high_vacuum_limit_and_le_high_pre_vacuum_limit"
    )
    GT_HIGH_VACUUM_LIMIT_AND_LT_BUCKET_SIZE_DISTANCE = (
        "gt_high_vacuum_limit_and_lt_bucket_size_distance"
    )
    GT_BUCKET_SIZE_DISTANCE_AND_LE_INFINIT = "gt_bucket_size_distance_and_le_infinit"


class SensorDistanceStateName(Enum):
    A = SensorDistanceState.LT_VACUUM_DISTANCE
    B = SensorDistanceState.GT_VACUUM_DISTANCE_AND_LE_HIGH_VACUUM_LIMIT
    C = SensorDistanceState.GT_HIGH_VACUUM_LIMIT_AND_LE_HIGH_PRE_VACUUM_LIMIT
    D = SensorDistanceState.GT_HIGH_VACUUM_LIMIT_AND_LT_BUCKET_SIZE_DISTANCE
    E = SensorDistanceState.GT_BUCKET_SIZE_DISTANCE_AND_LE_INFINIT


def sensor_distance_state(
    sensor_distance: int, sensor_distance_params: SensorDistanceParams
) -> SensorDistanceState:
    if sensor_distance < sensor_distance_params.vacuum_distance:
        return SensorDistanceStateName.A
    if (
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


@config.when(sensor_distance_state=SensorDistanceStateName.A)
def sensor_laser_on__a(sensor_distance_state: SensorDistanceState, control: RelControl):
    logger.debug("sensor_distance_state -> ", sensor_distance_state)
    control.apply_tower_state(TowerState.VACUUM)
    control.apply_tower_state(TowerState.ACOSTIC_ALARM_ON)
    time.sleep(1)  # TBD
    control.apply_tower_state(TowerState.BUCKET_CHANGE)
    pass


@config.when(sensor_distance_state=SensorDistanceStateName.B)
def sensor_laser_on__b(sensor_distance_state: SensorDistanceState):
    pass


@config.when(sensor_distance_state=SensorDistanceStateName.C)
def sensor_laser_on__c(sensor_distance_state: SensorDistanceState):
    pass


@config.when(sensor_distance_state=SensorDistanceStateName.D)
def sensor_laser_on__d(sensor_distance_state: SensorDistanceState):
    pass


@config.when(sensor_distance_state=SensorDistanceStateName.E)
def sensor_laser_on__e(sensor_distance_state: SensorDistanceState):
    pass
