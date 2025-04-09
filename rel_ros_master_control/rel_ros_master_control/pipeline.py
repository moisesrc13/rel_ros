import time

from hamilton.function_modifiers import config

from rel_ros_master_control.constants import (
    Constants,
    HMIWriteAction,
    ManifoldActions,
    Params,
    SensorDistanceParams,
    SensorDistanceState,
    SensorDistanceStateName,
    SensorLaserLectureState,
    Sensors,
)
from rel_ros_master_control.control import RegisterType, RelControl, SlaveType
from rel_ros_master_control.logger import new_logger
from rel_ros_master_control.models.status_device_m import TowerState

logger = new_logger(__name__)


def wait_for_sensor_laser():
    time.sleep(Constants.wait_for_sensor_laser_ms / 1000)


def hmi_hr_data(control: RelControl) -> dict:
    return control.get_hmi_hr_data()


def hmi_cr_data(control: RelControl) -> dict:
    return control.get_hmi_cr_data()


def iolink_hr_data(control: RelControl) -> dict:
    return control.get_iolink_hr_data()


def bucket_distance(param_bucket_size_selection: int, hmi_hr_data: dict) -> int:
    distance = hmi_hr_data.get(Params.PARAM_DISTANCE_BUCKET_1.value)  # default
    match param_bucket_size_selection:
        case 1:
            return distance
        case 2:
            distance = hmi_hr_data.get(Params.PARAM_DISTANCE_BUCKET_2.value)
        case 3:
            distance = hmi_hr_data.get(Params.PARAM_DISTANCE_BUCKET_3.value)
    return distance


def sensor_distance_params(bucket_distance: int, hmi_hr_data: dict) -> SensorDistanceParams:
    return SensorDistanceParams(
        bucket_distance=bucket_distance,
        high_pre_vacuum_limit=hmi_hr_data.get(Params.PARAM_PRE_VACUUM_LIMIT_HIGH.value),
        high_vacuum_limit=hmi_hr_data.get(Params.PARAM_VACUUM_LIMIT_HIGH.value),
        vacuum_distance=hmi_hr_data.get(Params.PARAM_VACUUM_DISTANCE.value),
    )


def sensor_distance_state(
    control_iolink_data: dict, hmi_hr_data: dict, sensor_distance_params: SensorDistanceParams
) -> SensorDistanceState:
    sensor_distance = control_iolink_data.get(Sensors.SENSOR_LASER_DISTANCE.value)
    if sensor_distance < hmi_hr_data.get(sensor_distance_params.vacuum_distance):
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


def bucket_state(bucket_distance: int, control: RelControl) -> TowerState:
    state = TowerState.BUCKET_CHANGE
    if bucket_distance >= 80 and TowerState <= 100:
        state = TowerState.FULL
    elif bucket_distance >= 50 and TowerState <= 79:
        state = TowerState.MEDIUM_HIGH
    elif bucket_distance >= 30 and TowerState <= 49:
        state = TowerState.MEDIUM_HIGH
    elif bucket_distance >= 10 and TowerState <= 20:
        state = TowerState.PRE_VACUUM
        control.apply_tower_state(TowerState.ACOSTIC_ALARM_ON)
    elif bucket_distance >= 2 and TowerState <= 5:
        control.apply_tower_state(TowerState.ACOSTIC_ALARM_ON)
        state = TowerState.VACUUM
    control.apply_tower_state(state)
    return state


def check_distance_sensor_for_electrovales(
    control: RelControl, iolink_hr_data: dict, hmi_hr_data: dict
):
    sensor_distance = iolink_hr_data.get(Sensors.SENSOR_LASER_DISTANCE.value)
    if sensor_distance < hmi_hr_data.get("param_vacuum_distance"):
        control.eletrovalve_off()
        control.apply_tower_state(TowerState.VACUUM)
        control.apply_tower_state(TowerState.ACOSTIC_ALARM_ON)


def not_holded_sensor_on_a(control: RelControl):
    control.apply_tower_state(TowerState.VACUUM)
    control.apply_tower_state(TowerState.ACOSTIC_ALARM_ON)
    control.apply_tower_state(TowerState.BUCKET_CHANGE)


@config.when(sensor_distance_state=SensorDistanceStateName.A)
def sensor_laser_on__a(sensor_distance_state: SensorDistanceState, control: RelControl):
    not_holded_sensor_on_a(control)
    return SensorLaserLectureState.WAITING_FOR_BUCKET


def not_holded_sensor_on_b(control: RelControl):
    control.write_register_by_address_name(
        name=HMIWriteAction.STATUS_VACUUM_ALARM.value,
        value=1,
        stype=SlaveType.HMI,
        rtype=RegisterType.COIL,
    )
    control.write_register_by_address_name(
        name=HMIWriteAction.ACTION_PULL_DOWN_PISTONS_BUCKET.value,
        value=1,
        stype=SlaveType.HMI,
        rtype=RegisterType.COIL,
    )
    control.apply_manifold_state(ManifoldActions.PISTONS_DOWN)


@config.when(sensor_distance_state=SensorDistanceStateName.B)
def sensor_laser_on__b(
    control: RelControl,
    sensor_distance_state: SensorDistanceState,
    bucket_state: TowerState,
    iolink_hr_data: dict,
    sensor_distance_params: SensorDistanceParams,
) -> SensorLaserLectureState:
    not_holded_sensor_on_b(control)
    wait_for_sensor_laser()
    sensor_distance = control.get_iolink_data_by_hr_name(Sensors.SENSOR_LASER_DISTANCE.value)
    if sensor_distance == iolink_hr_data.get(Sensors.SENSOR_LASER_DISTANCE.value):
        return SensorLaserLectureState.EMPTY_BUCKET
    while sensor_distance > sensor_distance_params.vacuum_distance:
        time.sleep(Constants.wait_read_laser)  # TBD
        sensor_distance = control.get_iolink_data_by_hr_name(Sensors.SENSOR_LASER_DISTANCE.value)

    control.write_register_by_address_name(
        name=HMIWriteAction.ACTION_PULL_DOWN_PISTONS_BUCKET.value,
        value=0,
        stype=SlaveType.HMI,
        rtype=RegisterType.COIL,
    )
    control.apply_manifold_state(ManifoldActions.DEACTIVATE)
    return SensorLaserLectureState.NOT_HOLD_TO_A


def not_holded_sensor_on_c(control: RelControl):
    control.apply_tower_state(bucket_state)
    control.write_register_by_address_name(
        name=HMIWriteAction.STATUS_ALARM_PRE_VACUUM.value,
        value=1,
        stype=SlaveType.HMI,
        rtype=RegisterType.COIL,
    )
    control.write_register_by_address_name(
        name=HMIWriteAction.ACTION_PULL_DOWN_PISTONS_BUCKET.value,
        value=1,
        stype=SlaveType.HMI,
        rtype=RegisterType.COIL,
    )
    control.apply_manifold_state(ManifoldActions.PISTONS_DOWN)


@config.when(sensor_distance_state=SensorDistanceStateName.C)
def sensor_laser_on__c(
    control: RelControl,
    bucket_state: TowerState,
    iolink_hr_data: dict,
) -> SensorLaserLectureState:
    not_holded_sensor_on_c(control)
    wait_for_sensor_laser()
    sensor_distance = control.get_iolink_data_by_hr_name(Sensors.SENSOR_LASER_DISTANCE.value)
    if sensor_distance == iolink_hr_data.get(Sensors.SENSOR_LASER_DISTANCE.value):
        control.write_register_by_address_name(
            name=HMIWriteAction.ACTION_TURN_ON_PUMPING_PROCESS.value,
            value=1,
            stype=SlaveType.HMI,
            rtype=RegisterType.COIL,
        )
        return SensorLaserLectureState.PREVACUUM_BUCKET_ON
    return SensorLaserLectureState.NOT_HOLD_TO_B


@config.when(sensor_distance_state=SensorDistanceStateName.D)
def sensor_laser_on__d(
    control: RelControl, sensor_distance_state: SensorDistanceState, iolink_hr_data: dict
) -> SensorLaserLectureState:
    control.apply_manifold_state(ManifoldActions.PISTONS_DOWN)
    wait_for_sensor_laser()
    sensor_distance = control.get_iolink_data_by_hr_name(Sensors.SENSOR_LASER_DISTANCE.value)
    if sensor_distance == iolink_hr_data.get(Sensors.SENSOR_LASER_DISTANCE.value):
        return SensorLaserLectureState.BUCKET_ON
    return SensorLaserLectureState.NOT_HOLD_TO_C


@config.when(sensor_distance_state=SensorDistanceStateName.E)
def sensor_laser_on__e(
    control: RelControl, sensor_distance_state: SensorDistanceState, hmi_hr_data: dict
) -> SensorLaserLectureState:
    control.apply_manifold_state(ManifoldActions.PISTONS_UP)
    sensor_distance = control.get_iolink_data_by_hr_name(Sensors.SENSOR_LASER_DISTANCE.value)
    security_distance = hmi_hr_data.get(Params.PARAM_SECURITY_DISTANCE.value)
    while sensor_distance < security_distance:
        time.sleep(Constants.wait_read_laser)
        sensor_distance = control.get_iolink_data_by_hr_name(Sensors.SENSOR_LASER_DISTANCE.value)

    control.apply_manifold_state(ManifoldActions.DEACTIVATE)
    control.apply_tower_state(TowerState.BUCKET_CHANGE)
    return SensorLaserLectureState.WAITING_FOR_BUCKET


def redirect_from_sensor_laser_state(
    control: RelControl, sensor_laser_on: SensorLaserLectureState
) -> SensorLaserLectureState:
    next_state = SensorLaserLectureState.DO_NOTHING
    match sensor_laser_on:
        case SensorLaserLectureState.HOLD:
            next_state = SensorLaserLectureState.HOLD  # do nothing
        case SensorLaserLectureState.NOT_HOLD_TO_A:
            not_holded_sensor_on_a(control)
            next_state = SensorLaserLectureState.DO_NOTHING
        case SensorLaserLectureState.NOT_HOLD_TO_B:
            not_holded_sensor_on_b(control)
            next_state = SensorLaserLectureState.DO_NOTHING
        case SensorLaserLectureState.NOT_HOLD_TO_C:
            not_holded_sensor_on_c(control)
            next_state = SensorLaserLectureState.DO_NOTHING
        case _:
            next_state = sensor_laser_on
    return next_state


@config.when(redirect_from_sensor_laser_state=SensorLaserLectureState.EMPTY_BUCKET)
def bucket_state_action__empty(
    control: RelControl, redirect_from_sensor_laser_state: SensorDistanceState
):
    control.apply_tower_state(TowerState.BUCKET_CHANGE)


@config.when(redirect_from_sensor_laser_state=SensorLaserLectureState.PREVACUUM_BUCKET_ON)
def bucket_state_action__prevacuum(
    control: RelControl, redirect_from_sensor_laser_state: SensorDistanceState
):
    control.write_register_by_address_name(
        name=HMIWriteAction.ACTION_TURN_ON_PUMPING_PROCESS.value,
        value=1,
        stype=SlaveType.HMI,
        rtype=RegisterType.COIL,
    )


@config.when(redirect_from_sensor_laser_state=SensorLaserLectureState.WAITING_FOR_BUCKET)
def bucket_state_action__setbucket(redirect_from_sensor_laser_state: SensorDistanceState):
    pass


@config.when_in(
    redirect_from_sensor_laser_state=[
        SensorLaserLectureState.BUCKET_ON,
        SensorLaserLectureState.PREVACUUM_BUCKET_ON,
    ]
)
def bucket_state_action__bucketon(redirect_from_sensor_laser_state: SensorDistanceState):
    pass
