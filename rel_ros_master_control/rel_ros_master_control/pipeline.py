import time
from timeit import default_timer as timer

from hamilton.function_modifiers import config

from rel_ros_master_control.constants import (
    CoilState,
    Constants,
    FlowStateAction,
    HMIWriteAction,
    ManifoldActions,
    Params,
    PressureState,
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


def bucket_distance(control: RelControl) -> int:
    bucket_size_selection = control.read_hmi_hregister_by_name(Params.PARAM_BUCKET_SIZE_SELECTION)
    distance = control.read_hmi_hregister_by_name(Params.PARAM_DISTANCE_BUCKET_1)
    match bucket_size_selection:
        case 1:
            return distance
        case 2:
            distance = control.read_hmi_hregister_by_name(Params.PARAM_DISTANCE_BUCKET_2)
        case 3:
            distance = control.read_hmi_hregister_by_name(Params.PARAM_DISTANCE_BUCKET_3)
    return distance


def sensor_distance_params(control: RelControl, bucket_distance: int) -> SensorDistanceParams:
    return SensorDistanceParams(
        bucket_distance=bucket_distance,
        high_pre_vacuum_limit=control.read_hmi_hregister_by_name(
            Params.PARAM_PRE_VACUUM_LIMIT_HIGH
        ),
        high_vacuum_limit=control.read_hmi_hregister_by_name(Params.PARAM_VACUUM_LIMIT_HIGH),
        vacuum_distance=control.read_hmi_hregister_by_name(Params.PARAM_VACUUM_DISTANCE),
    )


def sensor_distance_state(
    control: RelControl, sensor_distance_params: SensorDistanceParams
) -> SensorDistanceState:
    sensor_distance = control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)

    if sensor_distance < sensor_distance_params.vacuum_distance:
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


def set_visual_alarm_for_bucket_state(bucket_distance: int, control: RelControl):
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


def check_distance_sensor_for_electrovales(control: RelControl):
    sensor_distance = control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)
    vacuum_distance = control.read_hmi_hregister_by_name(Params.PARAM_VACUUM_DISTANCE)
    if sensor_distance < vacuum_distance:
        control.eletrovalve_off()
        control.apply_tower_state(TowerState.VACUUM)
        control.apply_tower_state(TowerState.ACOSTIC_ALARM_ON)


def not_holded_sensor_on_a(control: RelControl):
    control.apply_tower_state(TowerState.VACUUM)
    control.apply_tower_state(TowerState.ACOSTIC_ALARM_ON)
    control.apply_tower_state(TowerState.BUCKET_CHANGE)


#  ---------------------------------------------------------
#  SensorDistanceStateName
#  These are the 5 main paths from the laser distance state
#  ---------------------------------------------------------
@config.when(sensor_distance_state=SensorDistanceStateName.A)
def sensor_laser_on__a(
    sensor_distance_state: SensorDistanceState,
    sensor_distance_params: SensorDistanceParams,
    control: RelControl,
) -> FlowStateAction:
    set_visual_alarm_for_bucket_state(sensor_distance_params.bucket_distance, control)
    return FlowStateAction.WAITING_FOR_BUCKET


def not_holded_sensor_on_b(control: RelControl):
    control.write_hmi_coil_by_address_name(HMIWriteAction.STATUS_VACUUM_ALARM, CoilState.ON)
    control.write_hmi_coil_by_address_name(
        HMIWriteAction.ACTION_PULL_DOWN_PISTONS_BUCKET, CoilState.ON
    )
    control.apply_manifold_state(ManifoldActions.PISTONS_DOWN)


@config.when(sensor_distance_state=SensorDistanceStateName.B)
def sensor_laser_on__b(
    control: RelControl,
    sensor_distance_state: SensorDistanceState,
    bucket_state: TowerState,
    sensor_distance_params: SensorDistanceParams,
) -> FlowStateAction:
    control.write_hmi_coil_by_address_name(HMIWriteAction.STATUS_VACUUM_ALARM, CoilState.ON)
    control.write_hmi_coil_by_address_name(
        HMIWriteAction.ACTION_PULL_DOWN_PISTONS_BUCKET, CoilState.ON
    )
    logger.info("moving pistons down")
    sensor_distance = control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)
    control.apply_manifold_state(ManifoldActions.ACTIVATE)
    control.apply_manifold_state(ManifoldActions.PISTONS_DOWN)
    wait_for_sensor_laser()
    if sensor_distance == control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE):
        control.apply_tower_state(TowerState.BUCKET_CHANGE)
        return FlowStateAction.WAITING_FOR_BUCKET

    logger.info("turn off pistons down till laser distance < vacuum_distance")
    while (
        control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)
        >= sensor_distance_params.vacuum_distance
    ):
        sensor_distance = control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)

    control.apply_manifold_state(ManifoldActions.DEACTIVATE)
    control.write_hmi_coil_by_address_name(
        HMIWriteAction.ACTION_PULL_DOWN_PISTONS_BUCKET, CoilState.OFF
    )
    logger.info("returning to state A")
    return FlowStateAction.RETURN_TO_STATE_A


def not_holded_sensor_on_c(control: RelControl):
    control.apply_tower_state(set_visual_alarm_for_bucket_state)
    control.write_register_by_address_name(
        name=HMIWriteAction.STATUS_ALARM_PRE_VACUUM.value,
        enum_value=1,
        stype=SlaveType.HMI,
        rtype=RegisterType.COIL,
    )
    control.write_register_by_address_name(
        name=HMIWriteAction.ACTION_PULL_DOWN_PISTONS_BUCKET.value,
        enum_value=1,
        stype=SlaveType.HMI,
        rtype=RegisterType.COIL,
    )
    control.apply_manifold_state(ManifoldActions.PISTONS_DOWN)


@config.when(sensor_distance_state=SensorDistanceStateName.C)
def sensor_laser_on__c(
    control: RelControl,
    bucket_state: TowerState,
) -> SensorLaserLectureState:
    not_holded_sensor_on_c(control)
    wait_for_sensor_laser()
    sensor_distance = control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)
    laser_distance = control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)
    if sensor_distance == laser_distance:
        control.write_hmi_coil_by_address_name(
            HMIWriteAction.ACTION_TURN_ON_PUMPING_PROCESS, CoilState.ON
        )
        return SensorLaserLectureState.PREVACUUM_BUCKET_ON
    return SensorLaserLectureState.NOT_HOLD_TO_B


@config.when(sensor_distance_state=SensorDistanceStateName.D)
def sensor_laser_on__d(
    control: RelControl,
    sensor_distance_state: SensorDistanceState,
) -> SensorLaserLectureState:
    control.apply_manifold_state(ManifoldActions.PISTONS_DOWN)
    wait_for_sensor_laser()
    sensor_distance = control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)
    laser_distance = control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)
    if sensor_distance == laser_distance:
        return SensorLaserLectureState.BUCKET_ON
    return SensorLaserLectureState.NOT_HOLD_TO_C


@config.when(sensor_distance_state=SensorDistanceStateName.E)
def sensor_laser_on__e(
    control: RelControl,
    sensor_distance_state: SensorDistanceState,
) -> SensorLaserLectureState:
    control.apply_manifold_state(ManifoldActions.PISTONS_UP)
    sensor_distance = control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)
    laser_distance = control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)
    security_distance = laser_distance
    while sensor_distance < security_distance:
        time.sleep(Constants.wait_read_laser)
        sensor_distance = control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)

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
) -> FlowStateAction:
    control.apply_tower_state(TowerState.BUCKET_CHANGE)
    return FlowStateAction.CONTINUE_BUCKET_CHANGE


@config.when_in(
    redirect_from_sensor_laser_state=[
        SensorLaserLectureState.BUCKET_ON,
        SensorLaserLectureState.PREVACUUM_BUCKET_ON,
    ]
)
def bucket_state_action__prevacuum(
    control: RelControl,
    redirect_from_sensor_laser_state: SensorDistanceState,
) -> FlowStateAction:
    control.write_register_by_address_name(
        name=HMIWriteAction.ACTION_TURN_ON_PUMPING_PROCESS.value,
        enum_value=1,
        stype=SlaveType.HMI,
        rtype=RegisterType.COIL,
    )
    target_pressure = control.read_hmi_hregister_by_name(Params.PARAM_REGULATOR_PRESSURE_SET)
    pressure = control.read_iolink_hregister_by_name(Sensors.SENSOR_PRESSURE_REGULATOR_READ_REAL)
    if pressure != target_pressure:
        control.apply_pressure_regulator_state(PressureState.ON)
    while pressure != target_pressure:
        pressure = control.read_iolink_hregister_by_name(
            Sensors.SENSOR_PRESSURE_REGULATOR_READ_REAL
        )
    control.apply_pressure_regulator_state(PressureState.OFF)
    control.apply_pwm_state()
    if control.read_hmi_cregister_by_name(HMIWriteAction.ACTION_RECYCLE) > 0:
        return FlowStateAction.RECYCLE_ENABLED
    return FlowStateAction.RECYCLE_DISABLED


@config.when(redirect_from_sensor_laser_state=SensorLaserLectureState.WAITING_FOR_BUCKET)
def bucket_state_action__setbucket(
    redirect_from_sensor_laser_state: SensorDistanceState,
) -> FlowStateAction:
    return FlowStateAction.CONTINUE_BUCKET_CHANGE


@config.when(bucket_state_action=FlowStateAction.RECYCLE_ENABLED)
def after_bucket_state_action__recycleon(
    control: RelControl,
    bucket_state_action: FlowStateAction,
):
    while (
        control.read_hmi_hregister_by_name(
            HMIWriteAction.ACTION_RECYCLE,
        )
        != 1
    ):
        continue
    control.apply_pressure_regulator_state(PressureState.ON)
    while control.read_iolink_hregister_by_name(
        Sensors.SENSOR_PRESSURE_REGULATOR_READ_REAL
    ) != control.read_hmi_hregister_by_name(Params.PARAM_TARGET_PRESSURE_HYD_HOME):
        continue
    control.apply_pressure_regulator_state(PressureState.OFF)
    control.apply_manifold_state(ManifoldActions.RECYCLE)
    recycle_time_ms = control.read_hmi_hregister_by_name(
        Params.PARAM_RECYCLE_TIME_CYCLE
    )  # assume ms
    time.sleep(recycle_time_ms)
    control.apply_manifold_state(ManifoldActions.DEACTIVATE)
    if (
        control.read_hmi_cregister_by_name(HMIWriteAction.ACTION_RECYCLE) > 0
    ):  # repeat cycle if recycle is activated
        after_bucket_state_action__recycleon(
            control=control, bucket_state_action=FlowStateAction.RECYCLE_ENABLED
        )


@config.when(bucket_state_action=FlowStateAction.RECYCLE_DISABLED)
def after_bucket_state_action__recycleoff(
    control: RelControl, bucket_state_action: FlowStateAction
) -> FlowStateAction:
    recycle_time = control.read_hmi_hregister_by_name(Params.PARAM_RECYCLE_TIME)
    start = timer()
    control.apply_pressure_regulator_state(PressureState.ON)
    is_timeout = False
    while control.read_iolink_hregister_by_name(
        Sensors.SENSOR_PRESSURE_REGULATOR_READ_REAL
    ) != control.read_hmi_hregister_by_name(Params.PARAM_TARGET_PRESSURE_HYD_HOME):
        control.apply_tower_state(TowerState.PRE_VACUUM)
        is_timeout = (timer() - start) > recycle_time
        if is_timeout:
            break
    if is_timeout:
        logger.warning("recycle has timeout")
        return FlowStateAction.RECYCLE_TIMEOUT
    logger.info("recycle in time ok")
    return FlowStateAction.RECYCLE_CYCLE_OK


@config.when(after_bucket_state_action=FlowStateAction.RECYCLE_TIMEOUT)
def recycle_state__timeout(
    after_bucket_state_action: FlowStateAction, control: RelControl
) -> FlowStateAction:
    control.apply_tower_state(TowerState.VACUUM)
    control.apply_pressure_regulator_state(PressureState.OFF)
    # TODO
    # wait for confirmation to continue
    # lets assume this is manual recycle
    logger.info("waiting for manual recycle, STANDBY ⏲ ...")
    while control.read_hmi_hregister_by_name(Params.PARAM_RECYCLE_TIME_MANUAL) != 0:
        continue
    logger.info("recycle manual received")
    recycle_manual_count = control.read_hmi_hregister_by_name(Sensors.SENSOR_MANUAL_RECYCLE_COUNT)
    control.write_hmi_hregister_by_name(
        Sensors.SENSOR_MANUAL_RECYCLE_COUNT.value, recycle_manual_count + 1
    )
    return FlowStateAction.STANDBY_EXIT_BY_MANUAL


@config.when(after_bucket_state_action=FlowStateAction.RECYCLE_CYCLE_OK)
def recycle_state__ok(
    after_bucket_state_action: FlowStateAction, control: RelControl
) -> FlowStateAction:
    control.apply_pressure_regulator_state(PressureState.OFF)
    current_pressure = control.read_iolink_hregister_by_name(
        Sensors.SENSOR_PRESSURE_REGULATOR_VALVE_READ_STATE
    )
    pressure_bares_limit = control.read_hmi_hregister_by_name(Params.PARAM_PRESSURE_BARES_LIMIT)
    if current_pressure < pressure_bares_limit:
        return FlowStateAction.PRESSURE_NOT_ON_TARGET

    logger.info("waiting to turn on pump process ⏲ ...")
    while (
        control.read_hmi_cregister_by_name(HMIWriteAction.ACTION_TURN_ON_PUMPING_PROCESS)
        != CoilState.OFF.value
    ):
        continue
    logger.info("turn off PWM")
    control.stop_pwm()
    return FlowStateAction.CONTINUE_BUCKET_CHANGE


@config.when(bucket_state_action=FlowStateAction.CONTINUE_BUCKET_CHANGE)
def after_bucket_state_action__continue(
    control: RelControl, bucket_state_action: FlowStateAction
) -> FlowStateAction:
    return bucket_state_action


def init_flow_state() -> FlowStateAction:
    pass


def bucket_change(control: RelControl):
    """
    this node runs after first flow is completed
    """
    pass
