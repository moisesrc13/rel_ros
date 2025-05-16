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
    Sensors,
)
from rel_ros_master_control.control import RelControl
from rel_ros_master_control.logger import new_logger
from rel_ros_master_control.models.status_device_m import TowerState

logger = new_logger(__name__)


def wait_for_sensor_laser():
    time.sleep(Constants.wait_for_sensor_laser_ms / 1000)


def check_distance_sensor_for_electrovales(control: RelControl):
    sensor_distance = control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)
    vacuum_distance = control.read_hmi_hregister_by_name(Params.PARAM_VACUUM_DISTANCE)
    if sensor_distance < vacuum_distance:
        control.eletrovalve_off()
        control.apply_tower_state(TowerState.VACUUM)
        control.apply_tower_state(TowerState.ACOSTIC_ALARM_ON)


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


def set_visual_alarm_for_bucket_state(control: RelControl):
    """_summary_

    calculate the alarm distance which depends on the bucket distance (size) selected
    """
    laser_distance = control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)
    state = TowerState.BUCKET_CHANGE
    if laser_distance >= 80 and laser_distance <= 100:
        state = TowerState.FULL
    elif laser_distance >= 50 and laser_distance <= 79:
        state = TowerState.MEDIUM_HIGH
    elif laser_distance >= 30 and laser_distance <= 49:
        state = TowerState.MEDIUM_HIGH
    elif laser_distance >= 10 and laser_distance <= 20:
        state = TowerState.PRE_VACUUM
        control.apply_tower_state(TowerState.ACOSTIC_ALARM_ON)
    elif laser_distance >= 2 and laser_distance <= 5:
        control.apply_tower_state(TowerState.ACOSTIC_ALARM_ON)
        state = TowerState.VACUUM
    control.apply_tower_state(state)


#  ---------------------------------------------------------
#  SensorDistanceStateName
#  These are the 5 main paths from the laser distance state
#  ---------------------------------------------------------

#  --------------------------
#  A) Sensor laser d<Z
#  --------------------------
@config.when(sensor_distance_state=SensorDistanceStateName.A)
def sensor_laser_on__a(
    sensor_distance_state: SensorDistanceStateName,
    control: RelControl,
) -> FlowStateAction:
    logger.info("no bucket in place - State A")
    set_visual_alarm_for_bucket_state(control)
    return FlowStateAction.WAITING_FOR_BUCKET


#  --------------------------
#  B) Sensor laser d>Z && d<=Y
#  --------------------------
@config.when(sensor_distance_state=SensorDistanceStateName.B)
def sensor_laser_on__b(
    control: RelControl,
    sensor_distance_params: SensorDistanceParams,
    sensor_distance_state: SensorDistanceStateName,
) -> FlowStateAction:
    set_visual_alarm_for_bucket_state(control)
    control.write_hmi_cregister_by_address_name(HMIWriteAction.STATUS_VACUUM_ALARM, CoilState.ON)
    control.write_hmi_cregister_by_address_name(
        HMIWriteAction.ACTION_PULL_DOWN_PISTONS_BUCKET, CoilState.ON
    )
    logger.info("moving pistons down")
    sensor_distance = control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)
    control.apply_manifold_state(ManifoldActions.ACTIVATE)
    control.apply_manifold_state(ManifoldActions.PISTONS_DOWN)
    wait_for_sensor_laser()
    if sensor_distance == control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE):
        logger.info("bucket in place vacuum - State B")
        control.apply_tower_state(TowerState.BUCKET_CHANGE)
        return FlowStateAction.WAITING_FOR_BUCKET

    logger.info("turn off pistons down till laser distance < vacuum_distance")
    while (
        control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)
        >= sensor_distance_params.vacuum_distance
    ):
        continue

    control.apply_manifold_state(ManifoldActions.DEACTIVATE)
    control.write_hmi_cregister_by_address_name(
        HMIWriteAction.ACTION_PULL_DOWN_PISTONS_BUCKET, CoilState.OFF
    )
    logger.info("returning to state A")
    return FlowStateAction.RETURN_TO_STATE_A


#  --------------------------
#  C) Sénsor laser d>Y && d<=X
#  --------------------------
@config.when(sensor_distance_state=SensorDistanceStateName.C)
def sensor_laser_on__c(
    control: RelControl,
    sensor_distance_state: SensorDistanceStateName,
) -> FlowStateAction:
    set_visual_alarm_for_bucket_state(control)
    control.write_hmi_cregister_by_address_name(
        HMIWriteAction.STATUS_ALARM_PRE_VACUUM, CoilState.ON
    )
    control.write_hmi_cregister_by_address_name(
        HMIWriteAction.ACTION_PULL_DOWN_PISTONS_BUCKET, CoilState.ON
    )
    control.apply_tower_state(TowerState.PRE_VACUUM)
    sensor_distance = control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)
    control.apply_manifold_state(ManifoldActions.ACTIVATE)
    control.apply_manifold_state(ManifoldActions.PISTONS_DOWN)
    wait_for_sensor_laser()
    if sensor_distance == control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE):
        logger.info("bucket in place for pre-vacuum - State C")
        return FlowStateAction.PREPARE_FOR_RECYCLE_PROCESS
    return FlowStateAction.RETURN_TO_STATE_B


#  --------------------------
#  D) Sensor laser d>X && d<W
#  --------------------------
@config.when(sensor_distance_state=SensorDistanceStateName.D)
def sensor_laser_on__d(
    control: RelControl,
    sensor_distance_state: SensorDistanceStateName,
) -> FlowStateAction:
    control.apply_manifold_state(ManifoldActions.ACTIVATE)
    control.apply_manifold_state(ManifoldActions.PISTONS_DOWN)
    laser_distance = control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)
    logger.info("verify laser state after move - State D")
    wait_for_sensor_laser()
    if laser_distance == control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE):
        return FlowStateAction.PREPARE_FOR_RECYCLE_PROCESS
    return FlowStateAction.RETURN_TO_STATE_C


#  --------------------------
#  E) Sensor laser d>W && d<= ∞
#  --------------------------
@config.when(sensor_distance_state=SensorDistanceStateName.E)
def sensor_laser_on__e(
    control: RelControl,
    sensor_distance_params: SensorDistanceStateName,
) -> FlowStateAction:
    control.apply_manifold_state(ManifoldActions.ACTIVATE)
    control.apply_manifold_state(ManifoldActions.PISTONS_UP)
    security_distance = control.read_hmi_hregister_by_name(Params.PARAM_SECURITY_DISTANCE)
    logger.info("reaching security distance for laser sensor - State E")
    while control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE) < security_distance:
        time.sleep(Constants.wait_read_laser)

    control.apply_manifold_state(ManifoldActions.DEACTIVATE)
    control.apply_tower_state(TowerState.BUCKET_CHANGE)
    return FlowStateAction.WAITING_FOR_BUCKET


def init_flow_state(sensor_laser_on: FlowStateAction) -> FlowStateAction:
    return sensor_laser_on


def prepare_for_recycle_process(control: RelControl):
    control.write_hmi_cregister_by_address_name(
        HMIWriteAction.ACTION_TURN_ON_PUMPING_PROCESS, CoilState.ON
    )
    target_pressure = control.read_hmi_hregister_by_name(Params.PARAM_REGULATOR_PRESSURE_SET)
    logger.info("chekcing target pressure on")
    pressure = control.read_iolink_hregister_by_name(Sensors.SENSOR_PRESSURE_REGULATOR_READ_REAL)
    if pressure != target_pressure:
        control.apply_pressure_regulator_state(PressureState.ON)
    while (
        control.read_iolink_hregister_by_name(Sensors.SENSOR_PRESSURE_REGULATOR_READ_REAL)
        != target_pressure
    ):
        continue
    control.apply_pressure_regulator_state(PressureState.OFF)
    return FlowStateAction.START_PWM


def start_pwm(prepare_for_recycle_process: FlowStateAction, control: RelControl) -> FlowStateAction:
    logger.info("starting pwm")
    control.apply_pwm_state()
    return FlowStateAction.PWM_STARTED


def validate_recycle(start_pwm: FlowStateAction, control: RelControl):
    logger.info("validate if recycle is needed")
    if control.read_hmi_cregister_by_name(HMIWriteAction.ACTION_RECYCLE) > 0:
        logger.info("recycle enabled")
        return FlowStateAction.RECYCLE_ENABLED
    logger.info("recycle disabled")
    return FlowStateAction.RECYCLE_DISABLED


@config.when(validate_recycle=FlowStateAction.RECYCLE_DISABLED)
def recycle__disabled(control: RelControl, validate_recycle: FlowStateAction) -> FlowStateAction:
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


@config.when(validate_recycle=FlowStateAction.RECYCLE_ENABLED)
def recycle__enabled(
    control: RelControl,
    validate_recycle: FlowStateAction,
):
    while (
        control.read_hmi_hregister_by_name(
            HMIWriteAction.ACTION_RECYCLE,
        )
        != 1
    ):
        continue
    logger.info("apply pressure after recycle enabled")
    control.apply_pressure_regulator_state(PressureState.ON)
    while control.read_iolink_hregister_by_name(
        Sensors.SENSOR_PRESSURE_REGULATOR_READ_REAL
    ) != control.read_hmi_hregister_by_name(Params.PARAM_TARGET_PRESSURE_HYD_HOME):
        continue
    control.apply_pressure_regulator_state(PressureState.OFF)
    control.apply_manifold_state(ManifoldActions.ACTIVATE)
    logger.info("turn on recycle")
    control.apply_manifold_state(ManifoldActions.RECYCLE)
    recycle_time_ms = control.read_hmi_hregister_by_name(
        Params.PARAM_RECYCLE_TIME_CYCLE
    )  # assume milliseconds
    logger.info("waiting for recycle time")
    time.sleep(recycle_time_ms)
    control.apply_manifold_state(ManifoldActions.DEACTIVATE)
    return FlowStateAction.TO_PWM


@config.when(recycle=FlowStateAction.RECYCLE_CYCLE_OK)
def recycle_flow_state__ok(recycle: FlowStateAction, control: RelControl) -> FlowStateAction:
    logger.info("stop pump on pressure target")
    control.apply_pressure_regulator_state(PressureState.OFF)
    current_pressure = control.read_iolink_hregister_by_name(
        Sensors.SENSOR_PRESSURE_REGULATOR_VALVE_READ_STATE
    )
    logger.info("waiting for pump to reach target pressure")
    pressure_bares_limit = control.read_hmi_hregister_by_name(Params.PARAM_PRESSURE_BARES_LIMIT)
    if current_pressure < pressure_bares_limit:
        return FlowStateAction.PRESSURE_NOT_ON_TARGET_BARES

    logger.info("waiting to turn off pump process from HMI ⏲ ...")
    while (
        control.read_hmi_cregister_by_name(HMIWriteAction.ACTION_TURN_ON_PUMPING_PROCESS)
        != CoilState.OFF.value
    ):
        continue
    logger.info("turn off PWM")
    control.stop_pwm()
    return FlowStateAction.WAITING_FOR_BUCKET


@config.when(recycle=FlowStateAction.RECYCLE_TIMEOUT)
def recycle_flow_state__timeout(recycle: FlowStateAction, control: RelControl) -> FlowStateAction:
    control.apply_tower_state(TowerState.VACUUM)
    control.apply_manifold_state(ManifoldActions.DEACTIVATE)
    control.apply_pressure_regulator_state(PressureState.OFF)
    # TODO
    # wait for confirmation to continue
    # lets assume this is manual recycle
    logger.info("waiting for manual recycle, STANDBY ⏲ ...")
    while control.read_hmi_cregister_by_name(HMIWriteAction.ACTION_RECYCLE_RETRACTIL) == 0:
        continue
    logger.info("recycle retractil received")
    recycle_manual_count = control.read_hmi_hregister_by_name(Sensors.SENSOR_MANUAL_RECYCLE_COUNT)
    count = recycle_manual_count + 1
    logger.info("new count for manual retractil action %s", count)
    control.write_hmi_hregister_by_name(Sensors.SENSOR_MANUAL_RECYCLE_COUNT, count)
    return FlowStateAction.TO_INIT_FLOW


@config.when(recycle=FlowStateAction.TO_PWM)
def recycle_flow_state__pwm(recycle: FlowStateAction) -> FlowStateAction:
    return recycle


def bucket_change(
    control: RelControl, sensor_distance_params: SensorDistanceParams
) -> FlowStateAction:
    """
    this node runs after first flow is completed
    """
    logger.info("🪣 start bucket change")
    control.write_hmi_cregister_by_address_name(HMIWriteAction.ENTER_SCREEN_3_0, CoilState.ON)
    control.stop_pwm()
    control.apply_tower_state(TowerState.ACOSTIC_ALARM_OFF)
    control.apply_tower_state(TowerState.BUCKET_CHANGE)
    logger.info("evaluate sensor laser position")
    # TODO verify
    logger.info("waiting for bucket change action step 1")
    while (
        control.read_hmi_cregister_by_name(HMIWriteAction.ACTION_BUTTON_START_BUCKET_CHANGE_1) == 0
    ):
        continue
    logger.info("activate electro-valves retractil")
    control.apply_manifold_state(ManifoldActions.VENTING_RETRACTIL_UP)
    sensor_distance = control.read_iolink_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)
    if sensor_distance < sensor_distance_params.bucket_distance:
        return FlowStateAction.BUCKET_CHANGE_UNDER_W
    # TODO what if is equal?
    return FlowStateAction.BUCKET_CHANGE_OVER_W


@config.when(bucket_change=FlowStateAction.BUCKET_CHANGE_UNDER_W)
def bucket_change_frame__underw(
    control: RelControl,
    sensor_distance_params: SensorDistanceParams,
    bucket_change: FlowStateAction,
):
    logger.info("wait for bucket change confirmation ...")
    while (
        control.read_hmi_cregister_by_name(HMIWriteAction.ACTION_BUTTON_START_BUCKET_CHANGE_2) == 0
    ):
        control.apply_manifold_state(ManifoldActions.ACTIVATE)
        control.apply_manifold_state(ManifoldActions.PISTONS_UP)
        if (
            control.read_hmi_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)
            > sensor_distance_params.bucket_distance
        ):
            control.apply_manifold_state(ManifoldActions.DEACTIVATE)
            break
        continue
    control.apply_manifold_state(ManifoldActions.AIR_FOR_VACUUM)
    control.apply_manifold_state(ManifoldActions.ACTIVATE)
    control.apply_manifold_state(ManifoldActions.PISTONS_UP)
    logger.info("when laser distance > W turn off vacuum electro-vale")
    while (
        control.read_hmi_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)
        < sensor_distance_params.bucket_distance
    ):
        continue
    control.apply_manifold_state(ManifoldActions.DEACTIVATE)


@config.when(bucket_change=FlowStateAction.BUCKET_CHANGE_OVER_W)
def bucket_change_frame__overw(control: RelControl, bucket_change: FlowStateAction):
    control.apply_manifold_state(ManifoldActions.ACTIVATE)
    control.apply_manifold_state(ManifoldActions.PISTONS_DOWN)
    while (
        control.read_hmi_hregister_by_name(Sensors.SENSOR_LASER_DISTANCE)
        < sensor_distance_params.bucket_distance
    ):
        continue
    control.apply_manifold_state(ManifoldActions.DEACTIVATE)
