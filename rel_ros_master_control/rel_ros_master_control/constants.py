from enum import IntEnum, StrEnum

from pydantic import BaseModel


class SensorLaserLectureState(IntEnum):
    DO_NOTHING = 0
    HOLD = 1
    NOT_HOLD_TO_A = 2
    NOT_HOLD_TO_B = 3
    NOT_HOLD_TO_C = 4
    WAITING_FOR_BUCKET = 5
    EMPTY_BUCKET = 6
    PREVACUUM_BUCKET_ON = 7
    BUCKET_ON = 8


class CoilState(IntEnum):
    ON = 1
    OFF = 0


class FlowStateAction(IntEnum):
    CONTINUE_BUCKET_CHANGE = 1
    RECYCLE_ENABLED = 2
    RECYCLE_DISABLED = 3
    RECYCLE_TIMEOUT = 4
    RECYCLE_CYCLE_OK = 5
    STANDBY_EXIT_BY_MANUAL = 6
    PRESSURE_NOT_ON_TARGET = 7
    WAITING_FOR_BUCKET = 8
    RETURN_TO_STATE_A = 9
    RETURN_TO_STATE_B = 10
    RETURN_TO_STATE_C = 11
    PREPARE_FOR_RECYCLE_PROCESS = 12
    START_PWM = 13
    TO_RECYCLE_PROCESS = 14


class DigitalOutput(StrEnum):
    DIGITAL_OUT_HYD_VALVE = "digital_out_hyd_valve"


class DigitalInput(StrEnum):
    DIGITAL_IN_EMERGENCY_STOP = "digital_in_emergency_stop"


class DigitalHydValve(IntEnum):
    OUT1_OFF_OUT2_OFF: int = 1
    OUT1_ON_OUT2_OFF: int = 3
    OUT1_OFF_OUT2_ON: int = 5
    OUT1_ON_OUT2_ON: int = 7


class PWMPulseSet(IntEnum):
    LOW = 0
    MEDIUM = 1
    HIGH = 3


class PressureSet(StrEnum):
    REGULATOR_ACTIVATE_VALVE = "regulator_activate_valve"


class PressureState(IntEnum):
    ON = 257
    OFF = 256


class ManifoldActions(IntEnum):
    DEACTIVATE: int = int("0000_0000_0000_0000", 2)
    ACTIVATE: int = int("0000_0000_0000_0001", 2)
    EXTRA: int = int("0000_0001_0000_0000", 2)
    RECYCLE: int = int("0000_0100_0000_0000", 2)
    VENTING_RETRACTIL_UP: int = int("0001_0000_0000_0000", 2)
    VENTING_RETRACTIL_DOWN: int = int("0010_0000_0000_0000", 2)
    PISTONS_UP: int = int("0100_0000_0000_0000", 2)
    PISTONS_DOWN: int = int("1000_0000_0000_0000", 2)


class SensorDistanceParams(BaseModel):
    vacuum_distance: int  # Z
    bucket_distance: int  # W
    high_pre_vacuum_limit: int  # X
    high_vacuum_limit: int  # Y


class SensorDistanceState(StrEnum):
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


class SensorDistanceStateName(StrEnum):
    A = SensorDistanceState.LT_VACUUM_DISTANCE.value
    B = SensorDistanceState.GT_VACUUM_DISTANCE_AND_LE_HIGH_VACUUM_LIMIT.value
    C = SensorDistanceState.GT_HIGH_VACUUM_LIMIT_AND_LE_HIGH_PRE_VACUUM_LIMIT.value
    D = SensorDistanceState.GT_HIGH_VACUUM_LIMIT_AND_LT_BUCKET_SIZE_DISTANCE.value
    E = SensorDistanceState.GT_BUCKET_SIZE_DISTANCE_AND_LE_INFINITY.value


class HMIWriteAction(StrEnum):
    ACTION_TURN_ON_PUMPING_PROCESS = "action_turn_on_pumping_process"
    ACTION_PRE_FILL_LINE = "action_pre_fill_line"
    ACTION_PULL_DOWN_PISTONS_MANUAL = "action_pull_down_pistons_manual"
    ACTION_PULL_UP_PISTONS_MANUAL = "action_pull_up_pistons_manual"
    ACTION_VACUUM_AIR = "action_vacuum_air"
    ACTION_DEPRESSURIZE = "action_depressurize"
    ACTION_RECYCLE_CAR = "action_recycle_car"
    ACTION_RECYCLE_RETRACTIL = "action_recycle_retractil"
    ACTION_BUTTON_START_BUCKET_CHANGE_1 = "action_button_start_bucket_change_1"
    ACTION_BUTTON_START_BUCKET_CHANGE_2 = "action_button_start_bucket_change_2"
    ACTION_PULL_DOWN_PISTONS_BUCKET = "action_pull_down_pistons_bucket"
    ACTION_PULL_UP_PISTONS_BUCKET = "action_pull_up_pistons_bucket"
    ACTION_BUTTON_CHANGE_BUCKET_3 = "action_button_change_bucket_3"
    ACTION_RECYCLE = "action_recycle"
    ACTION_MANIFOLD = "action_manifold"
    STATUS_ALARM_PRE_VACUUM = "status_alarm_pre_vacuum"
    STATUS_VACUUM_ALARM = "status_vacuum_alarm"
    STATUS_NO_AIR_PRESSURE = "status_no_air_pressure"
    STATUS_ERROR_CHANGE_BUCKET = "status_error_change_bucket"
    STATUS_PUMP_NO_PRESSURE = "status_pump_no_pressure"
    STATUS_PUMP_NO_DEPRESSURIZED = "status_pump_no_depressurized"


class Sensors(StrEnum):
    SENSOR_MATERIAL_PRESSURE = "sensor_material_pressure"
    SENSOR_MATERIAL_TEMPERATURE = "sensor_material_temperature"
    SENSOR_LASER_DISTANCE = "sensor_laser_distance"
    SENSOR_PRESSURE_REGULATOR_READ_SET = "sensor_pressure_regulator_read_set"
    SENSOR_PRESSURE_REGULATOR_READ_REAL = "sensor_pressure_regulator_read_real"
    SENSOR_PRESSURE_REGULATOR_VALVE_READ_STATE = "sensor_pressure_regulator_valve_read_state"
    SENSOR_MANUAL_RECYCLE_COUNT = "sensor_manual_recycle_count"


class Params(StrEnum):
    PARAM_VACUUM_DISTANCE = "param_vacuum_distance"
    PARAM_DISTANCE_BUCKET_1 = "param_distance_bucket_1"
    PARAM_DISTANCE_BUCKET_2 = "param_distance_bucket_2"
    PARAM_DISTANCE_BUCKET_3 = "param_distance_bucket_3"
    PARAM_PRE_VACUUM_LIMIT_HIGH = "param_pre_vacuum_limit_high"
    PARAM_VACUUM_LIMIT_HIGH = "param_vacuum_limit_high"
    PARAM_SECURITY_DISTANCE = "param_security_distance"
    PARAM_REGULATOR_PRESSURE_SET = "param_regulator_pressure_set"  # this is in IOLink
    PARAM_PULSE_TRAIN_LOW = "param_pulse_train_low"
    PARAM_PULSE_TRAIN_MEDIUM = "param_pulse_train_medium"
    PARAM_PULSE_TRAIN_HIGH = "param_pulse_train_high"
    PARAM_PULSE_TRAIN_SELECTION = "param_pulse_train_selection"
    PARAM_TARGET_PRESSURE_PISTONS_MANUAL = "param_target_pressure_pistons_manual"
    PARAM_TARGET_PRESSURE_HYD_MANUAL = "param_target_pressure_hyd_manual"
    PARAM_RECYCLE_TIME_MANUAL = "param_recycle_time_manual"
    PARAM_RECYCLE_TIME_CYCLE = "param_recycle_time_cycle"
    PARAM_TARGET_PRESSURE_HYD_HOME = "param_target_pressure_hyd_home"
    PARAM_RECYCLE_TIME_INTERVAL = "param_recycle_time_interval"
    PARAM_RECYCLE_TIME = "param_recycle_time"
    PARAM_PRESSURE_BARES_LIMIT = "param_pressure_bares_limit"
    PARAM_BUCKET_SIZE_SELECTION = "param_bucket_size_selection"


class Constants:
    flow_tasks_init_state: list[str] = [
        "check_distance_sensor_for_electrovales",
        "bucket_distance",
        "sensor_distance_params",
        "sensor_distance_state",
        "sensor_laser_on",
        "init_flow_state",
    ]
    flow_tasks_bucket_change: list[str] = ["bucket_change"]
    laser_infinity: int = 1000
    wait_for_sensor_laser_ms: int = 2000
    wait_read_laser: float = 0.20
