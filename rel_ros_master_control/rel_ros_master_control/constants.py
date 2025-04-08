from enum import Enum

from pydantic import BaseModel


class SensorLaserLectureState(Enum):
    HOLD = 1
    NOT_HOLD_TO_A = 2
    NOT_HOLD_TO_B = 3
    NOT_HOLD_TO_C = 4


class DigitalOutput(Enum):
    DIGITAL_OUT_HYD_VALVE = "digital_out_hyd_valve"


class DigitalInput(Enum):
    DIGITAL_IN_EMERGENCY_STOP = "digital_in_emergency_stop"


class DigitalHydValve(BaseModel):
    out1_off_out2_off: int = 1
    out1_on_out2_off: int = 3
    out1_off_out2_on: int = 5
    out1_on_out2_on: int = 7


class ManifoldActions(BaseModel):
    DEACTIVATE: int = int("0000_0000_0000_0000", 2)
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


class HMIWriteAction(Enum):
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
    STATUS_ALARM = "status_alarm"
    STATUS_NO_AIR_PRESSURE = "status_no_air_pressure"
    STATUS_ERROR_CHANGE_BUCKET = "status_error_change_bucket"
    STATUS_PUMP_NO_PRESSURE = "status_pump_no_pressure"
    STATUS_PUMP_NO_DEPRESSURIZED = "status_pump_no_depressurized"


class Sensors(Enum):
    SENSOR_MATERIAL_PRESSURE = "sensor_material_pressure"
    SENSOR_MATERIAL_TEMPERATURE = "sensor_material_temperature"
    SENSOR_LASER_DISTANCE = "sensor_laser_distance"
    SENSOR_PRESSURE_REGULATOR_READ_SET = "sensor_pressure_regulator_read_set"
    SENSOR_PRESSURE_REGULATOR_READ_REAL = "sensor_pressure_regulator_read_real"
    SENSOR_PRESSURE_REGULATOR_VALVE_READ_STATE = "sensor_pressure_regulator_valve_read_state"


class Params(Enum):
    PARAM_VACUUM_DISTANCE = "param_vacuum_distance"
    PARAM_DISTANCE_BUCKET_1 = "param_distance_bucket_1"
    PARAM_DISTANCE_BUCKET_2 = "param_distance_bucket_2"
    PARAM_DISTANCE_BUCKET_3 = "param_distance_bucket_3"
    PARAM_PRE_VACUUM_LIMIT_HIGH = "param_pre_vacuum_limit_high"
    PARAM_VACUUM_LIMIT_HIGH = "param_vacuum_limit_high"


class Constants:
    flow_tasks: list[str] = [
        "hmi_hr_data",
        "hmi_cr_data",
        "iolink_hr_data",
        "bucket_distance",
        "sensor_distance_params",
        "sensor_distance_state",
        "bucket_state",
        "check_distance_sensor_for_electrovales",
        "sensor_laser_on",
        "redirect_sensor_laser_on_not_holded",
    ]
    laser_infinity: int = 1000
    wait_for_sensor_laser_ms: int = 2000
