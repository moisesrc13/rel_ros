from enum import Enum


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
    STATUS_ALARM_PRE_VACUUM = "status_alarm_pre_vacuum"
    STATUS_ALARM = "status_alarm"
    STATUS_NO_AIR_PRESSURE = "status_no_air_pressure"
    STATUS_ERROR_CHANGE_BUCKET = "status_error_change_bucket"
    STATUS_PUMP_NO_PRESSURE = "status_pump_no_pressure"
    STATUS_PUMP_NO_DEPRESSURIZED = "status_pump_no_depressurized"


class Constants:
    flow_tasks: list[str] = [
        "bucket_distance",
        "sensor_distance_params",
        "bucket_state",
        "check_distance_sensor_for_electrovales",
        "sensor_distance_state",
        "sensor_laser_on",
    ]
    laser_infinity: int = 1000
