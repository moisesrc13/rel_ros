from pydantic import BaseModel


class HMI(BaseModel):
    param_target_pressure_pistons: int = 0
    param_target_pressure_material: int = 0
    param_system_state: int = 0
    param_pulse_train_high: int = 0
    param_pulse_train_medium: int = 0
    param_pulse_train_low: int = 0
    param_emergency_stop_state: int = 0
    param_solenoid_valves_state: int = 0
    param_alarm_state: int = 0
    param_alarm_pre_vacuum_limit_high: int = 0
    param_alarm_pre_vacuum_limit_low: int = 0
    param_alarm_pre_vacuum_distance: int = 0
    sensor_material_pressure: int = 0
    sensor_material_temperature: int = 0
    sensor_laser_distance: int = 0


class HMIConfig(BaseModel):
    hmi: HMI
