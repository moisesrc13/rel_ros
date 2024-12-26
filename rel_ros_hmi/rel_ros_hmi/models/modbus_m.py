from enum import Enum
from typing import Optional

from pydantic import BaseModel


class RegisterDataType(Enum):
    float16 = "float16"
    uint16 = "uint16"
    float32 = "float32"
    uint32 = "uint32"


class Register(BaseModel):
    address: int
    words: int = 1
    value: int = 0
    data_type: RegisterDataType = RegisterDataType.uint16

    def __post_init__(self):
        if self.data_type in [RegisterDataType.uint32, RegisterDataType.float32]:
            self.words = 2
        else:
            self.words = 1


class Parameters(BaseModel):
    target_pressure_pistons: Register
    target_pressure_material: Register
    system_state: Register
    pulse_train_high: Register
    pulse_train_medium: Register
    pulse_train_low: Register
    emergency_stop_state: Register
    solenoid_valves_state: Register
    alarm_state: Register
    alarm_pre_vacuum_limit_high: Register
    alarm_pre_vacuum_limit_low: Register
    alarm_pre_vacuum_distance: Register


class Sensors(BaseModel):
    material_pressure: Register
    material_temperature: Register
    laser_distance: Register


class HoldingRegisters(BaseModel):
    parameters: Parameters
    sensors: Sensors


class SlaveTCP(BaseModel):
    host: str
    port: int
    framer: str = "socket"
    timeout_seconds: int = 5
    holding_registers: HoldingRegisters


class ModbusConfig(BaseModel):
    modbus: SlaveTCP
