from enum import Enum
from typing import Optional

from pydantic import BaseModel


class IOLinkHR(Enum):
    SENSOR_MATERIAL_PRESSURE = "sensor_material_pressure"
    SENSOR_MATERIAL_TEMPERATURE = "sensor_material_temperature"
    SENSOR_LASER_DISTANCE = "sensor_laser_distance"
    SENSOR_PRESSURE_REGULATOR_READ_SET = "sensor_pressure_regulator_read_set"
    SENSOR_PRESSURE_REGULATOR_REAL = "sensor_pressure_regulator_real"
    SENSOR_PRESSURE_REGULATOR_VALVE_STATE = "sensor_pressure_regulator_valve_state"
    SENSOR_PRESSURE_REGULATOR_WRITE_SET = "sensor_pressure_regulator_write_set"
    SENSOR_PRESSURE_REGULATOR_ACTIVATE_VALVE = "sensor_pressure_regulator_activate_valve"
    MANIFOLD = "manifold"
    DIGITAL_OUT_HYD_VALVE = "digital_out_hyd_valve"


class DigitalHydValveState(Enum):
    OUT1_OFF_OUT2_OFF = "out1_off_out2_off"
    OUT1_ON_OUT2_OFF = "out1_on_out2_off"
    OUT1_OFF_OUT2_ON = "out1_off_out2_on"
    OUT1_ON_OUT2_ON = "out1_on_out2_on"


class DigitalHydValve(BaseModel):
    out1_off_out2_off: int = 1
    out1_on_out2_off: int = 3
    out1_off_out2_on: int = 5
    out1_on_out2_on: int = 7


class RegisterDataType(Enum):
    float16 = "float16"
    uint16 = "uint16"
    float32 = "float32"
    uint32 = "uint32"


class CRegister(BaseModel):
    address: int
    value: int = 0  # this will be only valid for 0 or 1


class HRegister(BaseModel):
    address: int
    words: int = 1
    data_type: RegisterDataType = RegisterDataType.uint16
    name: str = ""
    value: int = 0

    def __post_init__(self):
        if self.data_type in [RegisterDataType.uint32, RegisterDataType.float32]:
            self.words = 2
        else:
            self.words = 1


class SlaveTCP(BaseModel):
    host: str
    port: int
    name: str = ""
    hmi_id: int = 0
    hmi_name: str = "HMI"
    framer: str = "socket"
    timeout_seconds: int = 5
    offset: int = 0


class SlaveSerial(BaseModel):
    name: str = ""
    hmi_id: int = 0
    hmi_name: str = "HMI"
    baudrate: int = 115_200
    data_bit: int = 8
    stop_bit: int = 1
    parity: str = "N"
    timeout_seconds: int = 5
    framer: str = "rtu"
    port: str = "/dev/ptyp0"


class ModbusConfig(BaseModel):
    iolinks: list[SlaveTCP | SlaveSerial]
    holding_registers: list[HRegister]


def get_register_by_address(
    registers: list[HRegister], address: int
) -> Optional[tuple[HRegister, int]]:
    for index, r in enumerate(registers):
        if r.address == address:
            return (r, index)
    return (None, None)
