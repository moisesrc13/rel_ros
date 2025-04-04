from enum import Enum
from typing import Optional

from pydantic import BaseModel


class ManifoldActions(BaseModel):
    ACTIVATE: int = int("0000_0000_0000_0001", 2)
    EXTRA: int = int("0000_0001_0000_0000", 2)
    RECYCLE: int = int("0000_0100_0000_0000", 2)
    VENTING_RETRACTIL_UP: int = int("0001_0000_0000_0000", 2)
    VENTING_RETRACTIL_DOWN: int = int("0010_0000_0000_0000", 2)
    PISTONS_UP: int = int("0100_0000_0000_0000", 2)
    PISTONS_DOWN: int = int("1000_0000_0000_0000", 2)


class IOLinkHR(Enum):
    SENSOR_MATERIAL_PRESSURE = "sensor_material_pressure"
    SENSOR_MATERIAL_TEMPERATURE = "sensor_material_temperature"
    SENSOR_LASER_DISTANCE = "sensor_laser_distance"
    SENSOR_PRESSURE_REGULATOR_READ_SET = "sensor_pressure_regulator_read_set"
    SENSOR_PRESSURE_REGULATOR_READ_REAL = "sensor_pressure_regulator_read_real"
    SENSOR_PRESSURE_REGULATOR_VALVE_READ_STATE = "sensor_pressure_regulator_valve_read_state"
    REGULATOR_PRESSURE_SET = "regulator_pressure_set"
    REGULATOR_ACTIVATE_VALVE = "regulator_activate_valve"
    MANIFOLD = "manifold"
    DIGITAL_OUT_HYD_VALVE = "digital_out_hyd_valve"


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


class RegisterMode(Enum):
    RW = "rw"
    R = "r"
    W = "w"


class HRegister(BaseModel):
    address: int
    words: int = 1
    data_type: RegisterDataType = RegisterDataType.uint16
    name: str = ""
    value: int = 0
    mode: RegisterMode = RegisterMode.R

    def __post_init__(self):
        if self.data_type in [RegisterDataType.uint32, RegisterDataType.float32]:
            self.words = 2
        else:
            self.words = 1


class SlaveTCP(BaseModel):
    host: str
    port: int
    framer: str = "socket"
    timeout_seconds: int = 5
    offset: int = 0


class SlaveIOLink(BaseModel):
    name: str = ""
    hmi_id: int = 0
    hmi_name: str = "HMI"
    slave_tcp: SlaveTCP


class SlaveHMI(BaseModel):
    name: str = ""
    id: int = 0
    slave_tcp: SlaveTCP


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
    iolinks: list[SlaveIOLink]
    holding_registers: list[HRegister]
    hmis: list[SlaveHMI]


def get_register_by_address(
    registers: list[HRegister], address: int
) -> Optional[tuple[HRegister, int]]:
    for index, r in enumerate(registers):
        if r.address == address:
            return (r, index)
    return (None, None)


def get_register_by_name(registers: list[HRegister], name: str) -> Optional[HRegister]:
    return next((r for r in registers if r.name == name), None)
