from enum import Enum
from typing import Optional

from pydantic import BaseModel


class RegisterDataType(Enum):
    float16 = "float16"
    uint16 = "uint16"
    float32 = "float32"
    uint32 = "uint32"


class CRegister(BaseModel):
    name: str
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


def get_register_by_address(
    registers: list[HRegister] | list[CRegister], address: int
) -> Optional[tuple[HRegister, int] | tuple[CRegister, int]]:
    for index, r in enumerate(registers):
        if r.address == address:
            return (r, index)
    return (None, None)


def get_register_by_name(
    registers: list[HRegister] | list[CRegister], name: str
) -> Optional[HRegister | CRegister]:
    return next((r for r in registers if r.name == name), None)
