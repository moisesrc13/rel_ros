from enum import Enum
from typing import Optional

from pydantic import BaseModel


class RegisterModbusType(Enum):
    HR = "holding"
    CR = "coil"


class RegisterDataType(Enum):
    float16 = "float16"
    uint16 = "uint16"
    float32 = "float32"
    uint32 = "uint32"


class CRegister(BaseModel):
    name: str
    address: int
    value: int = 0


class HRegister(BaseModel):
    name: str
    address: int
    value: int = 0
    words: int = 1
    data_type: RegisterDataType = RegisterDataType.uint16

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


class SlaveHMI(BaseModel):
    hmi_name: str = ""
    hmi_id: int = 0
    slave_tcp: SlaveTCP


class ModbusConfig(BaseModel):
    hmis: list[SlaveHMI]
    holding_registers: list[HRegister]
    coil_registers: list[CRegister]


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


def get_hr_addresses(registers: list[HRegister]) -> list[int]:
    addresses = [r.address for r in registers]
    addresses.sort()
    return addresses
