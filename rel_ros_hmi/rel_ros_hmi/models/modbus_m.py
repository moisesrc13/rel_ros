from enum import Enum
from typing import Optional

from pydantic import BaseModel


class RegisterModbusType(Enum):
    HR = "hr"
    CO = "co"


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
    name: str
    host: str
    port: int
    id: int
    framer: str = "socket"
    timeout_seconds: int = 1


class ModbusConfig(BaseModel):
    slaves: list[SlaveTCP]
    holding_registers: list[HRegister]
    coil_registers: list[CRegister]


def get_register_by_address(
    registers: list[HRegister], address: int
) -> Optional[tuple[HRegister, int]]:
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
