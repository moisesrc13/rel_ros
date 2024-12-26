from enum import Enum
from typing import Optional

from pydantic import BaseModel


class RegisterDataType(Enum):
    float16 = "float16"
    uint16 = "uint16"
    float32 = "float32"
    uint32 = "uint32"


class Register(BaseModel):
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
    holding_registers: list[Register]


class ModbusConfig(BaseModel):
    modbus: SlaveTCP


def get_register_by_address(
    registers: list[Register], address: int
) -> Optional[tuple[Register, int]]:
    for index, r in enumerate(registers):
        if r.address == address:
            return (r, index)
    return (None, None)
