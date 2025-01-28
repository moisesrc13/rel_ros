from enum import Enum
from typing import Optional

from pydantic import BaseModel


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
    words: int
    data_type: RegisterDataType = RegisterDataType.uint16
    name: str = ""
    value: int = 0

    def __post_init__(self):
        if self.data_type in [RegisterDataType.uint32, RegisterDataType.float32]:
            self.words = 2
        else:
            self.words = 1


class DevicePortHoldingRegisters(BaseModel):
    data_input_status: Optional[HRegister] = None
    data_input: Optional[HRegister] = None
    data_output_status: Optional[HRegister] = None
    data_output: Optional[HRegister] = None


class DevicePort(BaseModel):
    init_address: int
    name: Optional[str] = None
    holding_registers: DevicePortHoldingRegisters


class DevicePorts(BaseModel):
    port_1: Optional[DevicePort] = None
    port_2: Optional[DevicePort] = None
    port_3: Optional[DevicePort] = None
    port_4: Optional[DevicePort] = None
    port_5: Optional[DevicePort] = None
    port_6: Optional[DevicePort] = None
    port_7: Optional[DevicePort] = None
    port_8: Optional[DevicePort] = None


class SlaveTCP(BaseModel):
    host: str
    port: int
    device_ports: DevicePorts
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
