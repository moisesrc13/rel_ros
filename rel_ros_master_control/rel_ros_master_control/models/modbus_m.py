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
    words: int
    data_type: RegisterDataType = RegisterDataType.uint16
    name: str = ""
    value: int = 0

    def __post_init__(self):
        if self.data_type in [RegisterDataType.uint32, RegisterDataType.float32]:
            self.words = 2
        else:
            self.words = 1


class HoldingRegisters(BaseModel):
    data_input_status: Optional[Register] = None
    data_input: Optional[Register] = None
    data_output_status: Optional[Register] = None
    data_output: Optional[Register] = None


class DevicePort(BaseModel):
    init_address: int
    name: Optional[str] = None
    holding_registers: HoldingRegisters


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
    framer: str = "socket"
    timeout_seconds: int = 5
    offset: int = 0
    device_ports: DevicePorts


class SlaveSerial(BaseModel):
    baudrate: int = 115_200
    data_bit: int = 8
    stop_bit: int = 1
    parity: str = "N"
    timeout_seconds: int = 5
    framer: str = "rtu"
    port: str = "/dev/ptyp0"


class ModbusSlaves(BaseModel):
    master_io_link: SlaveTCP | SlaveSerial


class ModbusConfig(BaseModel):
    slaves: ModbusSlaves
