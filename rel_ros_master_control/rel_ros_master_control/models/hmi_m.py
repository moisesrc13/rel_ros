from pydantic import BaseModel

from rel_ros_master_control.models.modbus_m import CRegister, HRegister, SlaveTCP


class ModbusConfig(BaseModel):
    slaves: list[SlaveTCP]
    holding_registers: list[HRegister]
    coil_registers: list[CRegister]
