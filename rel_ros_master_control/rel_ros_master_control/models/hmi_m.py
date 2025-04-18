from pydantic import BaseModel

from rel_ros_master_control.models.modbus_m import CRegister, HRegister, SlaveTCP


class SlaveHMI(BaseModel):
    hmi_name: str = ""
    hmi_id: int = 0
    slave_tcp: SlaveTCP


class ModbusHMIConfig(BaseModel):
    hmis: list[SlaveHMI]
    holding_registers: list[HRegister]
    coil_registers: list[CRegister]
