from enum import Enum

from pydantic import BaseModel


class StatusRegister(BaseModel):
    address: int
    value: int


class BasicActions(Enum):
    TURN_OFF = 0
    TURN_ON = 1
    FLASH = 2
    ANIMATION = 3
    AUDIBLE = 3


class StatusSegment(BaseModel):
    octect: int
    bits: list[int]


class BasicSegments(BaseModel):
    audible: StatusSegment
    segment_1: StatusSegment
    segment_2: StatusSegment
    segment_3: StatusSegment
    segment_4: StatusSegment
    segment_5: StatusSegment
    segment_6: StatusSegment
    segment_7: StatusSegment
    segment_8: StatusSegment
    segment_9: StatusSegment
    segment_10: StatusSegment


class StatusDevice(BaseModel):
    segments: BasicSegments


class ControlStatus(BaseModel):
    full: list[StatusRegister]
    medium_high: list[StatusRegister]
    medium: list[StatusRegister]
    pre_vacuum: list[StatusRegister]
    vacuum: list[StatusRegister]
    bucket_change: list[StatusRegister]


class StatusDeviceConfig(BaseModel):
    basic: StatusDevice
    control_status: ControlStatus


class TowerControlStates(BaseModel):
    full: list[int]
    medium_high: list[int]
    medium: list[int]
    pre_vacuum: list[int]
    vacuum: list[int]
    bucket_change: list[int]


class TowerStatus(BaseModel):
    start_address: int
    total_registers: int
    states: TowerControlStates


class TowerStatusDevice:
    tower_status: TowerStatus

    def __init__(self, tower_status: TowerStatus):
        self.tower_status = tower_status
