from enum import Enum

from pydantic import BaseModel


class StatusRegister(BaseModel):
    address: int
    value: int


class TowerState(Enum):
    FULL = "full"
    MEDIUM_HIGH = "medium_high"
    MEDIUM = "medium"
    PRE_VACUUM = "pre_vacuum"
    VACUUM = "vacuum"
    BUCKET_CHANGE = "bucket_change"
    ACOSTIC_ALARM = "acoustic_alarm"


class BasicActions(Enum):
    TURN_OFF = 0
    TURN_ON = 1
    FLASH = 2
    ANIMATION = 3
    AUDIBLE = 3


class StatusSegment(BaseModel):
    octect: int
    bits: list[int]


class TowerControlStates(BaseModel):
    full: list[int]
    medium_high: list[int]
    medium: list[int]
    pre_vacuum: list[int]
    vacuum: list[int]
    bucket_change: list[int]
    acoustic_alarm: list[int]


class TowerStatus(BaseModel):
    start_address: int
    total_registers: int
    states: TowerControlStates


class TowerStatusDevice:
    tower_status: TowerStatus

    def __init__(self, tower_status: TowerStatus):
        self.tower_status = tower_status
