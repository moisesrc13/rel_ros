from enum import Enum

from pydantic import BaseModel


class StatusRegister(BaseModel):
    address: int
    value: int


class TowerState(Enum):
    TOFF = "toff"
    FULL = "full"
    MEDIUM_HIGH = "medium_high"
    MEDIUM = "medium"
    PRE_VACUUM = "pre_vacuum"
    VACUUM = "vacuum"
    BUCKET_CHANGE = "bucket_change"
    ACOUSTIC_ALARM_ON = "acoustic_alarm_on"
    ACOUSTIC_ALARM_OFF = "acoustic_alarm_off"


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
    toff: list[int]
    full: list[int]
    medium_high: list[int]
    medium: list[int]
    pre_vacuum: list[int]
    vacuum: list[int]
    bucket_change: list[int]
    acoustic_alarm_on: list[int]
    acoustic_alarm_off: list[int]


class TowerStatus(BaseModel):
    start_address: int
    total_registers: int
    alarm_address: int
    states: TowerControlStates


class TowerStatusDevice:
    tower_status: TowerStatus

    def __init__(self, tower_status: TowerStatus):
        self.tower_status = tower_status
