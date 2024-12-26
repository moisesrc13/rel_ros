from enum import Enum

from pydantic import BaseModel


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


class StatusDeviceConfig(BaseModel):
    basic: StatusDevice
