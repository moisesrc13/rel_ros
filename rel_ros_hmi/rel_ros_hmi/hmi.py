import json
from enum import Enum

from pydantic import BaseModel


class MessageType(Enum):
    PARAMETER = "parameter"
    SENSOR = "sensor"


class HMIMessage(BaseModel):
    msg_type: MessageType
    value: int
    name: str


def _serialize(message: HMIMessage) -> str:
    try:
        return message.model_dump_json()
    except Exception as err:
        raise err


def _deserialize(msg: str) -> HMIMessage:
    try:
        msg_dict = json.loads(msg)
        return HMIMessage.model_validate_json(msg_dict)
    except Exception as err:
        raise err
