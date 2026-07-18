from enum import Enum

import serial
from pydantic import BaseModel

from rel_ros_master_control.logger import new_logger

logger = new_logger(__name__)


class PWMSpeed(Enum):
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"


class PWMTurn(Enum):
    CLOCKWISE = "clockwise"
    COUNTERCLOCKWISE = "counterclockwise"


class PWMTTLConfig(BaseModel):
    speed_cmd: str = "SPEED:"
    ramp_command: str = "RAMP:"
    serial_port: str = "COM3"
    baudrate: int = 115_200
    timeout: int = 3


class PWMTTL:
    def __init__(self, config: PWMTTLConfig = PWMTTLConfig()):
        self.config = config
        try:
            self.serial = serial.Serial(
                config.serial_port, baudrate=config.baudrate, timeout=config.timeout
            )
            logger.info("💡 PWM serial module connected %s", self.serial.name)
        except serial.SerialException as err:
            logger.error("error connecting to serial 🏮")
            raise err

    def run_pwm(self, speed: PWMSpeed, ramp: bool = True):
        if ramp:
            self.serial.write(b"RAMP:\n")
        match speed:
            case PWMSpeed.HIGH:
                self.serial.write(b"SPEED:1000\n")
            case PWMSpeed.MEDIUM:
                self.serial.write(b"SPEED:500\n")
            case PWMSpeed.LOW:
                self.serial.write(b"SPEED:250\n")
            case _:
                self.serial.write(b"SPEED:500\n")

    def run_turn(self, turn: PWMTurn):
        match turn:
            case PWMTurn.COUNTERCLOCKWISE:
                self.serial.write(b"LOW\n")
            case _:
                self.serial.write(b"HIGH\n")

    def stop(self):
        self.serial.write(b"SPEED:0\n")
