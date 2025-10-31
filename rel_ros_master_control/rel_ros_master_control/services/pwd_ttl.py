from pydantic import BaseModel


class PWMTTLConfig(BaseModel):
    speed_cmd: str = "SPEED:"
    high_command: str = "HIGH"
    low_command: str = "LOW"
    low_motor_speed: str = "250"
    medium_motor_speed: str = "500"
    high_motor_speed: str = "1000"
    serial_port: str = "COM3"


class PWMTTL:
    def __init__(self):
        pass
