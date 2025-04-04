from pydantic import BaseModel


class PWMConfig(BaseModel):
    pin: int = 12  # this is default PWM0 in RPi
    frequency: int = 1000
