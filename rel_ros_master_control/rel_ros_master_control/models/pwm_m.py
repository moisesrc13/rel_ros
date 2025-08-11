from pydantic import BaseModel, computed_field
from typing import Literal

class PWM(BaseModel):
    frequency: int = 1000
    duty_cycle: int = 50

class PWMConfig(BaseModel):
    chip: int = 0
    pin: int = 13  # Physical Pin 33 corresponds to BCM GPIO 13
    ramp_time: int = 2  # Time in seconds for the ramp (fade up and down).
    steps: int = 50  # Number of steps in the duty cycle change for smoother transitions.
    option: Literal ["high", "medium", "low"]
    
    @computed_field
    @property
    def params(self) -> PWM:
        match self.option:
            case "high":
                return PWM(frequency=1500)
            case "medium":
                return PWM(frequency=1000)
            case "low":
                return PWM(frequency=500)
            case _:
                return PWM()
