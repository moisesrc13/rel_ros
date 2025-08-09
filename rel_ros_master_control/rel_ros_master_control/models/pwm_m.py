from pydantic import BaseModel


class PWMConfig(BaseModel):
    chip: int = 0
    pin: int = 13  # Physical Pin 33 corresponds to BCM GPIO 13
    frequency: int = 1000
    duty_cycle: int = 50
    ramp_time: int = 2  # Time in seconds for the ramp (fade up and down).
    steps: int = 50  # Number of steps in the duty cycle change for smoother transitions.
