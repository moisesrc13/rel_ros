import time
from time import sleep
from pydantic import BaseModel

import RPi.GPIO as GPIO

class PWMConfig(BaseModel):
    pin: int = 12  # this is default PWM0 in RPi
    frequency: int = 1000

class RelPWM:
    def __init__(self, config: PWMConfig):
        try:
            GPIO.setwarnings(False)  # disable warnings
            GPIO.setmode(GPIO.BOARD)  # set pin numbering system
            GPIO.setup(config.pin, GPIO.OUT)
            self.pi_pwm = GPIO.PWM(
                config.pin, config.frequency
            )  # create PWM instance with frequency
        except Exception as err:
            print(f"error setting up PWM - {err}")

    def run_duty(
        self, time_seconds: int = 0, duty: int = 100
    ):  # provide duty cycle in the range 0-100
        t_end = time.time() + time_seconds
        try:
            print(f"start running pwm with duty {duty}")
            while time.time() < t_end:
                for d in range(0, duty, 1):
                    self.pi_pwm.ChangeDutyCycle(d)
                    sleep(0.01)
                sleep(0.25)

                for d in range(duty, -1, -1):
                    self.pi_pwm.ChangeDutyCycle(d)
                    sleep(0.01)
                sleep(0.25)
            print(f"end running pwm with duty {duty}")
        except Exception as err:
            print(f"error running PWM - {err}")

    def start_duty(self, duty: int = 100):  # provide duty cycle in the range 0-100
        try:
            self.pi_pwm.start(0)
            self.pi_pwm.ChangeDutyCycle(duty)
        except Exception as err:
            print(f"error running PWM - {err}")

    def change_duty(self, duty: int = 100):
        try:
            self.pi_pwm.ChangeDutyCycle(duty)
        except Exception as err:
            print(f"error changing PWM duty - {err}")

    def stop_duty(self):
        try:
            self.pi_pwm.stop()
        except Exception as err:
            print(f"error stopping PWM - {err}")


if __name__ == "__main__":
    my_pwm = RelPWM(PWMConfig())
    my_pwm.run_duty(120)
