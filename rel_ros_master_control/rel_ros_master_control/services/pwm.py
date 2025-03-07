import time
from time import sleep

import RPi.GPIO as GPIO

from rel_ros_master_control.logger import new_logger
from rel_ros_master_control.models.pwm_m import PWMConfig

logger = new_logger(__name__)


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
            logger.error("error setting up PWM - %s", err)

    def run(self, time_seconds: int = 0, duty: int = 100):  # provide duty cycle in the range 0-100
        t_end = time.time() + time_seconds
        try:
            while time.time() < t_end:
                for duty in range(0, duty, 1):
                    self.pi_pwm.ChangeDutyCycle(duty)
                    sleep(0.01)
                sleep(0.25)

                for duty in range(duty, -1, -1):
                    self.pi_pwm.ChangeDutyCycle(duty)
                    sleep(0.01)
                sleep(0.25)
        except Exception as err:
            logger.error("error running PWN - %s", err)
