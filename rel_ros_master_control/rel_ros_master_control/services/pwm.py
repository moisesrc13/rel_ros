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

    def run_duty(
        self, time_seconds: int = 0, duty: int = 100
    ):  # provide duty cycle in the range 0-100
        t_end = time.time() + time_seconds
        try:
            logger.info("start running pwm with duty %s", duty)
            while time.time() < t_end:
                for d in range(0, duty, 1):
                    self.pi_pwm.ChangeDutyCycle(d)
                    sleep(0.01)
                sleep(0.25)

                for d in range(duty, -1, -1):
                    self.pi_pwm.ChangeDutyCycle(d)
                    sleep(0.01)
                sleep(0.25)
            logger.info("end running pwm with duty %s", duty)
        except Exception as err:
            logger.error("error running PWM - %s", err)

    def start_duty(self, duty: int = 100):  # provide duty cycle in the range 0-100
        try:
            self.pi_pwm.start(0)
            self.pi_pwm.ChangeDutyCycle(duty)
        except Exception as err:
            logger.error("error starting PWM duty %s - %s", duty, err)

    def change_duty(self, duty: int = 100):
        try:
            self.pi_pwm.ChangeDutyCycle(duty)
        except Exception as err:
            logger.error("error changing PWM duty %s - %s", duty, err)

    def stop_duty(self):
        try:
            self.pi_pwm.stop()
        except Exception as err:
            logger.error("error stopping PWM - %s", err)


if __name__ == "__main__":
    logger.info("running pwm for 5 secs...")
    my_pwm = RelPWM(PWMConfig())
    my_pwm.run_duty(5)
