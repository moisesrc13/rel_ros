import signal
import time

import lgpio

from rel_ros_master_control.logger import new_logger
from rel_ros_master_control.models.pwm_m import PWMConfig

logger = new_logger(__name__)


class RelPWM:
    def __init__(self, config=PWMConfig()):
        try:
            self.config = config
            self.handler = lgpio.gpiochip_open(config.chip)
            lgpio.gpio_claim_output(self.handler, config.pin)
        except lgpio.LgpiodError as ec:
            logger.error("error opening chip for PWM %s", ec)
        except Exception as err:
            logger.error("error setting up PWM - %s", err)

    def run(self):
        try:
            logger.info("ramping up PWM ...")
            for duty_cycle in range(0, self.config.steps, 1):
                lgpio.tx_pwm(self.handler, self.config.pin, self.config.frequency, duty_cycle)
                time.sleep(0.01)
            logger.info("running full PWM ...")
            lgpio.tx_pwm(
                self.handler, self.config.pin, self.config.frequency, self.config.duty_cycle
            )
            while True:
                time.sleep(0.5)
        finally:
            self.stop()

    def stop(self):
        try:
            lgpio.tx_pwm(self.handler, self.config.pin, 0, 0)
            lgpio.gpiochip_close(self.handler)
            logger.info("GPIO chip closed.")
        except:
            logger.error("error stopping PWM")


def stop_pwm_handler(pwm: RelPWM):
    def signal_handler(signum, frame):
        pwm.stop()

    return signal_handler


if __name__ == "__main__":
    pwm = RelPWM()
    handler = stop_pwm_handler(pwm)
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
