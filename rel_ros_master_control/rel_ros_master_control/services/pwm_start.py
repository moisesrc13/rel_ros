import os
import sys
import signal
import time

import lgpio

from rel_ros_master_control.logger import new_logger
from rel_ros_master_control.models.pwm_m import PWMConfig

logger = new_logger(__name__)


class RelPWM:
    def __init__(self, config=PWMConfig()):
        self.is_running = False
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
            self.is_running = True
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
            if self.is_running:
                self.stop()

    def stop(self):
        try:
            lgpio.tx_pwm(self.handler, self.config.pin, 0, 0)
            lgpio.gpiochip_close(self.handler)
            logger.info("GPIO chip closed.")
            self.is_running = False
        except:
            logger.error("error stopping PWM")


def stop_pwm_handler(pwm: RelPWM):
    def signal_handler(signum, frame):
        pwm.stop()
        sys.exit(0)

    return signal_handler


if __name__ == "__main__":
    pwm = RelPWM()
    handler = stop_pwm_handler(pwm)
    with open("/tmp/pwm_control.pid", "w") as f:
        f.write(str(os.getpid()))
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
    pwm.run()
