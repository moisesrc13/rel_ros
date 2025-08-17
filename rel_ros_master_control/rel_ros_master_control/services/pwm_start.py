import argparse
import os
import signal
import subprocess
import sys
import time

import lgpio

from rel_ros_master_control.logger import new_logger
from rel_ros_master_control.models.pwm_m import PWMConfig

logger = new_logger(__name__)


class RelPWM:
    def __init__(self, option: str = "medium"):
        self.is_running = False
        self.option = option
        try:
            self.config = PWMConfig(option=option)
            self.handler = lgpio.gpiochip_open(self.config.chip)
            lgpio.gpio_claim_output(self.handler, self.config.pin)
        except lgpio.LgpiodError as ec:
            logger.error("error opening chip for PWM %s", ec)
        except Exception as err:
            logger.error("error setting up PWM - %s", err)

    def run(self):
        try:
            self.is_running = True
            logger.info("ramping up PWM for option %s ...", self.option)
            for duty_cycle in range(0, self.config.steps, 1):
                lgpio.tx_pwm(
                    self.handler, self.config.pin, self.config.params.frequency, duty_cycle
                )
                time.sleep(0.01)
            logger.info("running full PWM ...")
            lgpio.tx_pwm(
                self.handler,
                self.config.pin,
                self.config.params.frequency,
                self.config.params.duty_cycle,
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


def do_start_pwm_process(option: str = "medium"):
    logger.info("running ♾️ pwm with option %s", option)
    root_dir = os.path.abspath(os.path.dirname(__file__))
    script_file = os.path.basename(__file__)
    subprocess.run(["python", f"{root_dir}/{script_file}", "--option", option])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-o",
        "--option",
        help="pwm type, e.g 'high', 'medium', 'low'",
        dest="option",
        default="medium",
        type=str,
    )
    args = parser.parse_args()
    pwm = RelPWM(option=args.option)
    handler = stop_pwm_handler(pwm)
    with open("/tmp/pwm_control.pid", "w") as f:
        f.write(str(os.getpid()))
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
    pwm.run()
