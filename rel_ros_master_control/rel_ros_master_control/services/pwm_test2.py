import lgpio
import time

# Pin number (BCM numbering)
PWM_PIN = 32

# Open the GPIO chip
h = lgpio.gpiochip_open(0)

# Set up PWM on the pin
# The frequency is 1000 Hz, and the duty cycle is 50%
# (duty cycle is 500, since it's a value from 0-1000)
lgpio.pwm(h, PWM_PIN, 1000, 500)

print("Starting PWM with 50% duty cycle...")

try:
    # Let it run for 10 seconds
    time.sleep(10)
    
finally:
    # Stop PWM and close the chip
    lgpio.pwm(h, PWM_PIN, 0, 0)
    lgpio.gpiochip_close(h)
    print("PWM stopped.")