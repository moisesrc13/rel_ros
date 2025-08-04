import gpiod
import time

# Define the GPIO chip and line
chip = gpiod.Chip('gpiochip0')  # Or 'gpiochip1' or other, depending on your RPi model
pwm_line = chip.get_line(17)  # Replace 17 with your desired GPIO pin number

# Request the line as an output
pwm_line.request(consumer="PWM_Example", type=gpiod.LINE_REQ_DIR_OUT)

# PWM parameters
frequency = 100  # Hz
duty_cycle = 50  # %

# Calculate pulse durations
period = 1 / frequency
high_time = period * (duty_cycle / 100)
low_time = period - high_time

try:
    while True:
        # Set pin high
        pwm_line.set_value(1)
        time.sleep(high_time)

        # Set pin low
        pwm_line.set_value(0)
        time.sleep(low_time)

except KeyboardInterrupt:
    pass

finally:
    # Release the GPIO line when done
    pwm_line.release()
    chip.close()