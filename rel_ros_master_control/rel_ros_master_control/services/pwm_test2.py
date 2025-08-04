import lgpio
import time

# --- Configuration ---
# GPIO pin number (using BCM numbering)
PWM_PIN = 32

# Frequency of the PWM signal in Hz
FREQUENCY = 10

# Open the GPIO chip
# On a Raspberry Pi, the main GPIO chip is usually #0
try:
    h = lgpio.gpiochip_open(0)
    print("Successfully opened GPIO chip 0.")
except lgpio.LgpiodError as e:
    print(f"Error opening GPIO chip: {e}")
    print("Is the lgpiod daemon running? Try 'sudo systemctl start lgpiod.service'")
    exit()

# It's good practice to claim the GPIO as an output first, though tx_pwm handles this.
# You can uncomment this line, but the tx_pwm function will also do it internally.
# lgpio.gpio_claim_output(h, PWM_PIN)

print(f"Starting PWM on GPIO {PWM_PIN} with a frequency of {FREQUENCY} Hz.")
print("The LED will fade in and out.")

try:
    # Fade in: increase duty cycle from 0 to 100
    for duty_cycle in range(0, 101, 1):
        # tx_pwm(handle, gpio, frequency, duty_cycle_percentage)
        lgpio.tx_pwm(h, PWM_PIN, FREQUENCY, duty_cycle)
        time.sleep(0.02) # Small delay for the fade effect

    # Hold at 100% duty cycle for a moment
    time.sleep(3)

    # Fade out: decrease duty cycle from 100 to 0
    for duty_cycle in range(100, -1, -1):
        lgpio.tx_pwm(h, PWM_PIN, FREQUENCY, duty_cycle)
        time.sleep(0.02) # Small delay for the fade effect

    # Hold at 0% duty cycle for a moment
    time.sleep(3)
    
finally:
    # --- Cleanup ---
    # To stop the PWM, set the frequency to 0.
    # The duty cycle value in this call is ignored.
    print(f"Stopping PWM and releasing GPIO {PWM_PIN}.")
    lgpio.tx_pwm(h, PWM_PIN, 0, 0)
    
    # Close the GPIO chip
    lgpio.gpiochip_close(h)
    print("GPIO chip closed.")