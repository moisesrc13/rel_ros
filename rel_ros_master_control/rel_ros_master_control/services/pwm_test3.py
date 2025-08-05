import lgpio
import time

# --- Configuration ---
# GPIO pin number (using BCM numbering)
# Physical Pin 32 corresponds to BCM GPIO 12
PWM_PIN = 12

# Frequency of the PWM signal in Hz
# Note: 500,000 Hz (500 kHz) is a realistic high frequency.
# 10,000,000 Hz (10 MHz) will not work.
FREQUENCY = 5

# Duty cycle as a percentage (0-100)
DUTY_CYCLE = 50

# Open the GPIO chip
try:
    h = lgpio.gpiochip_open(0)
    print("Successfully opened GPIO chip 0.")
except lgpio.LgpiodError as e:
    print(f"Error opening GPIO chip: {e}")
    print("Is the lgpiod daemon running? Try 'sudo systemctl start lgpiod.service'")
    exit()

print(f"Starting PWM on BCM GPIO {PWM_PIN} (Physical Pin 32)")
print(f"Frequency: {FREQUENCY} Hz")
print(f"Duty Cycle: {DUTY_CYCLE}%")

try:
    # Start the PWM with the specified frequency and duty cycle
    # tx_pwm(handle, gpio, frequency, duty_cycle_percentage)
    lgpio.tx_pwm(h, PWM_PIN, FREQUENCY,  DUTY_CYCLE * 10000)

    # Let the PWM run indefinitely until interrupted
    print("PWM is now running. Press Ctrl+C to stop.")
    while True:
        time.sleep(1)

finally:
    # --- Cleanup ---
    print("\nStopping PWM and cleaning up.")
    # Stop the PWM signal by setting the frequency to 0
    lgpio.tx_pwm(h, PWM_PIN, 0, 0)
    
    # Close the GPIO chip
    lgpio.gpiochip_close(h)
    print("GPIO chip closed.")