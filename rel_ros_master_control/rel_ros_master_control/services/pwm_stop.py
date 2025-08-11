import os
import signal

if __name__ == "__main__":
    try:
        with open("/tmp/pwm_control.pid", "r") as f:
            pid = int(f.read())

        # Send SIGTERM to the PWM process
        os.kill(pid, signal.SIGTERM)
        print(f"Sent SIGTERM to PWM process with PID: {pid}")

    except FileNotFoundError:
        print("pwm process ID file not found")
    except Exception as e:
        print(f"Error: {e}")
