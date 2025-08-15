import os
import subprocess
import signal

def do_stop_process():
    root_dir = os.path.abspath(os.path.dirname(__file__))
    script_file = os.path.basename(__file__)
    subprocess.run(["python", f"{root_dir}/{script_file}"])

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
