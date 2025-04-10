import functools
import os

import rclpy
from rclpy.node import Node

from rel_interfaces.msg import IOLinkData
from rel_ros_master_control.config import load_hmi_config, load_iolink_config
from rel_ros_master_control.control import RelControl, run_masters_to_iolinks
from rel_ros_master_control.flow_control import run as run_control
from rel_ros_master_control.models.pwm_m import PWMConfig
from rel_ros_master_control.services.pwm import RelPWM


class RelROSNode(Node):
    def __init__(self):
        super().__init__("rel_ros_master_control_node")
        self.get_logger().info("creating PWM service ✨...")
        self.pwm = RelPWM(PWMConfig())
        self.is_control_running = False
        self.iolink_config = load_iolink_config()
        self.hmi_config = load_hmi_config()
        self.get_logger().info("creating Relant master control 🚀...")
        self.masters = run_masters_to_iolinks(
            iolink_slaves=self.iolink_config.iolinks,
            hr=self.iolink_config.holding_registers,
            hmi_slaves=self.hmi_config.hmis,
            hmi_hr=self.hmi_config.holding_registers,
            hmi_cr=self.hmi_config.coil_registers,
        )
        self.create_timers_for_iolink_masters()
        self.create_hmi_subscribers(len(self.masters))
        self.get_logger().info("creating publisher for rel/iolink topic 📨 ...")
        self.iolink_publisher = self.create_publisher(IOLinkData, "rel/iolink", 10)
        self.get_logger().info("======= creating MAIN CONTROL timers 🤖 =======")
        self.create_timers_for_main_control()

    def create_timers_for_main_control(self):
        if not self.masters:
            self.get_logger().error("no iolink masters available")
            return
        self.get_logger().info("creating main control timers ⏱ ...")
        for master in self.masters:
            if isinstance(master, RelControl):
                self.get_logger().info(
                    f"creating timer for main control with hmi id {master.master_io_link.hmi_id}"
                )
                self.create_timer(
                    0.5,
                    functools.partial(
                        self.timer_callback_main_control, hmi_id=master.master_io_link.hmi_id
                    ),
                )

    def timer_callback_main_control(self, hmi_id: int = 0):
        """
        Main control function
        """
        if self.is_control_running:
            return
        if not (os.getenv("ENABLE_CONTROL", "true").lower() in ["true", "yes"]):
            return
        self.get_logger().info(f"🎮 starting main control for node id {hmi_id}")
        control: RelControl = self.masters[hmi_id]
        self.is_control_running = True
        run_control(control=control, pwm=self.pwm)
        self.is_control_running = False

    def create_timers_for_iolink_masters(self):
        if not self.masters:
            self.get_logger().error("no iolink masters available")
            raise RuntimeError("no iolink masters available")

        self.get_logger().info("creating iolink data timers ⏱ ...")
        for master in self.masters:
            if isinstance(master, RelControl):
                self.get_logger().info(
                    f"creating timer for iolink master with hmi id {master.master_io_link.hmi_id}"
                )
                self.create_timer(
                    0.5,
                    functools.partial(
                        self.timer_callback_iolink_data, hmi_id=master.master_io_link.hmi_id
                    ),
                )

    def get_io_link_data(self, hmi_id: int = 0):
        master: RelControl = self.masters[hmi_id]
        msg = IOLinkData()
        msg.hmi_name = master.master_io_link.slave.hmi_name
        msg.hmi_id = master.master_io_link.slave.hmi_id
        registers = master.get_iolink_hr_data()
        for reg in registers:
            setattr(msg, reg.name, reg.value)
        self.get_logger().info(f"📨 Publishing IOLinkData message: {msg}")
        self.iolink_publisher.publish(msg)

    def timer_callback_iolink_test(self, hmi_id: int = 0):
        self.get_logger().info(f"test hmi_id {hmi_id}")

    def timer_callback_iolink_data(self, hmi_id: int = 0):
        self.get_logger().info("Relant ROS2 Master Control 🤖 - get iolink data 🤘 ...")
        self.get_logger().info(f"hmi_id {hmi_id}")
        self.get_io_link_data(hmi_id)


def main():
    try:
        rclpy.init()
        node = RelROSNode()
        rclpy.spin(node)
        rclpy.shutdown()

    except Exception as err:
        print(f"terminating ROS node {err}")


if __name__ == "__main__":
    main()
