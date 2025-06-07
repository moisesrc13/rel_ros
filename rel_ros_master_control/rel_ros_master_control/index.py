import functools
import os

import rclpy
from rclpy.node import Node

from rel_interfaces.msg import HMIUserTask
from rel_ros_master_control.config import load_hmi_config, load_iolink_config
from rel_ros_master_control.constants import Constants
from rel_ros_master_control.control import RelControl, run_masters_to_iolinks
from rel_ros_master_control.flow_control import run as run_control


class RelROSNode(Node):
    def __init__(self):
        super().__init__("rel_ros_master_control_node")
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
        self.get_logger().info("======= creating consumer for UserTasks 🤖 =======")
        self.create_hmi_user_task_subscribers(len(self.masters))
        self.get_logger().info("======= creating MAIN CONTROL timers 🤖 =======")
        self.create_timers_for_main_control()
        self.get_logger().info("======= creating timers for control actions 🤖 =======")
        self.create_timers_for_control_actions()

    def create_hmi_user_task_subscribers(self, count: int = 1):
        for s in range(count):
            topic = f"rel/hmi_user_task_{s}"
            self.get_logger().info(f"creating hmi subscriber for user task topic {topic} 📨")
            self.create_subscription(
                HMIUserTask,
                topic,
                functools.partial(self.listener_hmi_user_task_callback, hmi_id=s),
                10,
            )
        self.get_logger().info("creating hmi consumers is done ...")

    def listener_hmi_user_task_callback(self, msg: HMIUserTask, hmi_id: int = 0):
        self.get_logger().info(f"📨 I got an HMI {hmi_id} user task message 📺 {msg}")
        hmiData = get_hmi_from_cluster_with_id(self.hmi_cluster, msg.hmi_id)
        hmiData.hmi = msg
        self.hmi_cluster[msg.hmi_id] = hmiData

    def create_timers_for_control_actions(self):
        if not self.masters:
            self.get_logger().error("no iolink masters available")
            return
        self.get_logger().info("creating control action timers ⏱ ...")
        for master in self.masters:
            if isinstance(master, RelControl):
                self.get_logger().info(
                    f"creating control action timer for hmi id {master.master_io_link.hmi_id}"
                )
                self.create_timer(
                    0.25,
                    functools.partial(
                        self.timer_callback_main_control, hmi_id=master.master_io_link.hmi_id
                    ),
                )

    def timer_callback_control_actions(self, hmi_id: int = 0):
        control: RelControl = self.masters[hmi_id]
        control.check_actions()

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
        run_control(control, Constants.flow_tasks_init_state)
        self.is_control_running = False

    def timer_callback_iolink_test(self, hmi_id: int = 0):
        self.get_logger().info(f"test hmi_id {hmi_id}")


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
