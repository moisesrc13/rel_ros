import functools
import os
from queue import Queue
from threading import Thread

import rclpy
from rclpy.node import Node

from rel_interfaces.msg import HMIUserTask
from rel_ros_master_control.config import load_hmi_config, load_iolink_config
from rel_ros_master_control.constants import Constants
from rel_ros_master_control.control import run_masters_to_iolinks
from rel_ros_master_control.flow_control import run_control


class RelROSNode(Node):
    def __init__(self):
        super().__init__("rel_ros_master_control_node")
        self.is_control_running = False
        self.queue = Queue()
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
        self.get_logger().info("apply initial state")
        self.apply_initial_state()
        self.get_logger().info("======= creating MAIN CONTROL 🤖 =======")
        self.start_main_control()
        self.create_timer(5.0, self.timer_callback_text)

    def timer_callback_text(self):
        self.get_logger().info("I'm alive in ROS 👾")

    def apply_initial_state(self):
        for m in self.masters:
            m.apply_initial_state()

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
        self.masters[hmi_id].run_user_actions(msg.coil_address, msg.value)

    def start_main_control(self):
        if not (os.getenv("ENABLE_CONTROL", "true").lower() in ["true", "yes"]):
            return
        for idx, m in enumerate(self.masters):
            self.get_logger().info(f"🚀 🎮 starting main control for node id {m.hmi_id}")
            self.queue.put(idx)
            m_thread = Thread(
                target=run_control,
                args=(m, Constants.flow_calculate_distance_sensor_case, self.queue),
            )
            m_thread.start()
        try:
            self.queue.join()
        except Exception as err:
            self.get_logger().warning(f"error waiting for main control to finish - {err}")


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
