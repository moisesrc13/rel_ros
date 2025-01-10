import functools
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from rel_interfaces.msg import HMI, IOLinkData
from rel_ros_master_control.config import load_modbus_config
from rel_ros_master_control.control import RelControl, run_masters_to_iolinks


@dataclass
class HMIData:
    hmi_id: int = 0
    hmi: HMI = None


def create_hmi_cluster(size: int) -> list[HMIData]:
    cluster = []
    for n in range(size):
        cluster.append(HMIData(hmi_id=n))
    return cluster


def get_hmi_from_cluster_with_id(cluster: list[HMIData], hmi_id: int) -> Optional[HMIData]:
    return next((hmid for hmid in cluster if hmid.hmi_id == hmi_id), None)


class RelROSNode(Node):
    def __init__(self):
        super().__init__("rel_ros_master_control_node")
        self.config = load_modbus_config()
        self.get_logger().info("creating Relant master control 🚀...")
        self.masters = run_masters_to_iolinks(
            slaves=self.config.slaves, hr=self.config.holding_registers
        )
        self.create_timers_for_iolink_masters()
        self.hmi_cluster = create_hmi_cluster(size=1)
        # self.control = RelControl()
        self.get_logger().info("creating subscriber for rel/hmi topic 📨 ...")
        self.subscription = self.create_subscription(HMI, "rel/hmi", self.listener_hmi_callback, 10)
        self.get_logger().info("creating publisher for rel/iolink topic 📨 ...")
        self.rel_publisher = self.create_publisher(IOLinkData, "rel/iolink", 10)

    def create_timers_for_iolink_masters(self):
        if not self.masters:
            self.get_logger().error("no iolink masters available")
            return
        self.get_logger().info("creating timers ⏱ ...")
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

    def create_subscribers(self):
        self.get_logger().info("creating subscriber for rel/hmi topic 📨 ...")
        self.subscription = self.create_subscription(HMI, "rel/hmi", self.listener_hmi_callback, 10)

    def listener_hmi_callback(self, msg: HMI):
        self.get_logger().info("📨 I got an HMI message")
        hmiData = get_hmi_from_cluster_with_id(self.hmi_cluster, msg.hmi_id)
        hmiData.hmi = msg
        self.hmi_cluster[msg.hmi_id] = hmiData

    def timer_callback_iolink_data(self, hmi_id: int = 0):
        self.get_logger().info("Relant ROS2 Master Control 🤖 - get iolink data 🤘 ...")
        self.get_logger().info(f"hmi_id {hmi_id}")
        # self.get_logger().info("Relant ROS2 Master Control 🤖 - get iolink data 🤘 ...")
        # self.get_logger().info(f"this is the current HMI cluster {self.hmi_cluster}")
        # msg = IOLinkData()
        # msg.hmi_name = self.control.master_io_link.hmi_name
        # msg.hmi_id = self.control.master_io_link.hmi_id
        # registers = self.control.get_data()
        # for reg in registers:
        #    setattr(msg, reg.name, reg.value)
        # self.rel_publisher.publish(msg)
        # self.get_logger().info(f"📨 Publishing IOLinkData message: {msg}")


def main():
    try:
        rclpy.init()
        node = RelROSNode()
        rclpy.spin(node)
        rclpy.shutdown()

    except Exception as err:
        print(f"terminating program {err}")


if __name__ == "__main__":
    main()
