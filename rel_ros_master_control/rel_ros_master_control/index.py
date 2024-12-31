from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from rel_interfaces.msg import HMI, IOLinkData
from rel_ros_master_control.control import RelControl


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
        self.get_logger().info("creating Relant master control 🚀...")
        self.create_timer(1.0, self.timer_callback_iolink_data)
        self.hmi_cluster = create_hmi_cluster(size=1)
        self.control = RelControl()
        self.get_logger().info("creating subscriber for rel/hmi topic 📨 ...")
        self.subscription = self.create_subscription(HMI, "rel/hmi", self.listener_hmi_callback, 10)
        self.get_logger().info("creating publisher for rel/iolink topic 📨 ...")
        self.rel_publisher = self.create_publisher(IOLinkData, "rel/iolink", 10)

    def listener_hmi_callback(self, msg: HMI):
        self.get_logger().info(f"📨 I got an HMI message")
        hmiData = get_hmi_from_cluster_with_id(self.hmi_cluster, msg.hmi_id)
        hmiData.hmi = msg
        self.hmi_cluster[msg.hmi_id] = hmiData

    def timer_callback_iolink_data(self):
        self.get_logger().info("Relant ROS2 Master Control 🤖 - get iolink data 🤘 ...")
        self.get_logger().info(f"this is the current HMI cluster {self.hmi_cluster}")
        msg = IOLinkData()
        msg.hmi_name = self.control.master_io_link.hmi_name
        msg.hmi_id = self.control.master_io_link.hmi_id
        registers = self.control.get_data()
        for reg in registers:
            setattr(msg, reg.name, reg.value)
        self.rel_publisher.publish(msg)
        self.get_logger().info(f"📨 Publishing IOLinkData message: {msg}")


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
