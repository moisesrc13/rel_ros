from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from rel_interfaces.msg import HMI
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
        self.create_timer(1.0, self.timer_callback)
        self.hmi_cluster = create_hmi_cluster(size=1)
        # self.control = RelControl()
        self.get_logger().info("creating subscriber 📨 ...")
        self.subscription = self.create_subscription(HMI, "rel/hmi", self.listener_hmi_callback, 10)

    def listener_hmi_callback(self, msg: HMI):
        self.get_logger().info(f"📨 I got an HMI message {msg}")
        hmiData = get_hmi_from_cluster_with_id(self.hmi_cluster, msg.hmi_id)
        hmiData.hmi = msg
        self.hmi_cluster[msg.hmi_id] = hmiData

    def timer_callback(self):
        self.get_logger().info("Relant ROS2 Master Control 🤖 Node running 🤘 ...")
        self.get_logger().info(f"this is the current HMI cluster {self.hmi_cluster}")


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
