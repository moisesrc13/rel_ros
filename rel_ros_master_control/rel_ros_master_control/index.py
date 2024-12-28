from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from rel_interfaces.msg import HMI
from rel_ros_master_control.control import RelControl


@dataclass
class HMIData:
    id: int = 0
    hmi: HMI = None


@dataclass
class HMICluster:
    cluster: list[HMIData]


def create_hmi_cluster(size: int) -> HMICluster:
    cluster = []
    for n in range(size):
        cluster.append(HMIData(id=n))
    return cluster


class RelROSNode(Node):
    def __init__(self):
        super().__init__("rel_ros_master_control_node")
        self.get_logger().info("creating Relant master control 🚀...")
        self.create_timer(1.0, self.timer_callback)
        self.hmi_cluster = create_hmi_cluster(1)
        # self.control = RelControl()
        self.get_logger().info("creating subscriber 📨 ...")
        self.subscription = self.create_subscription(HMI, "rel/hmi", self.listener_hmi_callback, 10)

    def listener_hmi_callback(self, msg):
        self.get_logger().info(f"📨 I got an HMI message {msg}")
        setattr(self.hmi, msg.name, msg.value)
        updated_attr = getattr(self.hmi, msg.name)
        self.get_logger().info(f"hmi updated config {updated_attr}")

    def timer_callback(self):
        self.get_logger().info("Relant ROS2 Master Control 🤖 Node running 🤘 ...")
        self.get_logger().info(f"current HMI config {self.hmi}")


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
