import rclpy
from rclpy.node import Node

from rel_interfaces.msg import HMI
from rel_ros_master_control.config import load_hmi_control_config
from rel_ros_master_control.control import RelControl


class RelROSNode(Node):
    def __init__(self):
        super().__init__("rel_ros_master_control_node")
        self.get_logger().info("creating Relant master control 🚀...")
        self.create_timer(1.0, self.timer_callback)
        # self.control = RelControl()
        hmi_config = load_hmi_control_config()
        self.hmi = hmi_config.hmi
        self.get_logger().info("creating subscriber 📨 ...")
        self.subscription = self.create_subscription(HMI, "rel/hmi", self.listener_hmi_callback, 10)

    def listener_hmi_callback(self, msg):
        self.get_logger().info(f"📨 I got an HMI message {msg}")
        setattr(self.hmi, msg.name, msg.value)
        self.get_logger().info(f"hmi config {self.hmi}")

    def timer_callback(self):
        self.get_logger().info("Relant ROS2 Master Control 🤖 Node running 🤘 ...")


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
