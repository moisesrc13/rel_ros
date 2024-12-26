import rclpy
from rclpy.node import Node

from rel_ros_master_control.control import RelControl


class RelROSNode(Node):
    def __init__(self):
        super().__init__("rel_ros_master_control_node")
        self.get_logger().info("creating Relant master control 🚀...")
        self.create_timer(1.0, self.timer_callback)
        self.control = RelControl()

    def timer_callback(self):
        self.get_logger().info("Relant ROS2 Master Control Node running 🤖...")


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
