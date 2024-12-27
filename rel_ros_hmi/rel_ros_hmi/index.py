import rclpy
from rclpy.node import Node

from rel_interfaces.msg import HMI
from rel_ros_hmi.config import load_modbus_config
from rel_ros_hmi.modbus_slave import run


class RelROSNode(Node):
    def __init__(self):
        super().__init__("rel_ros_hmi_node")
        self.get_logger().info("creating publisher for hmi topic")
        self.rel_publisher = self.create_publisher(HMI, "rel/hmi", 10)
        self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("running modbus slave ...")
        self.config = load_modbus_config()
        run(self.config.modbus, self.rel_publisher)

    def timer_callback(self):
        self.get_logger().info("Relant ROS2 HMI Node running 🤖...")
        # msg = HMIMsg()
        msg = HMI()
        msg.name = "target_pressure_pistons"
        msg.type = "parameter"
        msg.value = 1200
        self.rel_publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg}")


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
