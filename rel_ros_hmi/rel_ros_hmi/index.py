import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rel_ros_hmi.hmi import HMIMessage, MessageType, _serialize


class RelROSNode(Node):
    def __init__(self):
        super().__init__("rel_ros_hmi_node")
        self.rel_publisher = self.create_publisher(String, "hmi", 10)
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Relant ROS2 HMI Node running 🤖...")
        # msg = HMIMsg()
        msg = String()
        custom_msg = HMIMessage(
            msg_type=MessageType.PARAMETER, name="target_pressure_pistons", value=100
        )
        msg.data = _serialize(custom_msg)
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
