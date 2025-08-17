import rclpy
from rclpy.node import Node


class RelROSPWMNode(Node):
    def __init__(self):
        super().__init__("rel_ros_pwm_node")
