import rclpy
from rclpy.node import Node

from rel_interfaces.msg import HMI
from rel_ros_hmi.config import load_modbus_config
from rel_ros_hmi.modbus_master import create_masters_for_hmis


class RelROSNode(Node):
    def __init__(self):
        super().__init__("rel_ros_hmi_node")
        self.get_logger().info("creating publisher for rel/hmi topic 📨")
        self.rel_publisher = self.create_publisher(HMI, "rel/hmi", 10)
        self.create_timer(0.5, self.timer_callback)
        self.create_hmi_timers()
        self.get_logger().info("running modbus slave ...")
        config = load_modbus_config()
        self.masters = create_masters_for_hmis(config.slaves, config.holding_registers)

    def create_hmi_timers(self):
        self.create_timer(0.5, self.timer_callback_hmi_0)

    def timer_callback_hmi_0(self):
        self.get_logger().info("HMI 0 timer running 👾...")
        msg = HMI()
        master = self.masters[0]
        msg.hmi_name = master.hmi_name
        msg.hmi_id = master.hmi_id
        registers = master.get_holding_registers_data()
        for reg in registers:
            setattr(msg, reg.name, reg.value)
        self.rel_publisher.publish(msg)
        self.get_logger().info(f"📨 Publishing HMI message: {msg}")

    def timer_callback(self):
        self.get_logger().info("Relant ROS2 HMI Node running 🤖...")


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
