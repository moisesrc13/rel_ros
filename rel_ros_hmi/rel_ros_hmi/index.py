import rclpy
from rclpy.node import Node

from rel_interfaces.msg import HMI, IOLinkData
from rel_ros_hmi.config import load_modbus_config
from rel_ros_hmi.modbus_master import create_masters_for_hmis
from rel_ros_hmi.modbus_slave import run_modbus_slaves


class RelROSNode(Node):
    def __init__(self):
        super().__init__("rel_ros_hmi_node")
        self.get_logger().info("creating subscriber for rel/iolink topic 📨")
        self.subscription = self.create_subscription(
            IOLinkData, "rel/iolink", self.listener_iolink_data_callback, 10
        )
        self.create_hmi_timers()
        self.get_logger().info("running modbus slaves 🤖 ...")
        config = load_modbus_config()
        run_modbus_slaves(
            config.slaves, config.holding_registers, self.create_hmi_publishers(len(config.slaves))
        )
        self.get_logger().info("creating modbus hmi master connections 👾 ...")
        self.masters = create_masters_for_hmis(config.slaves, config.holding_registers)

    def create_hmi_publishers(self, count: int = 1) -> dict:
        publishers = {}
        for p in range(count):
            topic = f"rel/hmi_{p}"
            self.get_logger().info(f"creating publisher for {topic} topic 📨")
            publishers[p] = self.create_publisher(HMI, topic, 10)
        return publishers

    def create_hmi_timers(self):
        self.create_timer(0.5, self.timer_callback_hmi_0)

    def listener_iolink_data_callback(self, msg: IOLinkData):
        self.get_logger().info(f"📨 I got an IOLinkData message {msg}")
        self.save_hmi_iolink_data(msg.hmi_id, msg)

    def save_hmi_iolink_data(self, master_id: int, msg: IOLinkData):
        try:
            master = self.masters[master_id]
            for register in master.hr:
                if value := getattr(msg, register.name, None):
                    self.get_logger().info(
                        f"writing iolink data into hmi with address: {register.address}, value: {value}"
                    )
                    self.get_logger().info(
                        f"📺 write HMI {master_id} register {register.address} value {value}"
                    )
                    master.do_write(register.address, value)
            self.get_logger().info(f"complete write into hmi master id {master_id}")
        except Exception as err:
            self.get_logger().error(f"error saving iolink data {err}")

    def publish_hmi_data(self, master_id: int):
        master = self.masters[master_id]
        msg = HMI()
        msg.hmi_name = master.hmi_name
        msg.hmi_id = master.hmi_id
        registers = master.get_holding_registers_data()
        for reg in registers:
            setattr(msg, reg.name, reg.value)
        self.rel_publisher.publish(msg)
        self.get_logger().info(f"📨 Publishing HMI message: {msg}")

    def timer_callback_hmi_0(self):
        self.get_logger().info("HMI 0 timer running 👾...")
        self.publish_hmi_data(0)


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
