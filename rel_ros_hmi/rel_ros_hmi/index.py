import rclpy
from rclpy.node import Node

from rel_interfaces.msg import HMIAction, IOLinkData
from rel_ros_hmi.config import load_modbus_config
from rel_ros_hmi.modbus_master import create_masters_for_hmis
from rel_ros_hmi.modbus_slave import run_modbus_slaves
from rel_ros_hmi.models.modbus_m import get_register_by_name


class RelROSNode(Node):
    def __init__(self):
        super().__init__("rel_ros_hmi_node")
        self.get_logger().info("creating subscriber for rel/iolink topic 📨")
        self.subscription_iolink = self.create_subscription(
            IOLinkData, "rel/iolink", self.listener_iolink_data_callback, 10
        )
        self.subscription_hmi_status = self.create_subscription(
            HMIAction, "rel/hmiaction", self.listener_hmi_action_callback, 10
        )
        self.get_logger().info("running modbus slaves 🤖 ...")
        config = load_modbus_config()
        run_modbus_slaves(config.hmis, config.holding_registers, config.coil_registers)
        self.get_logger().info("creating modbus hmi master connections 👾 ...")
        self.masters = create_masters_for_hmis(
            config.hmis, config.holding_registers, config.coil_registers
        )

    def listener_iolink_data(self, msg: IOLinkData, id: int = 0):
        self.get_logger().info(f"📨 I got an IOLinkData {id} message {msg}")
        self.save_hmi_iolink_data(msg.hmi_id, msg)

    def listener_iolink_data_callback(self, msg: IOLinkData):
        self.get_logger().info(f"📨 I got an IOLinkData message {msg}")
        self.save_hmi_iolink_data(msg.hmi_id, msg)

    def listener_hmi_action_callback(self, msg: HMIAction):
        self.get_logger().info(f"📨 I got an HMIAction message {msg}")
        self.write_hmi_action(msg.hmi_id, msg)

    def write_hmi_action(self, master_id: int, msg: HMIAction):
        try:
            master = self.masters[master_id]
            if register := get_register_by_name(
                master.coils, msg.action_name
            ):  # holding action on coils only for now
                self.get_logger().info(
                    f"📺 write HMI {master_id} coil {register.address} value: {msg.action_value}"
                )
                master.do_write(register.address, msg.action_value)
                self.get_logger().info(f"complete write status into hmi master id {master_id}")
        except Exception as err:
            self.get_logger().error(f"error saving hmi status {err}")

    def save_hmi_iolink_data(self, master_id: int, msg: IOLinkData):
        try:
            master = self.masters[master_id]
            for register in master.hr:
                if value := getattr(msg, register.name, None):
                    self.get_logger().info(
                        f"📺 write HMI {master_id} register {register.address} value: {value}"
                    )
                    master.do_write(register.address, value)
            self.get_logger().info(f"complete write into hmi master id {master_id}")
        except Exception as err:
            self.get_logger().error(f"error saving iolink data {err}")


def main():
    try:
        rclpy.init()
        node = RelROSNode()
        rclpy.spin(node)
        rclpy.shutdown()

    except Exception as err:
        print(f"terminating ROS node {err}")


if __name__ == "__main__":
    main()
