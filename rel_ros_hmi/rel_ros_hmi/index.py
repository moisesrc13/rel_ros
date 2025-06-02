from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node

from rel_interfaces.msg import HMIAction
from rel_ros_hmi.config import load_modbus_config
from rel_ros_hmi.modbus_master import create_masters_for_hmis
from rel_ros_hmi.modbus_slave import run_modbus_slaves
from rel_ros_hmi.models.modbus_m import get_register_by_name


class HMIActionName(Enum):
    WRITE = "write"
    READ = "read"
    NONE = "none"


class RelROSNode(Node):
    def __init__(self):
        super().__init__("rel_ros_hmi_node")
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

    def listener_hmi_action_callback(self, msg: HMIAction):
        self.get_logger().info(f"📨 I got an HMIAction message {msg}")
        self.do_hmi_action(msg.hmi_id, msg)

    def do_hmi_action(self, master_id: int, msg: HMIAction):
        def get_action_name() -> Optional[HMIActionName]:
            try:
                return HMIActionName(msg.action)
            except Exception as ex:
                self.get_logger().error(f"error getting hmi action for message {msg} - {ex}")
                return None

        try:
            master = self.masters[master_id]
            action_name = get_action_name()
            if not action_name:
                self.get_logger().info(f"No valid action from HMI action message {msg}")
                return

            match action_name:
                case HMIActionName.READ:
                    self.get_logger().info("read HMI action")
                    master.do_read(msg.register_address)
                case HMIActionName.WRITE:
                    self.get_logger().info("write HMI action")
                    master.do_write(msg.register_address, msg.value)
                case _:
                    self.get_logger().info("do nothing HMI action")
                    return
        except Exception as err:
            self.get_logger().error(f"error saving hmi status {err}")


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
