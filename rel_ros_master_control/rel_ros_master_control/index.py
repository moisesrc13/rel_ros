import functools
import os
from typing import Optional

import rclpy
from pydantic import BaseModel
from rclpy.node import Node
from rclpy_message_converter.message_converter import convert_ros_message_to_dictionary

from rel_interfaces.msg import HMI, HMIStatus, IOLinkData
from rel_ros_master_control.config import load_modbus_config
from rel_ros_master_control.control import RelControl, run_masters_to_iolinks
from rel_ros_master_control.flow_control import FlowControlConfig, FlowControlInputs
from rel_ros_master_control.flow_control import run as run_control


class ControlHMIData(BaseModel):
    hmi_id: int = 0
    data: Optional[HMI] = None


class ControlIOLinkData(BaseModel):
    hmi_id: int = 0
    data: Optional[IOLinkData] = None


class ControlNode(BaseModel):
    hmi_data: ControlHMIData
    iolink_data: ControlIOLinkData


def create_control_cluster(size: int) -> list[ControlNode]:
    """
    used to get data from sensors and user input in the HMI.
    This data will be used for the control logic
    """
    cluster = []
    for n in range(size):
        control_node = ControlNode(
            hmi_data=ControlHMIData(hmi_id=n), iolink_data=ControlIOLinkData(hmi_id=n)
        )
        cluster.append(control_node)
    return cluster


def get_control_node_with_id(cluster: list[ControlNode], hmi_id: int) -> Optional[ControlNode]:
    return next((node for node in cluster if node.hmi_data.hmi_id == hmi_id), None)


class RelROSNode(Node):
    def __init__(self):
        super().__init__("rel_ros_master_control_node")
        self.config = load_modbus_config()
        self.get_logger().info("creating Relant master control 🚀...")
        self.masters = run_masters_to_iolinks(
            iolink_slaves=self.config.iolinks, hr=self.config.holding_registers
        )
        self.create_timers_for_iolink_masters()
        self.control_cluster = create_control_cluster(size=len(self.masters))
        self.get_logger().info("creating subscriber for rel/hmi topics 📨 ...")
        self.create_hmi_subscribers(len(self.masters))
        self.get_logger().info("creating publisher for rel/iolink topic 📨 ...")
        self.iolink_publisher = self.create_publisher(IOLinkData, "rel/iolink", 10)
        self.get_logger().info("creating publisher for rel/hmistatus topic 📨 ...")
        self.hmi_status_publisher = self.create_publisher(HMIStatus, "rel/hmistatus", 10)

        self.get_logger().info("======= creating MAIN CONTROL timers 🤖 =======")
        self.create_timers_for_main_control()

    def create_timers_for_main_control(self):
        if not self.masters:
            self.get_logger().error("no iolink masters available")
            return
        self.get_logger().info("creating main control timers ⏱ ...")
        for master in self.masters:
            if isinstance(master, RelControl):
                self.get_logger().info(
                    f"creating timer for main control with hmi id {master.master_io_link.hmi_id}"
                )
                self.create_timer(
                    0.5,
                    functools.partial(
                        self.timer_callback_main_control, hmi_id=master.master_io_link.hmi_id
                    ),
                )

    """ Main control function """

    def timer_callback_main_control(self, hmi_id: int = 0):
        if not (os.getenv("ENABLE_CONTROL", "true").lower() in ["true", "yes"]):
            return
        self.get_logger().info(f"🎮 starting main control for node id {hmi_id}")
        node: ControlNode = get_control_node_with_id(self.control_cluster, hmi_id)
        control_iolink_data = convert_ros_message_to_dictionary(node.iolink_data.data)
        control_hmi_data = convert_ros_message_to_dictionary(node.iolink_data.data)
        master_control: RelControl = self.masters[hmi_id]
        inputs = FlowControlInputs(
            master_control=master_control,
            control_hmi_data=control_hmi_data,
            control_iolink_data=control_iolink_data,
            hmi_status_publisher=self.hmi_status_publisher,
        )
        run_control(
            FlowControlConfig(
                inputs=inputs,
            )
        )

    def create_hmi_subscribers(self, count: int = 1):
        for s in range(count):
            topic = f"rel/hmi_{s}"
            self.get_logger().info(f"creating hmi subscriber for {topic} topic 📨")
            self.create_subscription(
                HMI, topic, functools.partial(self.listener_hmi_callback, hmi_id=s), 10
            )
        self.get_logger().info("creating hmi consumers is done ...")

    def create_timers_for_iolink_masters(self):
        if not self.masters:
            self.get_logger().error("no iolink masters available")
            raise RuntimeError("no iolink masters available")

        self.get_logger().info("creating iolink data timers ⏱ ...")
        for master in self.masters:
            if isinstance(master, RelControl):
                self.get_logger().info(
                    f"creating timer for iolink master with hmi id {master.master_io_link.hmi_id}"
                )
                self.create_timer(
                    0.5,
                    functools.partial(
                        self.timer_callback_iolink_data, hmi_id=master.master_io_link.hmi_id
                    ),
                )

    def listener_hmi_callback(self, msg: HMI, hmi_id: int = 0):
        self.get_logger().info(f"📨 I got an HMI {hmi_id} data message 📺 {msg}")
        node: ControlNode = get_control_node_with_id(self.control_cluster, msg.hmi_id)
        node.hmi_data.data = msg
        # self.control_cluster[msg.hmi_id] = node  # may not be required

    def get_io_link_data(self, hmi_id: int = 0):
        master: RelControl = self.masters[hmi_id]
        msg = IOLinkData()
        msg.hmi_name = master.master_io_link.slave.hmi_name
        msg.hmi_id = master.master_io_link.slave.hmi_id
        registers = master.get_data()
        for reg in registers:
            setattr(msg, reg.name, reg.value)

        node: ControlNode = get_control_node_with_id(self.control_cluster, msg.hmi_id)
        node.iolink_data = msg
        # self.control_cluster[msg.hmi_id] = node  # may not be required
        self.get_logger().info(f"📨 Publishing IOLinkData message: {msg}")
        self.iolink_publisher.publish(msg)

    def timer_callback_iolink_test(self, hmi_id: int = 0):
        self.get_logger().info(f"test hmi_id {hmi_id}")

    def timer_callback_iolink_data(self, hmi_id: int = 0):
        self.get_logger().info("Relant ROS2 Master Control 🤖 - get iolink data 🤘 ...")
        self.get_logger().info(f"hmi_id {hmi_id}")
        self.get_io_link_data(hmi_id)


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
