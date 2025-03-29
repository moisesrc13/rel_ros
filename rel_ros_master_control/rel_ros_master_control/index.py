import functools
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from rel_interfaces.msg import HMI, IOLinkData
from rel_ros_master_control.config import load_modbus_config
from rel_ros_master_control.control import RelControl, run_masters_to_iolinks


@dataclass
class HMIData:
    hmi_id: int = 0
    hmi: HMI = None


@dataclass
class ControlIOLinkData:
    data: IOLinkData
    hmi_id: int = 0


def create_hmi_cluster(size: int) -> list[HMIData]:
    """
    used to get data from sensors and user input in the HMI.
    This data will be used for the control logic
    """
    cluster = []
    for n in range(size):
        cluster.append(HMIData(hmi_id=n))
    return cluster


def get_hmi_from_cluster_with_id(cluster: list[HMIData], hmi_id: int) -> Optional[HMIData]:
    return next((hmid for hmid in cluster if hmid.hmi_id == hmi_id), None)


class RelROSNode(Node):
    def __init__(self):
        super().__init__("rel_ros_master_control_node")
        self.config = load_modbus_config()
        self.get_logger().info("creating Relant master control 🚀...")
        self.masters = run_masters_to_iolinks(
            slaves=self.config.iolinks, hr=self.config.holding_registers
        )
        self.create_timers_for_iolink_masters()
        self.hmi_cluster = create_hmi_cluster(size=len(self.masters))
        self.get_logger().info("creating subscriber for rel/hmi topics 📨 ...")
        self.create_hmi_subscribers(len(self.masters))
        self.get_logger().info("creating publisher for rel/iolink topic 📨 ...")
        self.rel_publisher = self.create_publisher(IOLinkData, "rel/iolink", 10)
        self.control_iolink_data = {}

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
            return
        self.get_logger().info("creating timers ⏱ ...")
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
        self.get_logger().info(f"📨 I got an HMI {hmi_id} message 📺 {msg}")
        hmiData = get_hmi_from_cluster_with_id(self.hmi_cluster, msg.hmi_id)
        hmiData.hmi = msg
        self.hmi_cluster[msg.hmi_id] = hmiData

    def get_io_link_data(self, hmi_id: int = 0):
        master: RelControl = self.masters[hmi_id]
        msg = IOLinkData()
        msg.hmi_name = master.master_io_link.slave.hmi_name
        msg.hmi_id = master.master_io_link.slave.hmi_id
        registers = master.get_data()
        for reg in registers:
            setattr(msg, reg.name, reg.value)

        self.control_iolink_data[hmi_id] = ControlIOLinkData(data=msg, hmi_id=hmi_id)
        self.rel_publisher.publish(msg)
        self.get_logger().info(f"📨 Publishing IOLinkData message: {msg}")

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
