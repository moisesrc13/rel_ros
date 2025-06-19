import os
from typing import Union

import pymodbus.client as modbusClient
from pymodbus.framer import FramerType
from tenacity import retry, stop_after_attempt, wait_exponential

from rel_ros_master_control.logger import new_logger
from rel_ros_master_control.models.hmi_m import SlaveHMI
from rel_ros_master_control.models.modbus_m import SlaveIOLink, SlaveTCP

logger = new_logger(__name__)


def setup_sync_client(
    slave: SlaveIOLink | SlaveHMI,
) -> Union[modbusClient.ModbusTcpClient, modbusClient.ModbusSerialClient]:
    """Run client setup."""
    logger.info("creating modbus master 👾 ...")
    is_hmi = isinstance(slave, SlaveHMI)
    try:
        host = slave.slave_tcp.host
        port = slave.slave_tcp.port
        if not is_hmi and os.getenv("USE_TEST_MODBUS", "false").lower() in ["yes", "true"]:
            logger.info("connecting to test modbus slave")
            host = "0.0.0.0"
            port = 8844
        client = None
        if isinstance(slave.slave_tcp, SlaveTCP):
            logger.info(
                "Creating TCP master connection to slave on host %s port %s ⭐",
                host,
                port,
            )
            client = modbusClient.ModbusTcpClient(
                host=host,
                port=port,
                framer=FramerType.SOCKET,
                timeout=slave.slave_tcp.timeout_seconds,
            )
        else:
            logger.error("modbus master type not supported")
            raise RuntimeError("modbus master not supported")
        return client
    except Exception as err:
        logger.error("❌ Error creating modbus client - %s", err)
        raise err


class RelModbusMaster:
    def __init__(self, slave: SlaveIOLink | SlaveHMI) -> None:
        logger.info("✨ Starting modbus master to slave %s...", slave)
        self.slave_conn = setup_sync_client(slave)
        self.slave = slave
        self.hmi_id = slave.hmi_id
        self.hmi_name = slave.hmi_name
        self.connection_state = None

    def _connection_state(self):
        if self.slave_conn.connected:
            return "connected 🍰"
        return self.connection_state or "❌ error not connected"

    @retry(
        stop=stop_after_attempt(5),
        wait=wait_exponential(multiplier=2, max=10),
        reraise=True,
    )
    def do_connect(self):
        try:
            logger.info("connecting to modbus slave %s", self.slave_conn)
            assert self.slave_conn.connect()
            logger.info(
                "modbus slave connected 🤘 is socked opened %s, transport %s",
                self.slave_conn.is_socket_open(),
                self.slave_conn.transport,
            )
            logger.info("modbus socket %s", self.slave_conn.socket)
        except Exception as err:
            logger.error("❌ Error connecting to modbus slave - %s", err)
            raise err

    def do_close(self):
        try:
            logger.info("closing connection to modbus slave")
            self.slave_conn.close()
            logger.info("slave connection closed")
            self.connection_state = "closed"
        except Exception as err:
            logger.error("❌ Error closing connection to modbus server - %s", err)
            self.connection_state = "❌ error closing"
