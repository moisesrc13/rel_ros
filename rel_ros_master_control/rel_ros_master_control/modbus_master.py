import os
from typing import Union

import pymodbus.client as modbusClient
from pymodbus.framer import FramerType
from tenacity import retry, stop_after_attempt, wait_exponential

from rel_ros_master_control.logger import new_logger
from rel_ros_master_control.models.modbus_m import SlaveSerial, SlaveTCP

logger = new_logger(__name__)


def setup_sync_client(
    slave: SlaveTCP | SlaveSerial,
) -> Union[modbusClient.ModbusTcpClient, modbusClient.ModbusSerialClient]:
    """Run client setup."""
    logger.info("creating modbus master 👾 ...")
    try:
        host = slave.host
        port = slave.port
        if os.getenv("USE_TEST_MODBUS", "false").lower() in ["yes", "true"]:
            host = "0.0.0.0"
            port = 8844
        client = None
        if isinstance(slave, SlaveTCP):
            logger.info("Creating TCP master connecion to slave ⭐ %s", slave)
            client = modbusClient.ModbusTcpClient(
                host=host,
                port=port,
                framer=FramerType.SOCKET,
                timeout=slave.timeout_seconds,
            )
        elif isinstance(slave, SlaveSerial):
            logger.info("Creating Serial client 🕯️ on port %s", slave)
            client = modbusClient.ModbusSerialClient(
                port=slave.port,  # serial port
                # Common optional parameters:
                framer=FramerType.RTU,
                timeout=slave.timeout_seconds,
                baudrate=slave.baudrate,
                bytesize=slave.data_bit,
                parity=slave.parity,
                stopbits=slave.stop_bit,
            )
        else:
            logger.error("modbus master type not supported")
            raise RuntimeError("modbus master not supported")
        return client
    except Exception as err:
        logger.error("❌ Error creating modbus client - %s", err)
        raise err


class RelModbusMaster:
    def __init__(self, slave: SlaveTCP | SlaveSerial) -> None:
        logger.info("✨ Starting modbus master ...")
        self.slave_conn = setup_sync_client(slave)
        self.slave = slave

    def connection_state(self):
        if self.slave_conn.connected:
            return "connected 🍰"
        return "❌ error not connected"

    @retry(
        stop=stop_after_attempt(5),
        wait=wait_exponential(multiplier=2, max=10),
        reraise=True,
    )
    def do_connect(self):
        try:
            logger.info("connecting to modbus slave")
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
