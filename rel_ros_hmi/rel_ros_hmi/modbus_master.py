from typing import Union

import pymodbus.client as modbusClient
from pymodbus.framer import FramerType
from tenacity import retry, stop_after_attempt, wait_exponential

from rel_ros_hmi.config import load_modbus_config
from rel_ros_hmi.logger import new_logger
from rel_ros_hmi.models.modbus_m import SlaveTCP

logger = new_logger(__name__)


def setup_sync_client(
    slave: SlaveTCP,
) -> Union[modbusClient.ModbusTcpClient, modbusClient.ModbusSerialClient]:
    """Run client setup."""
    logger.info("creating modbus master 👾 ...")
    try:
        client = None
        if isinstance(slave, SlaveTCP):
            logger.info(
                "Creating TCP master connecion to slave on host %s port %s ⭐",
                slave.host,
                slave.port,
            )
            client = modbusClient.ModbusTcpClient(
                host=slave.host,
                port=slave.port,
                framer=FramerType.SOCKET,
                timeout=slave.timeout_seconds,
            )
        else:
            logger.error("modbus master type not supported")
            raise RuntimeError("modbus master not supported")
        return client
    except Exception as err:
        logger.error("❌ Error creating modbus client - %s", err)
        raise err


class RelModbusMaster:
    def __init__(self, slave: SlaveTCP) -> None:
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

    def do_write(self, address: int, value: int):
        respose = self.slave_conn.write_register(address=address, value=value)
        if respose.isError():
            logger.error("error writing register %s", address)
        else:
            logger.info("register written ok ✨")


def run():
    config = load_modbus_config()
    modbus_master = RelModbusMaster(config.modbus)
    modbus_master.do_connect()
    modbus_master.do_write(40001, 1010)


if __name__ == "__main__":
    run()
