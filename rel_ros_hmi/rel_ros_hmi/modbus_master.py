import argparse
from typing import Union

import pymodbus.client as modbusClient
from pymodbus.constants import Endian
from pymodbus.framer import FramerType
from pymodbus.payload import BinaryPayloadDecoder
from tenacity import retry, stop_after_attempt, wait_exponential

from rel_ros_hmi.config import load_modbus_config
from rel_ros_hmi.logger import new_logger
from rel_ros_hmi.models.modbus_m import (
    Register,
    RegisterDataType,
    SlaveTCP,
    get_hr_addresses,
    get_register_by_address,
)

logger = new_logger(__name__)


def get_decoder_from_rr(rr) -> BinaryPayloadDecoder:
    return BinaryPayloadDecoder.fromRegisters(rr, wordorder=Endian.LITTLE, byteorder=Endian.BIG)


def get_decoder(response) -> BinaryPayloadDecoder:
    logger.debug("calling decoder with response %s", response)
    return get_decoder_from_rr(response.registers)


def get_value(decoder: BinaryPayloadDecoder, data_type: RegisterDataType) -> int | float:
    match data_type:
        case RegisterDataType.float16:
            return decoder.decode_16bit_float()
        case RegisterDataType.float32:
            return decoder.decode_32bit_float()
        case RegisterDataType.uint16:
            return decoder.decode_16bit_uint()
        case RegisterDataType.uint32:
            return decoder.decode_32bit_uint()
        case _:
            return -1


def setup_sync_client(
    slave: SlaveTCP,
) -> Union[modbusClient.ModbusTcpClient, modbusClient.ModbusSerialClient]:
    """Run client setup."""
    logger.info("creating modbus master for slave %s 👾 ...", slave.id)
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
    def __init__(self, slave: SlaveTCP, hr: list[Register]) -> None:
        logger.info("✨ Starting modbus master ...")
        self.slave_conn = setup_sync_client(slave)
        self.slave = slave
        self.hr = hr

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
        logger.info("writing to register %s value %s", address, value)
        respose = self.slave_conn.write_register(address=address, value=value)
        if respose.isError():
            logger.error("error writing register %s", address)
        else:
            logger.info("register written ok ✨")

    def do_read(self, register: int) -> int:
        logger.info("reading register %s", register)
        rr = self.slave_conn.read_holding_registers(address=register, count=1)
        if rr.isError():
            logger.error("error reading register")
            return
        logger.info("reading ok ✨ %s", rr.registers)
        decoder = get_decoder(rr)
        return get_value(decoder, RegisterDataType.uint16)

    def get_holding_registers_data(self) -> list[Register]:
        logger.info("reading holding register data")
        addresses = get_hr_addresses(self.hr)
        rr = self.slave_conn.read_holding_registers(address=addresses[0], count=len(addresses))
        decoder = get_decoder(rr)
        updated_registers = []
        for addr in addresses:
            register, _ = get_register_by_address(self.hr, addr)
            register.value = get_value(decoder, RegisterDataType.uint16)
            updated_registers.append(register)
        return updated_registers


def create_masters_for_hmis(slaves: list[SlaveTCP], hr: list[Register]) -> list[RelModbusMaster]:
    return [RelModbusMaster(s, hr) for s in slaves]


def run():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-a",
        "--action",
        choices=["read", "write"],
        help="modbus action",
        dest="action",
        default="write",
        type=str,
    )
    parser.add_argument(
        "-r",
        "--register",
        help="register address int value",
        dest="register",
        type=int,
    )
    parser.add_argument(
        "-v",
        "--value",
        help="register write value",
        dest="value",
        type=int,
    )
    args = parser.parse_args()
    config = load_modbus_config()
    modbus_master = RelModbusMaster(config.slaves[0], config.holding_registers)
    modbus_master.do_connect()
    if args.action == "write":
        modbus_master.do_write(args.register, args.value)
    elif args.action == "read":
        value = -1
        if args.register == 0:
            value = modbus_master.get_holding_registers_data()
        else:
            value = modbus_master.do_read(args.register)
        logger.info("read value %s", value)


if __name__ == "__main__":
    run()
