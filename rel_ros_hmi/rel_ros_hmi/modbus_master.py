import argparse
import os
from typing import Union

import pymodbus.client as modbusClient
from pymodbus.constants import Endian
from pymodbus.framer import FramerType
from pymodbus.payload import BinaryPayloadDecoder
from tenacity import retry, stop_after_attempt, wait_exponential

from rel_ros_hmi.config import load_modbus_config
from rel_ros_hmi.logger import new_logger
from rel_ros_hmi.models.modbus_m import (
    CRegister,
    HRegister,
    RegisterDataType,
    RegisterModbusType,
    SlaveHMI,
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
    slave: SlaveHMI,
) -> Union[modbusClient.ModbusTcpClient, modbusClient.ModbusSerialClient]:
    """Run client setup."""
    logger.info("creating modbus master for HMI slave %s 👾 ...", slave.hmi_id)
    try:
        host = slave.slave_tcp.host
        port = slave.slave_tcp.port
        if os.getenv("USE_TEST_MODBUS", "false").lower() in ["yes", "true"]:
            logger.info("connecting to test modbus slave")
            host = "0.0.0.0"
            port = 8845
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
    def __init__(self, slave: SlaveHMI, hr: list[HRegister], coils: list[CRegister]) -> None:
        logger.info("✨ Starting modbus HMI master ...")
        self.hmi_name = slave.name
        self.hmi_id = slave.hmi_id
        self.slave_conn = setup_sync_client(slave)
        self.slave = slave
        self.hr = hr
        self.coils = coils

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
            logger.info("connecting to modbus HMI slave")
            assert self.slave_conn.connect()
            logger.info(
                "modbus HMI slave connected 🤘 is socked opened %s, transport %s",
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

    def do_write(self, address: int, value: int, reg_type=RegisterModbusType.HR):
        def _write():
            if reg_type == RegisterModbusType.HR:
                return self.slave_conn.write_register(address=address, value=value)
            else:
                return self.slave_conn.write_coil(address=address, value=value)

        logger.info("writing %s to register %s value %s", reg_type, address, value)
        respose = _write()
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

    def get_holding_registers_data(self) -> list[HRegister]:
        addresses = get_hr_addresses(self.hr)
        logger.info("reading holding register data, addresses %s", addresses)
        logger.info("reading total records %s", len(addresses))
        rr = self.slave_conn.read_holding_registers(
            address=addresses[0], count=len(addresses)
        )  # start with first address
        logger.info("results %s", rr)
        decoder = get_decoder(rr)
        updated_registers = []
        for addr in addresses:
            register, _ = get_register_by_address(self.hr, addr)
            register.value = get_value(decoder, register.data_type)
            updated_registers.append(register)
        return updated_registers


def create_masters_for_hmis(
    slaves: list[SlaveTCP], hr: list[HRegister], coils: list[CRegister]
) -> list[RelModbusMaster]:
    return [RelModbusMaster(s, hr, coils) for s in slaves]


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
    parser.add_argument(
        "-t",
        "--type",
        help="register type",
        dest="type",
        type=str,
        choices=["hr", "co"],
        default="hr",
    )
    args = parser.parse_args()
    config = load_modbus_config()
    modbus_master = RelModbusMaster(
        config.slaves[0], config.holding_registers, config.coil_registers
    )
    modbus_master.do_connect()
    if args.action == "write":
        modbus_master.do_write(args.register, args.value, RegisterModbusType(args.type))
    elif args.action == "read":
        value = -1
        if args.register == 0:
            value = modbus_master.get_holding_registers_data()  # read all data
        else:
            value = modbus_master.do_read(args.register)
        logger.info("read value %s", value)


if __name__ == "__main__":
    run()
