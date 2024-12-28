import argparse

from pydantic import BaseModel
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadBuilder, BinaryPayloadDecoder

from rel_ros_master_control.config import load_modbus_config
from rel_ros_master_control.logger import new_logger
from rel_ros_master_control.modbus_master import RelModbusMaster
from rel_ros_master_control.models.modbus_m import DevicePort, Register, RegisterDataType
from rel_ros_master_control.util import get_master

logger = new_logger(__name__)


def get_builder():
    return BinaryPayloadBuilder(wordorder=Endian.LITTLE, byteorder=Endian.BIG)  # this is for CDAB


def get_decoder_from_rr(rr) -> BinaryPayloadDecoder:
    return BinaryPayloadDecoder.fromRegisters(rr, wordorder=Endian.LITTLE, byteorder=Endian.BIG)


def get_decoder(response) -> BinaryPayloadDecoder:
    logger.debug("calling decoder with response %s", response)
    return get_decoder_from_rr(response.registers)


def get_value(
    decoder: BinaryPayloadDecoder, data_type: RegisterDataType = RegisterDataType.uint16
) -> int | float:
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


class ControlStatus(BaseModel):
    error: str = None
    status: str = "ok"
    value: int = -1


class RelControl:
    def __init__(self) -> None:
        logger.info("starting main control...")
        self.master_io_link: RelModbusMaster = get_master(
            load_modbus_config().slaves, "master_io_link"
        )  # this is open to work with other masters in the future
        logger.info("connecting master io_link")
        self.master_io_link.do_connect()
        logger.info("master_io_link connected .✨")

    def read_device_port_input_status(self, port: str) -> int:
        port: DevicePort = getattr(self.master_io_link.slave.device_ports, port)
        logger.debug(
            "port_num %s status_register %s", port, port.holding_registers.data_input_status
        )
        rr = self.master_io_link.slave_conn.read_holding_registers(
            address=port.holding_registers.data_input_status.address,
            count=port.holding_registers.data_input_status.words,
        )
        decoder = get_decoder(rr)
        return get_value(decoder, port.holding_registers.data_input_status.data_type)

    def read_device_port_register(self, register: Register) -> int:
        logger.debug("read device register %s", register)
        rr = self.master_io_link.slave_conn.read_holding_registers(
            address=register.address,
            count=register.words,
        )
        decoder = get_decoder(rr)
        return get_value(decoder, register.data_type)

    def write_device_port_register(self, register: Register, value: int) -> int:
        logger.debug("write device register %s", register)
        response = self.master_io_link.slave_conn.write_register(
            address=register.address, value=value
        )
        if response.isError():
            logger.error("error writing register")
            return
        logger.info("writing ok ✨")

    def write_holding_register(self, register_addr: int, value: int):
        logger.info("writing to register %s value %s", register_addr, value)
        response = self.master_io_link.slave_conn.write_register(address=register_addr, value=value)
        if response.isError():
            logger.error("error writing register")
            return
        logger.info("writing ok ✨")

    def read_holding_register(self, register: int) -> ControlStatus:
        logger.info("reading register %s", register)
        response = self.master_io_link.slave_conn.read_holding_registers(address=register, count=1)
        status = ControlStatus()
        if response.isError():
            logger.error("error reading register")
            status.error = response
        logger.info("reading ok ✨ %s", response.registers)
        decoder = get_decoder(response)
        status.value = get_value(decoder)
        status.status = "read ok"
        return status


def run():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-a",
        "--action",
        choices=["read", "write"],
        help="modbus action",
        dest="action",
        default="read",
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
    control = RelControl()
    if args.action == "write":
        control.write_holding_register(args.register, args.value)
    elif args.action == "read":
        control.read_holding_register(args.register)


# for testing outside ROS
if __name__ == "__main__":
    run()
