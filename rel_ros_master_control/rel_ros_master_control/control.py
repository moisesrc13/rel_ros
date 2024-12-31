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
    status: str = ""
    value: int = 0


class RelControl:
    def __init__(self) -> None:
        logger.info("starting main control...")
        config = load_modbus_config()
        self.master_io_link: RelModbusMaster = get_master(
            config.slaves, "master_io_link"
        )  # this is open to work with other masters in the future
        self.hr = config.holding_registers
        logger.info("connecting master io_link")
        self.master_io_link.do_connect()
        logger.info("master_io_link connected .✨")

    def get_register_with_offset(self, register: int) -> int:
        register = register + self.master_io_link.slave.offset
        logger.debug("register with offset %s", register)
        return register

    def read_device_port_input_status(self, port: str) -> int:
        port: DevicePort = getattr(self.master_io_link.slave.device_ports, port)
        logger.debug(
            "port_num %s status_register %s", port, port.holding_registers.data_input_status
        )
        rr = self.master_io_link.slave_conn.read_holding_registers(
            address=self.get_register_with_offset(port.holding_registers.data_input_status.address),
            count=port.holding_registers.data_input_status.words,
        )
        decoder = get_decoder(rr)
        return get_value(decoder, port.holding_registers.data_input_status.data_type)

    def read_device_port_register(self, register: Register) -> int:
        logger.debug("read device register %s", register)
        rr = self.master_io_link.slave_conn.read_holding_registers(
            address=self.get_register_with_offset(register.address),
            count=register.words,
        )
        decoder = get_decoder(rr)
        return get_value(decoder, register.data_type)

    def write_device_port_register(self, register: Register, value: int) -> int:
        logger.debug("write device register %s", register)
        response = self.master_io_link.slave_conn.write_register(
            address=self.get_register_with_offset(register.address), value=value
        )
        if response.isError():
            logger.error("error writing register")
            return
        logger.info("writing ok ✨")

    def write_holding_register(self, register: int, value: int) -> ControlStatus:
        status = ControlStatus()
        logger.info("writing to register %s value %s", register, value)
        response = self.master_io_link.slave_conn.write_register(
            address=self.get_register_with_offset(register), value=value
        )
        if response.isError():
            logger.error("error writing register")
            status.error = response
            return status
        logger.info("writing ok ✨")
        status.status = "write ok"
        status.value = value
        return status

    def read_holding_register(self, register: int) -> ControlStatus:
        status = ControlStatus()
        logger.info("reading register %s", register)
        response = self.master_io_link.slave_conn.read_holding_registers(
            address=self.get_register_with_offset(register), count=1
        )
        if response.isError():
            logger.error("error reading register")
            status.error = response
        logger.info("reading ok ✨ %s", response.registers)
        decoder = get_decoder(response)
        status.value = get_value(decoder)
        status.status = "read ok"
        return status

    def get_data(self) -> list[Register]:
        updated_registers = []
        for register in self.hr:
            register.value = self.read_holding_register(register.address)
            updated_registers.append(register)
        return updated_registers


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
        value = -1
        if args.register == 0:
            value = control.get_data()
        else:
            value = control.read_holding_register(args.register)
        logger.info("read value %s", value)


# for testing outside ROS
if __name__ == "__main__":
    run()
