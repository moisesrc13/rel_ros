import argparse
import concurrent.futures

from pydantic import BaseModel
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadBuilder, BinaryPayloadDecoder

from rel_ros_master_control.config import (
    load_hmi_config,
    load_modbus_config,
    load_status_device_config,
)
from rel_ros_master_control.constants import DigitalHydValve, DigitalOutput, HMIWriteAction
from rel_ros_master_control.logger import new_logger
from rel_ros_master_control.modbus_master import RelModbusMaster
from rel_ros_master_control.models.hmi_m import SlaveHMI
from rel_ros_master_control.models.modbus_m import (
    CRegister,
    HRegister,
    RegisterDataType,
    RegisterMode,
    SlaveIOLink,
    get_register_by_name,
)
from rel_ros_master_control.models.status_device_m import TowerState, TowerStatusDevice

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


class ModbusStatus(BaseModel):
    error: str = None
    status: str = ""
    value: int = 0


class RelControl:
    """Main class for control"""

    def __init__(
        self,
        iolink_slave: SlaveIOLink,
        iolink_hr: list[HRegister],
        hmi_slave: SlaveHMI,
        hmi_hr: list[HRegister],
        hmi_cr: list[CRegister],
    ) -> None:
        self.tower_devive = TowerStatusDevice(load_status_device_config())
        self.hyd_valve_io = DigitalHydValve()
        self.master_io_link = RelModbusMaster(iolink_slave)
        self.master_hmi = RelModbusMaster(hmi_slave)
        self.iolink_hr = iolink_hr
        self.hmi_hr = hmi_hr
        self.hmi_cr = hmi_cr
        self.hmi_id = iolink_slave.hmi_id
        logger.info("connecting master io_link")
        self.master_io_link.do_connect()
        logger.info("master_io_link connected .✨")

    def apply_state(self, hr: HRegister, state_value: int):
        try:
            self.master_io_link.slave_conn.write_register(
                self.get_register_with_offset(hr.address),
                state_value,
            )
        except Exception as err:
            logger.error("error writing state %s - %s", state_value, err)

    def apply_manifold_state(self, state_value: int):
        self.apply_state(
            get_register_by_name(self.iolink_hr, HMIWriteAction.ACTION_MANIFOLD.value), state_value
        )

    def apply_hyd_valve_state(self, state_value: int):
        self.apply_state(
            get_register_by_name(self.iolink_hr, DigitalOutput.DIGITAL_OUT_HYD_VALVE.value),
            state_value,
        )

    def apply_tower_state(self, state: TowerState):
        registers = []
        start_address = self.tower_devive.tower_status.start_address
        match state:
            case TowerState.FULL:
                registers = self.tower_devive.tower_status.states.full
            case TowerState.MEDIUM_HIGH:
                registers = self.tower_devive.tower_status.states.medium_high
            case TowerState.MEDIUM:
                registers = self.tower_devive.tower_status.states.medium
            case TowerState.PRE_VACUUM:
                registers = self.tower_devive.tower_status.states.pre_vacuum
            case TowerState.VACUUM:
                registers = self.tower_devive.tower_status.states.vacuum
            case TowerState.BUCKET_CHANGE:
                registers = self.tower_devive.tower_status.states.bucket_change
            case TowerState.ACOSTIC_ALARM_ON:
                start_address = self.tower_devive.tower_status.alarm_address
                registers = self.tower_devive.tower_status.states.acoustic_alarm_on
            case TowerState.ACOSTIC_ALARM_OFF:
                start_address = self.tower_devive.tower_status.alarm_address
                registers = self.tower_devive.tower_status.states.acoustic_alarm_off
            case _:
                logger.warning("no match state found for tower device - %s", state.value)
        if registers:
            try:
                self.master_io_link.slave_conn.write_registers(
                    start_address,
                    registers,
                )
            except Exception as err:
                logger.error("error writing tower state status %s - %s", state.value, err)

    def get_register_with_offset(self, register: int) -> int:
        register = register + self.master_io_link.slave.slave_tcp.offset
        logger.debug("register with offset %s", register)
        return register

    def write_holding_register(self, register: int, value: int) -> ModbusStatus:
        status = ModbusStatus()
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

    def eletrovalve_on(self):
        register = get_register_by_name(self.iolink_hr, "digital_out_hyd_valve")
        self.write_holding_register(register.address, 3)

    def eletrovalve_off(self):
        register = get_register_by_name(self.iolink_hr, "digital_out_hyd_valve")
        self.write_holding_register(register.address, 5)

    def read_holding_register(self, register: int) -> ModbusStatus:
        status = ModbusStatus()
        logger.info("reading register %s", register)
        response = self.master_io_link.slave_conn.read_holding_registers(
            address=self.get_register_with_offset(register), count=1
        )
        if response.isError():
            logger.error("error reading register")
            status.error = response
            return status
        logger.info("reading ok ✨ %s", response.registers)
        decoder = get_decoder(response)
        status.value = get_value(decoder)
        status.status = "read ok"
        return status

    def get_data_by_hr_name(self, register_name: str) -> int:
        register = get_register_by_name(self.iolink_hr, register_name)
        return self.read_holding_register(register.address).value

    def get_data(self) -> list[HRegister]:
        updated_registers = []

        def worker(register: HRegister):
            register.value = self.read_holding_register(register.address).value
            updated_registers.append(register)

        futures = []
        with concurrent.futures.ThreadPoolExecutor() as executor:
            for register in self.iolink_hr:
                if register.mode == RegisterMode.R:
                    futures.append(executor.submit(worker, register))
        if not futures:
            logger.warning("getting no data for read mode registers...")
            return futures
        concurrent.futures.wait(futures)
        logger.info(
            "getting data total %s from master %s", len(updated_registers), updated_registers
        )
        return updated_registers


def run_masters_to_iolinks(
    iolink_slaves: list[SlaveIOLink], hr: list[HRegister]
) -> list[RelControl]:
    masters = []
    for iolink_slave in iolink_slaves:
        masters.append(RelControl(iolink_slave=iolink_slave, iolink_hr=hr))
    logger.info("finish to run masters ...")
    return masters


def run():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-i",
        "--id",
        help="modbus master iolink id",
        dest="id",
        default=0,
        type=int,
    )
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
    logger.info("starting main control for master io link %s ...", args.id)
    iolink_config = load_modbus_config()
    hmi_config = load_hmi_config()
    control = RelControl(
        iolink_slave=iolink_config.iolinks[args.id],
        iolink_hr=iolink_config.holding_registers,
        hmi_slave=hmi_config.hmis[args.id],
        hmi_hr=hmi_config.holding_registers,
        hmi_cr=hmi_config.coil_registers,
    )
    if args.action == "write":
        control.write_holding_register(args.register, args.value)
    elif args.action == "read":
        value = -1
        if args.register == 0:
            value = control.get_data()
            logger.info("value size %s", len(value))
        else:
            value = control.read_holding_register(args.register)
        logger.info("read value %s", value)


# for testing outside ROS
if __name__ == "__main__":
    run()
