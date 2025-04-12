import argparse
import concurrent.futures
from enum import Enum

from pydantic import BaseModel
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadBuilder, BinaryPayloadDecoder

from rel_ros_master_control.config import (
    load_hmi_config,
    load_iolink_config,
    load_status_device_config,
)
from rel_ros_master_control.constants import (
    DigitalHydValve,
    DigitalOutput,
    HMIWriteAction,
    Params,
    PressureSet,
    PressureState,
    PWMPulseSet,
)
from rel_ros_master_control.logger import new_logger
from rel_ros_master_control.modbus_master import RelModbusMaster
from rel_ros_master_control.models.hmi_m import SlaveHMI
from rel_ros_master_control.models.modbus_m import (
    CRegister,
    HRegister,
    RegisterDataType,
    RegisterMode,
    SlaveIOLink,
    get_register_by_address,
    get_register_by_name,
)
from rel_ros_master_control.models.pwm_m import PWMConfig
from rel_ros_master_control.models.status_device_m import TowerState, TowerStatusDevice

logger = new_logger(__name__)
try:
    from rel_ros_master_control.services.pwm import RelPWM
except Exception as err:
    logger.warning("expected error if not running on RPi - %s", err)


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


class SlaveType(Enum):
    IOLINK = "iolink"
    HMI = "hmi"


class RegisterType(Enum):
    HOLDING = "holiding"
    COIL = "coil"


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
        self.iolink_hr = iolink_hr
        self.hmi_hr = hmi_hr
        self.hmi_cr = hmi_cr
        self.hmi_id = iolink_slave.hmi_id
        self.init_control(iolink_slave, hmi_slave)

    def init_control(
        self,
        iolink_slave: SlaveIOLink,
        hmi_slave: SlaveHMI,
    ):
        self.tower_devive = TowerStatusDevice(load_status_device_config())
        self.hyd_valve_io = DigitalHydValve()
        self.master_io_link = RelModbusMaster(iolink_slave)
        self.master_hmi = RelModbusMaster(hmi_slave)
        logger.info("connecting master io_link")
        self.master_io_link.do_connect()
        logger.info("master_io_link connected ✨")
        logger.info("connecting master HMI")
        self.master_hmi.do_connect()
        logger.info("master_HMI connected ✨")
        logger.info("creating PWM")
        try:
            self.pwm = RelPWM(PWMConfig())
            logger.info("PWM set ✨")
        except Exception as err:
            logger.warning("error creating pwm service - %s", err)

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

    def get_register_with_offset(self, register: int, stype: SlaveType = SlaveType.IOLINK) -> int:
        if stype == SlaveType.IOLINK:
            offset_value = self.master_io_link.slave.slave_tcp.offset
        else:
            offset_value = self.master_hmi.slave.slave_tcp.offset
        register = register + offset_value
        logger.debug("register with offset %s", register)
        return register

    def write_iolink_hregister(self, register: int, value: int) -> ModbusStatus:
        return self.write_register(register, value, SlaveType.IOLINK)

    def eletrovalve_on(self):
        register = get_register_by_name(self.iolink_hr, "digital_out_hyd_valve")
        self.write_iolink_hregister(register.address, 3)

    def eletrovalve_off(self):
        register = get_register_by_name(self.iolink_hr, "digital_out_hyd_valve")
        self.write_iolink_hregister(register.address, 5)

    def get_master_connection(self, stype: SlaveType) -> RelModbusMaster:
        if stype == SlaveType.IOLINK:
            return self.master_io_link
        return self.master_hmi

    def read_hregister(self, register: int, stype: SlaveType = SlaveType.IOLINK) -> ModbusStatus:
        status = ModbusStatus()
        logger.info("reading register %s", register)
        master = self.get_master_connection(stype)
        response = master.slave_conn.read_holding_registers(
            address=self.get_register_with_offset(register, stype), count=1
        )
        if response.isError():
            logger.error("error reading register on %s", stype)
            status.error = response
            return status
        logger.info("reading ok ✨ %s - %s", response.registers, stype)
        decoder = get_decoder(response)
        status.value = get_value(decoder)
        status.status = f"read ok {stype.value}"
        return status

    def read_hmi_cregister(self, register: int) -> ModbusStatus:
        status = ModbusStatus()
        logger.info("reading coil register %s", register)
        master = self.get_master_connection(SlaveType.HMI)
        response = master.slave_conn.read_coils(
            address=self.get_register_with_offset(register, SlaveType.HMI), count=1
        )
        if response.isError():
            logger.error("error reading hmi coil register on")
            status.error = response
            return status
        logger.info("reading coil ok ✨ %s", response.registers)
        status.value = int(response.bits[0])
        status.status = "read coil ok"
        return status

    def read_hmi_cregister_by_name(self, name: str) -> ModbusStatus:
        register = get_register_by_name(self.hmi_cr, name)
        return self.read_hmi_cregister(register.address)

    def apply_pressure_state(self, state: PressureState):
        self.write_register_by_address_name(
            name=PressureSet.REGULATOR_ACTIVATE_VALVE.value,
            stype=SlaveType.IOLINK,
            rtype=RegisterType.HOLDING,
            value=state.value,
        )

    def apply_pwm_state(self):
        option_register = get_register_by_name(
            self.hmi_hr, Params.PARAM_PULSE_TRAIN_SELECTION.value
        )
        option = self.read_hmi_register(option_register.address)
        register_name = Params.PARAM_PULSE_TRAIN_LOW
        match option:
            case PWMPulseSet.HIGH.value:
                register_name = Params.PARAM_PULSE_TRAIN_HIGH
            case PWMPulseSet.MEDIUM.value:
                register_name = Params.PARAM_PULSE_TRAIN_MEDIUM
            case _:
                register_name = Params.PARAM_PULSE_TRAIN_LOW
        register = get_register_by_name(self.hmi_hr, register_name.value)
        pulse_value = self.read_hmi_register(register.address)
        self.pwm.run(time_seconds=10, duty=pulse_value)  # TODO define time

    def write_register_by_address_name(
        self,
        name: str,
        value: int,
        stype: SlaveType = SlaveType.IOLINK,
        rtype: RegisterType = RegisterType.HOLDING,
    ) -> ModbusStatus:
        registers = self.iolink_hr  # no coils for IOLINK
        if stype == SlaveType.HMI and rtype == RegisterType.HOLDING:
            registers = self.hmi_hr
        elif stype == SlaveType.HMI and rtype == RegisterType.COIL:
            registers = self.hmi_cr
        register = get_register_by_name(registers, name)
        return self.write_register(register=register.address, value=value, rtype=rtype, stype=stype)

    def write_register(
        self,
        register: int,
        value: int,
        stype: SlaveType = SlaveType.IOLINK,
        rtype: RegisterType = RegisterType.HOLDING,
    ) -> ModbusStatus:
        status = ModbusStatus()
        master = self.get_master_connection(stype)
        logger.info("writing to %s register %s value %s", rtype, register, value)
        if rtype == RegisterType.HOLDING:
            response = master.slave_conn.write_register(
                address=self.get_register_with_offset(register, stype), value=value
            )
        else:
            logger.info("writing coil with %s", bool(value))
            response = master.slave_conn.write_coil(
                address=self.get_register_with_offset(register, stype), value=bool(value)
            )
        if response.isError():
            logger.error("error writing register on %s", stype)
            status.error = response
            return status
        logger.info("writing ok ✨ %s", stype)
        status.status = f"write ok {stype.value}"
        status.value = value
        return status

    def read_iolink_hregister(self, register: int) -> ModbusStatus:
        return self.read_hregister(register, SlaveType.IOLINK)

    def read_hmi_register(
        self, register: int, rtype: RegisterType = RegisterType.HOLDING
    ) -> ModbusStatus:
        if rtype == RegisterType.HOLDING:
            return self.read_hregister(register, SlaveType.HMI)
        return self.read_hmi_cregister(register)

    def get_iolink_data_by_hr_name(self, register_name: str) -> int:
        register = get_register_by_name(self.iolink_hr, register_name)
        return self.read_iolink_hregister(register.address).value

    def get_iolink_hr_data(self) -> list[HRegister]:
        updated_registers = []

        def worker(register: HRegister):
            register.value = self.read_iolink_hregister(register.address).value
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

    def get_hmi_hr_data(self) -> dict:
        master = self.get_master_connection(SlaveType.HMI)
        addresses = [r.address for r in self.hmi_hr]
        addresses.sort()
        logger.info(
            "reading HMI %s holding register data, addresses %s", master.slave_conn, addresses
        )
        logger.info("reading total records %s", len(addresses))
        registers_data = {}
        try:
            rr = master.slave_conn.read_holding_registers(
                address=addresses[0], count=len(addresses)
            )  # start with first address and request the total
            logger.info("results %s", rr)
            decoder = get_decoder(rr)
            for addr in addresses:
                register, _ = get_register_by_address(self.hmi_hr, addr)
                if not register:
                    continue
                registers_data[register.name] = get_value(decoder, register.data_type)
            return registers_data
        except Exception as err:
            logger.error("Error getting hmi holding registers data - %s", err)
            return {}

    def get_hmi_cr_data(self) -> dict:
        master = self.get_master_connection(SlaveType.HMI)
        addresses = [r.address for r in self.hmi_cr]
        addresses.sort()
        logger.info(
            "reading HMI %s coils register data, addresses %s", master.slave_conn, addresses
        )
        logger.info("reading total coil records %s", len(addresses))
        registers_data = {}
        try:
            response = master.slave_conn.read_coils(
                address=addresses[0], count=len(addresses)
            )  # start with first address and request the total
            logger.info("results %s", response)
            for addr, dx in enumerate(addresses):
                register, _ = get_register_by_address(self.hmi_cr, addr)
                if not register:
                    continue
                registers_data[register.name] = int(response.bits[dx])
            return registers_data
        except Exception as err:
            logger.error("Error getting hmi coils data - %s", err)
            return {}


def run_masters_to_iolinks(
    iolink_slaves: list[SlaveIOLink],
    hr: list[HRegister],
    hmi_slaves: list[SlaveHMI],
    hmi_hr: list[HRegister],
    hmi_cr: list[CRegister],
) -> list[RelControl]:
    masters = []
    for iolink_slave in iolink_slaves:
        hmi_slave = next((s for s in hmi_slaves if s.hmi_id == iolink_slave.hmi_id), None)
        masters.append(
            RelControl(
                iolink_slave=iolink_slave,
                iolink_hr=hr,
                hmi_slave=hmi_slave,
                hmi_hr=hmi_hr,
                hmi_cr=hmi_cr,
            )
        )
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
    parser.add_argument(
        "-m",
        "--mode",
        help="connection mode",
        dest="mode",
        default=SlaveType.IOLINK.value,
        type=str,
    )
    parser.add_argument(
        "-x",
        "--addresstype",
        help="address type",
        dest="addresstype",
        default=RegisterType.HOLDING.value,
        type=str,
    )

    args = parser.parse_args()
    logger.info("starting main control for master io link %s ...", args.id)
    iolink_config = load_iolink_config()
    hmi_config = load_hmi_config()
    control = RelControl(
        iolink_slave=iolink_config.iolinks[args.id],
        iolink_hr=iolink_config.holding_registers,
        hmi_slave=hmi_config.hmis[args.id],
        hmi_hr=hmi_config.holding_registers,
        hmi_cr=hmi_config.coil_registers,
    )
    slave_type = SlaveType(args.mode)
    if slave_type == SlaveType.IOLINK:
        if args.action == "write":
            control.write_iolink_hregister(args.register, args.value)
        elif args.action == "read":
            value = -1
            if args.register == 0:
                value = control.get_iolink_hr_data()
                logger.info("value size %s", len(value))
            else:
                value = control.read_iolink_hregister(args.register)
            logger.info("read results %s", value)
    else:
        register_type = RegisterType(args.addresstype)
        if args.action == "write":
            control.write_register(
                register=args.register,
                value=args.value,
                stype=slave_type,
                rtype=register_type,
            )
        elif args.action == "read":
            value = -1
            if args.register == 0 and register_type == RegisterType.HOLDING:
                value = control.get_hmi_hr_data()
                logger.info("value size %s", len(value))
            elif args.register == 0 and register_type == RegisterType.COIL:
                value = control.get_hmi_cr_data()
                logger.info("value size %s", len(value))
            else:
                value = control.read_hmi_register(args.register, rtype=register_type)
            logger.info("read results %s", value)


# for testing outside ROS
if __name__ == "__main__":
    run()
