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
    ElectroValveState,
    HMIWriteAction,
    ManualTasks,
    ManifoldActions,
    Params,
    PressureSet,
    PressureState,
    PWMPulseSet,
    Constants,
    SensorDistanceStateName,
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
from rel_ros_master_control.models.status_device_m import TowerState, TowerStatusDevice
from flow_control import run_flow

logger = new_logger(__name__)
try:
    from rel_ros_master_control.services.pwm_start import do_run_process as run_pwm
    from rel_ros_master_control.services.pwm_stop import do_stop_process as stop_pwm
except Exception as err:
    logger.warning("expected error if not running on RPi - %s", err)


def get_builder():
    return BinaryPayloadBuilder(wordorder=Endian.LITTLE, byteorder=Endian.BIG)  # this is for CDAB


def get_decoder_from_rr(rr) -> BinaryPayloadDecoder:
    return BinaryPayloadDecoder.fromRegisters(rr, wordorder=Endian.LITTLE, byteorder=Endian.BIG)


def get_decoder(response) -> BinaryPayloadDecoder:
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
        self.pwm_started = False
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
        self.master_io_link = RelModbusMaster(iolink_slave)
        self.master_hmi = RelModbusMaster(hmi_slave)
        logger.info("connecting master io_link")
        self.master_io_link.do_connect()
        logger.info("master_io_link connected ✨")
        logger.info("connecting master HMI")
        self.master_hmi.do_connect()
        logger.info("master_HMI connected ✨")

    def apply_initial_state(self):
        self.apply_hyd_valve_state(DigitalHydValve.OUT1_OFF_OUT2_OFF)
        self.apply_manifold_state(ManifoldActions.DEACTIVATE)
        self.apply_pressure_regulator_state(PressureState.OFF)
        self.apply_tower_state(TowerState.ACOUSTIC_ALARM_OFF)

    def run_user_tasks(self, coil_address: str, value: int):
        """
        This are all user tasks. Only valid in Manual mode
        For pistons 

        Args:
            coil_address (int): _description_
            value (int): _description_
        """
        def do_manifold_state(value: int, state: ManifoldActions):
            if value:
                self.apply_manifold_state(ManifoldActions.ACTIVATE)
                self.apply_manifold_state(state)
            else:
                self.apply_manifold_state(ManifoldActions.DEACTIVATE)
        
        # check manual is ON
        if not self.read_hmi_cregister_by_name(ManualTasks.ENTER_MANUAL_MODE_SCREEN):
            return
        logger.info("manual mode is ON")
        register = get_register_by_name(self.hmi_cr, coil_address)
        if not register:
            logger.info("resgiter with coil_address %s for user task not found", coil_address)
            return
        logger.info("running user task on %s - value %s", register.name, value)
        try:
            user_task = ManualTasks(register.name)
        except:
            logger.error("user task for register %s not supported", register.name)
            return        
        
        match user_task:
            case ManualTasks.ACTION_PRE_FILL_LINE:
                if not value:
                    self.stop_pwm()
                    return                    
                inputs = {"control": self}
                outputs = run_flow(inputs, Constants.flow_manual_pre_fill_line)
                sensor_distance_state = outputs.get("sensor_distance_state")
                if sensor_distance_state == SensorDistanceStateName.D and value:
                    self.apply_pwm_state()
            case ManualTasks.ACTION_RECYCLE_RETRACTIL:
                do_manifold_state(value, ManifoldActions.RECYCLE)
            case ManualTasks.ACTION_PULL_DOWN_PISTONS_MANUAL:
                do_manifold_state(value, ManifoldActions.PISTONS_DOWN)
            case ManualTasks.ACTION_PULL_UP_PISTONS_MANUAL:
                do_manifold_state(value, ManifoldActions.PISTONS_UP)
            case ManualTasks.ACTION_VACUUM_AIR:
                do_manifold_state(value, ManifoldActions.AIR_FOR_VACUUM)
                
            
                    
                    
                
                    
                    
                
                
            
            
            
        
        user_action = HMIWriteAction(register.name)
        if value == 0:
            self.apply_manifold_state(ManifoldActions.DEACTIVATE)
            return
        self.apply_manifold_state(ManifoldActions.ACTIVATE)
        match user_action:
            case HMIWriteAction.ACTION_PULL_DOWN_PISTONS_MANUAL:
                self.apply_manifold_state(ManifoldActions.PISTONS_DOWN)
            case HMIWriteAction.ACTION_PULL_UP_PISTONS_MANUAL:
                self.apply_manifold_state(ManifoldActions.PISTONS_UP)
            case HMIWriteAction.ACTION_DEPRESSURIZE:
                self.apply_manifold_state(
                    ManifoldActions.VENTING_RETRACTIL_UP
                )  # need to check if is up or down
            case HMIWriteAction.ACTION_VACUUM_AIR:
                self.apply_manifold_state(ManifoldActions.AIR_FOR_VACUUM)
            case HMIWriteAction.ACTION_RECYCLE:
                self.apply_manifold_state(ManifoldActions.RECYCLE)
            

    def apply_state(self, hr: HRegister, state_value: int):
        try:
            self.master_io_link.slave_conn.write_register(
                self.get_register_with_offset(hr.address),
                state_value,
            )
        except Exception as err:
            logger.error("error writing state %s - %s", state_value, err)

    def apply_manifold_state(self, state: ManifoldActions):
        logger.info("✨ apply manifold state - %s", state)
        self.apply_state(
            get_register_by_name(self.iolink_hr, HMIWriteAction.ACTION_MANIFOLD.value), state.value
        )

    def apply_hyd_valve_state(self, state: DigitalHydValve):
        logger.info("✨ apply hyd valve state - %s", state)
        self.apply_state(
            get_register_by_name(self.iolink_hr, DigitalOutput.DIGITAL_OUT_HYD_VALVE.value),
            state.value,
        )

    def apply_tower_state(self, state: TowerState):
        logger.info("✨ apply tower status state %s", state)
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
            case TowerState.ACOUSTIC_ALARM_ON:
                start_address = self.tower_devive.tower_status.alarm_address
                registers = self.tower_devive.tower_status.states.acoustic_alarm_on
            case TowerState.ACOUSTIC_ALARM_OFF:
                start_address = self.tower_devive.tower_status.alarm_address
                registers = self.tower_devive.tower_status.states.acoustic_alarm_off
            case _:
                logger.warning("no match state found for tower device - %s", state.value)
        if registers:
            try:
                self.master_io_link.slave_conn.write_registers(
                    self.get_register_with_offset(start_address),
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
        return register

    def write_iolink_hregister(self, register: int, value: int) -> ModbusStatus:
        return self.write_register(register, value, SlaveType.IOLINK)

    def write_iolink_hregister_by_name(self, enum_name: Enum, value: int) -> ModbusStatus:
        register = get_register_by_name(self.iolink_hr, enum_name.value)
        return self.write_register(register.address, value, SlaveType.IOLINK)

    def write_hmi_hregister_by_name(self, enum_name: Enum, value: int) -> ModbusStatus:
        register = get_register_by_name(self.hmi_hr, enum_name.value)
        return self.write_register(register.address, value, SlaveType.HMI)

    def eletrovalve_on(self):
        register = get_register_by_name(self.iolink_hr, DigitalOutput.DIGITAL_OUT_HYD_VALVE.value)
        self.write_iolink_hregister(register.address, ElectroValveState.ON.value)

    def eletrovalve_off(self):
        register = get_register_by_name(self.iolink_hr, DigitalOutput.DIGITAL_OUT_HYD_VALVE.value)
        self.write_iolink_hregister(register.address, ElectroValveState.OFF.value)

    def get_master_connection(self, stype: SlaveType) -> RelModbusMaster:
        if stype == SlaveType.IOLINK:
            return self.master_io_link
        return self.master_hmi

    def read_hregister(self, register: int, stype: SlaveType = SlaveType.IOLINK) -> ModbusStatus:
        status = ModbusStatus()
        master = self.get_master_connection(stype)
        response = master.slave_conn.read_holding_registers(
            address=self.get_register_with_offset(register, stype), count=1
        )
        if response.isError():
            logger.error("error reading register %s on %s", register, stype)
            status.error = response
            return status
        decoder = get_decoder(response)
        status.value = get_value(decoder)
        status.status = f"read ok {stype.value}"
        return status

    def read_hmi_cregister(self, register: int) -> ModbusStatus:
        status = ModbusStatus()
        master = self.get_master_connection(SlaveType.HMI)
        response = master.slave_conn.read_coils(
            address=self.get_register_with_offset(register, SlaveType.HMI), count=1
        )
        if response.isError():
            logger.error("error hmi reading hmi coil register %s", register)
            status.error = response
            return status
        status.value = int(response.bits[0])
        status.status = "read coil ok"
        return status

    def read_hmi_hregister(self, register: int) -> ModbusStatus:
        status = ModbusStatus()
        master = self.get_master_connection(SlaveType.HMI)
        response = master.slave_conn.read_holding_registers(
            address=self.get_register_with_offset(register, SlaveType.HMI), count=1
        )
        if response.isError():
            logger.error("error reading HMI hr %s", register)
            status.error = response
            return status
        decoder = get_decoder(response)
        status.value = get_value(decoder)
        status.status = "read hr ok"
        return status

    def read_hmi_cregister_by_name(self, enum_name: Enum) -> int:
        register = get_register_by_name(self.hmi_cr, enum_name.value)
        value = self.read_hmi_cregister(register.address).value
        logger.info(
            "reading 📺 HMI coil %s (%s) ok | value: ✨ %s", enum_name.value, register.address, value
        )
        return self.read_hmi_cregister(register.address).value

    def read_hmi_hregister_by_name(self, enum_name: Enum) -> int:
        register = get_register_by_name(self.hmi_hr, enum_name.value)
        value = self.read_hmi_hregister(register.address).value
        logger.info(
            "reading 📺 HMI hr %s (%s) ok | value: ✨ %s", enum_name.value, register.address, value
        )
        return self.read_hmi_hregister(register.address).value

    def apply_pressure_regulator_state(self, state: PressureState):
        logger.info("✨ apply pressure regulator state - %s", state)
        self.write_register_by_address_name(
            name=PressureSet.REGULATOR_ACTIVATE_VALVE.value,
            stype=SlaveType.IOLINK,
            rtype=RegisterType.HOLDING,
            enum_value=state,
        )

    def get_pwm_option(self):
        option_register = get_register_by_name(
            self.hmi_hr, Params.PARAM_PULSE_TRAIN_SELECTION.value
        )
        option = self.read_hmi_register(option_register.address).value
        pwm_ption = "low"
        match option:
            case PWMPulseSet.HIGH.value:
                pwm_ption = "high"
            case PWMPulseSet.MEDIUM.value:
                pwm_ption = "medium"
            case _:
                pwm_ption = "low"
        return pwm_ption

    def apply_pwm_state(self):
        if self.pwm_started:
            return
        if not self.pwm_started:
            run_pwm(option=self.get_pwm_option())
            self.pwm_started = True

    def stop_pwm(self):
        if self.pwm_started:
            stop_pwm()
            self.pwm_started = False

    def write_hmi_cregister_by_address_name(self, enum_name: Enum, enum_value: Enum):
        self.write_register_by_address_name(
            name=enum_name.value,
            enum_value=enum_value,
            stype=SlaveType.HMI,
            rtype=RegisterType.COIL,
        )

    def write_register_by_address_name(
        self,
        name: str,
        enum_value: Enum,
        stype: SlaveType = SlaveType.IOLINK,
        rtype: RegisterType = RegisterType.HOLDING,
    ) -> ModbusStatus:
        registers = self.iolink_hr  # no coils for IOLINK
        if stype == SlaveType.HMI and rtype == RegisterType.HOLDING:
            registers = self.hmi_hr
        elif stype == SlaveType.HMI and rtype == RegisterType.COIL:
            registers = self.hmi_cr
        register = get_register_by_name(registers, name)
        return self.write_register(
            register=register.address, value=enum_value.value, rtype=rtype, stype=stype
        )

    def write_register(
        self,
        register: int,
        value: int,
        stype: SlaveType = SlaveType.IOLINK,
        rtype: RegisterType = RegisterType.HOLDING,
    ) -> ModbusStatus:
        status = ModbusStatus()
        master = self.get_master_connection(stype)
        if rtype == RegisterType.HOLDING:
            response = master.slave_conn.write_register(
                address=self.get_register_with_offset(register, stype), value=value
            )
        else:
            response = master.slave_conn.write_coil(
                address=self.get_register_with_offset(register, stype), value=bool(value)
            )
        if response.isError():
            logger.error("error writing register on %s", stype)
            status.error = response
            return status
        status.value = value
        logger.info(
            "writing %s register %s value: ✨ %s",
            stype.value,
            register,
            value,
        )
        return status

    def read_iolink_hregister(self, register: int) -> ModbusStatus:
        return self.read_hregister(register, SlaveType.IOLINK)

    def read_iolink_hregister_by_name(self, enum_name: Enum) -> int:
        register = get_register_by_name(self.iolink_hr, enum_name.value)
        value = self.read_hregister(register.address, SlaveType.IOLINK).value
        logger.info(
            "reading 🛸 IOLink hr %s (%s) ok | value: ✨ %s", register.name, register.address, value
        )
        return value

    def read_hmi_register(
        self, register: int, rtype: RegisterType = RegisterType.HOLDING
    ) -> ModbusStatus:
        if rtype == RegisterType.HOLDING:
            return self.read_hregister(register, SlaveType.HMI)
        return self.read_hmi_cregister(register)

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
            "reading 📺 HMI %s holding register data, addresses %s", master.slave_conn, addresses
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
            "reading 📺 HMI %s coils register data, addresses %s", master.slave_conn, addresses
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
