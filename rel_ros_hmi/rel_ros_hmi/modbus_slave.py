"""
this is our modbus slave(s) implementation, mainly for testing
"""
import random

from pymodbus.constants import Endian
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusServerContext, ModbusSlaveContext
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.framer import FramerType
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.server import StartTcpServer

from rel_ros_hmi.logger import new_logger
from rel_ros_hmi.models.modbus_m import Register, SlaveTCP

logger = new_logger(__name__)


class ModbusServerBlock(ModbusSequentialDataBlock):
    def __init__(self, addr, values, slave: SlaveTCP):
        """Initialize."""
        self.hr = slave.holding_registers
        logger.info("initializing modbus 👾 slave %s", slave)
        super().__init__(addr, values)

    def setValues(self, address, value):
        """
        Set the requested values of the datastore.
        Automation Outputs (to dispenser from modbus)
        """
        logger.debug("calling SET values ...")

        super().setValues(address - 1, value)
        address = address - 1
        logger.debug("setValues with address %s, value %s", address, value)
        value = value[0]
        if address == self.hr.parameters.target_pressure_pistons.address:
            logger.debug("write parameter target_pressure_pistons = %s", value)
            self.hr.parameters.target_pressure_pistons.value = value
        elif address == self.hr.parameters.target_pressure_material.address:
            logger.debug("write parameter target_pressure_material = %s", value)
            self.hr.parameters.target_pressure_material.value = value
        elif address == self.hr.parameters.system_state.address:
            logger.debug("write parameter system_state = %s", value)
            self.hr.parameters.system_state.value = value

    def getValues(self, address, count=1):
        """
        Return the requested values from the datastore.
        Automation inputs (from dispenser to modbus)
        """
        logger.debug("calling GET values ...")

        def get_random_float(start: int, end: int) -> float:
            return round(random.uniform(start, end), 2)

        address = address - 1
        builder = BinaryPayloadBuilder(wordorder=Endian.LITTLE, byteorder=Endian.BIG)
        try:
            address_value = super().getValues(address, count=count)
            logger.info(
                "modbus getValues with address %s, address_value: %s",
                address,
                address_value[0],
            )

            if address == self.test_address:
                logger.info(
                    "requesting value for test address %s, return value %s",
                    self.test_address,
                    address_value,
                )
                return address_value
            if address == self.test_address_software_version:
                data = random.randint(0, 100)
                logger.info("reading software version data %s", data)
                builder.add_16bit_uint(data)
                return builder.to_registers()
            if address == self.test_address_int8:
                data = random.randint(0, 100)
                logger.info("test int8 data %s", data)
                builder.add_8bit_uint(data)
                return builder.to_registers()
            if address == self.test_address_int16:
                data = random.randint(0, 100)
                logger.info("test int16 data %s", data)
                builder.add_16bit_uint(data)
                return builder.to_registers()
            if address == self.test_address_float16:
                data = get_random_float(5, 80)
                logger.info("test float16 data %s", data)
                builder.add_16bit_float(data)
                return builder.to_registers()
            if address == self.test_address_int32:
                data = random.randint(0, 100)
                logger.info("test int32 data %s", data)
                builder.add_32bit_uint(data)
                return builder.to_registers()
            if address == self.test_address_float32:
                data = get_random_float(5, 180)
                logger.info("test float32 data %s", data)
                builder.add_32bit_float(data)
                return builder.to_registers()
            return [11, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        except Exception as ex:
            logger.error("Error getting values from modbus address %s - %s", address, ex)

    def validate(self, address, count=1):
        """Check to see if the request is in range."""
        logger.info("modbus validate address %s - count %s...", address, count)
        result = super().validate(address - 1, count=count)
        return result


def run_sync_modbus_server(slave: SlaveTCP):
    try:
        nreg = 50_000  # number of registers
        block = ModbusServerBlock(0x00, [0] * nreg, slave)
        store = {}
        # creating two slaves 0 & 1
        store[0] = ModbusSlaveContext(hr=block)
        store[1] = ModbusSlaveContext(hr=block)
        store[2] = ModbusSlaveContext(hr=block)
        context = ModbusServerContext(slaves=store, single=False)
        # initialize the server information
        identity = ModbusDeviceIdentification()
        identity.VendorName = "TestGasStation"
        identity.ProductName = "Test"
        identity.ModelName = "Test Modbus Server"
        identity.MajorMinorRevision = "0.1.0"
        modbus_slave = None
        if isinstance(slave, SlaveTCP):
            # Start Modbus TCP Server
            logger.info(
                "running TCP modbus slave 🤖 on %s port %s",
                slave.host,
                slave.port,
            )
            modbus_slave = StartTcpServer(
                context=context,
                host=slave.host,
                identity=identity,
                framer=FramerType.SOCKET,
                address=(slave.host, slave.port),
            )
        else:
            raise RuntimeError("slave type not supported")
        return modbus_slave
    except Exception as err:
        logger.error("error starting modbus slave - %s", err)
        raise err


def run(slave: SlaveTCP):
    server = run_sync_modbus_server(slave=slave)
    if server:
        server.shutdown()


if __name__ == "__main__":
    run(
        SlaveTCP(
            host="0.0.0.0",
            port=8844,  # matching one slave from config
            address_offset=0,
            device_ports=[],
            timeout_seconds=5,
        )
    )
