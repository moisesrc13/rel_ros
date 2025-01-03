"""
This is a modbus slave for testing within ROS
"""
import random

from pymodbus.constants import Endian
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusServerContext, ModbusSlaveContext
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.framer import FramerType
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.server import StartTcpServer

from rel_ros_hmi.config import load_modbus_config
from rel_ros_hmi.logger import new_logger
from rel_ros_hmi.models.modbus_m import Register, SlaveTCP, get_register_by_address

logger = new_logger(__name__)


class ModbusServerBlock(ModbusSequentialDataBlock):
    def __init__(self, addr, values, slave: SlaveTCP, hr: list[Register]):
        """Initialize."""
        self.hr = hr
        logger.info("initializing modbus 👾 slave on port %s", slave.port)
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
        logger.debug("getting from parameters ...")
        register, idx = get_register_by_address(self.hr, address)
        if not register:
            logger.debug("Not getting a valid register for address %s", address)
            return

        logger.debug("write register %s with value %s", register, value)
        register.value = value
        self.hr[idx] = register

    def getValues(self, address, count=1):
        """
        Return the requested values from the datastore.
        Automation inputs (from dispenser to modbus)
        """
        logger.debug("calling GET values from modbus slave ...")

        address = address - 1
        builder = BinaryPayloadBuilder(wordorder=Endian.LITTLE, byteorder=Endian.BIG)
        try:
            address_value = super().getValues(address, count=count)
            logger.info(
                "modbus getValues with address %s, address_value: %s and count: %s",
                address,
                address_value[0],
                count,
            )

            addresses = list(range(address, address + count))
            logger.info("addresses to get %s", addresses)
            for addr in addresses:
                register, _ = get_register_by_address(self.hr, addr)
                if not register:
                    continue
                if register.name.startswith("param") and register.value == 0:
                    # generate random
                    builder.add_16bit_uint(random.randint(10, 555))
                else:
                    builder.add_16bit_uint(register.value)
            values = builder.to_registers()
            logger.info("return values %s", values)
            return values
        except Exception as ex:
            logger.error("Error getting values from modbus address %s - %s", address, ex)

    def validate(self, address, count=1):
        """Check to see if the request is in range."""
        logger.info("modbus validate address %s - count %s...", address, count)
        result = super().validate(address - 1, count=count)
        return result


def run_sync_modbus_server(slave: SlaveTCP, hr: list[Register]):
    try:
        nreg = 50_000  # number of registers
        block = ModbusServerBlock(0x00, [0] * nreg, slave, hr)
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


def run(slave: SlaveTCP, hr: list[Register]):
    slave.host = "0.0.0.0"
    slave.port = 8845
    server = run_sync_modbus_server(slave, hr)
    if server:
        server.shutdown()


if __name__ == "__main__":
    #  note that this slave implementation is only for testing
    config = load_modbus_config()
    run(config.slaves[0], config.holding_registers)
