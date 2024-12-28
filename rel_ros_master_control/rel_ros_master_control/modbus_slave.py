"""
this is our modbus slave(s) implementation, mainly for testing
"""
import os
import random
import time

from pymodbus.constants import Endian
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusServerContext, ModbusSlaveContext
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.framer import FramerType
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.server import StartSerialServer, StartTcpServer

from rel_ros_master_control.logger import new_logger
from rel_ros_master_control.models.modbus_m import DevicePorts, SlaveSerial, SlaveTCP

logger = new_logger(__name__)


class ModbusServerBlock(ModbusSequentialDataBlock):
    def __init__(self, addr, values, slave: SlaveSerial | SlaveTCP):
        """Initialize."""
        self.settings = slave
        logger.info("initializing modbus 👾 local slave %s", slave)
        self.test_address = 0
        self.test_address_int8 = 1
        self.test_address_int16 = 2
        self.test_address_float16 = 3
        self.test_address_int32 = 4
        self.test_address_float32 = 5
        self.test_address_software_version = 40020
        super().__init__(addr, values)

    def setValues(self, address, values):
        """
        Set the requested values of the datastore.
        Automation Outputs (to dispenser from modbus)
        """
        logger.debug("calling SET values ...")

        super().setValues(address - 1, values)
        address = address - 1
        logger.info("setValues with address %s, value %s", address, values)
        values = values[0]
        if address == 100:  # TEST
            logger.info("hitting modbus 100 address")

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


def run_sync_modbus_server(slave: SlaveSerial | SlaveTCP):
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
        elif isinstance(slave, SlaveSerial):
            logger.info(
                "running SERIAL modbus slave 🤖 on port %s",
                slave.port,
            )
            logger.info("wait till %s is available", slave.port)
            while not os.path.exists(slave.port):
                time.sleep(1)
            logger.info(" %s is ready 🚀", slave.port)
            modbus_slave = StartSerialServer(
                context=context,  # Data storage
                identity=identity,  # server identify
                port=slave.port,  # serial port
                framer=FramerType.RTU,
                bytesize=slave.data_bit,
                parity=slave.parity,
                stopbits=slave.stop_bit,
                baudrate=slave.baudrate,
            )
        else:
            raise RuntimeError("slave type not supported")
        return modbus_slave
    except Exception as err:
        logger.error("error starting modbus slave - %s", err)
        raise err


def main():
    server = run_sync_modbus_server(
        slave=SlaveTCP(
            host="0.0.0.0",
            port=8844,  # matching one slave from config
            address_offset=0,
            device_ports=DevicePorts(),
            timeout_seconds=5,
        )
    )
    if server:
        server.shutdown()


if __name__ == "__main__":
    main()
