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

from rel_ros_master_control.config import load_modbus_config
from rel_ros_master_control.logger import new_logger
from rel_ros_master_control.models.modbus_m import (
    DevicePorts,
    Register,
    SlaveSerial,
    SlaveTCP,
    get_register_by_address,
)

logger = new_logger(__name__)


class ModbusServerBlock(ModbusSequentialDataBlock):
    def __init__(self, addr, values, slave: SlaveSerial | SlaveTCP, hr: list[Register]):
        """Initialize."""
        self.settings = slave
        self.hr = hr
        logger.info("initializing modbus 👾 local slave %s", slave)
        self.test_address_uint16 = 40021
        self.test_address_software_version = 40020
        self.test_address_uint16_value = 255
        self.test_address_software_version_value = 255
        super().__init__(addr, values)

    def setValues(self, address, values):
        """
        Set the requested values of the datastore.
        Automation Outputs (to dispenser from modbus)
        """
        logger.debug("calling SET values ...")

        super().setValues(address - 1, values)
        logger.info("setValues with address %s, value %s", address, values)
        value = values[0]
        logger.debug("getting register by address ...")
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
        logger.debug("calling GET values ...")
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


def run_sync_modbus_server(slave: SlaveSerial | SlaveTCP, hr: list[Register]):
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
        identity.VendorName = "Relant"
        identity.ProductName = "Iolink Slave Test"
        identity.ModelName = "relros"
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
    config = load_modbus_config()
    server = run_sync_modbus_server(
        slave=SlaveTCP(
            host="0.0.0.0",
            port=8844,  # matching one slave from config
            address_offset=0,
            device_ports=DevicePorts(),
            timeout_seconds=5,
        ),
        hr=config.holding_registers,
    )
    if server:
        server.shutdown()


if __name__ == "__main__":
    main()
