"""
This is a modbus slave for testing within ROS
"""
from threading import Thread

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


class PublishHMIData:
    def __init__(self, slave: SlaveTCP, publisher) -> None:
        self.publisher = publisher
        self.slave = slave

    def __enter__(self):
        pass

    def __exit__(self, exc_type, exc_value, traceback):
        pass

    def do_publish(self):
        from rel_interfaces.msg import HMI

        # publish the data
        msg = HMI()
        msg.hmi_name = self.slave.name
        msg.hmi_id = self.slave.id
        for register in self.hr:
            setattr(msg, register.name, register.value)
        logger.info("📨 publishing message from HMI set value %s", msg)
        self.publisher.publish(msg)


class ModbusServerBlock(ModbusSequentialDataBlock):
    def __init__(self, addr, values, slave: SlaveTCP, hr: list[Register], publisher):
        """Initialize."""
        self.hr = hr
        self.slave = slave
        self.publisher = publisher
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
        logger.debug("getting register by address ...")
        register, idx = get_register_by_address(self.hr, address)
        if not register:
            logger.debug("Not getting a valid register for address %s", address)
            return

        logger.debug("write register %s with value %s", register, value)
        register.value = value
        self.hr[idx] = register
        # publish the data
        with PublishHMIData(self.slave, self.publisher) as hmi_publisher:
            hmi_publisher.do_publish()

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


def run_sync_modbus_server(slave: SlaveTCP, hr: list[Register], publisher):
    try:
        nreg = 50_000  # number of registers
        block = ModbusServerBlock(0x00, [0] * nreg, slave, hr, publisher)
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
                address=(slave.host, slave.port, publisher),
            )
        else:
            raise RuntimeError("slave type not supported")
        return modbus_slave
    except Exception as err:
        logger.error("error starting modbus slave - %s", err)
        raise err


class ModbusSlaveThread(Thread):
    """main workflow thread"""

    def __init__(self, slave: SlaveTCP, hr: list[Register], publisher) -> None:
        Thread.__init__(self)
        self.slave = slave
        self.hr = hr
        self.publisher = publisher

    def run(self):
        logger.info("running slave thread %s", self.slave.id)
        try:
            server = run_sync_modbus_server(slave=self.slave, hr=self.hr, publisher=self.publisher)
            if server:
                server.shutdown()
        except Exception as ex:
            logger.error("Error %s running slave thread %s", ex, self.slave.id)
        self.queue.task_done()


def run(slave: SlaveTCP, publisher=None):
    server = run_sync_modbus_server(slave=slave, publisher=publisher)
    if server:
        server.shutdown()


def run_modbus_slaves(slaves: list[SlaveTCP], hr: list[Register], publishers: dict):
    for slave in slaves:
        slave_thread = ModbusSlaveThread(slave=slave, hr=hr, publisher=publishers.get(slave.id))
        slave_thread.daemon = True
        slave_thread.start()
    try:
        logger.info("all slaves has been started...")
    except Exception as err:
        logger.error("Error running slave threads %s", err)
        raise err


if __name__ == "__main__":
    # this main method is basically for local test
    try:
        config = load_modbus_config()
        run_modbus_slaves(config.slaves, config.holding_registers, {})
        logger.info("slaves launched ...")
        while True:
            pass
    except KeyboardInterrupt:
        logger.info("bye 👋...")
