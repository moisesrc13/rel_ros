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
from rel_ros_hmi.models.modbus_m import (
    CRegister,
    HRegister,
    RegisterModbusType,
    SlaveHMI,
    SlaveTCP,
    get_register_by_address,
)

logger = new_logger(__name__)


class PublishHMIUserTask:
    def __init__(self, slave_name: str, slave_id: str) -> None:
        self.slave_name = slave_name
        self.slave_id = slave_id
        self.msg = None

    def __enter__(self):
        try:
            from rel_interfaces.msg import HMIUserTask

            self.msg = HMIUserTask()
            self.msg.hmi_name = self.slave_name
            self.msg.hmi_id = self.slave_id
            return self.msg
        except Exception as ex:
            logger.error("error getting user task message interface %s", ex)
            return None

    def __exit__(self, exc_type, exc_value, traceback):
        self.msg = None


class ModbusServerBlock(ModbusSequentialDataBlock):
    def __init__(
        self,
        addr,
        values,
        slave: SlaveHMI,
        hr: list[HRegister],
        cr: list[CRegister],
        user_task_publisher=None,
    ):
        """Initialize."""
        self.hr = hr
        self.cr = cr
        self.slave = slave
        self.user_task_publisher = user_task_publisher
        logger.info("initializing modbus 👾 slave on port %s", self.slave.slave_tcp.port)
        super().__init__(addr, values)

    def setValues(self, address, value):
        """
        Set the requested values of the datastore.
        Automation Outputs (to dispenser from modbus)
        """
        logger.debug("calling SET values ...")
        address = address - 1
        super().setValues(address, value)
        value = value[0]
        if isinstance(value, bool):  # this is required for coils
            value = int(value)
        logger.debug("setValues with address %s, value %s", address, value)
        logger.debug("getting register by address ...")
        register, idx = get_register_by_address(self.hr, address)
        rtype = RegisterModbusType.HR
        if not register:
            register, idx = get_register_by_address(self.cr, address)
            if not register:
                logger.debug("Not getting a valid register for address %s", address)
                return
            rtype = RegisterModbusType.CR
            logger.info("raising a UserTask 🤠 ...")
            with PublishHMIUserTask(self.slave.name, self.slave.id) as msg:
                if msg:
                    setattr(msg, "coil_address", register)
                    setattr(msg, "value", value)
                    logger.info("📨 publishing message on HMIUserTask - %s", msg)
                    self.user_task_publisher.publish(msg)

        logger.debug("write register %s with value %s", register, value)
        register.value = value
        if rtype == RegisterModbusType.HR:
            self.hr[idx] = register
        else:
            self.cr[idx] = register

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
            total = 0
            register, _ = get_register_by_address(self.hr, address)
            register_list = self.hr
            if not register:
                register, _ = get_register_by_address(self.cr, address)
                if not register:
                    logger.warning("register with address %s not found", addresses)
                    return
                logger.info("this is a coil register")
                register_list = self.cr
            for addr in addresses:
                register, _ = get_register_by_address(register_list, addr)
                if not register:
                    continue
                builder.add_16bit_uint(register.value)
                total += 1
            values = builder.to_registers()
            logger.info("returning total values %s", total)
            logger.info("return values %s", values)
            return values
        except Exception as ex:
            logger.error("Error getting values from modbus address %s - %s", address, ex)

    def validate(self, address, count=1):
        """Check to see if the request is in range."""
        address = address - 1
        logger.info("modbus validate address %s - count %s...", address, count)
        result = super().validate(address, count=count)
        return result


def run_sync_modbus_server(
    slave: SlaveHMI, hr: list[HRegister], cr: list[CRegister], user_task_publisher=None
):
    try:
        nreg = 50_000  # number of registers
        block = ModbusServerBlock(0x00, [0] * nreg, slave, hr, cr, user_task_publisher)
        # creating two slaves 0 & 1
        store = ModbusSlaveContext(
            di=block, co=block, hr=block, ir=block
        )  # coils only write 1 (True) or 0 (False)
        context = ModbusServerContext(slaves=store, single=True)
        # initialize the server information
        identity = ModbusDeviceIdentification()
        identity.VendorName = f"Relant HMI - {slave.hmi_id}"
        identity.ProductName = "Relant-On-ROS"
        identity.ModelName = "relros"
        identity.MajorMinorRevision = "0.1.0"
        modbus_slave = None
        if isinstance(slave.slave_tcp, SlaveTCP):
            # Start Modbus TCP Server
            logger.info(
                "running TCP modbus HMI slave 🤖 on %s port %s",
                slave.slave_tcp.host,
                slave.slave_tcp.port,
            )
            modbus_slave = StartTcpServer(
                context=context,
                host=slave.slave_tcp.host,
                identity=identity,
                framer=FramerType.SOCKET,
                address=(slave.slave_tcp.host, slave.slave_tcp.port),
            )
        else:
            raise RuntimeError("slave type not supported")
        return modbus_slave
    except Exception as err:
        logger.error("error starting modbus slave - %s", err)
        raise err


class ModbusSlaveThread(Thread):
    """main workflow thread"""

    def __init__(
        self, slave: SlaveHMI, hr: list[HRegister], cr: list[CRegister], user_task_publisher=None
    ) -> None:
        Thread.__init__(self)
        self.slave = slave
        self.hr = hr
        self.cr = cr
        self.hmi_id = slave.hmi_id
        self.user_task_publisher = user_task_publisher

    def run(self):
        logger.info("running slave thread %s", self.hmi_id)
        try:
            server = run_sync_modbus_server(
                slave=self.slave,
                hr=self.hr,
                cr=self.cr,
                user_task_publisher=self.user_task_publisher,
            )
            if server:
                server.shutdown()
        except Exception as ex:
            logger.error("Error %s running slave thread %s", ex, self.hmi_id)


def run_modbus_slaves(
    slaves: list[SlaveHMI], hr: list[HRegister], cr: list[CRegister], publishers: dict
):
    for slave in slaves:
        slave_thread = ModbusSlaveThread(
            slave=slave, hr=hr, cr=cr, user_task_publisher=publishers.get(slave.id)
        )
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
        run_modbus_slaves(config.hmis, config.holding_registers, config.coil_registers, {})
        logger.info("slaves launched ...")
        while True:
            pass
    except KeyboardInterrupt:
        logger.info("bye 👋...")
