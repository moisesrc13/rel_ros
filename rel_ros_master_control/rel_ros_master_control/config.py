import os
from enum import Enum

import yaml

from rel_ros_master_control.logger import new_logger
from rel_ros_master_control.models.hmi_m import HMIConfig
from rel_ros_master_control.models.modbus_m import ModbusConfig
from rel_ros_master_control.models.status_device_m import StatusDeviceConfig

logger = new_logger(__name__)


class ConfigType(Enum):
    MODBUS = "modbus"
    STATUS_DEVICE = "status_device"
    HMI = "hmi"


def abs_path(path: str) -> str:
    __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
    return os.path.join(__location__, path)


def load_config(file_path: str, config_type: ConfigType):
    try:
        file_location = abs_path(file_path)
        with open(file_location, "r", encoding="utf-8") as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
            logger.info("config file %s loaded ok", file_location)
            logger.info("converting to global config...")
            if config_type == ConfigType.MODBUS:
                return ModbusConfig.model_validate(config)
            if config_type == ConfigType.HMI:
                return HMIConfig.model_validate(config)
            return StatusDeviceConfig.model_validate(config)
    except Exception as err:
        logger.error("Exception loading configuration - %s", err)
        raise err


def load_hmi_control_config() -> HMIConfig:
    return load_config("./config/hmi.yml", ConfigType.HMI)


def load_modbus_config() -> ModbusConfig:
    return load_config("./config/modbus.yml", ConfigType.MODBUS)


def load_status_device_config() -> StatusDeviceConfig:
    return load_config("./config/status_device.yml", ConfigType.STATUS_DEVICE)
