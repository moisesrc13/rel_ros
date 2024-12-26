import os
from enum import Enum

import yaml

from rel_ros_hmi.logger import new_logger
from rel_ros_hmi.models.modbus_m import SlaveTCP

logger = new_logger(__name__)


class ConfigType(Enum):
    MODBUS = "modbus"
    STATUS_DEVICE = "status_device"


def abs_path(path: str) -> str:
    __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
    return os.path.join(__location__, path)


def load_config(file_path: str, config_type: ConfigType):
    try:
        file_location = abs_path(file_path)
        with open(file_location, "r", encoding="utf-8") as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
            logger.info("config file %s loaded ok", file_location)
            logger.info("converting to global config..")
            if config_type == ConfigType.MODBUS:
                print(config)
                return ModbusConfig.model_validate(config)
            elif config_type == ConfigType.STATUS_DEVICE:
                return StatusDeviceConfig.model_validate(config)
    except Exception as err:
        logger.error("Exception loading configuration - %s", err)
        raise err


def load_modbus_config() -> ModbusConfig:
    return load_config("./config/modbus.yml", ConfigType.MODBUS)


def load_status_device_config() -> StatusDeviceConfig:
    return load_config("./config/status_device.yml", ConfigType.STATUS_DEVICE)
