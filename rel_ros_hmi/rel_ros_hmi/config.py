import os

import yaml

from rel_ros_hmi.logger import new_logger
from rel_ros_hmi.models.modbus_m import ModbusConfig

logger = new_logger(__name__)


def abs_path(path: str) -> str:
    __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
    return os.path.join(__location__, path)


def load_config(file_path: str):
    try:
        file_location = abs_path(file_path)
        with open(file_location, "r", encoding="utf-8") as file:
            config = yaml.load(file, Loader=yaml.FullLoader)
            logger.info("config file %s loaded ok", file_location)
            logger.info("converting to global config..")
            return ModbusConfig.model_validate(config)
    except Exception as err:
        logger.error("Exception loading configuration - %s", err)
        raise err


def load_modbus_config() -> ModbusConfig:
    return load_config("./config/hmi.yml")
