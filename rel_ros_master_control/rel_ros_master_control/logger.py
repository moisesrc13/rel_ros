"""
Main logger
"""
import logging
import os

from pymodbus.logging import pymodbus_apply_logging_config


def new_logger(name):
    """main logger implementation"""
    logger = logging.getLogger(name)
    logger.setLevel(os.environ.get("LOGLEVEL", "INFO"))
    pymodbus_apply_logging_config(level=os.environ.get("DEBUG_MODBUS", "ERROR"))
    handler = logging.StreamHandler()
    formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    return logger
