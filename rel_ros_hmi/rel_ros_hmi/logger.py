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
    debug_modbus = os.environ.get("DEBUG_MODBUS", "false").lower() in ["true", "yes"]
    if debug_modbus:
        pymodbus_apply_logging_config(level=os.environ.get("LOGLEVEL", "INFO"))
    handler = logging.StreamHandler()
    formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    return logger
