import dataclasses
import json
import sqlite3
from dataclasses import dataclass
from datetime import datetime
from enum import Enum
from operator import itemgetter
from typing import Optional

from rel_ros_hmi.config import load_modbus_config
from rel_ros_hmi.logger import new_logger

logger = new_logger(__name__)


class DBService:
    def __init__(self, db_file: str = "/home/relant/ros2_ws/hmi.db") -> None:
        logger.info("loading config")
        config = load_modbus_config()
        if config is None:
            raise RuntimeError("error loading configuration")
        self.hr = config.modbus.holding_registers
        logger.debug("holding registers %s", self.hr)
        try:
            self.conn = sqlite3.connect(db_file)
        except Exception as ex:
            logger.error("error creating database - %s", ex)

    def create_tables(self) -> None:
        try:
            cur = self.conn.cursor()
            # task_outputs is an array of ids from output_task
            cur.execute(
                """CREATE TABLE IF NOT EXISTS hmi (
                        param_target_pressure_pistons INTEGER DEFAULT 0,
                        param_target_pressure_material INTEGER DEFAULT 0,
                        param_system_state INTEGER DEFAULT 0,
                        param_pulse_train_high INTEGER DEFAULT 0,
                        param_pulse_train_medium INTEGER DEFAULT 0,
                        param_pulse_train_low INTEGER DEFAULT 0,
                        param_emergency_stop_state INTEGER DEFAULT 0,
                        param_solenoid_valves_state INTEGER DEFAULT 0,
                        param_alarm_state INTEGER DEFAULT 0,
                        param_alarm_pre_vacuum_limit_high INTEGER DEFAULT 0,
                        param_alarm_pre_vacuum_limit_low INTEGER DEFAULT 0,
                        param_alarm_pre_vacuum_distance INTEGER DEFAULT 0,
                        sensor_material_pressure INTEGER DEFAULT 0,
                        sensor_material_temperature INTEGER DEFAULT 0,
                        sensor_laser_distance INTEGER DEFAULT 0)"""
            )
            self.conn.commit()
            logger.info("tables created ok")
        except Exception as ex:
            logger.error("Error creating tables. Error %s", ex)

    def update_hmi(self, name: str, value: int = 0) -> bool:
        try:
            stmt = f"UPDATE hmi SET {name} = {value}"
            cur = self.conn.cursor()
            res = cur.execute(stmt)
            self.conn.commit()
            if res.rowcount == 1:
                logger.info(" %s updated ok", name)
                return True
            return False
        except Exception as err:
            logger.error("error updating %s - %s", name, err)
            return False


if __name__ == "__main__":
    logger.info("Starting sqlite db...")
    db_service = DBService()
    db_service.create_tables()
