""" main fast api application """
import os
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from rel_ros_master_control.config import load_modbus_config
from rel_ros_master_control.control import RelControl
from rel_ros_master_control.logger import new_logger

logger = new_logger(__name__)


@asynccontextmanager
async def lifespan_func(app: FastAPI):
    config = load_modbus_config()
    iolink_master_id = int(os.getenv("APP_MASTER_IOLINK_ID", "0"))
    logger.info("creating modbus io link master id %s connection 🚀...", iolink_master_id)
    control = RelControl(
        iolink_slave=config.iolinks[iolink_master_id], iolink_hr=config.holding_registers
    )
    app.state.control = control
    yield


def build_app():
    from router import api_router

    # Initialize FastAPI app
    title = "👾 modbus io link master connection"
    description = "Connect to modbus io link slave to get data"
    app = FastAPI(title=title, description=description, lifespan=lifespan_func)
    app.include_router(api_router)
    app.add_middleware(
        CORSMiddleware,
        allow_origins="*",
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )
    return app


if __name__ == "__main__":
    uvicorn.run(
        app=build_app(),
        host="0.0.0.0",
        port=9080,
        access_log=False,
    )
