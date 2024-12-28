""" main fast api application """
import os
from contextlib import asynccontextmanager
from threading import Thread

import uvicorn
from dotenv import find_dotenv, load_dotenv
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from rel_ros_master_control.logger import new_logger

logger = new_logger(__name__)


@asynccontextmanager
async def lifespan_func(app: FastAPI):
    logger.info("creating modbus master 🚀...")
    yield


def build_app():
    from router import api_router

    # Initialize FastAPI app
    title = "modbus gas client for dispenser"
    description = "Connect to modbus server and get metrics from dispenser"
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
    app_settings = AppSettings()
    if app_settings.enable_poll:
        master_poll_thread = Thread(target=run_poll)
        master_poll_thread.start()
    if app_settings.app_secure:
        uvicorn.run(
            app=build_app(),
            ssl_certfile=abs_path("./certs/ssc.crt"),
            ssl_keyfile=abs_path("./certs/ssc.key"),
            host=app_settings.app_host,
            port=app_settings.app_port,
            access_log=False,
        )
    else:
        uvicorn.run(
            app=build_app(),
            host=app_settings.app_host,
            port=app_settings.app_port,
            access_log=False,
        )
