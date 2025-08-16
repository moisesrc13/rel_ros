from fastapi import APIRouter, BackgroundTasks, HTTPException, Request
from pydantic import BaseModel

from rel_ros_master_control.control import RelControl
from rel_ros_master_control.logger import new_logger
from rel_ros_master_control.services.pwm_start import do_start_pwm_process as run_pwm
from rel_ros_master_control.services.pwm_stop import do_stop_pwm_process as stop_pwm

logger = new_logger(__name__)

api_router = APIRouter()


class WriteRequest(BaseModel):
    register: int
    value: int


class PWMRequest(BaseModel):
    frequency: int = 1000
    time_seconds: int = 10
    duty: int = 100


@api_router.get("/")
def read_root():
    return "modbus IOLink master connection 🤖"


@api_router.get("/health")
def health():
    """This is a generic endpoint to check the service is alive"""
    return "OK"


@api_router.get("/modbus/read/{register}")
async def read_register(
    request: Request,
    register: int,
):
    logger.debug("getting register %s data", register)
    control: RelControl = request.app.state.control
    control_status = control.read_iolink_hregister(register)
    if control_status.error:
        raise HTTPException(
            status_code=control_status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting register data: {control_status.error}",
        )
    return control_status.model_dump()


@api_router.post("/modbus/write")
async def write_register(
    request: Request,
    write_request: WriteRequest,
):
    logger.debug("write register %s with value %s", write_request.register, write_request.value)
    control: RelControl = request.app.state.control
    control_status = control.write_iolink_hregister(write_request.register, write_request.value)
    if control_status.error:
        raise HTTPException(
            status_code=control_status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting register data: {control_status.error}",
        )
    return control_status.model_dump()


@api_router.post("/pwm/run")
async def run_pwm_api(
    background_tasks: BackgroundTasks,
):
    background_tasks.add_task(run_pwm)
    return {"message": "pwm running..."}


@api_router.post("/pwm/stop")
async def stop_pwm_api(
    background_tasks: BackgroundTasks,
):
    background_tasks.add_task(stop_pwm)
    return {"message": "pwm stopping..."}
