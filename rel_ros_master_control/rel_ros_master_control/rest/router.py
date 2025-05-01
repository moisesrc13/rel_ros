from fastapi import APIRouter, BackgroundTasks, HTTPException, Request
from pydantic import BaseModel

from rel_ros_master_control.control import RelControl
from rel_ros_master_control.logger import new_logger
from rel_ros_master_control.services.pwm import PWMConfig, RelPWM

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


@api_router.post("/pwdm/run")
async def run_pwm(
    pwm_request: PWMRequest,
    background_tasks: BackgroundTasks,
):
    r_pwm = RelPWM(PWMConfig(frequency=pwm_request.frequency))
    background_tasks.add_task(
        r_pwm.run_duty, time_seconds=pwm_request.time_seconds, duty=pwm_request.duty
    )
    return {"message": "pwm running..."}
