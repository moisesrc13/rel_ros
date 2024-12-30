from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

from rel_ros_master_control.control import RelControl
from rel_ros_master_control.logger import new_logger

logger = new_logger(__name__)

api_router = APIRouter()


class WriteRequest(BaseModel):
    register: int
    value: int


@api_router.get("/")
def read_root():
    return "modbus IOLink master connection 🤖"


@api_router.get("/health")
def health():
    """This is a generic endpoint to check the service is alive"""
    return "OK"


@api_router.get("/read/{register}")
async def read_register(
    request: Request,
    register: int,
):
    logger.debug("getting register %s data", register)
    control: RelControl = request.app.state.control
    control_status = control.read_holding_register(register)
    if control_status.error:
        raise HTTPException(
            status_code=control_status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting register data: {control_status.error}",
        )
    return control_status.model_dump()


@api_router.post("/write")
async def write_register(
    request: Request,
    write_request: WriteRequest,
):
    logger.debug("write register %s with value %s", write_request.register, write_request.value)
    control: RelControl = request.app.state.control
    control_status = control.write_holding_register(write_request.register, write_request.value)
    if control_status.error:
        raise HTTPException(
            status_code=control_status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting register data: {control_status.error}",
        )
    return control_status.model_dump()
