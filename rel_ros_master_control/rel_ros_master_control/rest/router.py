from fastapi import APIRouter, HTTPException, Request, Response, status
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


@api_router.post("/read/{register}")
async def get_dispense_registers_data(
    request: Request,
    register: int,
):
    logger.debug("getting register %s data", register)
    control: RelControl = request.app.state.control
    data = control.read_holding_register(register)
    if error := data.get("error"):
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting register data from modbus slave: {error}",
        )
    return data
