from fastapi import APIRouter, BackgroundTasks, HTTPException, Request
from pydantic import BaseModel

from rel_ros_master_control.control import PWMOption, RegisterType, RelControl, SlaveType
from rel_ros_master_control.logger import new_logger
from rel_ros_master_control.services.pwm_start import do_start_pwm_process as run_pwm
from rel_ros_master_control.services.pwm_stop import do_stop_pwm_process as stop_pwm

logger = new_logger(__name__)

api_router = APIRouter()


class WriteRequest(BaseModel):
    register: int
    value: int
    register_type: RegisterType = RegisterType("coil")
    slave_type: SlaveType = SlaveType("hmi")


class ReadRequest(BaseModel):
    register: int
    register_type: RegisterType = RegisterType("coil")
    slave_type: SlaveType = SlaveType("hmi")


class PWMRequest(BaseModel):
    option: PWMOption = PWMOption("low")


@api_router.get("/")
def read_root():
    return "modbus IOLink master connection 🤖"


@api_router.get("/health")
def health():
    """This is a generic endpoint to check the service is alive"""
    return "OK"


@api_router.post("/control/read")
async def read_register(
    request: Request,
    read_request: ReadRequest,
):
    logger.debug(
        "getting register %s data for slave type %s and register type",
        read_request.register,
        read_request.slave_type,
        read_request.register_type,
    )
    control: RelControl = request.app.state.control

    if read_request.slave_type == SlaveType.IOLINK:
        if read_request.register == 0:
            return control.get_iolink_hr_data()
        control_status = control.read_iolink_hregister(read_request.register)
    else:
        if read_request.register == 0 and read_request.register_type == RegisterType.HOLDING:
            return control.get_hmi_hr_data()
        if read_request.register == 0 and read_request.register_type == RegisterType.COIL:
            return control.get_hmi_cr_data()
        else:
            control_status = control.read_hmi_register(
                read_request.register, rtype=read_request.register_type
            )
    if control_status.error:
        raise HTTPException(
            status_code=control_status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting register data: {control_status.error}",
        )
    return control_status.model_dump()


@api_router.post("/control/write")
async def write_register(
    request: Request,
    write_request: WriteRequest,
):
    logger.debug(
        "write slave type %s register %s with value %s",
        write_request.slave_type,
        write_request.register,
        write_request.value,
    )
    control: RelControl = request.app.state.control
    if write_request.slave_type == SlaveType.IOLINK:
        modebus_status = control.write_iolink_hregister(write_request.register, write_request.value)
    else:
        modebus_status = control.write_register(
            register=write_request.register,
            value=write_request.value,
            stype=write_request.slave_type,
            rtype=write_request.register_type,
        )
    if modebus_status.error:
        raise HTTPException(
            status_code=modebus_status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error getting register data: {modebus_status.error}",
        )
    return modebus_status.model_dump()


@api_router.post("/pwm/run")
async def run_pwm_api(background_tasks: BackgroundTasks, pwm_request: PWMRequest):
    background_tasks.add_task(run_pwm, pwm_request.option)
    return {"message": "pwm running..."}


@api_router.post("/pwm/stop")
async def stop_pwm_api(
    background_tasks: BackgroundTasks,
):
    background_tasks.add_task(stop_pwm)
    return {"message": "pwm stopping..."}
