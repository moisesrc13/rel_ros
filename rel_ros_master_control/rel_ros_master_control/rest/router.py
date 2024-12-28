from fastapi import APIRouter, Depends, HTTPException, Request, Response, status

from rel_ros_master_control.logger import new_logger

logger = new_logger(__name__)

api_router = APIRouter()


@api_router.get("/")
def read_root():
    return "modbus master connection 🤖"


@api_router.get("/health")
def health():
    """This is a generic endpoint to check the service is alive"""
    return "OK"
