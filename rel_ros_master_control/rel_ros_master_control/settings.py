from pydantic_settings import BaseSettings


class ModbusSettings(BaseSettings):
    version: str = "0.1.0"
