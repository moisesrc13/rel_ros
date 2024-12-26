from enum import Enum


class PortMode(Enum):
    NO_DEVICE = 0
    DEACTIVATED = 1
    PORT_DIAG = 2
    PREOPERATE = 3
    OPERATE = 4
    DI_CO = 5
    DO_CO = 6
