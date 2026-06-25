from __future__ import annotations

from enum import Enum, auto


class RobotControllerState(Enum):
    CREATED = auto()
    INIT_SHM = auto()
    START_CAN_DAEMON = auto()
    BRINGUP_IMU = auto()
    START_CHILD_PROCESSES = auto()
    RUNNING = auto()
    SHUTTING_DOWN = auto()
    STOPPED = auto()
    ERROR = auto()
