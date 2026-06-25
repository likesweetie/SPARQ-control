"""
hal.can_bus

CAN communication package for robot control systems.

This package exposes the public CAN API:
- CAN frame abstraction
- CAN bus abstraction
- SocketCAN implementation
- CAN daemon
- CAN dispatcher
- CAN device information
"""

from .bus import CANBus, SocketCANBus
from .can_types import CANFrameCallback
from .daemon import CANDaemon
from .dispatcher import CANDispatcher
from .frame import CANFrame
from .process_client import CANProcessClient
from .process_transport import CANProcessTransport


__all__ = [
    "CANFrame",
    "CANBus",
    "SocketCANBus",
    "CANDispatcher",
    "CANDaemon",
    "CANFrameCallback",
    "CANProcessClient",
    "CANProcessTransport",
]
