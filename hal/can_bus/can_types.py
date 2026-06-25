from collections.abc import Callable
from .frame import CANFrame

CANFrameCallback = Callable[[CANFrame], None]
