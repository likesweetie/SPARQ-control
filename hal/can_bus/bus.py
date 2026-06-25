from abc import ABC, abstractmethod
from .frame import CANFrame
import socket
import struct


CLASSIC_CAN_FRAME_FMT = "=IB3x8s" #Only for Classic CAN
CLASSIC_CAN_FRAME_SIZE = struct.calcsize(CLASSIC_CAN_FRAME_FMT)


class CANBus(ABC):
    """
    A base abstract class for CAN BUS implementation. 
    """
    @abstractmethod
    def send_frame(self, frame: CANFrame) -> None:
        raise NotImplementedError

    @abstractmethod
    def recv_frame(self, timeout: float = 0.0) -> CANFrame | None:
        raise NotImplementedError
    
    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError


class SocketCANBus(CANBus):
    """
    A CAN bus implementation using SocketCAN.
    Only for Classic CAN.

    """
    def __init__(self, interface_name: str):
        self._interface_name = interface_name

        self._socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._socket_bind()

    @property
    def interface_name(self):
        return self._interface_name
    
    def _socket_bind(self):
        self._socket.bind((self._interface_name,))

    def send_frame(self, frame: CANFrame) -> None:
        if len(frame.data) > 8:
            raise ValueError(f"[{self.__class__.__name__}] Classical CAN payload must be <= 8 bytes")
        
        raw = struct.pack(CLASSIC_CAN_FRAME_FMT, 
                          frame.can_id, 
                          frame.dlc, 
                          frame.data.ljust(8, b"\x00"))
        self._socket.send(raw)

    def recv_frame(self, timeout: float = 0.0) -> CANFrame | None:
        self._socket.settimeout(timeout)
        try:
            raw = self._socket.recv(CLASSIC_CAN_FRAME_SIZE)
        except (socket.timeout, TimeoutError, OSError):
            return None

        can_id, dlc, data = struct.unpack(CLASSIC_CAN_FRAME_FMT, raw)
        can_id &= socket.CAN_SFF_MASK
        return CANFrame(can_id=can_id, data=data[:dlc])

    def close(self) -> None:
        self._socket.close()
