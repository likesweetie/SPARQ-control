"""
IMU protocol base class.

Protocol classes are responsible for CAN-level encoding and decoding.
They should not own CANDaemon, CANDispatcher, state storage, or threads.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from hal.can_bus import CANFrame

from .state import RobotPoseState


class IMUProtocolBase(ABC):
    """
    Base interface for CAN-based IMU protocols.

    Product-specific subclasses define CAN IDs, request commands, payload
    layout, scale factors, and frame convention transforms.
    """

    @abstractmethod
    def rx_can_ids(self) -> list[int]:
        """Return CAN IDs that should be routed to this IMU driver."""
        raise NotImplementedError

    @abstractmethod
    def encode_request_quat(self) -> CANFrame:
        """Create a CAN frame that requests quaternion feedback."""
        raise NotImplementedError

    @abstractmethod
    def encode_request_gyro(self) -> CANFrame:
        """Create a CAN frame that requests gyro feedback."""
        raise NotImplementedError

    @abstractmethod
    def encode_request_all(self) -> CANFrame:
        """Create a CAN frame that requests all supported IMU feedback."""
        raise NotImplementedError

    @abstractmethod
    def decode_frame(self, frame: CANFrame) -> RobotPoseState | None:
        """
        Decode a received CAN frame into a partial RobotPoseState.

        Returns None when the frame is not handled by this protocol.
        """
        raise NotImplementedError

    def is_quat_frame(self, frame: CANFrame) -> bool:
        """Return True if the frame is a quaternion feedback frame."""
        return False

    def is_gyro_frame(self, frame: CANFrame) -> bool:
        """Return True if the frame is a gyro feedback frame."""
        return False
