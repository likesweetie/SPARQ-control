"""
Generic CAN IMU driver.

This driver is product-independent. Vendor-specific details should be
implemented in IMUProtocolBase subclasses.

Responsibilities:
- Provide RX callback entry point.
- Merge partial decoded IMU state.
- Track quaternion and gyro freshness separately.
- Generate request frames through the protocol.
"""

from __future__ import annotations

import logging
import threading
from copy import deepcopy

from hal.can_bus import CANFrame
from hal.hardware.can.device_driver import CANDeviceDriverBase
from hal.hardware.can.device_comm_manager import BestEffortCommManager

from .protocol import IMUProtocolBase
from .state import RobotPoseState


logger = logging.getLogger(__name__)


class IMUDriver(CANDeviceDriverBase):
    """
    Product-independent CAN IMU driver.

    The driver does not send frames directly and does not know CANDaemon.
    A manager or factory should register on_frame() to the dispatcher and
    send request frames through CANDaemon.
    """

    def __init__(
        self,
        name: str,
        protocol: IMUProtocolBase,
        quat_timeout: float,
        gyro_timeout: float,
    ):
        super().__init__(name=name, protocol=protocol)

        self.quat_comm = BestEffortCommManager(timeout=quat_timeout)
        self.gyro_comm = BestEffortCommManager(timeout=gyro_timeout)

        self._state = RobotPoseState()
        self._lock = threading.Lock()

    def rx_can_ids(self) -> list[int]:
        return self.protocol.rx_can_ids()

    def make_request_quat_frame(self) -> CANFrame:
        return self.protocol.encode_request_quat()

    def make_request_gyro_frame(self) -> CANFrame:
        return self.protocol.encode_request_gyro()

    def make_request_all_frame(self) -> CANFrame:
        return self.protocol.encode_request_all()

    def on_frame(self, frame: CANFrame) -> None:
        """
        Dispatcher callback for received IMU frames.

        Keep this method lightweight because it is usually called from the
        CANDaemon RX thread.
        """
        try:
            partial_state = self.protocol.decode_frame(frame)
        except Exception:
            logger.exception(
                "[%s] Failed to decode IMU frame: can_id=0x%X",
                self.name,
                frame.can_id,
            )
            self._mark_decode_error(frame)
            return

        if partial_state is None:
            return

        self._merge_state(partial_state)

    def _merge_state(self, partial_state: RobotPoseState) -> None:
        """Merge a partial decoded state into the latest full state."""
        with self._lock:
            if partial_state.quat_xyzw is not None:
                self._state.quat_xyzw = partial_state.quat_xyzw
                self._state.projected_gravity_b = partial_state.projected_gravity_b
                self._state.last_quat_t = partial_state.last_quat_t
                self.quat_comm.mark_rx(partial_state.last_quat_t)

            if partial_state.angular_velocity_rad_s is not None:
                self._state.angular_velocity_rad_s = partial_state.angular_velocity_rad_s
                self._state.last_gyro_t = partial_state.last_gyro_t
                self.gyro_comm.mark_rx(partial_state.last_gyro_t)

    def _mark_decode_error(self, frame: CANFrame) -> None:
        """Mark decode error on the matching feedback monitor."""
        if self.protocol.is_quat_frame(frame):
            self.quat_comm.mark_decode_error()
            return

        if self.protocol.is_gyro_frame(frame):
            self.gyro_comm.mark_decode_error()
            return

        if frame.can_id in self.protocol.rx_can_ids():
            self.quat_comm.mark_decode_error()
            self.gyro_comm.mark_decode_error()

    def update_fault_flags(self) -> None:
        self.quat_comm.update_fault_flags()
        self.gyro_comm.update_fault_flags()

    def is_fresh(self) -> bool:
        quat_status = self.quat_comm.update_fault_flags()
        gyro_status = self.gyro_comm.update_fault_flags()

        return (not quat_status.is_stale) and (not gyro_status.is_stale)

    def get_state(self) -> RobotPoseState:
        with self._lock:
            return deepcopy(self._state)

    @property
    def state(self) -> RobotPoseState:
        return self.get_state()

    def get_comm_status(self):
        return {
            "quat": self.quat_comm.get_status(),
            "gyro": self.gyro_comm.get_status(),
        }
