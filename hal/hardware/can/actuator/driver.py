"""
Generic CAN actuator driver.

This driver is product-independent. Vendor-specific details should be
implemented in ActuatorProtocolBase subclasses.

Design rule:
- Driver does not know CANDaemon.
- Driver exposes rx_can_ids() and on_frame() so a manager/factory can register it.
- Driver exposes make_*_frame() methods so a manager/controller can send frames.
"""

from __future__ import annotations

import logging
import threading
from copy import deepcopy

from hal.can_bus import CANFrame
from hal.hardware.can.device_comm_manager import BestEffortCommManager, TransactionManager
from hal.hardware.can.device_driver import CANDeviceDriverBase

from .protocol import ActuatorProtocolBase
from .state import ActuatorCommand, ActuatorState


logger = logging.getLogger(__name__)


class ActuatorDriver(CANDeviceDriverBase):
    """
    Product-independent actuator driver.

    This class manages:
    - Latest actuator feedback state
    - Best-effort feedback freshness
    - Optional request-response transaction tracking
    - RX callback entry point

    It does not send frames by itself. A manager or controller should call

    example:
    daemon.send(driver.make_*_frame(...)).
    """

    def __init__(
        self,
        name: str,
        protocol: ActuatorProtocolBase,
        feedback_timeout: float,
        comm_manager: BestEffortCommManager | None = None,
        transaction_manager: TransactionManager | None = None,
    ):
        super().__init__(
            name=name,
            protocol=protocol,
            comm_manager=comm_manager or BestEffortCommManager(timeout=feedback_timeout),
        )

        self.protocol: ActuatorProtocolBase = protocol
        self.transaction_manager = transaction_manager or TransactionManager()

        self._state = ActuatorState()
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # RX registration support
    # ------------------------------------------------------------------

    def rx_can_ids(self) -> list[int]:
        return self.protocol.rx_can_ids()

    def on_frame(self, frame: CANFrame) -> None:
        """
        CANDispatcher callback.

        This method should remain lightweight because it is usually called from
        the CANDaemon RX thread.
        """
        try:
            partial_state = self.protocol.decode_frame(frame)
        except Exception:
            logger.exception(
                "[%s] Failed to decode actuator frame: can_id=0x%X",
                self.name,
                frame.can_id,
            )
            self.comm_manager.mark_decode_error()
            return

        if partial_state is None:
            return

        self._merge_state(partial_state)
        self.transaction_manager.mark_rx(frame.can_id)

    def _merge_state(self, partial_state: ActuatorState) -> None:
        """
        Merge a partial ActuatorState into the latest full state.
        """
        with self._lock:
            if partial_state.position_rad is not None:
                self._state.position_rad = partial_state.position_rad

            if partial_state.velocity_rad_s is not None:
                self._state.velocity_rad_s = partial_state.velocity_rad_s

            if partial_state.torque_nm is not None:
                self._state.torque_nm = partial_state.torque_nm

            if partial_state.current_a is not None:
                self._state.current_a = partial_state.current_a

            if partial_state.temperature_c is not None:
                self._state.temperature_c = partial_state.temperature_c

            if partial_state.voltage_v is not None:
                self._state.voltage_v = partial_state.voltage_v

            if partial_state.fault_code is not None:
                self._state.fault_code = partial_state.fault_code

            if partial_state.is_enabled is not None:
                self._state.is_enabled = partial_state.is_enabled

            if partial_state.mode is not None:
                self._state.mode = partial_state.mode

            if partial_state.raw:
                self._state.raw.update(partial_state.raw)

            if partial_state.last_feedback_t > 0.0:
                self._state.last_feedback_t = partial_state.last_feedback_t
                self.comm_manager.mark_rx(partial_state.last_feedback_t)
            else:
                self.comm_manager.mark_rx()

    # ------------------------------------------------------------------
    # TX frame generation helpers
    # ------------------------------------------------------------------

    def make_enable_frame(self) -> CANFrame:
        return self.protocol.encode_enable_frame()

    def make_disable_frame(self) -> CANFrame:
        return self.protocol.encode_disable_frame()

    def make_damping_frame(self) -> CANFrame:
        return self.protocol.encode_damping_frame()

    def make_clear_fault_frame(self) -> CANFrame:
        return self.protocol.encode_clear_fault_frame()

    def make_zero_position_frame(self, offset_deg: float = 0.0) -> CANFrame:
        return self.protocol.encode_zero_position_frame(offset_deg=offset_deg)

    def make_torque_command_frame(self, torque_nm: float) -> CANFrame:
        return self.protocol.encode_torque_command_frame(torque_nm)

    def make_velocity_command_frame(self, velocity_rad_s: float) -> CANFrame:
        return self.protocol.encode_velocity_command_frame(velocity_rad_s)

    def make_position_command_frame(
        self,
        position_rad: float,
        velocity_rad_s: float | None = None,
    ) -> CANFrame:
        return self.protocol.encode_position_command_frame(
            position_rad=position_rad,
            velocity_rad_s=velocity_rad_s,
        )

    def make_impedance_command_frame(
        self,
        position_rad: float,
        velocity_rad_s: float,
        kp: float,
        kd: float,
        torque_ff_nm: float = 0.0,
    ) -> CANFrame:
        return self.protocol.encode_impedance_command_frame(
            position_rad=position_rad,
            velocity_rad_s=velocity_rad_s,
            kp=kp,
            kd=kd,
            torque_ff_nm=torque_ff_nm,
        )

    def make_command_frame(self, command: ActuatorCommand) -> CANFrame:
        return self.protocol.encode_command_frame(command)

    # ------------------------------------------------------------------
    # State and communication status
    # ------------------------------------------------------------------

    def get_state(self) -> ActuatorState:
        with self._lock:
            return deepcopy(self._state)

    @property
    def state(self) -> ActuatorState:
        """
        Return a copy of the latest actuator state.

        The returned object is detached from the internal state to prevent
        accidental external mutation.
        """
        return self.get_state()

    def update_fault_flags(self):
        return self.comm_manager.update_fault_flags()

    def get_comm_status(self):
        return self.comm_manager.get_status()

    def is_fresh(self) -> bool:
        return self.comm_manager.is_fresh()

    # ------------------------------------------------------------------
    # Optional transaction helper
    # ------------------------------------------------------------------

    def start_transaction(
        self,
        expected_ids: set[int],
        timeout: float,
    ):
        return self.transaction_manager.start_transaction(
            expected_ids=expected_ids,
            timeout=timeout,
        )

    def wait_transaction(self, timeout: float | None = None) -> bool:
        return self.transaction_manager.wait(timeout=timeout)

    def clear_transaction(self) -> None:
        self.transaction_manager.clear()
