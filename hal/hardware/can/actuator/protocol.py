"""
Actuator protocol base class.

Product-specific CAN actuator protocols should inherit from
ActuatorProtocolBase and implement frame encoding/decoding.

Design rule:
- Protocol classes know CAN IDs, opcodes, byte layout, scaling, and sign convention.
- Protocol classes do not own CANDaemon or CANDispatcher.
- ActuatorDriver owns state and communication freshness policy.
"""

from __future__ import annotations

from abc import abstractmethod

from hal.can_bus import CANFrame
from hal.hardware.can.device_protocol import CANDeviceProtocolBase

from .state import ActuatorCommand, ActuatorState


class ActuatorProtocolBase(CANDeviceProtocolBase):
    """
    Base class for CAN actuator protocols.

    Concrete subclasses should implement rx_can_ids() and decode_frame().
    Command encoders are optional at the base level because not every actuator
    supports every control mode.
    """

    @abstractmethod
    def rx_can_ids(self) -> list[int]:
        """
        Return CAN IDs that carry feedback or status frames for this actuator.
        """
        raise NotImplementedError

    @abstractmethod
    def decode_frame(self, frame: CANFrame) -> ActuatorState | None:
        """
        Decode a received CAN frame into a partial ActuatorState.

        Returns:
            ActuatorState:
                Partial or full decoded state.
            None:
                If this protocol does not handle the frame.
        """
        raise NotImplementedError

    def is_feedback_frame(self, frame: CANFrame) -> bool:
        """
        Return True if the frame belongs to this actuator's RX ID set.
        """
        return frame.can_id in self.rx_can_ids()

    # ------------------------------------------------------------------
    # Optional command encoders.
    # Product-specific protocols should override the supported methods.
    # ------------------------------------------------------------------

    def encode_enable_frame(self) -> CANFrame:
        raise NotImplementedError(f"{self.__class__.__name__} does not support enable command")

    def encode_disable_frame(self) -> CANFrame:
        raise NotImplementedError(f"{self.__class__.__name__} does not support disable command")

    def encode_damping_frame(self) -> CANFrame:
        raise NotImplementedError(f"{self.__class__.__name__} does not support damping command")

    def encode_clear_fault_frame(self) -> CANFrame:
        raise NotImplementedError(f"{self.__class__.__name__} does not support clear-fault command")

    def encode_zero_position_frame(self, offset_deg: float = 0.0) -> CANFrame:
        raise NotImplementedError(f"{self.__class__.__name__} does not support zero-position command")

    def encode_torque_command_frame(self, torque_nm: float) -> CANFrame:
        raise NotImplementedError(f"{self.__class__.__name__} does not support torque command")

    def encode_velocity_command_frame(self, velocity_rad_s: float) -> CANFrame:
        raise NotImplementedError(f"{self.__class__.__name__} does not support velocity command")

    def encode_position_command_frame(
        self,
        position_rad: float,
        velocity_rad_s: float | None = None,
    ) -> CANFrame:
        raise NotImplementedError(f"{self.__class__.__name__} does not support position command")

    def encode_impedance_command_frame(
        self,
        position_rad: float,
        velocity_rad_s: float,
        kp: float,
        kd: float,
        torque_ff_nm: float = 0.0,
    ) -> CANFrame:
        raise NotImplementedError(f"{self.__class__.__name__} does not support impedance command")


    def encode_MIT_command_frame(
        self,
        position_rad: float,
        velocity_rad_s: float,
        kp: float,
        kd: float,
        torque_ff_nm: float = 0.0,
    ) -> CANFrame:
        raise NotImplementedError(f"{self.__class__.__name__} does not support MIT command")


    def encode_command_frame(self, command: ActuatorCommand) -> CANFrame:
        """
        Optional generic command encoder.

        Concrete protocols can override this method if their command mode maps
        cleanly from ActuatorCommand.
        """
        raise NotImplementedError(f"{self.__class__.__name__} does not support generic ActuatorCommand")
