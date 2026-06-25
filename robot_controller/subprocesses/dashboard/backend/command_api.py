from __future__ import annotations

from typing import Callable

from .can_decode import (
    SPG_CMD_MIT_CONTROL,
    SPG_CMD_MIT_ENTER,
    SPG_CMD_MIT_EXIT,
    SPG_CMD_MIT_SET_ZERO,
)
from .socketcan_io import CAN_FRAME_SIZE
from .state import MonitorState
from hal.can_bus import CANFrame
from hal.can_bus.process_client import CANProcessClient


class CommandError(RuntimeError):
    pass


ENABLE_BLOCK_STATES = {
    "CREATED",
    "DISABLED",
    "DISARMED",
    "FAULT_LATCHED",
    "ESTOP",
    "SHUTTING_DOWN",
    "STOPPED",
}
ZERO_SET_ALLOWED_STATES = {"NORMAL"}


def normalize_state_name(value: str | None) -> str | None:
    if value is None:
        return None
    normalized = "".join(char if char.isalnum() else "_" for char in value.strip().upper())
    normalized = "_".join(part for part in normalized.split("_") if part)
    return normalized or None


def is_operator_estop_damping(state_name: str | None, reason: str | None) -> bool:
    return state_name == "DAMPING" and "E-STOP" in str(reason or "").upper()


def format_tx_result(can_id: int, data: bytes, sent_bytes: int) -> dict:
    return {
        "can_id": f"0x{can_id:03X}",
        "data": data.hex(" ").upper(),
        "sent_bytes": sent_bytes,
    }


def require_range(value: float, lo: float, hi: float, field: str) -> None:
    if value < lo or value > hi:
        raise ValueError(f"MIT {field} out of range: {value} not in [{lo}, {hi}]")


def float_to_uint(value: float, value_min: float, value_max: float, bits: int) -> int:
    require_range(value, value_min, value_max, "field")
    span = value_max - value_min
    max_int = (1 << bits) - 1
    return int(round((value - value_min) * max_int / span))


def pack_mit_payload(
    q_rad: float,
    qd_rad_s: float,
    kp: float,
    kd: float,
    tau_ff_nm: float,
    *,
    p_max: float = 12.5,
    v_max: float = 45.0,
    kp_max: float = 500.0,
    kd_max: float = 5.0,
    tau_max: float = 33.0,
) -> bytes:
    require_range(q_rad, -p_max, p_max, "q_rad")
    require_range(qd_rad_s, -v_max, v_max, "qd_rad_s")
    require_range(kp, 0.0, kp_max, "kp")
    require_range(kd, 0.0, kd_max, "kd")
    require_range(tau_ff_nm, -tau_max, tau_max, "tau_ff_nm")
    p_u = float_to_uint(q_rad, -p_max, p_max, 16)
    v_u = float_to_uint(qd_rad_s, -v_max, v_max, 12)
    kp_u = float_to_uint(kp, 0.0, kp_max, 12)
    kd_u = float_to_uint(kd, 0.0, kd_max, 8)
    t_u = float_to_uint(tau_ff_nm, -tau_max, tau_max, 8)

    data = bytearray(8)
    data[0] = SPG_CMD_MIT_CONTROL
    data[1] = (p_u >> 8) & 0xFF
    data[2] = p_u & 0xFF
    data[3] = (v_u >> 4) & 0xFF
    data[4] = ((v_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F)
    data[5] = kp_u & 0xFF
    data[6] = kd_u & 0xFF
    data[7] = t_u & 0xFF
    return bytes(data)


class CommandService:
    def __init__(
        self,
        state: MonitorState,
        can_client: CANProcessClient,
        *,
        controller_safety_state_provider: Callable[[], str | None] | None = None,
        controller_safety_reason_provider: Callable[[], str | None] | None = None,
    ) -> None:
        self.state = state
        self.can_client = can_client
        self.controller_safety_state_provider = controller_safety_state_provider
        self.controller_safety_reason_provider = controller_safety_reason_provider
        self._connected = False

    def lock_tx(self) -> None:
        self.state.tx_enabled = False

    def unlock_tx(self) -> None:
        self.state.tx_enabled = True

    def require_tx(self) -> CANProcessClient:
        if not self.state.tx_enabled:
            raise CommandError("TX is locked")
        if not self._connected:
            try:
                self.can_client.connect()
            except (OSError, RuntimeError) as exc:
                self.state.socket_error = str(exc)
                raise CommandError(f"CAN daemon is not connected: {exc}") from exc
            self._connected = True
        return self.can_client

    def send_raw(self, can_id: int, data: bytes) -> dict:
        self.reject_motor_enable_when_blocked(can_id, data)
        self.reject_motor_zero_when_not_normal(can_id, data)
        can_client = self.require_tx()
        try:
            can_client.send(CANFrame(can_id=can_id, data=data))
        except (OSError, RuntimeError) as exc:
            self._connected = False
            self.state.socket_error = str(exc)
            raise CommandError(str(exc)) from exc
        self.state.mark_tx(can_id, data)
        self.state.socket_error = None
        return format_tx_result(can_id, data, CAN_FRAME_SIZE)

    def reject_motor_enable_when_blocked(self, can_id: int, data: bytes) -> None:
        if not data or data[0] != SPG_CMD_MIT_ENTER:
            return
        if self.state.actuator_config_for_can_id(can_id) is None:
            return
        state_name = self.controller_safety_state()
        if self.controller_safety_state_provider is not None and state_name is None:
            raise CommandError("Motor enable is blocked because controller safety state is unavailable")
        if state_name in ENABLE_BLOCK_STATES:
            raise CommandError(
                f"Motor enable is blocked while controller safety state is {state_name}"
            )
        if is_operator_estop_damping(state_name, self.controller_safety_reason()):
            raise CommandError("Motor enable is blocked while controller is in operator E-stop damping")

    def reject_motor_zero_when_not_normal(self, can_id: int, data: bytes) -> None:
        if not data or data[0] != SPG_CMD_MIT_SET_ZERO:
            return
        if self.state.actuator_config_for_can_id(can_id) is None:
            return
        self.require_controller_state(
            action_name="Motor zero set",
            allowed_states=ZERO_SET_ALLOWED_STATES,
        )

    def require_controller_state(self, *, action_name: str, allowed_states: set[str]) -> None:
        state_name = self.controller_safety_state()
        if self.controller_safety_state_provider is None or state_name is None:
            raise CommandError(f"{action_name} is blocked because controller safety state is unavailable")
        if state_name not in allowed_states:
            allowed = ", ".join(sorted(allowed_states))
            raise CommandError(
                f"{action_name} is blocked while controller safety state is {state_name}; "
                f"required: {allowed}"
            )

    def controller_safety_state(self) -> str | None:
        if self.controller_safety_state_provider is None:
            return None
        return normalize_state_name(self.controller_safety_state_provider())

    def controller_safety_reason(self) -> str | None:
        if self.controller_safety_reason_provider is None:
            return None
        return self.controller_safety_reason_provider()

    def send_motor_command(self, can_id: int, opcode: int, suffix: bytes = b"") -> dict:
        if not self.state.allow_actuator_commands:
            raise CommandError("Actuator commands are disabled by config")
        if self.state.actuator_config_for_can_id(can_id) is None:
            raise CommandError(f"Unknown actuator CAN ID 0x{can_id:03X}")
        data = bytes([opcode]) + b"\x00" * 7
        if suffix:
            if len(suffix) > 7:
                raise CommandError(f"Actuator command suffix too long: {len(suffix)}/7 bytes")
            data = bytes([opcode]) + b"\x00" * (7 - len(suffix)) + suffix
        return self.send_raw(can_id, data)

    def motor_enter(self, can_id: int) -> dict:
        return self.send_motor_command(can_id, SPG_CMD_MIT_ENTER)

    def motor_exit(self, can_id: int) -> dict:
        return self.send_motor_command(can_id, SPG_CMD_MIT_EXIT)

    def motor_zero(self, can_id: int, offset_count: int = 0) -> dict:
        if not (-32768 <= int(offset_count) <= 32767):
            raise CommandError(f"MIT zero offset_count out of int16 range: {offset_count}")
        suffix = int(offset_count).to_bytes(2, "little", signed=True)
        return self.send_motor_command(can_id, SPG_CMD_MIT_SET_ZERO, suffix=suffix)

    def motor_mit_hold(self, can_id: int) -> dict:
        if not self.state.allow_actuator_commands:
            raise CommandError("Actuator commands are disabled by config")
        payload = pack_mit_payload(
            q_rad=0.0,
            qd_rad_s=0.0,
            kp=0.0,
            kd=0.5,
            tau_ff_nm=0.0,
            p_max=self.state.mit_p_max_rad,
            v_max=self.state.mit_v_max_rad_s,
            kp_max=self.state.mit_kp_max,
            kd_max=self.state.mit_kd_max,
            tau_max=self.state.mit_tau_max_nm,
        )
        return self.send_raw(can_id, payload)
