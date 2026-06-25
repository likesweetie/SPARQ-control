from __future__ import annotations

import time
import logging
from collections import deque
from dataclasses import dataclass, field
from typing import Deque

from .bus_load import BusLoadWindow, estimate_classical_can_bits
from .can_decode import (
    SPG_CMD_CLEAR_ERROR,
    SPG_CMD_MIT_CONTROL,
    SPG_CMD_MIT_ENTER,
    SPG_CMD_MIT_EXIT,
    SPG_CMD_MIT_SET_ZERO,
    decode_e2box_gyro,
    decode_e2box_quat,
    decode_spg_status,
    hex_data,
    spg_opcode_name,
)
from .socketcan_io import ParsedFrame


logger = logging.getLogger(__name__)


def monotonic() -> float:
    return time.monotonic()


def hex_id(can_id: int) -> str:
    return f"0x{can_id:03X}"


@dataclass
class RawFrameState:
    can_id: int
    count: int = 0
    last_t: float = 0.0
    last_dlc: int = 0
    last_data: bytes = b""


@dataclass
class NodeState:
    key: str
    name: str
    can_id: int
    role: str
    timeout_s: float
    heartbeat_window_s: float
    rx_count: int = 0
    last_t: float = 0.0
    last_data: bytes = b""
    timestamps: Deque[float] = field(default_factory=deque)

    def mark_rx(self, now: float, data: bytes) -> None:
        self.rx_count += 1
        self.last_t = now
        self.last_data = data
        self.timestamps.append(now)
        self.prune(now)

    def prune(self, now: float) -> None:
        cutoff = now - self.heartbeat_window_s
        while self.timestamps and self.timestamps[0] < cutoff:
            self.timestamps.popleft()

    def snapshot(self, now: float) -> dict:
        self.prune(now)
        age_s = None if self.last_t <= 0.0 else max(0.0, now - self.last_t)
        status = "never"
        if age_s is not None:
            status = "timeout" if age_s > self.timeout_s else "online"
        return {
            "key": self.key,
            "name": self.name,
            "can_id": hex_id(self.can_id),
            "role": self.role,
            "heartbeat_hz": self.heartbeat_hz(),
            "last_seen_s": age_s,
            "timeout_s": self.timeout_s,
            "status": status,
            "rx_count": self.rx_count,
            "last_data": hex_data(self.last_data),
        }

    def heartbeat_hz(self) -> float:
        if len(self.timestamps) < 2:
            return 0.0
        duration_s = self.timestamps[-1] - self.timestamps[0]
        if duration_s <= 1e-9:
            return 0.0
        return (len(self.timestamps) - 1) / duration_s


@dataclass
class E2BoxState:
    req_count: int = 0
    quat_count: int = 0
    gyro_count: int = 0
    quat_last_t: float = 0.0
    gyro_last_t: float = 0.0
    quat_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    projected_gravity_b: tuple[float, float, float] = (0.0, 0.0, -1.0)
    angular_velocity_rad_s: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class SPGMotorState:
    can_id: int
    name: str = ""
    rx_count: int = 0
    last_t: float = 0.0
    last_kind: str = "never"
    enabled_hint: bool | None = None
    temperature_c: int | None = None
    iq_a_approx: float | None = None
    speed_dps: int | None = None
    position_rad: float | None = None
    zero_offset_count: int | None = None
    raw_data: bytes = b""


@dataclass
class MonitorState:
    iface: str
    bitrate: int
    bus_window_s: float
    heartbeat_window_s: float
    node_timeout_s: float
    stuff_factor: float
    feedback_position_max_rad: float
    iq_full_scale_count: float
    iq_full_scale_current_a: float
    mit_p_max_rad: float
    mit_v_max_rad_s: float
    mit_kp_max: float
    mit_kd_max: float
    mit_tau_max_nm: float
    imu_request_id: int
    imu_quat_id: int
    imu_gyro_id: int
    imu_quat_scale: float
    imu_gyro_scale: float
    imu_normalize_quat: bool
    actuator_configs: tuple[dict, ...] = ()
    tx_enabled: bool = False
    allow_actuator_commands: bool = False
    socket_status: str = "disconnected"
    socket_error: str | None = None
    mit_poll_can_ids: set[int] = field(default_factory=set)
    mit_poll_hz: float = 50.0
    start_t: float = field(default_factory=monotonic)
    total_rx: int = 0
    total_tx: int = 0
    raw_frames: dict[int, RawFrameState] = field(default_factory=dict)
    nodes: dict[str, NodeState] = field(default_factory=dict)
    imu: E2BoxState = field(default_factory=E2BoxState)
    motors: dict[int, SPGMotorState] = field(default_factory=dict)
    bus_load: BusLoadWindow = field(default_factory=BusLoadWindow)

    def __post_init__(self) -> None:
        self.bus_load.window_s = self.bus_window_s
        self.bus_load.bitrate = float(self.bitrate)
        self._normalize_actuator_configs()
        self.ensure_known_nodes()

    def _normalize_actuator_configs(self) -> None:
        normalized = []
        for item in self.actuator_configs:
            can_id = int(item["can_id"])
            name = str(item["name"])
            normalized.append({"can_id": can_id, "name": name})
        self.actuator_configs = tuple(sorted(normalized, key=lambda item: item["can_id"]))

    def ensure_known_nodes(self) -> None:
        self._ensure_node("imu_req", "E2Box command", self.imu_request_id, "IMU")
        self._ensure_node("imu_quat", "E2Box quaternion", self.imu_quat_id, "IMU")
        self._ensure_node("imu_gyro", "E2Box gyro", self.imu_gyro_id, "IMU")

        for actuator in self.actuator_configs:
            can_id = int(actuator["can_id"])
            name = str(actuator["name"])
            self._ensure_node(f"can_{can_id:03X}", name, can_id, "Actuator")
            self.motors.setdefault(
                can_id,
                SPGMotorState(can_id=can_id, name=name),
            )

    def actuator_config_for_can_id(self, can_id: int) -> dict | None:
        for actuator in self.actuator_configs:
            if int(actuator["can_id"]) == can_id:
                return actuator
        return None

    def _ensure_node(self, key: str, name: str, can_id: int, role: str) -> NodeState:
        node = self.nodes.get(key)
        if node is None:
            node = NodeState(
                key=key,
                name=name,
                can_id=can_id,
                role=role,
                timeout_s=self.node_timeout_s,
                heartbeat_window_s=self.heartbeat_window_s,
            )
            self.nodes[key] = node
        return node

    def _node_for_can_id(self, can_id: int) -> NodeState:
        for node in self.nodes.values():
            if node.can_id == can_id:
                return node
        return self._ensure_node(f"can_{can_id:03X}", f"CAN {can_id:03X}", can_id, "Raw")

    def mark_rx(self, frame: ParsedFrame, now: float | None = None) -> None:
        now = monotonic() if now is None else now
        self.total_rx += 1
        bits = estimate_classical_can_bits(
            frame.dlc,
            is_eff=frame.is_eff,
            is_rtr=frame.is_rtr,
            stuff_factor=self.stuff_factor,
        )
        self.bus_load.add(now, bits, "rx")
        self._update_raw_frame(frame, now)
        self._node_for_can_id(frame.can_id).mark_rx(now, frame.data)
        self._update_imu(frame, now)
        self._update_motor(frame, now)

    def mark_tx(self, can_id: int, data: bytes, now: float | None = None) -> None:
        now = monotonic() if now is None else now
        self.total_tx += 1
        bits = estimate_classical_can_bits(len(data), stuff_factor=self.stuff_factor)
        self.bus_load.add(now, bits, "tx")
        if can_id == self.imu_request_id:
            self.imu.req_count += 1
        self._update_motor_tx_hint(can_id, data, now)

    def _update_motor_tx_hint(self, can_id: int, data: bytes, now: float) -> None:
        actuator = self.actuator_config_for_can_id(can_id)
        if not data or actuator is None:
            return

        motor = self.motors.setdefault(
            can_id,
            SPGMotorState(can_id=can_id, name=str(actuator["name"])),
        )

        opcode = data[0]
        if opcode == SPG_CMD_MIT_ENTER:
            motor.last_t = now
            motor.raw_data = data[:8]
            motor.last_kind = "MIT_ENTER_SENT"
            motor.enabled_hint = True
        elif opcode == SPG_CMD_MIT_EXIT:
            motor.last_t = now
            motor.raw_data = data[:8]
            motor.last_kind = "MIT_EXIT_SENT"
            motor.enabled_hint = False
        elif opcode == SPG_CMD_MIT_SET_ZERO:
            motor.last_t = now
            motor.raw_data = data[:8]
            motor.last_kind = "MIT_SET_ZERO_SENT"
            if len(data) >= 8:
                motor.zero_offset_count = int.from_bytes(data[6:8], "little", signed=True)

    def _update_raw_frame(self, frame: ParsedFrame, now: float) -> None:
        raw = self.raw_frames.get(frame.can_id)
        if raw is None:
            raw = RawFrameState(can_id=frame.can_id)
            self.raw_frames[frame.can_id] = raw
        raw.count += 1
        raw.last_t = now
        raw.last_dlc = frame.dlc
        raw.last_data = frame.data

    def _update_imu(self, frame: ParsedFrame, now: float) -> None:
        if frame.can_id == self.imu_request_id:
            self.imu.req_count += 1
            return
        if frame.can_id == self.imu_quat_id and frame.dlc == 8:
            try:
                quat, gravity = decode_e2box_quat(
                    frame.data,
                    quat_scale=self.imu_quat_scale,
                    normalize=self.imu_normalize_quat,
                )
            except ValueError as exc:
                logger.warning(
                    "Rejected invalid E2Box quaternion frame can_id=%s data=%s: %s",
                    hex_id(frame.can_id),
                    hex_data(frame.data),
                    exc,
                )
                return
            self.imu.quat_count += 1
            self.imu.quat_last_t = now
            self.imu.quat_xyzw = quat
            self.imu.projected_gravity_b = gravity
            return
        if frame.can_id == self.imu_gyro_id and frame.dlc == 8:
            try:
                gyro = decode_e2box_gyro(frame.data, gyro_scale=self.imu_gyro_scale)
            except ValueError as exc:
                logger.warning(
                    "Rejected invalid E2Box gyro frame can_id=%s data=%s: %s",
                    hex_id(frame.can_id),
                    hex_data(frame.data),
                    exc,
                )
                return
            self.imu.gyro_count += 1
            self.imu.gyro_last_t = now
            self.imu.angular_velocity_rad_s = gyro

    def _update_motor(self, frame: ParsedFrame, now: float) -> None:
        actuator = self.actuator_config_for_can_id(frame.can_id)
        if actuator is None:
            return

        motor = self.motors.setdefault(
            frame.can_id,
            SPGMotorState(can_id=frame.can_id, name=str(actuator["name"])),
        )
        motor.rx_count += 1
        motor.last_t = now
        motor.raw_data = frame.data

        if frame.dlc < 1:
            motor.last_kind = "empty"
            return

        opcode = frame.data[0]
        motor.last_kind = spg_opcode_name(opcode)
        if opcode == SPG_CMD_MIT_CONTROL and frame.dlc == 8:
            try:
                decoded = decode_spg_status(
                    frame.data,
                    feedback_position_max_rad=self.feedback_position_max_rad,
                    iq_full_scale_count=self.iq_full_scale_count,
                    iq_full_scale_current_a=self.iq_full_scale_current_a,
                )
            except ValueError as exc:
                logger.warning(
                    "Rejected invalid SPG status frame can_id=%s data=%s: %s",
                    hex_id(frame.can_id),
                    hex_data(frame.data),
                    exc,
                )
                return
            motor.temperature_c = decoded["temperature_c"]
            motor.iq_a_approx = decoded["iq_a_approx"]
            motor.speed_dps = decoded["speed_dps"]
            motor.position_rad = decoded["position_rad"]
        elif opcode == SPG_CMD_MIT_ENTER:
            motor.enabled_hint = True
        elif opcode in (SPG_CMD_MIT_EXIT, SPG_CMD_CLEAR_ERROR):
            motor.enabled_hint = False
        elif opcode == SPG_CMD_MIT_SET_ZERO and frame.dlc == 8:
            motor.zero_offset_count = int.from_bytes(frame.data[6:8], "little", signed=True)

    def snapshot(self) -> dict:
        now = monotonic()
        bus = self.bus_load.snapshot(now)
        return {
            "time": now,
            "uptime_s": now - self.start_t,
            "can": {
                "iface": self.iface,
                "socket_status": self.socket_status,
                "socket_error": self.socket_error,
                "total_rx": self.total_rx,
                "total_tx": self.total_tx,
                **bus,
            },
            "nodes": [node.snapshot(now) for node in sorted(self.nodes.values(), key=lambda item: item.can_id)],
            "imu": self._imu_snapshot(now),
            "motors": [self._motor_snapshot(motor, now) for motor in sorted(self.motors.values(), key=lambda item: item.can_id)],
            "recent_frames": self._recent_frames(now),
            "processes": [],
            "shm": {
                "control_state": {
                    "status": "disabled",
                    "shm_name": None,
                    "age_s": None,
                    "payload": None,
                },
                "dashboard_state": {
                    "status": "disabled",
                    "shm_name": None,
                    "age_s": None,
                    "payload": None,
                },
            },
            "safety": {
                "tx_enabled": self.tx_enabled,
                "allow_actuator_commands": self.allow_actuator_commands,
            },
            "controls": {
                "mit_polling": bool(self.mit_poll_can_ids),
                "mit_poll_can_ids": [hex_id(can_id) for can_id in sorted(self.mit_poll_can_ids)],
                "mit_poll_hz": self.mit_poll_hz,
            },
        }

    def _imu_snapshot(self, now: float) -> dict:
        return {
            "req_count": self.imu.req_count,
            "quat_count": self.imu.quat_count,
            "gyro_count": self.imu.gyro_count,
            "quat_xyzw": list(self.imu.quat_xyzw),
            "projected_gravity_b": list(self.imu.projected_gravity_b),
            "angular_velocity_rad_s": list(self.imu.angular_velocity_rad_s),
            "quat_age_s": None if self.imu.quat_last_t <= 0.0 else now - self.imu.quat_last_t,
            "gyro_age_s": None if self.imu.gyro_last_t <= 0.0 else now - self.imu.gyro_last_t,
        }

    def _motor_snapshot(self, motor: SPGMotorState, now: float) -> dict:
        age_s = None if motor.last_t <= 0.0 else now - motor.last_t
        status = "never" if age_s is None else ("timeout" if age_s > self.node_timeout_s else "online")
        return {
            "name": motor.name,
            "can_id": hex_id(motor.can_id),
            "rx_count": motor.rx_count,
            "age_s": age_s,
            "status": status,
            "last_kind": motor.last_kind,
            "enabled_hint": motor.enabled_hint,
            "mit_polling": motor.can_id in self.mit_poll_can_ids,
            "temperature_c": motor.temperature_c,
            "current_a": motor.iq_a_approx,
            "iq_a_approx": motor.iq_a_approx,
            "speed_dps": motor.speed_dps,
            "position_rad": motor.position_rad,
            "zero_offset_count": motor.zero_offset_count,
            "raw": hex_data(motor.raw_data),
        }

    def _recent_frames(self, now: float) -> list[dict]:
        frames = sorted(self.raw_frames.values(), key=lambda item: item.last_t, reverse=True)[:24]
        return [
            {
                "can_id": hex_id(frame.can_id),
                "dlc": frame.last_dlc,
                "data": hex_data(frame.last_data),
                "count": frame.count,
                "age_s": None if frame.last_t <= 0.0 else now - frame.last_t,
            }
            for frame in frames
        ]
