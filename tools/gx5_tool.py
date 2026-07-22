#!/usr/bin/env python3
"""PyQt6 standalone monitor/configurator for LORD 3DM-GX5-25.

This program opens the IMU serial port directly. Stop SensorConnect and every
other process using the same port before launching it.

Live monitor:
  - Estimation-filter quaternion (w, x, y, z)
  - Compensated angular rate (x, y, z) [rad/s]
  - Projected gravity in vehicle frame
  - Host receive interval, measured rate, jitter, sample age
  - Filter state/status flags

Interactive controls:
  - Sensor -> Vehicle Euler roll/pitch/yaw [deg]
  - Quaternion/gyro and status output rates
  - Auto initialization and pitch/roll aiding
  - Apply current settings, read back, reset filter
  - Restore launch snapshot, load startup, save startup

APPLY CURRENT only changes current runtime settings. SAVE STARTUP performs a
persistent write after a confirmation dialog.
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Optional, Sequence

try:
    from python_mscl import mscl  # type: ignore
except ImportError:
    try:
        import MSCL as mscl  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "MSCL Python binding is not installed. Try:\n"
            "  python3 -m pip install python-mscl\n"
            "or install the official MSCL Python binding."
        ) from exc

try:
    import pyqtgraph as pg
    from PyQt6 import QtCore, QtGui, QtWidgets
except ImportError as exc:
    raise SystemExit(
        "PyQt6/pyqtgraph is not installed. Install with:\n"
        "  python3 -m pip install PyQt6 pyqtgraph"
    ) from exc


FILTER_QUATERNION_FIELD = 0x8203
FILTER_COMP_ANG_RATE_FIELD = 0x820E
FILTER_STATUS_FIELD = 0x8210
TOOL_VERSION = "vectorfix-v2-20260712"

FILTER_STATE_NAMES = {
    0: "STARTUP",
    1: "INITIALIZING",
    2: "RUNNING_VALID",
    3: "RUNNING_ERROR",
}


def _resolve_mip_constant(candidates: Sequence[str], fallback: int) -> int:
    for name in candidates:
        value = getattr(mscl.MipTypes, name, None)
        if value is not None:
            return int(value)
    return fallback


QUAT_FIELD = _resolve_mip_constant(
    (
        "CH_FIELD_ESTFILTER_ATTITUDE_QUATERNION",
        "CH_FIELD_ESTFILTER_ORIENTATION_QUATERNION",
        "CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION",
        "CH_FIELD_ESTFILTER_ESTIMATED_ORIENTATION_QUATERNION",
    ),
    FILTER_QUATERNION_FIELD,
)
ANG_RATE_FIELD = _resolve_mip_constant(
    (
        "CH_FIELD_ESTFILTER_COMP_ANGULAR_RATE",
        "CH_FIELD_ESTFILTER_COMPENSATED_ANGULAR_RATE",
        "CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE",
    ),
    FILTER_COMP_ANG_RATE_FIELD,
)
STATUS_FIELD = _resolve_mip_constant(
    ("CH_FIELD_ESTFILTER_FILTER_STATUS", "CH_FIELD_ESTFILTER_STATUS"),
    FILTER_STATUS_FIELD,
)


def normalize_name(name: str) -> str:
    return "".join(ch.lower() for ch in name if ch.isalnum())


def safe_int(value: Any) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return -1


def read_scalar(data_point: Any) -> float:
    getters = (
        "as_float",
        "as_double",
        "as_uint16",
        "as_int16",
        "as_uint32",
        "as_int32",
        "as_uint8",
        "as_int8",
    )
    last_error: Optional[Exception] = None
    for getter_name in getters:
        getter = getattr(data_point, getter_name, None)
        if getter is None:
            continue
        try:
            return float(getter())
        except Exception as exc:
            last_error = exc
    try:
        return float(data_point.as_string())
    except Exception as exc:
        last_error = exc
    raise ValueError(
        f"Cannot convert data point {data_point.channelName()} to scalar"
    ) from last_error


def read_vector(data_point: Any) -> Optional[list[float]]:
    """Read an MSCL vector-valued data point as Python floats.

    GX5 quaternion and compensated angular-rate fields are commonly exposed
    as one Vector data point rather than separate scalar component points.
    """
    getter = getattr(data_point, "as_Vector", None)
    if getter is None:
        return None

    try:
        vector = getter()
        size_member = getattr(vector, "size", None)
        if size_member is None:
            return None
        size = int(size_member() if callable(size_member) else size_member)
    except Exception:
        return None

    values: list[float] = []
    for index in range(size):
        value = None
        for method_name in (
            "as_doubleAt",
            "as_floatAt",
            "as_uint16At",
            "as_uint8At",
        ):
            method = getattr(vector, method_name, None)
            if method is None:
                continue
            try:
                value = float(method(index))
                break
            except Exception:
                continue
        if value is None:
            return None
        values.append(value)

    return values


def detect_component(channel_name: str, components: Sequence[str]) -> Optional[str]:
    normalized = normalize_name(channel_name)
    for component in components:
        c = component.lower()
        if normalized.endswith(c):
            return component
        if f"{c}axis" in normalized or f"axis{c}" in normalized:
            return component
    return None


def quaternion_to_projected_gravity(
    q: Sequence[float],
) -> tuple[float, float, float]:
    """Project world gravity [0, 0, -1] into the vehicle frame."""
    w, x, y, z = (float(v) for v in q)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1.0e-12:
        return 0.0, 0.0, -1.0
    w /= norm
    x /= norm
    y /= norm
    z /= norm
    return (
        2.0 * (w * y - x * z),
        -2.0 * (w * x + y * z),
        2.0 * (x * x + y * y) - 1.0,
    )


def object_component(obj: Any, names: Iterable[str]) -> Optional[float]:
    for name in names:
        member = getattr(obj, name, None)
        if member is None:
            continue
        try:
            value = member() if callable(member) else member
            return float(value)
        except Exception:
            continue
    return None


def read_mount_euler(node: Any) -> tuple[Optional[tuple[float, float, float]], str]:
    method_names = (
        "getSensorToVehicleRotation_eulerAngles",
        "getSensorToVehicleTransform_eulerAngles",
    )
    for method_name in method_names:
        method = getattr(node, method_name, None)
        if method is None:
            continue
        try:
            angles = method()
            roll = object_component(angles, ("roll", "x"))
            pitch = object_component(angles, ("pitch", "y"))
            yaw = object_component(angles, ("yaw", "z"))
            if None not in (roll, pitch, yaw):
                return (float(roll), float(pitch), float(yaw)), method_name
        except Exception as exc:
            return None, f"{method_name}: {exc}"
    return None, "unsupported"


def set_mount_euler(node: Any, mount_deg: Sequence[float]) -> None:
    if len(mount_deg) != 3:
        raise ValueError("mount_deg must contain roll, pitch, yaw")
    angles = mscl.EulerAngles(
        float(math.radians(mount_deg[0])),
        float(math.radians(mount_deg[1])),
        float(math.radians(mount_deg[2])),
    )
    setter = getattr(node, "setSensorToVehicleRotation_eulerAngles", None)
    if setter is None:
        setter = getattr(node, "setSensorToVehicleTransform_eulerAngles", None)
    if setter is None:
        raise RuntimeError("Sensor-to-vehicle Euler setter is unsupported")
    setter(angles)


def read_optional_bool(node: Any, method_name: str) -> Optional[bool]:
    method = getattr(node, method_name, None)
    if method is None:
        return None
    try:
        return bool(method())
    except Exception:
        return None


def format_channels(channels: Any) -> list[str]:
    result: list[str] = []
    for channel in channels:
        try:
            field_value = int(channel.channelField())
        except Exception:
            field_value = -1
        try:
            rate = channel.sampleRate().prettyStr()
        except Exception:
            rate = "?"
        result.append(f"0x{field_value:04X}@{rate}")
    return result


def channel_rates(channels: Any) -> dict[int, float]:
    result: dict[int, float] = {}
    for channel in channels:
        try:
            field_value = int(channel.channelField())
            result[field_value] = float(channel.sampleRate().samplesPerSecond())
        except Exception:
            continue
    return result


def make_channel(field_value: int, rate_hz: int) -> Any:
    """SampleRate.Hertz requires a uint32-compatible integer."""
    rate = int(rate_hz)
    if rate <= 0:
        raise ValueError("sample rate must be a positive integer")
    return mscl.MipChannel(int(field_value), mscl.SampleRate.Hertz(rate))


def valid_rates(base_rate_hz: int) -> list[int]:
    return [rate for rate in range(1, base_rate_hz + 1) if base_rate_hz % rate == 0]


def validate_rate(rate_hz: int, base_rate_hz: int, name: str) -> int:
    rate = int(rate_hz)
    if rate <= 0:
        raise ValueError(f"{name} must be positive")
    if rate > base_rate_hz:
        raise ValueError(f"{name}={rate} exceeds base rate {base_rate_hz}")
    if base_rate_hz % rate != 0:
        raise ValueError(
            f"{name}={rate} is not an exact divisor of base rate {base_rate_hz}"
        )
    return rate


def configure_stream(
    node: Any,
    sample_rate_hz: int,
    status_rate_hz: int,
    base_rate_hz: int,
) -> None:
    sample_rate = validate_rate(sample_rate_hz, base_rate_hz, "sample rate")
    status_rate = validate_rate(status_rate_hz, base_rate_hz, "status rate")
    channels = mscl.MipChannels()
    channels.append(make_channel(QUAT_FIELD, sample_rate))
    channels.append(make_channel(ANG_RATE_FIELD, sample_rate))
    channels.append(make_channel(STATUS_FIELD, status_rate))
    node.setActiveChannelFields(mscl.MipTypes.CLASS_ESTFILTER, channels)


@dataclass(frozen=True)
class DeviceSettings:
    mount_deg: Optional[tuple[float, float, float]]
    auto_init: Optional[bool]
    pitch_roll_aid: Optional[bool]
    sample_rate_hz: Optional[int]
    status_rate_hz: Optional[int]
    stream_enabled: bool
    channels: tuple[str, ...]


def read_device_settings(node: Any) -> DeviceSettings:
    mount_rad, _ = read_mount_euler(node)
    mount_deg = (
        None
        if mount_rad is None
        else tuple(float(math.degrees(v)) for v in mount_rad)
    )
    auto_init = read_optional_bool(node, "getAutoInitialization")
    pitch_roll_aid = read_optional_bool(node, "getPitchRollAid")
    channels = node.getActiveChannelFields(mscl.MipTypes.CLASS_ESTFILTER)
    rates = channel_rates(channels)
    sample_rate = rates.get(QUAT_FIELD) or rates.get(ANG_RATE_FIELD)
    status_rate = rates.get(STATUS_FIELD)
    return DeviceSettings(
        mount_deg=mount_deg,
        auto_init=auto_init,
        pitch_roll_aid=pitch_roll_aid,
        sample_rate_hz=None if sample_rate is None else int(round(sample_rate)),
        status_rate_hz=None if status_rate is None else int(round(status_rate)),
        stream_enabled=bool(
            node.isDataStreamEnabled(mscl.MipTypes.CLASS_ESTFILTER)
        ),
        channels=tuple(format_channels(channels)),
    )


def make_setting_commands(include_stream: bool) -> Any:
    container_type = getattr(mscl, "MipCommands", None)
    if container_type is None:
        raise RuntimeError("This MSCL binding does not expose MipCommands")
    commands = container_type()
    names = [
        "CMD_EF_SENS_VEHIC_FRAME_ROTATION_EULER",
        "CMD_EF_AUTO_INIT_CTRL",
        "CMD_EF_PITCH_ROLL_AID_CTRL",
    ]
    if include_stream:
        names.extend(("CMD_EF_MESSAGE_FORMAT", "CMD_CONTINUOUS_DATA_STREAM"))
    for name in names:
        value = getattr(mscl.MipTypes, name, None)
        if value is None:
            raise RuntimeError(f"MSCL binding does not expose {name}")
        commands.append(value)
    return commands


@dataclass
class MonitorState:
    window_s: float
    max_rate_hz: int
    lock: threading.Lock = field(default_factory=threading.Lock)
    running: bool = True
    error: Optional[str] = None

    t: deque[float] = field(init=False)
    gyro_x: deque[float] = field(init=False)
    gyro_y: deque[float] = field(init=False)
    gyro_z: deque[float] = field(init=False)
    grav_x: deque[float] = field(init=False)
    grav_y: deque[float] = field(init=False)
    grav_z: deque[float] = field(init=False)
    quat_w: deque[float] = field(init=False)
    quat_x: deque[float] = field(init=False)
    quat_y: deque[float] = field(init=False)
    quat_z: deque[float] = field(init=False)
    dt_t: deque[float] = field(init=False)
    dt_ms: deque[float] = field(init=False)

    sample_count: int = 0
    packet_count: int = 0
    invalid_points: int = 0
    incomplete_packets: int = 0
    filter_state: Optional[int] = None
    status_flags: Optional[int] = None
    last_sample_time: Optional[float] = None
    first_sample_time: Optional[float] = None
    latest_q: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    latest_gyro: tuple[float, float, float] = (0.0, 0.0, 0.0)
    latest_gravity: tuple[float, float, float] = (0.0, 0.0, -1.0)

    # MSCL may deliver quaternion and angular-rate fields in separate
    # MipDataPacket objects. Keep the latest complete value from each field
    # and publish a monitor sample when both sides have advanced.
    pending_q: Optional[tuple[float, float, float, float]] = None
    pending_gyro: Optional[tuple[float, float, float]] = None
    q_generation: int = 0
    gyro_generation: int = 0
    committed_q_generation: int = 0
    committed_gyro_generation: int = 0

    discovered_channels: set[str] = field(default_factory=set)

    def __post_init__(self) -> None:
        maxlen = max(2_000, int(self.window_s * self.max_rate_hz * 1.5))
        self.t = deque(maxlen=maxlen)
        self.gyro_x = deque(maxlen=maxlen)
        self.gyro_y = deque(maxlen=maxlen)
        self.gyro_z = deque(maxlen=maxlen)
        self.grav_x = deque(maxlen=maxlen)
        self.grav_y = deque(maxlen=maxlen)
        self.grav_z = deque(maxlen=maxlen)
        self.quat_w = deque(maxlen=maxlen)
        self.quat_x = deque(maxlen=maxlen)
        self.quat_y = deque(maxlen=maxlen)
        self.quat_z = deque(maxlen=maxlen)
        self.dt_t = deque(maxlen=maxlen)
        self.dt_ms = deque(maxlen=maxlen)

    def append_sample(
        self,
        now: float,
        quat: tuple[float, float, float, float],
        gyro: tuple[float, float, float],
    ) -> None:
        gravity = quaternion_to_projected_gravity(quat)
        qnorm = math.sqrt(sum(v * v for v in quat))
        if not math.isfinite(qnorm) or qnorm < 1.0e-8:
            self.invalid_points += 1
            return
        if self.first_sample_time is None:
            self.first_sample_time = now
        rel_t = now - self.first_sample_time
        if self.last_sample_time is not None:
            self.dt_t.append(rel_t)
            self.dt_ms.append((now - self.last_sample_time) * 1_000.0)
        self.last_sample_time = now

        self.t.append(rel_t)
        self.quat_w.append(quat[0])
        self.quat_x.append(quat[1])
        self.quat_y.append(quat[2])
        self.quat_z.append(quat[3])
        self.gyro_x.append(gyro[0])
        self.gyro_y.append(gyro[1])
        self.gyro_z.append(gyro[2])
        self.grav_x.append(gravity[0])
        self.grav_y.append(gravity[1])
        self.grav_z.append(gravity[2])
        self.latest_q = quat
        self.latest_gyro = gyro
        self.latest_gravity = gravity
        self.sample_count += 1

        cutoff = rel_t - self.window_s
        while self.t and self.t[0] < cutoff:
            self.t.popleft()
            self.quat_w.popleft()
            self.quat_x.popleft()
            self.quat_y.popleft()
            self.quat_z.popleft()
            self.gyro_x.popleft()
            self.gyro_y.popleft()
            self.gyro_z.popleft()
            self.grav_x.popleft()
            self.grav_y.popleft()
            self.grav_z.popleft()
        while self.dt_t and self.dt_t[0] < cutoff:
            self.dt_t.popleft()
            self.dt_ms.popleft()

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            now = time.monotonic()
            age_ms = (
                math.inf
                if self.last_sample_time is None
                else (now - self.last_sample_time) * 1_000.0
            )
            dts = list(self.dt_ms)
            recent_dts = dts[-min(len(dts), 500) :]
            if recent_dts:
                median_dt = statistics.median(recent_dts)
                measured_hz = 1_000.0 / median_dt if median_dt > 0.0 else 0.0
                jitter_ms = (
                    statistics.pstdev(recent_dts) if len(recent_dts) > 1 else 0.0
                )
            else:
                measured_hz = 0.0
                jitter_ms = 0.0
            return {
                "t": list(self.t),
                "gyro": [list(self.gyro_x), list(self.gyro_y), list(self.gyro_z)],
                "gravity": [list(self.grav_x), list(self.grav_y), list(self.grav_z)],
                "quat": [
                    list(self.quat_w),
                    list(self.quat_x),
                    list(self.quat_y),
                    list(self.quat_z),
                ],
                "dt_t": list(self.dt_t),
                "dt_ms": dts,
                "sample_count": self.sample_count,
                "packet_count": self.packet_count,
                "invalid_points": self.invalid_points,
                "incomplete_packets": self.incomplete_packets,
                "filter_state": self.filter_state,
                "status_flags": self.status_flags,
                "latest_q": self.latest_q,
                "latest_gyro": self.latest_gyro,
                "latest_gravity": self.latest_gravity,
                "age_ms": age_ms,
                "measured_hz": measured_hz,
                "jitter_ms": jitter_ms,
                "error": self.error,
                "channels": sorted(self.discovered_channels),
            }


def process_packet(packet: Any, state: MonitorState) -> None:
    """Consume one MSCL packet and extract vector or scalar filter fields."""
    quat_parts: dict[str, float] = {}
    gyro_parts: dict[str, float] = {}
    packet_quat: Optional[tuple[float, float, float, float]] = None
    packet_gyro: Optional[tuple[float, float, float]] = None
    packet_channels: set[str] = set()

    for point in packet.data():
        try:
            channel_name = str(point.channelName())
        except Exception:
            channel_name = "unknown"

        packet_channels.add(channel_name)
        normalized = normalize_name(channel_name)

        try:
            field_value = safe_int(point.field())
        except Exception:
            field_value = -1

        try:
            valid = bool(point.valid())
        except Exception:
            valid = True

        if not valid:
            with state.lock:
                state.invalid_points += 1
            continue

        is_quaternion = (
            field_value == QUAT_FIELD
            or "quaternion" in normalized
            or "orientquat" in normalized
        )
        is_gyro = (
            field_value == ANG_RATE_FIELD
            or "compangularrate" in normalized
            or "compensatedangularrate" in normalized
            or ("angular" in normalized and "rate" in normalized)
            or "gyro" in normalized
        )
        is_status = (
            field_value == STATUS_FIELD
            or "filterstatus" in normalized
            or "filterstate" in normalized
        )

        # MSCL normally exposes quaternion/gyro as one vector-valued point.
        vector = read_vector(point)

        if is_quaternion:
            if vector is not None and len(vector) >= 4:
                packet_quat = (
                    float(vector[0]),
                    float(vector[1]),
                    float(vector[2]),
                    float(vector[3]),
                )
                continue

            try:
                value = read_scalar(point)
            except ValueError:
                continue
            component = detect_component(channel_name, ("w", "x", "y", "z"))
            if component is not None:
                quat_parts[component] = value
            continue

        if is_gyro:
            if vector is not None and len(vector) >= 3:
                packet_gyro = (
                    float(vector[0]),
                    float(vector[1]),
                    float(vector[2]),
                )
                continue

            try:
                value = read_scalar(point)
            except ValueError:
                continue
            component = detect_component(channel_name, ("x", "y", "z"))
            if component is not None:
                gyro_parts[component] = value
            continue

        if is_status:
            # Some MSCL builds expose status as [filter_state, status_flags].
            if vector is not None and len(vector) >= 2:
                with state.lock:
                    state.filter_state = int(vector[0])
                    state.status_flags = int(vector[1])
                continue

            try:
                value = read_scalar(point)
            except ValueError:
                continue

            with state.lock:
                if "state" in normalized and "flag" not in normalized:
                    state.filter_state = int(value)
                elif "flag" in normalized:
                    state.status_flags = int(value)
                elif state.filter_state is None:
                    state.filter_state = int(value)
                else:
                    state.status_flags = int(value)

    if packet_quat is None and all(k in quat_parts for k in ("w", "x", "y", "z")):
        packet_quat = (
            quat_parts["w"],
            quat_parts["x"],
            quat_parts["y"],
            quat_parts["z"],
        )

    if packet_gyro is None and all(k in gyro_parts for k in ("x", "y", "z")):
        packet_gyro = (
            gyro_parts["x"],
            gyro_parts["y"],
            gyro_parts["z"],
        )

    now = time.monotonic()

    with state.lock:
        state.packet_count += 1
        state.discovered_channels.update(packet_channels)

        # Only count a field as incomplete when that field was recognized but
        # lacked one or more required components.
        if quat_parts and packet_quat is None:
            state.incomplete_packets += 1
        if gyro_parts and packet_gyro is None:
            state.incomplete_packets += 1

        if packet_quat is not None:
            state.pending_q = packet_quat
            state.q_generation += 1

        if packet_gyro is not None:
            state.pending_gyro = packet_gyro
            state.gyro_generation += 1

        # Quaternion and gyro may arrive in separate packet objects. Commit
        # after both streams have produced a new complete value.
        if (
            state.pending_q is not None
            and state.pending_gyro is not None
            and state.q_generation > state.committed_q_generation
            and state.gyro_generation > state.committed_gyro_generation
        ):
            state.append_sample(now, state.pending_q, state.pending_gyro)
            state.committed_q_generation = state.q_generation
            state.committed_gyro_generation = state.gyro_generation


class ReaderThread(QtCore.QThread):
    fatal_error = QtCore.pyqtSignal(str)

    def __init__(
        self,
        node: Any,
        state: MonitorState,
        node_lock: threading.Lock,
        timeout_ms: int = 25,
    ) -> None:
        super().__init__()
        self.node = node
        self.state = state
        self.node_lock = node_lock
        self.timeout_ms = int(timeout_ms)
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()
        with self.state.lock:
            self.state.running = False

    def run(self) -> None:
        try:
            while not self._stop_event.is_set():
                with self.node_lock:
                    packets = self.node.getDataPackets(self.timeout_ms)
                for packet in packets:
                    process_packet(packet, self.state)
        except Exception as exc:
            with self.state.lock:
                self.state.error = str(exc)
                self.state.running = False
            self.fatal_error.emit(str(exc))


class DeviceActionThread(QtCore.QThread):
    result_ready = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)

    def __init__(
        self,
        node_lock: threading.Lock,
        action: Callable[[], Optional[DeviceSettings]],
    ) -> None:
        super().__init__()
        self.node_lock = node_lock
        self.action = action

    def run(self) -> None:
        try:
            with self.node_lock:
                result = self.action()
            self.result_ready.emit(result)
        except Exception as exc:
            self.failed.emit(str(exc))


class MetricCard(QtWidgets.QFrame):
    def __init__(self, title: str, value: str = "--") -> None:
        super().__init__()
        self.setObjectName("metricCard")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(2)
        title_label = QtWidgets.QLabel(title)
        title_label.setObjectName("metricTitle")
        self.value_label = QtWidgets.QLabel(value)
        self.value_label.setObjectName("metricValue")
        layout.addWidget(title_label)
        layout.addWidget(self.value_label)

    def set_value(self, text: str) -> None:
        self.value_label.setText(text)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(
        self,
        node: Any,
        node_lock: threading.Lock,
        state: MonitorState,
        device_info: dict[str, Any],
        launch_settings: DeviceSettings,
        original_channels: Any,
        original_enabled: bool,
        base_rate: int,
        args: argparse.Namespace,
    ) -> None:
        super().__init__()
        self.node = node
        self.node_lock = node_lock
        self.state = state
        self.device_info = device_info
        self.launch_settings = launch_settings
        self.original_channels = original_channels
        self.original_enabled = original_enabled
        self.base_rate = int(base_rate)
        self.args = args
        self.action_thread: Optional[DeviceActionThread] = None
        self._closing = False

        self.setWindowTitle("3DM-GX5-25 Standalone Monitor")
        self.resize(1720, 1000)
        self.setMinimumSize(1250, 760)

        pg.setConfigOptions(antialias=False)
        self._build_ui()
        self._apply_settings_to_controls(launch_settings)

        self.reader = ReaderThread(node, state, node_lock, timeout_ms=25)
        self.reader.fatal_error.connect(self._on_reader_error)
        self.reader.start()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._refresh_ui)
        self.timer.start(max(10, int(round(1_000.0 / args.ui_hz))))

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        header = QtWidgets.QHBoxLayout()
        title_box = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel("LORD 3DM-GX5-25 Debug Console")
        title.setObjectName("appTitle")
        subtitle = QtWidgets.QLabel(
            f"{self.device_info['model']}  ·  S/N {self.device_info['serial']}  ·  "
            f"FW {self.device_info['firmware']}  ·  {self.device_info['port']} @ "
            f"{self.device_info['baud']} bps"
        )
        subtitle.setObjectName("subtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header.addLayout(title_box, 1)
        self.connection_badge = QtWidgets.QLabel("● CONNECTED")
        self.connection_badge.setObjectName("connectedBadge")
        header.addWidget(self.connection_badge)
        root.addLayout(header)

        cards = QtWidgets.QHBoxLayout()
        self.rate_card = MetricCard("Measured rate", "0.0 Hz")
        self.age_card = MetricCard("Sample age", "No data")
        self.filter_card = MetricCard("Filter state", "UNKNOWN")
        self.mount_card = MetricCard("Sensor → Vehicle", "--")
        self.quality_card = MetricCard("Stream quality", "--")
        for card in (
            self.rate_card,
            self.age_card,
            self.filter_card,
            self.mount_card,
            self.quality_card,
        ):
            cards.addWidget(card)
        root.addLayout(cards)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        splitter.setChildrenCollapsible(False)
        root.addWidget(splitter, 1)

        plot_container = QtWidgets.QWidget()
        plot_grid = QtWidgets.QGridLayout(plot_container)
        plot_grid.setContentsMargins(0, 0, 0, 0)
        plot_grid.setSpacing(8)

        self.gyro_plot, self.gyro_curves = self._make_plot(
            "Compensated angular rate", "rad/s", ("x", "y", "z")
        )
        self.gravity_plot, self.gravity_curves = self._make_plot(
            "Projected gravity", "normalized", ("x", "y", "z"), fixed_y=(-1.2, 1.2)
        )
        self.quat_plot, self.quat_curves = self._make_plot(
            "Attitude quaternion", "component", ("w", "x", "y", "z"), fixed_y=(-1.2, 1.2)
        )
        self.timing_plot, self.timing_curves = self._make_plot(
            "Host receive interval", "ms", ("dt",)
        )
        plot_grid.addWidget(self.gyro_plot, 0, 0)
        plot_grid.addWidget(self.gravity_plot, 0, 1)
        plot_grid.addWidget(self.quat_plot, 1, 0)
        plot_grid.addWidget(self.timing_plot, 1, 1)
        splitter.addWidget(plot_container)

        settings_scroll = QtWidgets.QScrollArea()
        settings_scroll.setWidgetResizable(True)
        settings_scroll.setMinimumWidth(370)
        settings_panel = QtWidgets.QWidget()
        settings_scroll.setWidget(settings_panel)
        settings_layout = QtWidgets.QVBoxLayout(settings_panel)
        settings_layout.setContentsMargins(12, 8, 12, 12)
        settings_layout.setSpacing(10)

        mount_group = QtWidgets.QGroupBox("Sensor → Vehicle mounting")
        mount_form = QtWidgets.QFormLayout(mount_group)
        self.roll_spin = self._angle_spinbox()
        self.pitch_spin = self._angle_spinbox()
        self.yaw_spin = self._angle_spinbox()
        mount_form.addRow("Roll [deg]", self.roll_spin)
        mount_form.addRow("Pitch [deg]", self.pitch_spin)
        mount_form.addRow("Yaw [deg]", self.yaw_spin)
        settings_layout.addWidget(mount_group)

        stream_group = QtWidgets.QGroupBox("Estimation Filter stream")
        stream_form = QtWidgets.QFormLayout(stream_group)
        self.sample_rate_combo = QtWidgets.QComboBox()
        self.status_rate_combo = QtWidgets.QComboBox()
        rates = valid_rates(self.base_rate)
        for rate in reversed(rates):
            self.sample_rate_combo.addItem(f"{rate} Hz", rate)
            self.status_rate_combo.addItem(f"{rate} Hz", rate)
        stream_form.addRow("Quaternion / gyro", self.sample_rate_combo)
        stream_form.addRow("Filter status", self.status_rate_combo)
        stream_form.addRow("Base rate", QtWidgets.QLabel(f"{self.base_rate} Hz"))
        settings_layout.addWidget(stream_group)

        filter_group = QtWidgets.QGroupBox("Filter behavior")
        filter_layout = QtWidgets.QVBoxLayout(filter_group)
        self.auto_init_check = QtWidgets.QCheckBox("Auto initialization")
        self.pitch_roll_check = QtWidgets.QCheckBox("Pitch / roll aiding")
        self.reset_after_check = QtWidgets.QCheckBox("Reset filter after Apply")
        self.reset_after_check.setChecked(True)
        self.include_stream_check = QtWidgets.QCheckBox(
            "Include stream format when saving startup"
        )
        for widget in (
            self.auto_init_check,
            self.pitch_roll_check,
            self.reset_after_check,
            self.include_stream_check,
        ):
            filter_layout.addWidget(widget)
        settings_layout.addWidget(filter_group)

        button_grid = QtWidgets.QGridLayout()
        self.apply_button = QtWidgets.QPushButton("APPLY CURRENT")
        self.read_button = QtWidgets.QPushButton("READ DEVICE")
        self.reset_button = QtWidgets.QPushButton("RESET FILTER")
        self.restore_button = QtWidgets.QPushButton("RESTORE LAUNCH")
        self.load_button = QtWidgets.QPushButton("LOAD STARTUP")
        self.save_button = QtWidgets.QPushButton("SAVE STARTUP")
        self.apply_button.setObjectName("primaryButton")
        self.save_button.setObjectName("dangerButton")
        buttons = (
            self.apply_button,
            self.read_button,
            self.reset_button,
            self.restore_button,
            self.load_button,
            self.save_button,
        )
        for index, button in enumerate(buttons):
            button_grid.addWidget(button, index // 2, index % 2)
        settings_layout.addLayout(button_grid)

        live_group = QtWidgets.QGroupBox("Live values")
        live_form = QtWidgets.QFormLayout(live_group)
        self.quat_value = QtWidgets.QLabel("[+1.00000, +0.00000, +0.00000, +0.00000]")
        self.gyro_value = QtWidgets.QLabel("[+0.00000, +0.00000, +0.00000] rad/s")
        self.gravity_value = QtWidgets.QLabel("[+0.00000, +0.00000, -1.00000]")
        self.flags_value = QtWidgets.QLabel("----")
        for label in (
            self.quat_value,
            self.gyro_value,
            self.gravity_value,
            self.flags_value,
        ):
            label.setObjectName("monoValue")
            label.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        live_form.addRow("Quaternion wxyz", self.quat_value)
        live_form.addRow("Angular rate", self.gyro_value)
        live_form.addRow("Projected gravity", self.gravity_value)
        live_form.addRow("Status flags", self.flags_value)
        settings_layout.addWidget(live_group)

        self.channel_text = QtWidgets.QPlainTextEdit()
        self.channel_text.setReadOnly(True)
        self.channel_text.setMaximumHeight(120)
        self.channel_text.setPlaceholderText("Received channel names will appear here.")
        channels_group = QtWidgets.QGroupBox("Detected MSCL channels")
        channels_layout = QtWidgets.QVBoxLayout(channels_group)
        channels_layout.addWidget(self.channel_text)
        settings_layout.addWidget(channels_group)

        settings_layout.addStretch(1)
        splitter.addWidget(settings_scroll)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)
        splitter.setSizes([1300, 390])

        self.status_label = QtWidgets.QLabel("Ready")
        self.status_label.setObjectName("statusLabel")
        root.addWidget(self.status_label)

        self.apply_button.clicked.connect(self._apply_current)
        self.read_button.clicked.connect(self._read_device)
        self.reset_button.clicked.connect(self._reset_filter)
        self.restore_button.clicked.connect(self._restore_launch)
        self.load_button.clicked.connect(self._load_startup)
        self.save_button.clicked.connect(self._save_startup)

    def _make_plot(
        self,
        title: str,
        y_label: str,
        names: Sequence[str],
        fixed_y: Optional[tuple[float, float]] = None,
    ) -> tuple[pg.PlotWidget, list[Any]]:
        widget = pg.PlotWidget(title=title)
        widget.setLabel("bottom", "Time", units="s")
        widget.setLabel("left", y_label)
        widget.showGrid(x=True, y=True, alpha=0.25)
        widget.addLegend(offset=(8, 8))
        if fixed_y is not None:
            widget.setYRange(fixed_y[0], fixed_y[1], padding=0.0)
        colors = ("#52B6FF", "#FFB454", "#6DDE8A", "#C58CFF")
        curves = [
            widget.plot([], [], name=name, pen=pg.mkPen(colors[i], width=1.5))
            for i, name in enumerate(names)
        ]
        return widget, curves

    @staticmethod
    def _angle_spinbox() -> QtWidgets.QDoubleSpinBox:
        box = QtWidgets.QDoubleSpinBox()
        box.setRange(-360.0, 360.0)
        box.setDecimals(6)
        box.setSingleStep(0.1)
        box.setSuffix("°")
        box.setKeyboardTracking(False)
        return box

    def _set_combo_value(self, combo: QtWidgets.QComboBox, value: Optional[int]) -> None:
        if value is None:
            return
        index = combo.findData(int(value))
        if index >= 0:
            combo.setCurrentIndex(index)

    def _apply_settings_to_controls(self, settings: DeviceSettings) -> None:
        if settings.mount_deg is not None:
            self.roll_spin.setValue(settings.mount_deg[0])
            self.pitch_spin.setValue(settings.mount_deg[1])
            self.yaw_spin.setValue(settings.mount_deg[2])
        self._set_combo_value(self.sample_rate_combo, settings.sample_rate_hz)
        self._set_combo_value(self.status_rate_combo, settings.status_rate_hz)
        if settings.auto_init is not None:
            self.auto_init_check.setChecked(settings.auto_init)
        if settings.pitch_roll_aid is not None:
            self.pitch_roll_check.setChecked(settings.pitch_roll_aid)
        self._update_mount_card(settings)

    def _update_mount_card(self, settings: DeviceSettings) -> None:
        if settings.mount_deg is None:
            self.mount_card.set_value("Unsupported")
        else:
            r, p, y = settings.mount_deg
            self.mount_card.set_value(f"R {r:+.2f}°  P {p:+.2f}°  Y {y:+.2f}°")

    def _set_action_busy(self, busy: bool, message: str) -> None:
        for button in (
            self.apply_button,
            self.read_button,
            self.reset_button,
            self.restore_button,
            self.load_button,
            self.save_button,
        ):
            button.setEnabled(not busy)
        self.status_label.setText(message)

    def _start_action(
        self,
        label: str,
        action: Callable[[], Optional[DeviceSettings]],
    ) -> None:
        if self.action_thread is not None and self.action_thread.isRunning():
            self.status_label.setText("Another device command is still running.")
            return
        self._set_action_busy(True, f"{label} …")
        thread = DeviceActionThread(self.node_lock, action)
        self.action_thread = thread

        def on_result(result: object) -> None:
            if isinstance(result, DeviceSettings):
                self._apply_settings_to_controls(result)
            self._set_action_busy(False, f"{label}: OK")

        def on_error(message: str) -> None:
            self._set_action_busy(False, f"{label}: ERROR — {message}")
            QtWidgets.QMessageBox.critical(self, label, message)

        def cleanup() -> None:
            self.action_thread = None

        thread.result_ready.connect(on_result)
        thread.failed.connect(on_error)
        thread.finished.connect(cleanup)
        thread.start()

    def _apply_current(self) -> None:
        mount = (
            self.roll_spin.value(),
            self.pitch_spin.value(),
            self.yaw_spin.value(),
        )
        sample_rate = int(self.sample_rate_combo.currentData())
        status_rate = int(self.status_rate_combo.currentData())
        auto_init = self.auto_init_check.isChecked()
        pitch_roll = self.pitch_roll_check.isChecked()
        reset_after = self.reset_after_check.isChecked()

        def action() -> DeviceSettings:
            self.node.setToIdle()
            configure_stream(self.node, sample_rate, status_rate, self.base_rate)
            set_mount_euler(self.node, mount)
            self.node.setAutoInitialization(bool(auto_init))
            self.node.setPitchRollAid(bool(pitch_roll))
            if reset_after:
                self.node.resetFilter()
            self.node.enableDataStream(mscl.MipTypes.CLASS_ESTFILTER, True)
            return read_device_settings(self.node)

        self._start_action("Apply current settings", action)

    def _read_device(self) -> None:
        self._start_action("Read device settings", lambda: read_device_settings(self.node))

    def _reset_filter(self) -> None:
        def action() -> DeviceSettings:
            self.node.resetFilter()
            return read_device_settings(self.node)

        self._start_action("Reset filter", action)

    def _restore_launch(self) -> None:
        def action() -> DeviceSettings:
            self.node.setToIdle()
            if self.launch_settings.mount_deg is not None:
                set_mount_euler(self.node, self.launch_settings.mount_deg)
            if self.launch_settings.auto_init is not None:
                self.node.setAutoInitialization(self.launch_settings.auto_init)
            if self.launch_settings.pitch_roll_aid is not None:
                self.node.setPitchRollAid(self.launch_settings.pitch_roll_aid)
            self.node.setActiveChannelFields(
                mscl.MipTypes.CLASS_ESTFILTER, self.original_channels
            )
            self.node.enableDataStream(
                mscl.MipTypes.CLASS_ESTFILTER, self.original_enabled
            )
            self.node.resetFilter()
            return read_device_settings(self.node)

        self._start_action("Restore launch snapshot", action)

    def _load_startup(self) -> None:
        include_stream = self.include_stream_check.isChecked()

        def action() -> DeviceSettings:
            self.node.setToIdle()
            self.node.loadStartupSettings(make_setting_commands(include_stream))
            self.node.resetFilter()
            if not include_stream:
                self.node.enableDataStream(mscl.MipTypes.CLASS_ESTFILTER, True)
            return read_device_settings(self.node)

        self._start_action("Load startup settings", action)

    def _save_startup(self) -> None:
        answer = QtWidgets.QMessageBox.warning(
            self,
            "Persist device settings",
            "This writes the selected settings into the IMU startup configuration.\n\n"
            "The values will remain after power cycling. Continue?",
            QtWidgets.QMessageBox.StandardButton.Save
            | QtWidgets.QMessageBox.StandardButton.Cancel,
            QtWidgets.QMessageBox.StandardButton.Cancel,
        )
        if answer != QtWidgets.QMessageBox.StandardButton.Save:
            return
        include_stream = self.include_stream_check.isChecked()

        def action() -> DeviceSettings:
            self.node.saveSettingsAsStartup(make_setting_commands(include_stream))
            return read_device_settings(self.node)

        self._start_action("Save startup settings", action)

    def _on_reader_error(self, message: str) -> None:
        self.connection_badge.setText("● STREAM ERROR")
        self.connection_badge.setObjectName("errorBadge")
        self.connection_badge.style().unpolish(self.connection_badge)
        self.connection_badge.style().polish(self.connection_badge)
        self.status_label.setText(f"Reader error: {message}")

    def _refresh_ui(self) -> None:
        snap = self.state.snapshot()
        times = snap["t"]
        for curve, values in zip(self.gyro_curves, snap["gyro"]):
            curve.setData(times, values)
        for curve, values in zip(self.gravity_curves, snap["gravity"]):
            curve.setData(times, values)
        for curve, values in zip(self.quat_curves, snap["quat"]):
            curve.setData(times, values)
        self.timing_curves[0].setData(snap["dt_t"], snap["dt_ms"])

        if times:
            right = times[-1]
            left = max(0.0, right - self.state.window_s)
            for plot in (
                self.gyro_plot,
                self.gravity_plot,
                self.quat_plot,
                self.timing_plot,
            ):
                plot.setXRange(left, max(left + self.state.window_s, right), padding=0.0)

        age_ms = snap["age_ms"]
        self.rate_card.set_value(
            f"{snap['measured_hz']:.1f} Hz  ·  σ {snap['jitter_ms']:.3f} ms"
        )
        self.age_card.set_value("No data" if math.isinf(age_ms) else f"{age_ms:.1f} ms")

        filter_state = snap["filter_state"]
        filter_name = FILTER_STATE_NAMES.get(filter_state, "UNKNOWN")
        self.filter_card.set_value(
            filter_name if filter_state is None else f"{filter_name} ({filter_state})"
        )
        self.quality_card.set_value(
            f"{snap['packet_count']} packets  ·  {snap['sample_count']} samples  ·  "
            f"{snap['incomplete_packets']} incomplete"
        )

        q = snap["latest_q"]
        gyro = snap["latest_gyro"]
        gravity = snap["latest_gravity"]
        self.quat_value.setText(
            "[" + ", ".join(f"{v:+.5f}" for v in q) + "]"
        )
        self.gyro_value.setText(
            "[" + ", ".join(f"{v:+.5f}" for v in gyro) + "] rad/s"
        )
        self.gravity_value.setText(
            "[" + ", ".join(f"{v:+.5f}" for v in gravity) + "]"
        )
        flags = snap["status_flags"]
        self.flags_value.setText("----" if flags is None else f"0x{flags:04X}")

        channel_text = "\n".join(snap["channels"])
        if self.channel_text.toPlainText() != channel_text:
            self.channel_text.setPlainText(channel_text)

        if snap["error"]:
            self.status_label.setText(f"Stream error: {snap['error']}")

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        if self._closing:
            event.accept()
            return
        self._closing = True
        self.timer.stop()
        self.reader.stop()
        self.reader.wait(1_500)

        if self.action_thread is not None and self.action_thread.isRunning():
            self.action_thread.wait(2_000)

        if self.args.restore_on_exit:
            self.status_label.setText("Restoring launch snapshot …")
            QtWidgets.QApplication.processEvents()
            try:
                with self.node_lock:
                    self.node.setToIdle()
                    if self.launch_settings.mount_deg is not None:
                        set_mount_euler(self.node, self.launch_settings.mount_deg)
                    if self.launch_settings.auto_init is not None:
                        self.node.setAutoInitialization(self.launch_settings.auto_init)
                    if self.launch_settings.pitch_roll_aid is not None:
                        self.node.setPitchRollAid(self.launch_settings.pitch_roll_aid)
                    self.node.setActiveChannelFields(
                        mscl.MipTypes.CLASS_ESTFILTER, self.original_channels
                    )
                    self.node.enableDataStream(
                        mscl.MipTypes.CLASS_ESTFILTER, self.original_enabled
                    )
                    self.node.resetFilter()
            except Exception as exc:
                QtWidgets.QMessageBox.warning(
                    self,
                    "Restore failed",
                    f"Failed to restore launch settings:\n{exc}",
                )

        if self.args.show_channels:
            snap = self.state.snapshot()
            print("\nDiscovered data channels:")
            for name in snap["channels"]:
                print(f"  {name}")
        event.accept()


def apply_dark_style(app: QtWidgets.QApplication) -> None:
    app.setStyle("Fusion")
    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.ColorRole.Window, QtGui.QColor("#11151B"))
    palette.setColor(QtGui.QPalette.ColorRole.WindowText, QtGui.QColor("#E7EDF5"))
    palette.setColor(QtGui.QPalette.ColorRole.Base, QtGui.QColor("#171C24"))
    palette.setColor(QtGui.QPalette.ColorRole.AlternateBase, QtGui.QColor("#202631"))
    palette.setColor(QtGui.QPalette.ColorRole.Text, QtGui.QColor("#E7EDF5"))
    palette.setColor(QtGui.QPalette.ColorRole.Button, QtGui.QColor("#252D39"))
    palette.setColor(QtGui.QPalette.ColorRole.ButtonText, QtGui.QColor("#E7EDF5"))
    palette.setColor(QtGui.QPalette.ColorRole.Highlight, QtGui.QColor("#3388CC"))
    palette.setColor(QtGui.QPalette.ColorRole.HighlightedText, QtGui.QColor("#FFFFFF"))
    app.setPalette(palette)
    app.setStyleSheet(
        """
        QWidget { font-size: 13px; }
        QMainWindow { background: #11151B; }
        QLabel#appTitle { font-size: 24px; font-weight: 700; color: #F4F7FB; }
        QLabel#subtitle { color: #9EACBC; }
        QLabel#connectedBadge {
            background: #153827; color: #75E5A2; border: 1px solid #2B6C48;
            border-radius: 10px; padding: 6px 12px; font-weight: 700;
        }
        QLabel#errorBadge {
            background: #482020; color: #FF9B9B; border: 1px solid #8B3D3D;
            border-radius: 10px; padding: 6px 12px; font-weight: 700;
        }
        QFrame#metricCard {
            background: #181E27; border: 1px solid #2B3442; border-radius: 8px;
        }
        QLabel#metricTitle { color: #8291A3; font-size: 11px; }
        QLabel#metricValue { color: #F1F5F9; font-size: 15px; font-weight: 650; }
        QGroupBox {
            border: 1px solid #313B49; border-radius: 7px; margin-top: 11px;
            padding-top: 9px; font-weight: 650; color: #DCE4ED;
        }
        QGroupBox::title { subcontrol-origin: margin; left: 9px; padding: 0 5px; }
        QLineEdit, QDoubleSpinBox, QComboBox, QPlainTextEdit {
            background: #151A22; border: 1px solid #354152; border-radius: 5px;
            padding: 5px; selection-background-color: #3388CC;
        }
        QPushButton {
            background: #252D39; border: 1px solid #3A4759; border-radius: 5px;
            padding: 8px 10px; font-weight: 600;
        }
        QPushButton:hover { background: #303A49; }
        QPushButton:disabled { color: #667281; background: #1A2029; }
        QPushButton#primaryButton { background: #196AA5; border-color: #348CC8; }
        QPushButton#primaryButton:hover { background: #217DBF; }
        QPushButton#dangerButton { background: #733437; border-color: #9E4B4F; }
        QPushButton#dangerButton:hover { background: #8A3E42; }
        QLabel#monoValue { font-family: monospace; color: #DDE7F0; }
        QLabel#statusLabel {
            background: #181E27; border: 1px solid #2B3442; border-radius: 5px;
            padding: 7px 10px; color: #AEBAC8;
        }
        QScrollArea { border: none; }
        """
    )
    pg.setConfigOption("background", "#151A22")
    pg.setConfigOption("foreground", "#C9D3DE")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="PyQt6 standalone monitor/configurator for 3DM-GX5-25"
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="serial port")
    parser.add_argument(
        "--baud", type=int, default=115200, help="current device baud rate"
    )
    parser.add_argument(
        "--rate",
        type=int,
        default=500,
        help="temporary quaternion/gyro stream rate [integer Hz]",
    )
    parser.add_argument(
        "--status-rate",
        type=int,
        default=10,
        help="temporary filter-status stream rate [integer Hz]",
    )
    parser.add_argument("--window", type=float, default=10.0, help="plot history [s]")
    parser.add_argument("--ui-hz", type=float, default=24.0, help="GUI refresh rate [Hz]")
    parser.add_argument(
        "--no-config",
        action="store_true",
        help="do not configure a temporary monitor stream on startup",
    )
    parser.add_argument(
        "--restore-on-exit",
        action="store_true",
        help="restore the launch snapshot when the window closes",
    )
    parser.add_argument(
        "--show-channels",
        action="store_true",
        help="print received MSCL channel names on exit",
    )
    args = parser.parse_args()
    if args.rate <= 0 or args.status_rate <= 0 or args.window <= 0.0 or args.ui_hz <= 0.0:
        parser.error("rate, status-rate, window, and ui-hz must be positive")
    return args


def main() -> int:
    args = parse_args()
    app = QtWidgets.QApplication(sys.argv)
    apply_dark_style(app)

    print(f"GX5 Tool version: {TOOL_VERSION}")
    print(f"Connecting to {args.port} @ {args.baud} bps ...")
    try:
        connection = mscl.Connection.Serial(args.port, int(args.baud))
        node = mscl.InertialNode(connection)
        node_lock = threading.Lock()

        model = node.modelName()
        serial = node.serialNumber()
        firmware = node.firmwareVersion()
        print(f"Connected: {model}, S/N {serial}, firmware {firmware}")

        est_class = mscl.MipTypes.CLASS_ESTFILTER
        original_channels = node.getActiveChannelFields(est_class)
        original_enabled = bool(node.isDataStreamEnabled(est_class))
        base_rate = int(node.getDataRateBase(est_class))
        launch_settings = read_device_settings(node)

        print(f"Estimation Filter base rate: {base_rate} Hz")
        print(
            "Original filter channels:",
            ", ".join(format_channels(original_channels)) or "none",
        )
        print(f"Original filter stream enabled: {original_enabled}")
        if launch_settings.mount_deg is None:
            print("Sensor-to-vehicle RPY [deg]: unavailable")
        else:
            print(
                "Sensor-to-vehicle RPY [deg]: "
                f"({launch_settings.mount_deg[0]:+.3f}, "
                f"{launch_settings.mount_deg[1]:+.3f}, "
                f"{launch_settings.mount_deg[2]:+.3f})"
            )
        print(f"Auto initialization: {launch_settings.auto_init}")
        print(f"Pitch/roll aiding: {launch_settings.pitch_roll_aid}")
        print(
            f"Resolved fields: quat=0x{QUAT_FIELD:04X}, "
            f"gyro=0x{ANG_RATE_FIELD:04X}, status=0x{STATUS_FIELD:04X}"
        )

        validate_rate(args.rate, base_rate, "rate")
        validate_rate(args.status_rate, base_rate, "status-rate")

        if not args.no_config:
            with node_lock:
                node.setToIdle()
                configure_stream(node, args.rate, args.status_rate, base_rate)
                node.enableDataStream(est_class, True)
            print("Temporary monitor stream configured. Startup settings were not saved.")
        elif not original_enabled:
            with node_lock:
                node.enableDataStream(est_class, True)

        state = MonitorState(window_s=args.window, max_rate_hz=base_rate)
        device_info = {
            "model": model,
            "serial": serial,
            "firmware": firmware,
            "port": args.port,
            "baud": args.baud,
        }

        window = MainWindow(
            node=node,
            node_lock=node_lock,
            state=state,
            device_info=device_info,
            launch_settings=launch_settings,
            original_channels=original_channels,
            original_enabled=original_enabled,
            base_rate=base_rate,
            args=args,
        )
        window.show()
        return int(app.exec())

    except mscl.Error as exc:
        message = exc.what() if hasattr(exc, "what") else str(exc)
        QtWidgets.QMessageBox.critical(None, "MSCL error", message)
        print(f"MSCL error: {message}", file=sys.stderr)
        return 1
    except Exception as exc:
        QtWidgets.QMessageBox.critical(None, "GX5 monitor error", str(exc))
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())