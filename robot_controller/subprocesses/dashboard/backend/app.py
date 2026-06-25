from __future__ import annotations

import asyncio
import json
import logging
import os
from pathlib import Path
from typing import Any

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from .command_api import CommandError, CommandService
from .robot_state_shm import DashboardRobotStateReader
from .socketcan_io import CAN_FRAME_SIZE, open_can_socket, parse_can_frame
from .state import MonitorState
from robot_controller.core.config import load_robot_controller_config
from robot_controller.core.platform_config import (
    load_platform_config,
    load_yaml_mapping,
    resolve_config_path,
)
from robot_controller.shm.operator_command import OperatorCommandShmWriter
from robot_controller.supervisor import ProcessSupervisor
from hal.can_bus.process_client import CANProcessClient


ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = Path(__file__).resolve().parents[4]
CONFIG_PATH = Path(os.environ.get("DASHBOARD_CONFIG", PROJECT_ROOT / "config" / "app_config" / "dashboard.yaml"))
if not CONFIG_PATH.is_absolute():
    CONFIG_PATH = PROJECT_ROOT / CONFIG_PATH
FRONTEND_DIR = ROOT / "frontend"
logger = logging.getLogger(__name__)
PROTECTED_PROCESS_NAMES = frozenset({"can_daemon", "dashboard"})


class RawSendRequest(BaseModel):
    can_id: int | str
    data: str


class MotorZeroRequest(BaseModel):
    offset_count: int | None = None
    offset_deg: float | None = None

    def resolved_offset_count(self) -> int:
        if self.offset_count is not None and self.offset_deg is not None:
            raise ValueError("Use either offset_count or offset_deg, not both")
        if self.offset_deg is not None:
            return round_half_away_from_zero(float(self.offset_deg) * 100.0)
        if self.offset_count is not None:
            return int(self.offset_count)
        return 0


class OperatorZeroTargetRequest(MotorZeroRequest):
    can_id: int | str


class OperatorZeroSetRequest(BaseModel):
    targets: list[OperatorZeroTargetRequest] = Field(default_factory=list)


class ConfirmRequest(BaseModel):
    confirmed: bool = False


def require_no_platform_owned_keys(config: dict[str, Any]) -> None:
    checks = (
        ("can", ("iface", "bitrate")),
        ("can_daemon", ("ipc_socket_path",)),
        ("robot_controller_state", ("control_shm_name", "dashboard_shm_name", "operator_shm_name", "operator_shm_size_bytes")),
        (
            "imu",
            (
                "request_id",
                "quat_id",
                "gyro_id",
                "cmd_get_all",
                "cmd_get_quat",
                "cmd_get_gyro",
                "quat_scale",
                "gyro_scale",
                "normalize_quat",
            ),
        ),
        (
            "spg",
            (
                "feedback_position_max_rad",
                "iq_full_scale_count",
                "iq_full_scale_current_a",
                "p_max_rad",
                "v_max_rad_s",
                "kp_max",
                "kd_max",
                "tau_max_nm",
            ),
        ),
    )
    for section, keys in checks:
        value = config.get(section)
        if not isinstance(value, dict):
            continue
        for key in keys:
            if key in value:
                raise ValueError(f"Dashboard config must not override platform-owned key: {section}.{key}")
    if "actuators" in config:
        raise ValueError("Dashboard config must not define platform-owned key: actuators")


def resolve_transmit_ids(config: dict[str, Any], platform) -> None:
    dashboard = require_section(config, "dashboard")
    raw = dashboard.get("transmit_ids")
    if not isinstance(raw, list):
        raise ValueError("Dashboard config key 'dashboard.transmit_ids' must be a list")
    actuators = {actuator.name: actuator.can_id for actuator in platform.actuators}
    resolved = []
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise ValueError(f"dashboard.transmit_ids[{index}] must be a mapping")
        if "can_id" in item:
            raise ValueError(f"dashboard.transmit_ids[{index}].can_id must come from actuator")
        output = dict(item)
        if "platform_ref" in item:
            raise ValueError(
                f"dashboard.transmit_ids[{index}].platform_ref is not supported; "
                "dashboard transmit presets must reference an actuator"
            )
        if "actuator" not in item:
            raise ValueError(f"dashboard.transmit_ids[{index}] requires actuator")
        name = str(item["actuator"])
        if name not in actuators:
            raise ValueError(f"dashboard.transmit_ids[{index}] references unknown actuator: {name}")
        output["can_id"] = actuators[name]
        resolved.append(output)
    dashboard["transmit_ids"] = resolved


def resolve_zero_set_presets(config: dict[str, Any], platform) -> None:
    dashboard = require_section(config, "dashboard")
    raw = dashboard.get("zero_set_presets", [])
    if raw is None:
        raw = []
    if not isinstance(raw, list):
        raise ValueError("Dashboard config key 'dashboard.zero_set_presets' must be a list")

    actuators = {actuator.name: actuator.can_id for actuator in platform.enabled_actuators}
    resolved = []
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise ValueError(f"dashboard.zero_set_presets[{index}] must be a mapping")
        targets_raw = item.get("targets")
        if not isinstance(targets_raw, list) or not targets_raw:
            raise ValueError(f"dashboard.zero_set_presets[{index}].targets must be a non-empty list")

        targets = []
        for target_index, target in enumerate(targets_raw):
            if not isinstance(target, dict):
                raise ValueError(
                    f"dashboard.zero_set_presets[{index}].targets[{target_index}] must be a mapping"
                )
            if "can_id" in target:
                raise ValueError(
                    f"dashboard.zero_set_presets[{index}].targets[{target_index}].can_id "
                    "must come from actuator"
                )
            if "actuator" not in target:
                raise ValueError(
                    f"dashboard.zero_set_presets[{index}].targets[{target_index}] requires actuator"
                )
            name = str(target["actuator"])
            if name not in actuators:
                raise ValueError(
                    f"dashboard.zero_set_presets[{index}].targets[{target_index}] "
                    f"references unknown actuator: {name}"
                )
            if "offset_count" in target and "offset_deg" in target:
                raise ValueError(
                    f"dashboard.zero_set_presets[{index}].targets[{target_index}] "
                    "must use either offset_count or offset_deg, not both"
                )
            if "offset_deg" in target:
                offset_deg = float(target["offset_deg"])
                offset_count = round_half_away_from_zero(offset_deg * 100.0)
            elif "offset_count" in target:
                offset_count = int(target["offset_count"])
                offset_deg = offset_count * 0.01
            else:
                raise ValueError(
                    f"dashboard.zero_set_presets[{index}].targets[{target_index}] "
                    "requires offset_deg or offset_count"
                )
            if not (-32768 <= offset_count <= 32767):
                raise ValueError(
                    f"dashboard.zero_set_presets[{index}].targets[{target_index}] "
                    f"offset_count out of int16 range: {offset_count}"
                )
            targets.append(
                {
                    "actuator": name,
                    "can_id": actuators[name],
                    "offset_deg": offset_deg,
                    "offset_count": offset_count,
                }
            )

        resolved.append(
            {
                "id": str(item.get("id") or f"preset_{index + 1}"),
                "label": str(item.get("label") or f"Preset {index + 1}"),
                "targets": targets,
            }
        )
    dashboard["zero_set_presets"] = resolved


def load_config() -> tuple[dict[str, Any], Any]:
    raw = load_yaml_mapping(CONFIG_PATH)
    require_no_platform_owned_keys(raw)
    if "platform_config" not in raw:
        raise ValueError("Dashboard config key 'platform_config' is required")
    if "robot_controller_config" not in raw:
        raise ValueError("Dashboard config key 'robot_controller_config' is required")
    platform_path = resolve_config_path(
        CONFIG_PATH,
        str(raw["platform_config"]),
        "platform_config",
    )
    platform = load_platform_config(platform_path)
    controller_config_path = resolve_config_path(
        CONFIG_PATH,
        str(raw["robot_controller_config"]),
        "robot_controller_config",
    )
    controller_config = load_robot_controller_config(controller_config_path)
    if controller_config.platform.path != platform.path:
        raise ValueError(
            "Dashboard platform_config must match robot_controller_config platform_config"
        )

    config = dict(raw)
    resolve_transmit_ids(config, platform)
    resolve_zero_set_presets(config, platform)
    config.pop("platform_config", None)
    config.pop("robot_controller_config", None)
    config.setdefault("can", {})
    config["can"]["iface"] = platform.can.interface
    config["can"]["bitrate"] = platform.can.bitrate
    config.setdefault("can_daemon", {})
    config["can_daemon"]["ipc_socket_path"] = platform.can.daemon_socket
    config.setdefault("robot_controller_state", {})
    config["robot_controller_state"]["control_shm_name"] = platform.shm.control_state
    config["robot_controller_state"]["dashboard_shm_name"] = platform.shm.dashboard_state
    config["robot_controller_state"]["operator_shm_name"] = controller_config.shm.operator_command.name
    config["robot_controller_state"]["operator_shm_size_bytes"] = controller_config.shm.operator_command.size_bytes
    config.setdefault("imu", {})
    config["imu"]["request_id"] = platform.imu.request_id
    config["imu"]["quat_id"] = platform.imu.quat_id
    config["imu"]["gyro_id"] = platform.imu.gyro_id
    config["imu"]["quat_scale"] = platform.imu.quat_scale
    config["imu"]["gyro_scale"] = platform.imu.gyro_scale
    config["imu"]["normalize_quat"] = platform.imu.normalize_quat
    config.setdefault("spg", {})
    config["spg"]["feedback_position_max_rad"] = platform.spg_mit.feedback_position_max_rad
    config["spg"]["iq_full_scale_count"] = platform.spg_mit.iq_full_scale_count
    config["spg"]["iq_full_scale_current_a"] = platform.spg_mit.iq_full_scale_current_a
    config["spg"]["p_max_rad"] = platform.spg_mit.p_max_rad
    config["spg"]["v_max_rad_s"] = platform.spg_mit.v_max_rad_s
    config["spg"]["kp_max"] = platform.spg_mit.kp_max
    config["spg"]["kd_max"] = platform.spg_mit.kd_max
    config["spg"]["tau_max_nm"] = platform.spg_mit.tau_max_nm
    config["actuators"] = [
        {
            "name": actuator.name,
            "can_id": actuator.can_id,
        }
        for actuator in platform.enabled_actuators
    ]
    return config, controller_config


def require_section(config: dict[str, Any], section: str) -> dict[str, Any]:
    value = config.get(section)
    if not isinstance(value, dict):
        raise ValueError(f"Dashboard config section '{section}' is required")
    return value


def nested(config: dict[str, Any], section: str, key: str) -> Any:
    section_value = require_section(config, section)
    if key not in section_value:
        raise ValueError(f"Dashboard config key '{section}.{key}' is required")
    return section_value[key]


def parse_int_maybe_hex(value: Any) -> int:
    if isinstance(value, str):
        return int(value, 0)
    return int(value)


def require_hz(value: float, field: str, *, lo: float = 0.1, hi: float = 1000.0) -> float:
    value = float(value)
    if value < lo or value > hi:
        raise ValueError(f"{field} must be in [{lo}, {hi}] Hz")
    return value


def load_actuator_configs(config: dict[str, Any]) -> tuple[dict, ...]:
    raw = config.get("actuators")
    if not isinstance(raw, list):
        raise ValueError("Dashboard config key 'actuators' must be a list")
    if not raw:
        raise ValueError("Dashboard config key 'actuators' must not be empty")

    actuators = []
    seen_can_ids: set[int] = set()
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise ValueError(f"Dashboard actuator #{index} must be a mapping")
        if "can_id" not in item:
            raise ValueError(f"Dashboard actuator #{index} is missing 'can_id'")
        if "name" not in item:
            raise ValueError(f"Dashboard actuator #{index} is missing 'name'")
        can_id = parse_int_maybe_hex(item["can_id"])
        if can_id <= 0:
            raise ValueError(f"Dashboard actuator #{index} has invalid CAN ID 0x{can_id:X}")
        if can_id in seen_can_ids:
            raise ValueError(f"Duplicate Dashboard actuator CAN ID 0x{can_id:X}")
        seen_can_ids.add(can_id)
        actuators.append(
            {
                "name": str(item["name"]),
                "can_id": can_id,
            }
        )
    return tuple(actuators)


def make_state(config: dict[str, Any]) -> MonitorState:
    return MonitorState(
        iface=str(nested(config, "can", "iface")),
        bitrate=int(nested(config, "can", "bitrate")),
        bus_window_s=float(nested(config, "can", "bus_window_s")),
        heartbeat_window_s=float(nested(config, "can", "heartbeat_window_s")),
        node_timeout_s=float(nested(config, "can", "node_timeout_s")),
        stuff_factor=float(nested(config, "can", "stuff_factor")),
        feedback_position_max_rad=float(nested(config, "spg", "feedback_position_max_rad")),
        iq_full_scale_count=float(nested(config, "spg", "iq_full_scale_count")),
        iq_full_scale_current_a=float(nested(config, "spg", "iq_full_scale_current_a")),
        mit_p_max_rad=float(nested(config, "spg", "p_max_rad")),
        mit_v_max_rad_s=float(nested(config, "spg", "v_max_rad_s")),
        mit_kp_max=float(nested(config, "spg", "kp_max")),
        mit_kd_max=float(nested(config, "spg", "kd_max")),
        mit_tau_max_nm=float(nested(config, "spg", "tau_max_nm")),
        imu_request_id=parse_int_maybe_hex(nested(config, "imu", "request_id")),
        imu_quat_id=parse_int_maybe_hex(nested(config, "imu", "quat_id")),
        imu_gyro_id=parse_int_maybe_hex(nested(config, "imu", "gyro_id")),
        imu_quat_scale=float(nested(config, "imu", "quat_scale")),
        imu_gyro_scale=float(nested(config, "imu", "gyro_scale")),
        imu_normalize_quat=bool(nested(config, "imu", "normalize_quat")),
        actuator_configs=load_actuator_configs(config),
        tx_enabled=bool(nested(config, "safety", "tx_enabled_by_default")),
        allow_actuator_commands=bool(nested(config, "safety", "allow_actuator_commands")),
        mit_poll_hz=require_hz(nested(config, "spg", "default_mit_poll_hz"), "spg.default_mit_poll_hz"),
    )


def parse_can_id(value: int | str) -> int:
    if isinstance(value, int):
        return value
    return int(value.strip(), 0)


def parse_hex_payload(value: str) -> bytes:
    compact = value.replace(" ", "").replace("_", "").replace("-", "")
    if len(compact) % 2 != 0:
        raise ValueError("Payload hex string must contain whole bytes")
    data = bytes.fromhex(compact)
    if len(data) > 8:
        raise ValueError("Classical CAN payload must be <= 8 bytes")
    return data


def round_half_away_from_zero(value: float) -> int:
    if value >= 0.0:
        return int(value + 0.5)
    return int(value - 0.5)


def publish_operator_zero_set(targets: list[tuple[int, int]]) -> int:
    commands.require_controller_state(
        action_name="Operator zero set",
        allowed_states={"NORMAL"},
    )
    for can_id, offset_count in targets:
        if state.actuator_config_for_can_id(can_id) is None:
            raise ValueError(f"Unknown actuator CAN ID 0x{can_id:03X}")
        if not (-32768 <= int(offset_count) <= 32767):
            raise ValueError(f"MIT zero offset_count out of int16 range: {offset_count}")
    return operator_commands.publish_zero_set(targets)


config, controller_config = load_config()
state = make_state(config)
process_supervisor = ProcessSupervisor(controller_config.processes)
robot_state_reader = (
    DashboardRobotStateReader(
        control_shm_name=str(nested(config, "robot_controller_state", "control_shm_name")),
        dashboard_shm_name=str(nested(config, "robot_controller_state", "dashboard_shm_name")),
        stale_timeout_s=float(nested(config, "robot_controller_state", "stale_timeout_s")),
        state=state,
    )
    if bool(nested(config, "robot_controller_state", "enabled"))
    else None
)
operator_commands = OperatorCommandShmWriter(
    name=str(nested(config, "robot_controller_state", "operator_shm_name")),
    size_bytes=int(nested(config, "robot_controller_state", "operator_shm_size_bytes")),
    source="dashboard",
)


def current_controller_snapshot() -> dict | None:
    snapshot = dashboard_snapshot()
    controller = snapshot.get("robot_controller")
    if not isinstance(controller, dict):
        return None
    if controller.get("status") != "online":
        return None
    return controller


def current_controller_safety_state() -> str | None:
    controller = current_controller_snapshot()
    if controller is None:
        return None
    value = controller.get("safety_state")
    return None if value is None else str(value)


def current_controller_safety_reason() -> str | None:
    controller = current_controller_snapshot()
    if controller is None:
        return None
    value = controller.get("safety_reason")
    return None if value is None else str(value)


commands = CommandService(
    state,
    CANProcessClient(
        socket_path=str(nested(config, "can_daemon", "ipc_socket_path")),
        connect_timeout_s=float(nested(config, "can_daemon", "connect_timeout_s")),
        rx_enabled=False,
    ),
    controller_safety_state_provider=current_controller_safety_state if robot_state_reader is not None else None,
    controller_safety_reason_provider=current_controller_safety_reason if robot_state_reader is not None else None,
)
app = FastAPI(title="QHRR Robot State")
app.mount("/static", StaticFiles(directory=str(FRONTEND_DIR)), name="static")


def dashboard_snapshot() -> dict:
    if robot_state_reader is None:
        snapshot = state.snapshot()
    else:
        snapshot = robot_state_reader.dashboard_snapshot()
    snapshot["processes"] = process_status_rows()
    return snapshot


def process_status_rows() -> list[dict[str, Any]]:
    return [
        {
            "name": name,
            **dict(info),
            "manageable": name not in PROTECTED_PROCESS_NAMES,
            "management_reason": (
                "managed by robot controller supervisor"
                if name in PROTECTED_PROCESS_NAMES
                else ""
            ),
        }
        for name, info in sorted(process_supervisor.status().items())
    ]


def ensure_process_name(name: str) -> str:
    if name not in process_supervisor.process_configs:
        raise HTTPException(status_code=404, detail=f"Unknown process: {name}")
    return name


def ensure_process_manageable(name: str) -> None:
    if name in PROTECTED_PROCESS_NAMES:
        raise HTTPException(
            status_code=403,
            detail=f"Process '{name}' is managed by the robot controller supervisor",
        )


async def socketcan_loop() -> None:
    reconnect_delay_s = 1.0
    max_frames_per_tick = 4096
    sock = None
    while True:
        if sock is None:
            try:
                sock = open_can_socket(state.iface)
                state.socket_status = "connected"
                state.socket_error = None
            except OSError as exc:
                state.socket_status = "disconnected"
                state.socket_error = str(exc)
                await asyncio.sleep(reconnect_delay_s)
                continue

        try:
            frames_read = 0
            while frames_read < max_frames_per_tick:
                try:
                    frame_bytes = sock.recv(CAN_FRAME_SIZE)
                except BlockingIOError:
                    break
                state.mark_rx(parse_can_frame(frame_bytes))
                frames_read += 1
        except OSError as exc:
            state.socket_status = "disconnected"
            state.socket_error = str(exc)
            try:
                sock.close()
            except OSError as close_exc:
                logger.warning("CAN socket close failed after RX error: %s", close_exc)
            sock = None

        await asyncio.sleep(0 if frames_read else 0.001)


async def mit_poll_loop() -> None:
    next_send_t = asyncio.get_running_loop().time()
    while True:
        if state.mit_poll_can_ids:
            hz = float(state.mit_poll_hz)
            period_s = 1.0 / hz
            now = asyncio.get_running_loop().time()
            if now < next_send_t:
                await asyncio.sleep(next_send_t - now)

            try:
                for can_id in sorted(state.mit_poll_can_ids):
                    commands.motor_mit_hold(can_id)
            except CommandError as exc:
                state.socket_error = str(exc)

            now = asyncio.get_running_loop().time()
            next_send_t += period_s
            if next_send_t < now:
                next_send_t = now + period_s
        else:
            next_send_t = asyncio.get_running_loop().time()
            await asyncio.sleep(0.05)


@app.on_event("startup")
async def startup() -> None:
    app.state.tasks = [
        asyncio.create_task(socketcan_loop()),
        asyncio.create_task(mit_poll_loop()),
    ]


@app.on_event("shutdown")
async def shutdown() -> None:
    for task in getattr(app.state, "tasks", []):
        task.cancel()
    await asyncio.gather(*getattr(app.state, "tasks", []), return_exceptions=True)
    if robot_state_reader is not None:
        robot_state_reader.close()
    operator_commands.close()


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(FRONTEND_DIR / "index.html")


@app.get("/processes")
async def processes_page() -> FileResponse:
    return FileResponse(FRONTEND_DIR / "processes.html")


@app.get("/shm")
async def shm_page() -> FileResponse:
    return FileResponse(FRONTEND_DIR / "shm.html")


@app.get("/api/config")
async def api_config() -> dict:
    return config


@app.get("/api/state")
async def api_state() -> dict:
    return dashboard_snapshot()


@app.get("/api/processes")
async def api_processes() -> dict:
    return {"ok": True, "processes": process_status_rows()}


@app.post("/api/processes/{name}/start")
async def api_process_start(name: str) -> dict:
    name = ensure_process_name(name)
    ensure_process_manageable(name)
    logger.info("Dashboard requested process start: %s", name)
    try:
        await asyncio.to_thread(process_supervisor.start_by_name, name)
    except (KeyError, RuntimeError, ValueError, OSError) as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return {"ok": True, "processes": process_status_rows()}


@app.post("/api/processes/{name}/stop")
async def api_process_stop(name: str) -> dict:
    name = ensure_process_name(name)
    ensure_process_manageable(name)
    logger.info("Dashboard requested process stop: %s", name)
    timeout_s = float(controller_config.robot_controller.shutdown_timeout_s)
    try:
        await asyncio.to_thread(process_supervisor.stop_by_name, name, timeout_s)
    except (KeyError, RuntimeError, ValueError, OSError) as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return {"ok": True, "processes": process_status_rows()}


@app.post("/api/tx/lock")
async def tx_lock() -> dict:
    commands.lock_tx()
    return {"ok": True, "tx_enabled": state.tx_enabled}


@app.post("/api/tx/unlock")
async def tx_unlock() -> dict:
    commands.unlock_tx()
    return {"ok": True, "tx_enabled": state.tx_enabled}


@app.post("/api/can/send")
async def can_send(req: RawSendRequest) -> dict:
    try:
        tx = commands.send_raw(parse_can_id(req.can_id), parse_hex_payload(req.data))
    except (ValueError, CommandError, OSError) as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return {"ok": True, "tx": tx}


@app.post("/api/operator/fault-clear")
async def operator_fault_clear() -> dict:
    try:
        command_id = operator_commands.publish(clear_fault=True)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {"ok": True, "command_id": command_id}


@app.post("/api/operator/arm")
async def operator_arm() -> dict:
    try:
        command_id = operator_commands.publish(arm=True)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {"ok": True, "command_id": command_id}


@app.post("/api/operator/run")
async def operator_run() -> dict:
    try:
        command_id = operator_commands.publish(run=True)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {"ok": True, "command_id": command_id}


@app.post("/api/operator/damping")
async def operator_damping() -> dict:
    try:
        command_id = operator_commands.publish(damping=True)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {"ok": True, "command_id": command_id}


@app.post("/api/operator/zero-set")
async def operator_zero_set(req: OperatorZeroSetRequest | None = None) -> dict:
    try:
        targets: list[tuple[int, int]] = []
        request_targets = [] if req is None else req.targets
        for target in request_targets:
            can_id = parse_can_id(target.can_id)
            targets.append((can_id, target.resolved_offset_count()))
        command_id = publish_operator_zero_set(targets)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except CommandError as exc:
        raise HTTPException(status_code=403, detail=str(exc)) from exc
    except (FileNotFoundError, RuntimeError) as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {"ok": True, "command_id": command_id}


@app.post("/api/operator/estop")
async def operator_estop() -> dict:
    try:
        command_id = operator_commands.publish(estop=True)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {"ok": True, "command_id": command_id}


@app.post("/api/actuator/{can_id}/enter")
async def motor_enter(can_id: str) -> dict:
    try:
        tx = commands.motor_enter(parse_can_id(can_id))
    except (CommandError, OSError) as exc:
        raise HTTPException(status_code=403, detail=str(exc)) from exc
    return {"ok": True, "tx": tx}


@app.post("/api/actuator/{can_id}/exit")
async def motor_exit(can_id: str) -> dict:
    try:
        tx = commands.motor_exit(parse_can_id(can_id))
    except (CommandError, OSError) as exc:
        raise HTTPException(status_code=403, detail=str(exc)) from exc
    return {"ok": True, "tx": tx}


@app.post("/api/actuator/{can_id}/zero")
async def motor_zero(can_id: str, req: MotorZeroRequest) -> dict:
    try:
        offset_count = req.resolved_offset_count()
        command_id = publish_operator_zero_set([(parse_can_id(can_id), offset_count)])
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except CommandError as exc:
        raise HTTPException(status_code=403, detail=str(exc)) from exc
    except (FileNotFoundError, RuntimeError) as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {"ok": True, "command_id": command_id}


@app.post("/api/actuator/{can_id}/mit-poll/start")
async def motor_mit_poll_start(can_id: str, req: ConfirmRequest) -> dict:
    if not req.confirmed:
        raise HTTPException(status_code=400, detail="MIT polling start was not confirmed")
    parsed_can_id = parse_can_id(can_id)
    if parsed_can_id not in state.motors:
        raise HTTPException(status_code=404, detail=f"Unknown CAN ID 0x{parsed_can_id:03X}")
    state.mit_poll_can_ids.add(parsed_can_id)
    return {"ok": True, "can_id": f"0x{parsed_can_id:03X}", "mit_polling": True}


@app.post("/api/actuator/{can_id}/mit-poll/stop")
async def motor_mit_poll_stop(can_id: str) -> dict:
    parsed_can_id = parse_can_id(can_id)
    state.mit_poll_can_ids.discard(parsed_can_id)
    return {"ok": True, "can_id": f"0x{parsed_can_id:03X}", "mit_polling": False}


@app.websocket("/ws/state")
async def ws_state(websocket: WebSocket) -> None:
    await websocket.accept()
    state_hz = require_hz(nested(config, "dashboard", "state_hz"), "dashboard.state_hz", lo=1.0, hi=60.0)
    delay = 1.0 / state_hz
    try:
        while True:
            await websocket.send_text(json.dumps(dashboard_snapshot()))
            await asyncio.sleep(delay)
    except WebSocketDisconnect:
        return


def main() -> None:
    host = str(nested(config, "dashboard", "host"))
    port = int(nested(config, "dashboard", "port"))
    uvicorn.run("robot_controller.subprocesses.dashboard.backend.app:app", host=host, port=port, reload=False)


if __name__ == "__main__":
    main()
