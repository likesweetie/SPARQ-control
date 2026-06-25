from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .platform_config import (
    PlatformConfig,
    PlatformConfigError,
    load_platform_config,
    load_yaml_mapping,
    resolve_config_path,
)


@dataclass
class ProcessConfig:
    name: str
    command: list[str]
    start_order: int
    stop_order: int
    new_terminal: bool
    terminal_command: list[str]
    working_dir: str
    env: dict[str, str]


@dataclass
class MitCommandShmConfig:
    name: str
    target_count: int


@dataclass
class RobotStateShmConfig:
    name: str
    size_bytes: int
    publish_hz: float


@dataclass
class OperatorCommandShmConfig:
    name: str
    size_bytes: int


@dataclass
class ShmConfig:
    cleanup_stale_on_start: bool
    unlink_on_shutdown: bool
    mit_command: MitCommandShmConfig
    aux_command: RobotStateShmConfig
    operator_command: OperatorCommandShmConfig
    control_state: RobotStateShmConfig
    dashboard_state: RobotStateShmConfig


@dataclass
class MotorConfig:
    can_ids: list[int]
    enter_on_start: bool
    exit_on_shutdown: bool
    set_zero_on_start: bool


@dataclass
class ImuConfig:
    enabled: bool
    request_all_on_start: bool
    request_all_each_tick: bool
    startup_request_count: int
    startup_request_delay_s: float


@dataclass
class CANDaemonConfig:
    rx_timeout_s: float
    tx_timeout_s: float
    join_timeout_s: float
    max_tx_queue_size: int
    send_block: bool
    send_timeout_s: float | None
    ipc_socket_path: str
    connect_timeout_s: float


@dataclass
class MitProtocolRangeConfig:
    position_rad: float
    velocity_rad_s: float
    kp: float
    kd: float
    torque_ff_nm: float
    feedback_position_rad: float


@dataclass
class CanConfig:
    interface: str
    command_timeout_s: float
    bringup_delay_s: float
    daemon: CANDaemonConfig
    motors: MotorConfig
    imu: ImuConfig
    mit_protocol_range: MitProtocolRangeConfig


@dataclass
class RobotControllerCoreConfig:
    name: str
    control_hz: float
    shutdown_timeout_s: float


@dataclass
class StateMachineConfig:
    enable_duration_s: float


@dataclass
class RuntimeModeConfig:
    mode: str


@dataclass
class HardwareSafetyConfig:
    allow_real_can: bool
    require_manual_arm: bool
    require_estop: bool
    allow_enable_on_start: bool
    allowed_can_interfaces: list[str]


@dataclass
class SafetyPolicyConfig:
    velocity_damping_kd: float
    damping_timeout_s: float
    command_loss_action: str
    feedback_stale_action: str


@dataclass
class RobotControllerConfig:
    platform: PlatformConfig
    runtime: RuntimeModeConfig
    hardware: HardwareSafetyConfig
    safety: SafetyPolicyConfig
    state_machine: StateMachineConfig
    robot_controller: RobotControllerCoreConfig
    shm: ShmConfig
    can: CanConfig
    processes: list[ProcessConfig]


class ConfigError(PlatformConfigError):
    pass


def _require_key(mapping: dict[str, Any], key: str, path: str) -> Any:
    if key not in mapping:
        raise ConfigError(f"Missing required config key: {path}.{key}")
    return mapping[key]


def _require_mapping(mapping: dict[str, Any], key: str, path: str) -> dict[str, Any]:
    value = _require_key(mapping, key, path)
    if not isinstance(value, dict):
        raise ConfigError(f"Config key must be a mapping: {path}.{key}")
    return value


def _require_list(mapping: dict[str, Any], key: str, path: str) -> list[Any]:
    value = _require_key(mapping, key, path)
    if not isinstance(value, list):
        raise ConfigError(f"Config key must be a list: {path}.{key}")
    return value


def _require_int_list(mapping: dict[str, Any], key: str, path: str) -> list[int]:
    value = _require_list(mapping, key, path)
    return [int(item, 0) if isinstance(item, str) else int(item) for item in value]


def _require_bool(mapping: dict[str, Any], key: str, path: str) -> bool:
    value = _require_key(mapping, key, path)
    if not isinstance(value, bool):
        raise ConfigError(f"Config key must be a boolean: {path}.{key}")
    return value


def _require_float(mapping: dict[str, Any], key: str, path: str) -> float:
    return float(_require_key(mapping, key, path))


def _require_int(mapping: dict[str, Any], key: str, path: str) -> int:
    return int(_require_key(mapping, key, path))


def _optional_float_or_none(mapping: dict[str, Any], key: str, path: str) -> float | None:
    value = _require_key(mapping, key, path)
    return None if value is None else float(value)


def _process_config(item: Any, index: int) -> ProcessConfig:
    if not isinstance(item, dict):
        raise ConfigError(f"Config key must be a mapping: processes[{index}]")
    command = _require_list(item, "command", f"processes[{index}]")
    if not command:
        raise ConfigError(f"Config key must not be empty: processes[{index}].command")
    env_raw = _require_mapping(item, "env", f"processes[{index}]")
    if not isinstance(env_raw, dict):
        raise ConfigError(f"Config key must be a mapping: processes[{index}].env")
    return ProcessConfig(
        name=str(_require_key(item, "name", f"processes[{index}]")),
        command=[str(part) for part in command],
        start_order=_require_int(item, "start_order", f"processes[{index}]"),
        stop_order=_require_int(item, "stop_order", f"processes[{index}]"),
        new_terminal=_require_bool(item, "new_terminal", f"processes[{index}]"),
        terminal_command=[str(part) for part in _require_list(item, "terminal_command", f"processes[{index}]")],
        working_dir=str(_require_key(item, "working_dir", f"processes[{index}]")),
        env={str(key): str(value) for key, value in env_raw.items()},
    )


def _load_processes(path: Path) -> list[ProcessConfig]:
    raw = load_yaml_mapping(path)
    return [
        _process_config(item, index)
        for index, item in enumerate(_require_list(raw, "processes", str(path)))
    ]


def _validate_config(config: RobotControllerConfig) -> None:
    if config.runtime.mode not in ("simulation", "hardware"):
        raise ConfigError("runtime.mode must be 'simulation' or 'hardware'")
    if not config.hardware.allowed_can_interfaces:
        raise ConfigError("hardware.allowed_can_interfaces must not be empty")
    if config.safety.velocity_damping_kd < 0.0:
        raise ConfigError("safety.velocity_damping_kd must be >= 0")
    if config.safety.damping_timeout_s <= 0.0:
        raise ConfigError("safety.damping_timeout_s must be > 0")
    if config.safety.command_loss_action not in ("damping", "disable", "fault"):
        raise ConfigError("safety.command_loss_action must be damping, disable, or fault")
    if config.safety.feedback_stale_action not in ("damping", "disable", "fault"):
        raise ConfigError("safety.feedback_stale_action must be damping, disable, or fault")
    if config.robot_controller.control_hz <= 0.0:
        raise ConfigError("robot_controller.control_hz must be > 0")
    if config.robot_controller.shutdown_timeout_s < 0.0:
        raise ConfigError("robot_controller.shutdown_timeout_s must be >= 0")
    if config.state_machine.enable_duration_s < 0.0:
        raise ConfigError("state_machine.enable_duration_s must be >= 0")
    if config.shm.mit_command.target_count <= 0:
        raise ConfigError("shm.mit_command.target_count must be > 0")
    if config.shm.control_state.size_bytes < 4096:
        raise ConfigError("shm.control_state.size_bytes must be >= 4096")
    if config.shm.control_state.publish_hz <= 0.0:
        raise ConfigError("shm.control_state.publish_hz must be > 0")
    if config.shm.aux_command.size_bytes < 4096:
        raise ConfigError("shm.aux_command.size_bytes must be >= 4096")
    if config.shm.aux_command.publish_hz <= 0.0:
        raise ConfigError("shm.aux_command.publish_hz must be > 0")
    if config.shm.operator_command.size_bytes < 4096:
        raise ConfigError("shm.operator_command.size_bytes must be >= 4096")
    if config.shm.dashboard_state.size_bytes < 4096:
        raise ConfigError("shm.dashboard_state.size_bytes must be >= 4096")
    if config.shm.dashboard_state.publish_hz <= 0.0:
        raise ConfigError("shm.dashboard_state.publish_hz must be > 0")
    state_names = {
        config.shm.mit_command.name,
        config.shm.aux_command.name,
        config.shm.operator_command.name,
        config.shm.control_state.name,
        config.shm.dashboard_state.name,
    }
    if len(state_names) != 5:
        raise ConfigError("shm segment names must be unique")
    if config.can.command_timeout_s <= 0.0:
        raise ConfigError("can.command_timeout_s must be > 0")
    if config.can.bringup_delay_s < 0.0:
        raise ConfigError("can.bringup_delay_s must be >= 0")
    if config.can.daemon.rx_timeout_s < 0.0:
        raise ConfigError("can.daemon.rx_timeout_s must be >= 0")
    if config.can.daemon.tx_timeout_s < 0.0:
        raise ConfigError("can.daemon.tx_timeout_s must be >= 0")
    if config.can.daemon.join_timeout_s < 0.0:
        raise ConfigError("can.daemon.join_timeout_s must be >= 0")
    if config.can.daemon.max_tx_queue_size <= 0:
        raise ConfigError("can.daemon.max_tx_queue_size must be > 0")
    if config.can.daemon.send_timeout_s is not None and config.can.daemon.send_timeout_s < 0.0:
        raise ConfigError("can.daemon.send_timeout_s must be null or >= 0")
    if not config.can.daemon.ipc_socket_path:
        raise ConfigError("can.daemon.ipc_socket_path must not be empty")
    if config.can.daemon.connect_timeout_s <= 0.0:
        raise ConfigError("can.daemon.connect_timeout_s must be > 0")
    if not config.can.motors.can_ids:
        raise ConfigError("can.motors.can_ids must not be empty")
    if len(set(config.can.motors.can_ids)) != len(config.can.motors.can_ids):
        raise ConfigError("can.motors.can_ids must not contain duplicates")
    if config.shm.mit_command.target_count != len(config.can.motors.can_ids):
        raise ConfigError("shm.mit_command.target_count must match len(can.motors.can_ids)")
    if config.can.imu.startup_request_count < 0:
        raise ConfigError("can.imu.startup_request_count must be >= 0")
    if config.can.imu.startup_request_delay_s < 0.0:
        raise ConfigError("can.imu.startup_request_delay_s must be >= 0")
    if config.can.mit_protocol_range.position_rad <= 0.0:
        raise ConfigError("can.mit_protocol_range.position_rad must be > 0")
    if config.can.mit_protocol_range.velocity_rad_s <= 0.0:
        raise ConfigError("can.mit_protocol_range.velocity_rad_s must be > 0")
    if config.can.mit_protocol_range.kp < 0.0:
        raise ConfigError("can.mit_protocol_range.kp must be >= 0")
    if config.can.mit_protocol_range.kd < 0.5:
        raise ConfigError("can.mit_protocol_range.kd must be >= 0.5 for shutdown damping")
    if config.can.mit_protocol_range.torque_ff_nm <= 0.0:
        raise ConfigError("can.mit_protocol_range.torque_ff_nm must be > 0")
    if config.can.mit_protocol_range.feedback_position_rad <= 0.0:
        raise ConfigError("can.mit_protocol_range.feedback_position_rad must be > 0")
    if config.safety.velocity_damping_kd > config.can.mit_protocol_range.kd:
        raise ConfigError("safety.velocity_damping_kd must be <= can.mit_protocol_range.kd")
    process_names = [process.name for process in config.processes]
    if len(set(process_names)) != len(process_names):
        raise ConfigError("processes must not contain duplicate names")
    if "can_daemon" not in set(process_names):
        raise ConfigError("processes must include a 'can_daemon' subprocess")
    for process in config.processes:
        if not process.working_dir:
            raise ConfigError(f"process {process.name} working_dir must not be empty")
        if process.new_terminal and not process.terminal_command:
            raise ConfigError(f"process {process.name} terminal_command must not be empty when new_terminal is true")


def load_robot_controller_config(path: str | Path) -> RobotControllerConfig:
    config_path = Path(path)
    raw = load_yaml_mapping(config_path)
    platform_config_path = resolve_config_path(
        config_path,
        str(_require_key(raw, "platform_config", "<root>")),
        "platform_config",
    )
    platform = load_platform_config(platform_config_path)

    runtime_raw = _require_mapping(raw, "runtime", "<root>")
    hardware_raw = _require_mapping(raw, "hardware", "<root>")
    safety_raw = _require_mapping(raw, "safety", "<root>")
    state_machine_raw = _require_mapping(raw, "state_machine", "<root>")
    core_raw = _require_mapping(raw, "robot_controller", "<root>")
    shm_raw = _require_mapping(raw, "shm", "<root>")
    aux_command_raw = _require_mapping(shm_raw, "aux_command", "shm")
    operator_command_raw = _require_mapping(shm_raw, "operator_command", "shm")
    control_state_raw = _require_mapping(shm_raw, "control_state", "shm")
    dashboard_state_raw = _require_mapping(shm_raw, "dashboard_state", "shm")
    can_raw = _require_mapping(raw, "can", "<root>")
    daemon_raw = _require_mapping(can_raw, "daemon", "can")
    motors_raw = _require_mapping(can_raw, "motors", "can")
    imu_raw = _require_mapping(can_raw, "imu", "can")
    processes_config_path = resolve_config_path(
        config_path,
        str(_require_key(raw, "processes_config", "<root>")),
        "processes_config",
    )
    processes = _load_processes(processes_config_path)

    config = RobotControllerConfig(
        platform=platform,
        runtime=RuntimeModeConfig(
            mode=str(_require_key(runtime_raw, "mode", "runtime")),
        ),
        hardware=HardwareSafetyConfig(
            allow_real_can=_require_bool(hardware_raw, "allow_real_can", "hardware"),
            require_manual_arm=_require_bool(hardware_raw, "require_manual_arm", "hardware"),
            require_estop=_require_bool(hardware_raw, "require_estop", "hardware"),
            allow_enable_on_start=_require_bool(hardware_raw, "allow_enable_on_start", "hardware"),
            allowed_can_interfaces=[
                str(item)
                for item in _require_list(hardware_raw, "allowed_can_interfaces", "hardware")
            ],
        ),
        safety=SafetyPolicyConfig(
            velocity_damping_kd=_require_float(safety_raw, "velocity_damping_kd", "safety"),
            damping_timeout_s=_require_float(safety_raw, "damping_timeout_s", "safety"),
            command_loss_action=str(_require_key(safety_raw, "command_loss_action", "safety")),
            feedback_stale_action=str(_require_key(safety_raw, "feedback_stale_action", "safety")),
        ),
        state_machine=StateMachineConfig(
            enable_duration_s=_require_float(
                state_machine_raw,
                "enable_duration_s",
                "state_machine",
            ),
        ),
        robot_controller=RobotControllerCoreConfig(
            name=str(_require_key(core_raw, "name", "robot_controller")),
            control_hz=_require_float(core_raw, "control_hz", "robot_controller"),
            shutdown_timeout_s=_require_float(core_raw, "shutdown_timeout_s", "robot_controller"),
        ),
        shm=ShmConfig(
            cleanup_stale_on_start=_require_bool(shm_raw, "cleanup_stale_on_start", "shm"),
            unlink_on_shutdown=_require_bool(shm_raw, "unlink_on_shutdown", "shm"),
            mit_command=MitCommandShmConfig(
                name=platform.shm.mit_command,
                target_count=len(platform.enabled_actuators),
            ),
            aux_command=RobotStateShmConfig(
                name=platform.shm.aux_command,
                size_bytes=_require_int(aux_command_raw, "size_bytes", "shm.aux_command"),
                publish_hz=_require_float(aux_command_raw, "publish_hz", "shm.aux_command"),
            ),
            operator_command=OperatorCommandShmConfig(
                name=platform.shm.operator_command,
                size_bytes=_require_int(operator_command_raw, "size_bytes", "shm.operator_command"),
            ),
            control_state=RobotStateShmConfig(
                name=platform.shm.control_state,
                size_bytes=_require_int(control_state_raw, "size_bytes", "shm.control_state"),
                publish_hz=_require_float(control_state_raw, "publish_hz", "shm.control_state"),
            ),
            dashboard_state=RobotStateShmConfig(
                name=platform.shm.dashboard_state,
                size_bytes=_require_int(dashboard_state_raw, "size_bytes", "shm.dashboard_state"),
                publish_hz=_require_float(dashboard_state_raw, "publish_hz", "shm.dashboard_state"),
            ),
        ),
        can=CanConfig(
            interface=platform.can.interface,
            command_timeout_s=_require_float(can_raw, "command_timeout_s", "can"),
            bringup_delay_s=_require_float(can_raw, "bringup_delay_s", "can"),
            daemon=CANDaemonConfig(
                rx_timeout_s=_require_float(daemon_raw, "rx_timeout_s", "can.daemon"),
                tx_timeout_s=_require_float(daemon_raw, "tx_timeout_s", "can.daemon"),
                join_timeout_s=_require_float(daemon_raw, "join_timeout_s", "can.daemon"),
                max_tx_queue_size=_require_int(daemon_raw, "max_tx_queue_size", "can.daemon"),
                send_block=_require_bool(daemon_raw, "send_block", "can.daemon"),
                send_timeout_s=_optional_float_or_none(daemon_raw, "send_timeout_s", "can.daemon"),
                ipc_socket_path=platform.can.daemon_socket,
                connect_timeout_s=_require_float(daemon_raw, "connect_timeout_s", "can.daemon"),
            ),
            motors=MotorConfig(
                can_ids=[actuator.can_id for actuator in platform.enabled_actuators],
                enter_on_start=_require_bool(motors_raw, "enter_on_start", "can.motors"),
                exit_on_shutdown=_require_bool(motors_raw, "exit_on_shutdown", "can.motors"),
                set_zero_on_start=_require_bool(motors_raw, "set_zero_on_start", "can.motors"),
            ),
            imu=ImuConfig(
                enabled=_require_bool(imu_raw, "enabled", "can.imu"),
                request_all_on_start=_require_bool(imu_raw, "request_all_on_start", "can.imu"),
                request_all_each_tick=_require_bool(imu_raw, "request_all_each_tick", "can.imu"),
                startup_request_count=_require_int(imu_raw, "startup_request_count", "can.imu"),
                startup_request_delay_s=_require_float(imu_raw, "startup_request_delay_s", "can.imu"),
            ),
            mit_protocol_range=MitProtocolRangeConfig(
                position_rad=float(platform.spg_mit.p_max_rad),
                velocity_rad_s=float(platform.spg_mit.v_max_rad_s),
                kp=float(platform.spg_mit.kp_max),
                kd=float(platform.spg_mit.kd_max),
                torque_ff_nm=float(platform.spg_mit.tau_max_nm),
                feedback_position_rad=float(platform.spg_mit.feedback_position_max_rad),
            ),
        ),
        processes=processes,
    )
    _validate_config(config)
    return config
