from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class PlatformConfigError(ValueError):
    pass


@dataclass(frozen=True)
class PlatformRobotConfig:
    name: str


@dataclass(frozen=True)
class PlatformRobotAssetConfig:
    model_path: str
    policy_config_dir: str
    pd_config_path: str


@dataclass(frozen=True)
class PlatformCanConfig:
    interface: str
    bitrate: int
    daemon_socket: str


@dataclass(frozen=True)
class PlatformShmConfig:
    mit_command: str
    aux_command: str
    operator_command: str
    control_state: str
    dashboard_state: str


@dataclass(frozen=True)
class PlatformImuConfig:
    type: str
    request_id: int
    quat_id: int
    gyro_id: int
    cmd_get_quat: int
    cmd_get_gyro: int
    cmd_get_all: int
    quat_scale: float
    gyro_scale: float
    normalize_quat: bool


@dataclass(frozen=True)
class PlatformSpgMitConfig:
    p_max_rad: float
    v_max_rad_s: float
    kp_max: float
    kd_max: float
    tau_max_nm: float
    feedback_position_max_rad: float
    iq_full_scale_count: float
    iq_full_scale_current_a: float


@dataclass(frozen=True)
class PlatformActuatorConfig:
    name: str
    enabled: bool
    can_id: int
    mujoco_joint: str
    mujoco_actuator: str
    sign: float
    offset_rad: float


@dataclass(frozen=True)
class PlatformConfig:
    path: Path
    robot: PlatformRobotConfig
    robots: dict[str, PlatformRobotAssetConfig]
    can: PlatformCanConfig
    shm: PlatformShmConfig
    imu: PlatformImuConfig
    spg_mit: PlatformSpgMitConfig
    actuators: tuple[PlatformActuatorConfig, ...]

    @property
    def enabled_actuators(self) -> tuple[PlatformActuatorConfig, ...]:
        return tuple(actuator for actuator in self.actuators if actuator.enabled)


def load_yaml_mapping(path: str | Path) -> dict[str, Any]:
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as fp:
        raw = yaml.safe_load(fp)
    if not isinstance(raw, dict):
        raise PlatformConfigError(f"{config_path} must contain a YAML mapping")
    return raw


def resolve_config_path(base_path: str | Path, value: str, path: str) -> Path:
    if not value:
        raise PlatformConfigError(f"Config key must not be empty: {path}")
    candidate = Path(value)
    if not candidate.is_absolute():
        candidate = Path(base_path).parent / candidate
    return candidate.resolve()


def require_key(mapping: dict[str, Any], key: str, path: str) -> Any:
    if key not in mapping:
        raise PlatformConfigError(f"Missing required config key: {path}.{key}")
    return mapping[key]


def require_mapping(mapping: dict[str, Any], key: str, path: str) -> dict[str, Any]:
    value = require_key(mapping, key, path)
    if not isinstance(value, dict):
        raise PlatformConfigError(f"Config key must be a mapping: {path}.{key}")
    return value


def require_list(mapping: dict[str, Any], key: str, path: str) -> list[Any]:
    value = require_key(mapping, key, path)
    if not isinstance(value, list):
        raise PlatformConfigError(f"Config key must be a list: {path}.{key}")
    return value


def parse_int(value: Any) -> int:
    return int(value, 0) if isinstance(value, str) else int(value)


def parse_robot_assets(raw: dict[str, Any]) -> dict[str, PlatformRobotAssetConfig]:
    robots: dict[str, PlatformRobotAssetConfig] = {}
    for name, item in raw.items():
        if not isinstance(item, dict):
            raise PlatformConfigError(f"Config key must be a mapping: robots.{name}")
        robots[str(name)] = PlatformRobotAssetConfig(
            model_path=str(require_key(item, "model_path", f"robots.{name}")),
            policy_config_dir=str(require_key(item, "policy_config_dir", f"robots.{name}")),
            pd_config_path=str(require_key(item, "pd_config_path", f"robots.{name}")),
        )
    if not robots:
        raise PlatformConfigError("robots must not be empty")
    return robots


def parse_actuators(raw: list[Any]) -> tuple[PlatformActuatorConfig, ...]:
    actuators: list[PlatformActuatorConfig] = []
    seen_can_ids: set[int] = set()
    seen_names: set[str] = set()
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise PlatformConfigError(f"Config key must be a mapping: actuators[{index}]")
        name = str(require_key(item, "name", f"actuators[{index}]"))
        can_id = parse_int(require_key(item, "can_id", f"actuators[{index}]"))
        if name in seen_names:
            raise PlatformConfigError(f"Duplicate actuator name: {name}")
        if can_id in seen_can_ids:
            raise PlatformConfigError(f"Duplicate actuator CAN ID: 0x{can_id:X}")
        seen_names.add(name)
        seen_can_ids.add(can_id)
        actuators.append(
            PlatformActuatorConfig(
                name=name,
                enabled=bool(require_key(item, "enabled", f"actuators[{index}]")),
                can_id=can_id,
                mujoco_joint=str(require_key(item, "mujoco_joint", f"actuators[{index}]")),
                mujoco_actuator=str(require_key(item, "mujoco_actuator", f"actuators[{index}]")),
                sign=float(require_key(item, "sign", f"actuators[{index}]")),
                offset_rad=float(require_key(item, "offset_rad", f"actuators[{index}]")),
            )
        )
    if not actuators:
        raise PlatformConfigError("actuators must not be empty")
    if not any(actuator.enabled for actuator in actuators):
        raise PlatformConfigError("at least one actuator must be enabled")
    return tuple(actuators)


def load_platform_config(path: str | Path) -> PlatformConfig:
    config_path = Path(path).resolve()
    raw = load_yaml_mapping(config_path)
    robot_raw = require_mapping(raw, "robot", "<root>")
    can_raw = require_mapping(raw, "can", "<root>")
    shm_raw = require_mapping(raw, "shm", "<root>")
    imu_raw = require_mapping(raw, "imu", "<root>")
    spg_raw = require_mapping(raw, "spg_mit", "<root>")

    config = PlatformConfig(
        path=config_path,
        robot=PlatformRobotConfig(
            name=str(require_key(robot_raw, "name", "robot")),
        ),
        robots=parse_robot_assets(require_mapping(raw, "robots", "<root>")),
        can=PlatformCanConfig(
            interface=str(require_key(can_raw, "interface", "can")),
            bitrate=int(require_key(can_raw, "bitrate", "can")),
            daemon_socket=str(require_key(can_raw, "daemon_socket", "can")),
        ),
        shm=PlatformShmConfig(
            mit_command=str(require_key(shm_raw, "mit_command", "shm")),
            aux_command=str(require_key(shm_raw, "aux_command", "shm")),
            operator_command=str(require_key(shm_raw, "operator_command", "shm")),
            control_state=str(require_key(shm_raw, "control_state", "shm")),
            dashboard_state=str(require_key(shm_raw, "dashboard_state", "shm")),
        ),
        imu=PlatformImuConfig(
            type=str(require_key(imu_raw, "type", "imu")),
            request_id=parse_int(require_key(imu_raw, "request_id", "imu")),
            quat_id=parse_int(require_key(imu_raw, "quat_id", "imu")),
            gyro_id=parse_int(require_key(imu_raw, "gyro_id", "imu")),
            cmd_get_quat=parse_int(require_key(imu_raw, "cmd_get_quat", "imu")),
            cmd_get_gyro=parse_int(require_key(imu_raw, "cmd_get_gyro", "imu")),
            cmd_get_all=parse_int(require_key(imu_raw, "cmd_get_all", "imu")),
            quat_scale=float(require_key(imu_raw, "quat_scale", "imu")),
            gyro_scale=float(require_key(imu_raw, "gyro_scale", "imu")),
            normalize_quat=bool(require_key(imu_raw, "normalize_quat", "imu")),
        ),
        spg_mit=PlatformSpgMitConfig(
            p_max_rad=float(require_key(spg_raw, "p_max_rad", "spg_mit")),
            v_max_rad_s=float(require_key(spg_raw, "v_max_rad_s", "spg_mit")),
            kp_max=float(require_key(spg_raw, "kp_max", "spg_mit")),
            kd_max=float(require_key(spg_raw, "kd_max", "spg_mit")),
            tau_max_nm=float(require_key(spg_raw, "tau_max_nm", "spg_mit")),
            feedback_position_max_rad=float(require_key(spg_raw, "feedback_position_max_rad", "spg_mit")),
            iq_full_scale_count=float(require_key(spg_raw, "iq_full_scale_count", "spg_mit")),
            iq_full_scale_current_a=float(require_key(spg_raw, "iq_full_scale_current_a", "spg_mit")),
        ),
        actuators=parse_actuators(require_list(raw, "actuators", "<root>")),
    )
    if config.robot.name not in config.robots:
        raise PlatformConfigError(f"robot.name has no robots entry: {config.robot.name}")
    return config
