from __future__ import annotations

from dataclasses import dataclass

from robot_controller.state_machine import ControllerMode


@dataclass(frozen=True)
class ActuatorSnapshot:
    can_id: int
    position_rad: float | None
    velocity_rad_s: float | None
    torque_nm: float | None
    current_a: float | None
    temperature_c: float | None
    fault_code: int | None
    is_enabled: bool | None
    last_feedback_t: float
    age_s: float | None
    online: bool
    stale: bool


@dataclass(frozen=True)
class ImuSnapshot:
    quat_xyzw: tuple[float, float, float, float] | None
    projected_gravity_b: tuple[float, float, float] | None
    angular_velocity_rad_s: tuple[float, float, float] | None
    last_quat_t: float
    last_gyro_t: float
    quat_online: bool
    gyro_online: bool
    quat_stale: bool
    gyro_stale: bool


@dataclass(frozen=True)
class CommandTargetSnapshot:
    can_id: int
    p_target_rad: float
    v_target_rad_s: float
    kp: float
    kd: float
    tau_target_nm: float


@dataclass(frozen=True)
class CommandOutputSnapshot:
    source: str
    timestamp_monotonic: float
    targets: tuple[CommandTargetSnapshot, ...]


@dataclass(frozen=True)
class RobotSnapshot:
    mode: ControllerMode
    timestamp_monotonic: float
    timestamp_unix: float
    actuators: tuple[ActuatorSnapshot, ...]
    imu: ImuSnapshot
    command_output: CommandOutputSnapshot
