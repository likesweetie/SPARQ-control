from __future__ import annotations

import math
import time

from robot_controller.shm.robot_state import (
    COMMAND_OUTPUT_SOURCE_VALUES,
    MAX_ROBOT_STATE_ACTUATORS,
    RobotStateC,
    RobotStateShm,
)
from robot_controller.telemetry.robot_snapshot import RobotSnapshot


class ShmStatePublisher:
    def __init__(self, shm_name: str, publish_hz: float) -> None:
        self.writer = RobotStateShm.open_writer(shm_name)
        self.publish_period_s = 1.0 / float(publish_hz)
        self.last_publish_t = 0.0

    def close(self) -> None:
        self.writer.close()

    def publish(self, snapshot: RobotSnapshot, *, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self.last_publish_t < self.publish_period_s:
            return
        state = snapshot_to_cstruct(snapshot)
        self.writer.write(state)
        self.last_publish_t = now


def snapshot_to_cstruct(snapshot: RobotSnapshot) -> RobotStateC:
    state = RobotStateC()
    state.timestamp_ns = time.time_ns()
    state.timestamp_monotonic = float(snapshot.timestamp_monotonic)
    state.timestamp_unix = float(snapshot.timestamp_unix)
    state.controller_mode = int(snapshot.mode)
    state.actuator_count = min(len(snapshot.actuators), MAX_ROBOT_STATE_ACTUATORS)

    imu = snapshot.imu
    _fill_array(state.imu.quat_xyzw, imu.quat_xyzw, (0.0, 0.0, 0.0, 1.0))
    _fill_array(state.imu.projected_gravity_b, imu.projected_gravity_b, (0.0, 0.0, -1.0))
    _fill_array(state.imu.angular_velocity_rad_s, imu.angular_velocity_rad_s, (0.0, 0.0, 0.0))
    state.imu.last_quat_t = float(imu.last_quat_t)
    state.imu.last_gyro_t = float(imu.last_gyro_t)
    state.imu.quat_online = int(bool(imu.quat_online))
    state.imu.gyro_online = int(bool(imu.gyro_online))
    state.imu.quat_stale = int(bool(imu.quat_stale))
    state.imu.gyro_stale = int(bool(imu.gyro_stale))

    for index, item in enumerate(snapshot.actuators[:MAX_ROBOT_STATE_ACTUATORS]):
        out = state.actuators[index]
        out.can_id = int(item.can_id)
        out.position_rad = _float_or_zero(item.position_rad)
        out.velocity_rad_s = _float_or_zero(item.velocity_rad_s)
        out.torque_nm = _float_or_zero(item.torque_nm)
        out.current_a = _float_or_zero(item.current_a)
        out.temperature_c = _float_or_zero(item.temperature_c)
        out.fault_code = -1 if item.fault_code is None else int(item.fault_code)
        out.is_enabled = -1 if item.is_enabled is None else int(bool(item.is_enabled))
        out.last_feedback_t = float(item.last_feedback_t)
        out.age_s = -1.0 if item.age_s is None else float(item.age_s)
        out.online = int(bool(item.online))
        out.stale = int(bool(item.stale))

    command_output = snapshot.command_output
    state.command_output.timestamp_monotonic = float(command_output.timestamp_monotonic)
    state.command_output.source = int(COMMAND_OUTPUT_SOURCE_VALUES.get(command_output.source, 0))
    state.command_output.target_count = min(len(command_output.targets), MAX_ROBOT_STATE_ACTUATORS)
    for index, item in enumerate(command_output.targets[:MAX_ROBOT_STATE_ACTUATORS]):
        out = state.command_output.targets[index]
        out.can_id = int(item.can_id)
        out.p_target_rad = _float_or_zero(item.p_target_rad)
        out.v_target_rad_s = _float_or_zero(item.v_target_rad_s)
        out.kp = _float_or_zero(item.kp)
        out.kd = _float_or_zero(item.kd)
        out.tau_target_nm = _float_or_zero(item.tau_target_nm)

    return state


def _fill_array(target, values, fallback) -> None:
    source = fallback if values is None else values
    for index, value in enumerate(source):
        target[index] = float(value)


def _float_or_zero(value: float | None) -> float:
    if value is None or not math.isfinite(float(value)):
        return 0.0
    return float(value)
