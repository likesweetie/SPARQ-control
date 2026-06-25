"""
IMU state definitions.

This module defines product-independent IMU state containers used by
drivers and higher-level robot control code.

Responsibilities:
- Store decoded IMU state in robot-internal units.
- Allow partial updates from separate quaternion and gyro frames.
- Avoid vendor-specific CAN IDs, payload layouts, and scaling rules.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class RobotPoseState:
    """
    Robot body-frame pose-related IMU state.

    Unit convention:
    - quat_xyzw: (qx, qy, qz, qw)
    - angular_velocity_rad_s: rad/s
    - projected_gravity_b: body-frame unit gravity vector
    - timestamps: time.monotonic() seconds
    """

    quat_xyzw: tuple[float, float, float, float] | None = None
    angular_velocity_rad_s: tuple[float, float, float] | None = None
    projected_gravity_b: tuple[float, float, float] | None = None

    last_quat_t: float = 0.0
    last_gyro_t: float = 0.0


# Backward-compatible alias for code that still imports IMUState.
IMUState = RobotPoseState
