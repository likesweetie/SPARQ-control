from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class QHRR0ImuSpec:
    name: str
    request_id: int
    quat_id: int
    gyro_id: int
    cmd_get_quat: int
    cmd_get_gyro: int
    cmd_get_all: int
    quat_scale: float
    gyro_scale: float
    normalize_quat: bool


def imu_spec_from_platform(platform) -> QHRR0ImuSpec:
    imu = platform.imu
    return QHRR0ImuSpec(
        name=str(imu.type),
        request_id=int(imu.request_id),
        quat_id=int(imu.quat_id),
        gyro_id=int(imu.gyro_id),
        cmd_get_quat=int(imu.cmd_get_quat),
        cmd_get_gyro=int(imu.cmd_get_gyro),
        cmd_get_all=int(imu.cmd_get_all),
        quat_scale=float(imu.quat_scale),
        gyro_scale=float(imu.gyro_scale),
        normalize_quat=bool(imu.normalize_quat),
    )

