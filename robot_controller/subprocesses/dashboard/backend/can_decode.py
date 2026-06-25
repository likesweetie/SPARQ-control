from __future__ import annotations

import math
import struct


E2BOX_REQ_ID = 0x221
E2BOX_QUAT_ID = 0x2A1
E2BOX_GYRO_ID = 0x321
E2BOX_CMD_GET_ALL = 0x03

SPG_CMD_MIT_CONTROL = 0xC0
SPG_CMD_MIT_ENTER = 0xC1
SPG_CMD_MIT_EXIT = 0xC2
SPG_CMD_MIT_SET_ZERO = 0xC3
SPG_CMD_CLEAR_ERROR = 0x9B


def hex_data(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def normalize_quat_xyzw(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    qx, qy, qz, qw = q
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12 or not math.isfinite(n):
        return 0.0, 0.0, 0.0, 1.0
    return qx / n, qy / n, qz / n, qw / n


def projected_gravity_from_xyzw(q: tuple[float, float, float, float]) -> tuple[float, float, float]:
    qx, qy, qz, qw = q
    vx, vy, vz = 0.0, 0.0, -1.0

    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    vpx = vx - qw * tx + (qy * tz - qz * ty)
    vpy = vy - qw * ty + (qz * tx - qx * tz)
    vpz = vz - qw * tz + (qx * ty - qy * tx)
    return vpy, vpx, vpz


def decode_e2box_quat(
    data: bytes,
    *,
    quat_scale: float,
    normalize: bool,
) -> tuple[tuple[float, float, float, float], tuple[float, float, float]]:
    if len(data) != 8:
        raise ValueError("E2Box quaternion frame requires 8 bytes")

    qz_raw, qy_raw, qx_raw, qw_raw = struct.unpack("<hhhh", data)
    qz = qz_raw / quat_scale
    qy = qy_raw / quat_scale
    qx = -(qx_raw / quat_scale)
    qw = qw_raw / quat_scale

    quat = normalize_quat_xyzw((qx, qy, qz, qw)) if normalize else (qx, qy, qz, qw)
    return quat, projected_gravity_from_xyzw(quat)


def decode_e2box_gyro(data: bytes, *, gyro_scale: float) -> tuple[float, float, float]:
    if len(data) != 8:
        raise ValueError("E2Box gyro frame requires 8 bytes")

    gx_raw, gy_raw, gz_raw, _reserved = struct.unpack("<hhhh", data)
    gx = (gx_raw / gyro_scale) * math.pi / 180.0
    gy = (gy_raw / gyro_scale) * math.pi / 180.0
    gz = (gz_raw / gyro_scale) * math.pi / 180.0
    return gy, gx, gz


def spg_opcode_name(opcode: int) -> str:
    return {
        SPG_CMD_MIT_CONTROL: "MIT_STATUS",
        SPG_CMD_MIT_ENTER: "MIT_ENTER_ACK",
        SPG_CMD_MIT_EXIT: "MIT_EXIT_ACK",
        SPG_CMD_MIT_SET_ZERO: "MIT_SET_ZERO_ACK",
        SPG_CMD_CLEAR_ERROR: "CLEAR_ERROR_ACK",
    }.get(opcode, f"OP_{opcode:02X}")


def decode_spg_status(
    data: bytes,
    *,
    feedback_position_max_rad: float,
    iq_full_scale_count: float,
    iq_full_scale_current_a: float,
) -> dict:
    if len(data) != 8:
        raise ValueError("SPG status frame requires 8 bytes")

    temp_c = struct.unpack("<b", data[1:2])[0]
    iq_count = struct.unpack("<h", data[2:4])[0]
    speed_dps = struct.unpack("<h", data[4:6])[0]
    # OpenRobot v14 0xC0 exception: DATA[6:7] is signed MIT feedback
    # position in output-side zero coordinates, not a 14-bit encoder count.
    position_i16 = struct.unpack("<h", data[6:8])[0]

    return {
        "temperature_c": int(temp_c),
        "iq_count": int(iq_count),
        "iq_a_approx": (iq_count / iq_full_scale_count) * iq_full_scale_current_a,
        "speed_dps": int(speed_dps),
        "position_rad": (position_i16 / 32767.0) * feedback_position_max_rad,
    }
