import math
import struct
import time

from hal.can_bus import CANFrame
from hal.hardware.can.imu import IMUProtocolBase
from .robot_state import RobotPoseState


class E2BoxIMUProtocol(IMUProtocolBase):
    def __init__(
        self,
        *,
        request_id: int,
        quat_id: int,
        gyro_id: int,
        cmd_get_quat: int,
        cmd_get_gyro: int,
        cmd_get_all: int,
        quat_scale: float,
        gyro_scale: float,
        normalize_quat: bool,
    ) -> None:
        self.request_id = int(request_id)
        self.quat_id = int(quat_id)
        self.gyro_id = int(gyro_id)
        self.cmd_get_quat = int(cmd_get_quat)
        self.cmd_get_gyro = int(cmd_get_gyro)
        self.cmd_get_all = int(cmd_get_all)
        self.quat_scale = float(quat_scale)
        self.gyro_scale = float(gyro_scale)
        self.normalize_quat = bool(normalize_quat)

    def rx_can_ids(self) -> list[int]:
        return [self.quat_id, self.gyro_id]

    def encode_request_quat(self) -> CANFrame:
        return CANFrame(can_id=self.request_id, data=bytes([self.cmd_get_quat]))

    def encode_request_gyro(self) -> CANFrame:
        return CANFrame(can_id=self.request_id, data=bytes([self.cmd_get_gyro]))

    def encode_request_all(self) -> CANFrame:
        return CANFrame(can_id=self.request_id, data=bytes([self.cmd_get_all]))

    def decode_frame(self, frame: CANFrame) -> RobotPoseState | None:
        if frame.can_id == self.quat_id:
            return self._decode_quat(frame.data)

        if frame.can_id == self.gyro_id:
            return self._decode_gyro(frame.data)

        return None

    def _decode_quat(self, data: bytes) -> RobotPoseState:
        if len(data) != 8:
            raise ValueError(f"Quaternion payload must be 8 bytes, got {len(data)}")

        qz_raw, qy_raw, qx_raw, qw_raw = struct.unpack("<hhhh", data)

        qz = qz_raw / self.quat_scale
        qy = qy_raw / self.quat_scale
        qx = qx_raw / self.quat_scale
        qw = qw_raw / self.quat_scale

        # E2Box convention correction.
        qx = -qx

        quat_xyzw = (qx, qy, qz, qw)
        if self.normalize_quat:
            quat_xyzw = self._normalize_quat(quat_xyzw)
        projected_gravity_b = self._projected_gravity_from_xyzw(quat_xyzw)

        return RobotPoseState(
            quat_xyzw=quat_xyzw,
            projected_gravity_b=projected_gravity_b,
            last_quat_t=time.monotonic(),
        )

    def _decode_gyro(self, data: bytes) -> RobotPoseState:
        if len(data) != 8:
            raise ValueError(f"Gyro payload must be 8 bytes, got {len(data)}")

        gx_raw, gy_raw, gz_raw, _reserved = struct.unpack("<hhhh", data)

        gx = (gx_raw / self.gyro_scale) * math.pi / 180.0
        gy = (gy_raw / self.gyro_scale) * math.pi / 180.0
        gz = (gz_raw / self.gyro_scale) * math.pi / 180.0

        # E2Box convention correction: swap x/y.
        gx, gy = gy, gx

        return RobotPoseState(
            angular_velocity_rad_s=(gx, gy, gz),
            last_gyro_t=time.monotonic(),
        )

    @staticmethod
    def _normalize_quat(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        qx, qy, qz, qw = q
        n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if n < 1e-12:
            return 0.0, 0.0, 0.0, 1.0
        return qx / n, qy / n, qz / n, qw / n

    @staticmethod
    def _projected_gravity_from_xyzw(
        q: tuple[float, float, float, float],
    ) -> tuple[float, float, float]:
        qx, qy, qz, qw = q

        vx, vy, vz = 0.0, 0.0, -1.0

        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)

        vpx = vx - qw * tx + (qy * tz - qz * ty)
        vpy = vy - qw * ty + (qz * tx - qx * tz)
        vpz = vz - qw * tz + (qx * ty - qy * tx)

        # !!! E2Box/robot convention correction. !!!
        return vpy, vpx, vpz #becareful for the order
