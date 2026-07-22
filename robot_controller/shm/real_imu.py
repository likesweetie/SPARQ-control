from __future__ import annotations

import ctypes
from multiprocessing import shared_memory


# Mirrors robot_controller/subprocesses/imu_bridge/inc/RealImuShm.hpp
# field-for-field. That header is the source of truth for this layout; if
# either side changes, update both together.
class RealImuC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp_ns", ctypes.c_uint64),
        ("quat_wxyz", ctypes.c_float * 4),
        ("ang_vel_rad_s", ctypes.c_float * 3),
    ]


REAL_IMU_SIZE = ctypes.sizeof(RealImuC)


class RealImuShm:
    def __init__(self, name: str) -> None:
        self.name = str(name)
        self.shm = shared_memory.SharedMemory(name=self.name, create=False)
        if len(self.shm.buf) < REAL_IMU_SIZE:
            actual_size = len(self.shm.buf)
            self.close()
            raise RuntimeError(f"RealImuShm segment is too small: {actual_size}/{REAL_IMU_SIZE}")

    @classmethod
    def open_reader(cls, name: str) -> "RealImuShm":
        return cls(name)

    def close(self) -> None:
        if self.shm is not None:
            self.shm.close()
            self.shm = None

    def read_relaxed(self) -> RealImuC:
        return RealImuC.from_buffer_copy(self.shm.buf[:REAL_IMU_SIZE])
