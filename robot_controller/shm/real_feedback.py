from __future__ import annotations

import ctypes
from multiprocessing import shared_memory


MAX_REAL_FEEDBACK_MOTORS = 12


# Mirrors robot_controller/subprocesses/feedback_bridge/inc/RealFeedbackShm.hpp
# field-for-field. That header is the source of truth for this layout; if
# either side changes, update both together.
class RealFeedbackTargetC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("can_id", ctypes.c_uint32),
        ("pos", ctypes.c_float),
        ("vel", ctypes.c_float),
        ("torque", ctypes.c_float),
        ("temp", ctypes.c_float),
    ]


class RealFeedbackC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp_ns", ctypes.c_uint64),
        ("num_motors", ctypes.c_uint32),
        ("motors", RealFeedbackTargetC * MAX_REAL_FEEDBACK_MOTORS),
    ]


REAL_FEEDBACK_SIZE = ctypes.sizeof(RealFeedbackC)


class RealFeedbackShm:
    def __init__(self, name: str) -> None:
        self.name = str(name)
        self.shm = shared_memory.SharedMemory(name=self.name, create=False)
        if len(self.shm.buf) < REAL_FEEDBACK_SIZE:
            actual_size = len(self.shm.buf)
            self.close()
            raise RuntimeError(
                f"RealFeedbackShm segment is too small: {actual_size}/{REAL_FEEDBACK_SIZE}"
            )

    @classmethod
    def open_reader(cls, name: str) -> "RealFeedbackShm":
        return cls(name)

    def close(self) -> None:
        if self.shm is not None:
            self.shm.close()
            self.shm = None

    def read_relaxed(self) -> RealFeedbackC:
        return RealFeedbackC.from_buffer_copy(self.shm.buf[:REAL_FEEDBACK_SIZE])
