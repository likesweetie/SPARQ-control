from __future__ import annotations

import ctypes
import time
from dataclasses import dataclass
from multiprocessing import shared_memory


MAX_CONTROL_TARGETS = 12


class ControlTargetC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("can_id", ctypes.c_uint32),
        ("q", ctypes.c_float),
        ("dq", ctypes.c_float),
        ("kp", ctypes.c_float),
        ("kd", ctypes.c_float),
        ("tau", ctypes.c_float),
    ]


class ControlCommandC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp_ns", ctypes.c_uint64),
        ("num_targets", ctypes.c_uint32),
        ("targets", ControlTargetC * MAX_CONTROL_TARGETS),
    ]


CONTROL_COMMAND_SIZE = ctypes.sizeof(ControlCommandC)


@dataclass(frozen=True)
class ControlTarget:
    can_id: int
    q: float
    dq: float
    kp: float
    kd: float
    tau: float


class ControlCommandShm:
    def __init__(self, name: str, *, create: bool = False, size: int | None = None) -> None:
        self.name = str(name)
        requested_size = CONTROL_COMMAND_SIZE if size is None else int(size)
        if requested_size < CONTROL_COMMAND_SIZE:
            raise ValueError(
                f"ControlCommandShm size is too small: {requested_size}/{CONTROL_COMMAND_SIZE}"
            )
        self.shm = shared_memory.SharedMemory(
            name=self.name,
            create=bool(create),
            size=requested_size if create else 0,
        )
        if len(self.shm.buf) < CONTROL_COMMAND_SIZE:
            actual_size = len(self.shm.buf)
            self.close()
            raise RuntimeError(
                f"ControlCommandShm segment is too small: {actual_size}/{CONTROL_COMMAND_SIZE}"
            )

    @classmethod
    def open_reader(cls, name: str):
        return cls(name, create=False)

    @classmethod
    def open_writer(cls, name: str):
        return cls(name, create=False)

    @classmethod
    def create(cls, name: str, size: int | None = None):
        shm = cls(name, create=True, size=size)
        shm.clear()
        return shm

    def close(self) -> None:
        if self.shm is not None:
            self.shm.close()
            self.shm = None

    def unlink(self) -> None:
        if self.shm is not None:
            self.shm.unlink()

    def clear(self) -> None:
        self.shm.buf[: len(self.shm.buf)] = b"\x00" * len(self.shm.buf)

    def read_relaxed(self) -> ControlCommandC:
        return ControlCommandC.from_buffer_copy(self.shm.buf[:CONTROL_COMMAND_SIZE])

    def write(self, command: ControlCommandC) -> None:
        data = bytes(command)
        self.shm.buf[: len(data)] = data

    def write_targets(self, targets: list[ControlTarget]) -> None:
        if len(targets) > MAX_CONTROL_TARGETS:
            raise ValueError(f"too many control targets: {len(targets)}/{MAX_CONTROL_TARGETS}")
        command = ControlCommandC()
        command.timestamp_ns = time.time_ns()
        command.num_targets = len(targets)
        for index, target in enumerate(targets):
            command.targets[index].can_id = int(target.can_id)
            command.targets[index].q = float(target.q)
            command.targets[index].dq = float(target.dq)
            command.targets[index].kp = float(target.kp)
            command.targets[index].kd = float(target.kd)
            command.targets[index].tau = float(target.tau)
        self.write(command)
