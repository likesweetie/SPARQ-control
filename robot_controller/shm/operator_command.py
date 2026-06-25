from __future__ import annotations

import ctypes
import time
from collections.abc import Iterable
from enum import IntEnum
from multiprocessing import shared_memory


class OperatorCommandCode(IntEnum):
    NONE = 0
    ENABLE = 1
    DISABLE = 2
    DAMPING = 3
    ZERO_SET = 4
    ESTOP = 5
    RESET_FAULT = 6
    RUN = 7


OPERATOR_ZERO_TARGET_CAPACITY = 12
OPERATOR_ZERO_TARGET_MAGIC = 0x5A45524F


class OperatorZeroTargetC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("can_id", ctypes.c_uint32),
        ("offset_count", ctypes.c_int16),
        ("reserved", ctypes.c_uint16),
    ]


class OperatorCommandC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp_ns", ctypes.c_uint64),
        ("command", ctypes.c_uint32),
        ("target_mask", ctypes.c_uint32),
        ("zero_target_count", ctypes.c_uint32),
        ("zero_target_magic", ctypes.c_uint32),
        ("zero_targets", OperatorZeroTargetC * OPERATOR_ZERO_TARGET_CAPACITY),
    ]


OPERATOR_COMMAND_SIZE = ctypes.sizeof(OperatorCommandC)


class OperatorCommandShm:
    def __init__(self, name: str, *, create: bool = False, size: int | None = None) -> None:
        self.name = str(name)
        requested_size = OPERATOR_COMMAND_SIZE if size is None else int(size)
        if requested_size < OPERATOR_COMMAND_SIZE:
            raise ValueError(
                f"OperatorCommandShm size is too small: {requested_size}/{OPERATOR_COMMAND_SIZE}"
            )
        self.shm = shared_memory.SharedMemory(
            name=self.name,
            create=bool(create),
            size=requested_size if create else 0,
        )
        if len(self.shm.buf) < OPERATOR_COMMAND_SIZE:
            self.close()
            raise RuntimeError(
                f"OperatorCommandShm segment is too small: {len(self.shm.buf)}/{OPERATOR_COMMAND_SIZE}"
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

    def read_relaxed(self) -> OperatorCommandC:
        return OperatorCommandC.from_buffer_copy(self.shm.buf[:OPERATOR_COMMAND_SIZE])

    def write(self, command: OperatorCommandC) -> None:
        data = bytes(command)
        self.shm.buf[: len(data)] = data

    def publish(self, code: OperatorCommandCode | int, target_mask: int = 0) -> int:
        command = self._build_command(code, target_mask=target_mask)
        self.write(command)
        return int(command.timestamp_ns)

    def publish_zero_set(self, targets: Iterable[tuple[int, int]] = ()) -> int:
        command = self._build_command(
            OperatorCommandCode.ZERO_SET,
            zero_targets=targets,
        )
        self.write(command)
        return int(command.timestamp_ns)

    def _build_command(
        self,
        code: OperatorCommandCode | int,
        *,
        target_mask: int = 0,
        zero_targets: Iterable[tuple[int, int]] = (),
    ) -> OperatorCommandC:
        command = OperatorCommandC()
        command.timestamp_ns = time.time_ns()
        command.command = int(code)
        command.target_mask = int(target_mask)
        targets = tuple(zero_targets)
        if len(targets) > OPERATOR_ZERO_TARGET_CAPACITY:
            raise ValueError(
                f"zero_set target count exceeds capacity: "
                f"{len(targets)}/{OPERATOR_ZERO_TARGET_CAPACITY}"
            )
        command.zero_target_count = len(targets)
        command.zero_target_magic = OPERATOR_ZERO_TARGET_MAGIC if targets else 0
        for index, (can_id, offset_count) in enumerate(targets):
            can_id_int = int(can_id)
            offset_count_int = int(offset_count)
            if not (0 <= can_id_int <= 0x1FFFFFFF):
                raise ValueError(f"CAN ID out of range: {can_id_int}")
            if not (-32768 <= offset_count_int <= 32767):
                raise ValueError(f"MIT zero offset_count out of int16 range: {offset_count_int}")
            command.zero_targets[index].can_id = can_id_int
            command.zero_targets[index].offset_count = offset_count_int
        return command


class OperatorCommandShmWriter:
    def __init__(self, name: str, size_bytes: int | None = None, *, source: str = "") -> None:
        del size_bytes, source
        self.writer = OperatorCommandShm.open_writer(name)

    def close(self) -> None:
        self.writer.close()

    def publish(
        self,
        *,
        arm: bool = False,
        clear_fault: bool = False,
        estop: bool = False,
        damping: bool = False,
        zero_set: bool = False,
        disable: bool = False,
        run: bool = False,
    ) -> int:
        if arm:
            return self.writer.publish(OperatorCommandCode.ENABLE)
        if run:
            return self.writer.publish(OperatorCommandCode.RUN)
        if clear_fault:
            return self.writer.publish(OperatorCommandCode.RESET_FAULT)
        if estop:
            return self.writer.publish(OperatorCommandCode.ESTOP)
        if damping:
            return self.writer.publish(OperatorCommandCode.DAMPING)
        if zero_set:
            return self.publish_zero_set()
        if disable:
            return self.writer.publish(OperatorCommandCode.DISABLE)
        return self.writer.publish(OperatorCommandCode.NONE)

    def publish_zero_set(self, targets: Iterable[tuple[int, int]] = ()) -> int:
        return self.writer.publish_zero_set(targets)
