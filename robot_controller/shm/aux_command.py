from __future__ import annotations

import ctypes
import time
from multiprocessing import shared_memory


class AuxCommandC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp_ns", ctypes.c_uint64),
        ("lin_vel_target", ctypes.c_float * 3),
        ("ang_vel_target", ctypes.c_float * 3),
        ("button_mask", ctypes.c_uint32),
    ]


AUX_COMMAND_SIZE = ctypes.sizeof(AuxCommandC)


BUTTON_FIELDS = (
    "a_button",
    "b_button",
    "x_button",
    "y_button",
    "lb_button",
    "rb_button",
    "back_button",
    "start_button",
    "guide_button",
    "l3_button",
    "r3_button",
)


class AuxCommandShm:
    def __init__(self, name: str, *, create: bool = False, size: int | None = None) -> None:
        self.name = str(name)
        requested_size = AUX_COMMAND_SIZE if size is None else int(size)
        if requested_size < AUX_COMMAND_SIZE:
            raise ValueError(f"AuxCommandShm size is too small: {requested_size}/{AUX_COMMAND_SIZE}")
        self.shm = shared_memory.SharedMemory(
            name=self.name,
            create=bool(create),
            size=requested_size if create else 0,
        )
        if len(self.shm.buf) < AUX_COMMAND_SIZE:
            actual_size = len(self.shm.buf)
            self.close()
            raise RuntimeError(f"AuxCommandShm segment is too small: {actual_size}/{AUX_COMMAND_SIZE}")

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

    def read_relaxed(self) -> AuxCommandC:
        return AuxCommandC.from_buffer_copy(self.shm.buf[:AUX_COMMAND_SIZE])

    def write(self, command: AuxCommandC) -> None:
        data = bytes(command)
        self.shm.buf[: len(data)] = data

    def publish(
        self,
        lin_vel_target: list[float],
        ang_vel_target: list[float],
        buttons: dict[str, bool],
    ) -> int:
        command = AuxCommandC()
        command.timestamp_ns = time.time_ns()
        for index in range(3):
            command.lin_vel_target[index] = float(lin_vel_target[index])
            command.ang_vel_target[index] = float(ang_vel_target[index])
        command.button_mask = buttons_to_mask(buttons)
        self.write(command)
        return int(command.timestamp_ns)


def buttons_to_mask(buttons: dict[str, bool]) -> int:
    mask = 0
    for index, name in enumerate(BUTTON_FIELDS):
        if bool(buttons.get(name, False)):
            mask |= 1 << index
    return mask


def mask_to_buttons(mask: int) -> dict[str, bool]:
    return {
        name: bool(int(mask) & (1 << index))
        for index, name in enumerate(BUTTON_FIELDS)
    }
