from __future__ import annotations

import ctypes
from multiprocessing import shared_memory


def struct_size(struct_type: type[ctypes.Structure]) -> int:
    return ctypes.sizeof(struct_type)


def clear_buffer(buf) -> None:
    buf[:] = b"\x00" * len(buf)


class SharedCStruct:
    struct_type: type[ctypes.Structure]

    def __init__(self, name: str, *, create: bool, size: int | None = None) -> None:
        self.name = str(name)
        required_size = ctypes.sizeof(self.struct_type)
        requested_size = required_size if size is None else int(size)
        if requested_size < required_size:
            raise ValueError(
                f"SHM size for {self.name} is too small: {requested_size}/{required_size}"
            )
        self.shm = shared_memory.SharedMemory(
            name=self.name,
            create=bool(create),
            size=requested_size if create else 0,
        )
        if len(self.shm.buf) < required_size:
            self.close()
            raise RuntimeError(
                f"SHM segment {self.name} is too small: {len(self.shm.buf)}/{required_size}"
            )

    def close(self) -> None:
        if self.shm is not None:
            self.shm.close()
            self.shm = None

    def unlink(self) -> None:
        if self.shm is not None:
            self.shm.unlink()

    def read_struct(self):
        return self.struct_type.from_buffer_copy(self.shm.buf[: ctypes.sizeof(self.struct_type)])

    def write_struct(self, value) -> None:
        data = bytes(value)
        self.shm.buf[: len(data)] = data

