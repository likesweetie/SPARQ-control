from __future__ import annotations

from multiprocessing import shared_memory

from robot_controller.core.config import ShmConfig
from robot_controller.shm.aux_command import AuxCommandShm
from robot_controller.shm.control_command import ControlCommandShm
from robot_controller.shm.operator_command import OperatorCommandShm
from robot_controller.shm.robot_state import RobotStateShm


class ShmManager:
    def __init__(self, config: ShmConfig):
        self.config = config
        self._segments: dict[str, shared_memory.SharedMemory] = {}

    def cleanup_stale(self) -> None:
        for name in self._segment_names():
            try:
                stale = shared_memory.SharedMemory(name=name, create=False)
            except FileNotFoundError:
                continue
            stale.close()
            stale.unlink()

    def create_all(self) -> None:
        control_command = ControlCommandShm.create(self.config.mit_command.name)
        self._segments[self.config.mit_command.name] = control_command.shm

        aux_command = AuxCommandShm.create(
            self.config.aux_command.name,
            size=int(self.config.aux_command.size_bytes),
        )
        self._segments[self.config.aux_command.name] = aux_command.shm

        operator_command = OperatorCommandShm.create(
            self.config.operator_command.name,
            size=int(self.config.operator_command.size_bytes),
        )
        self._segments[self.config.operator_command.name] = operator_command.shm

        control_state = RobotStateShm.create(
            name=self.config.control_state.name,
            size=int(self.config.control_state.size_bytes),
        )
        self._segments[self.config.control_state.name] = control_state.shm

        dashboard_state = RobotStateShm.create(
            name=self.config.dashboard_state.name,
            size=int(self.config.dashboard_state.size_bytes),
        )
        self._segments[self.config.dashboard_state.name] = dashboard_state.shm

    def close_all(self) -> None:
        for segment in self._segments.values():
            segment.close()
        self._segments.clear()

    def unlink_all(self) -> None:
        for name in self._segment_names():
            try:
                segment = shared_memory.SharedMemory(name=name, create=False)
            except FileNotFoundError:
                continue
            segment.close()
            segment.unlink()

    def _segment_names(self) -> tuple[str, ...]:
        return (
            self.config.mit_command.name,
            self.config.aux_command.name,
            self.config.operator_command.name,
            self.config.control_state.name,
            self.config.dashboard_state.name,
        )
