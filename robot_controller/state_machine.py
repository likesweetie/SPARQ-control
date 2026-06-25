from __future__ import annotations

import time
from dataclasses import dataclass
from enum import IntEnum

from robot_controller.shm.operator_command import OperatorCommandC, OperatorCommandCode


class ControllerMode(IntEnum):
    DISABLED = 0
    ENABLING = 1
    NORMAL = 2
    DAMPING = 3
    ZERO_SETTING = 4
    ESTOP = 5


@dataclass
class ControllerStateMachine:
    enable_duration_s: float
    mode: ControllerMode = ControllerMode.DISABLED
    mode_enter_time: float = 0.0

    def __post_init__(self) -> None:
        if self.enable_duration_s < 0.0:
            raise ValueError("enable_duration_s must be >= 0")
        if self.mode_enter_time <= 0.0:
            self.mode_enter_time = time.monotonic()

    def update(self, command: OperatorCommandC | None, now: float) -> ControllerMode:
        code = OperatorCommandCode.NONE
        if command is not None:
            try:
                code = OperatorCommandCode(int(command.command))
            except ValueError:
                code = OperatorCommandCode.NONE

        if code == OperatorCommandCode.ESTOP:
            self.enter(ControllerMode.ESTOP, now)
            return self.mode

        if self.mode == ControllerMode.ESTOP:
            if code == OperatorCommandCode.RESET_FAULT:
                self.enter(ControllerMode.DISABLED, now)
            return self.mode

        if code == OperatorCommandCode.DISABLE:
            self.enter(ControllerMode.DISABLED, now)
            return self.mode
        if code == OperatorCommandCode.ENABLE:
            if self.mode in (ControllerMode.DISABLED, ControllerMode.ZERO_SETTING):
                self.enter(ControllerMode.ENABLING, now)
                return self.mode
            if self.mode != ControllerMode.ENABLING:
                return self.mode
        if code == OperatorCommandCode.RUN:
            if self.mode == ControllerMode.DAMPING:
                self.enter(ControllerMode.NORMAL, now)
                return self.mode
            if self.mode == ControllerMode.NORMAL:
                return self.mode
        if code == OperatorCommandCode.DAMPING:
            self.enter(ControllerMode.DAMPING, now)
            return self.mode
        if code == OperatorCommandCode.ZERO_SET:
            self.enter(ControllerMode.ZERO_SETTING, now)
            return self.mode
        if code == OperatorCommandCode.RESET_FAULT:
            self.enter(ControllerMode.DISABLED, now)
            return self.mode

        if self.mode == ControllerMode.ZERO_SETTING and code == OperatorCommandCode.NONE:
            self.enter(ControllerMode.DISABLED, now)
            return self.mode

        if self.mode == ControllerMode.ENABLING:
            if now - self.mode_enter_time >= self.enable_duration_s:
                self.enter(ControllerMode.DAMPING, now)

        return self.mode

    def enter(self, mode: ControllerMode, now: float) -> None:
        if self.mode == mode:
            return
        self.mode = mode
        self.mode_enter_time = now
