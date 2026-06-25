from __future__ import annotations

import argparse
import errno
import os
import signal
import struct
import time
from pathlib import Path

from robot_controller.core.config import load_robot_controller_config
from robot_controller.shm.aux_command import AuxCommandShm


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JS_EVENT_STRUCT = struct.Struct("IhBB")


RUNNING = True


def _handle_signal(signum: int, _frame) -> None:
    global RUNNING
    print(f"[aux_reader] signal {signum}, shutting down", flush=True)
    RUNNING = False


def _axis_value(value: int) -> float:
    return max(-1.0, min(1.0, float(value) / 32767.0))


def _deadband(value: float, threshold: float) -> float:
    return 0.0 if abs(value) < threshold else value


def _axis_targets(axes: list[float]) -> tuple[list[float], list[float]]:
    mappings = (
        (1, 0, 1.0, True, 0.05, "lin"),
        (0, 1, 1.0, True, 0.05, "lin"),
        (3, 2, 1.0, True, 0.05, "ang"),
    )
    lin_vel_target = [0.0, 0.0, 0.0]
    ang_vel_target = [0.0, 0.0, 0.0]
    for axis, index, scale, invert, deadband, target in mappings:
        value = _deadband(axes[axis], deadband)
        if invert:
            value = -value
        if target == "lin":
            lin_vel_target[index] = value * scale
        else:
            ang_vel_target[index] = value * scale
    return lin_vel_target, ang_vel_target


def _button_targets(buttons: list[bool]) -> dict[str, bool]:
    fields = (
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
    return {field: bool(buttons[index]) for index, field in enumerate(fields)}


def _publish(writer: AuxCommandShm, axes: list[float], buttons: list[bool]) -> None:
    lin_vel_target, ang_vel_target = _axis_targets(axes)
    writer.publish(
        lin_vel_target=lin_vel_target,
        ang_vel_target=ang_vel_target,
        buttons=_button_targets(buttons),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="QHRR MuJoCo joystick auxiliary reader")
    parser.add_argument("--controller-config", type=Path, default=Path(os.environ.get("ROBOT_CONTROLLER_CONFIG", "config/app_config/robot_controller.yaml")))
    parser.add_argument("--joystick-dev", default=os.environ.get("JOYSTICK_DEV", "/dev/input/js0"))
    parser.add_argument("--poll-sleep-s", type=float, default=0.001)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    config = load_robot_controller_config(args.controller_config)
    writer = AuxCommandShm.open_writer(config.shm.aux_command.name)
    print(f"[aux_reader] publishing aux command shm: {config.shm.aux_command.name}", flush=True)

    fd = os.open(args.joystick_dev, os.O_RDONLY | os.O_NONBLOCK)
    print(f"[aux_reader] joystick device: {args.joystick_dev}", flush=True)

    axes = [0.0] * 32
    buttons = [False] * 32
    try:
        _publish(writer, axes, buttons)
        print("[aux_reader] published neutral aux command", flush=True)

        while RUNNING:
            try:
                packet = os.read(fd, JS_EVENT_STRUCT.size)
            except BlockingIOError:
                time.sleep(args.poll_sleep_s)
                continue
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    time.sleep(args.poll_sleep_s)
                    continue
                raise

            if len(packet) != JS_EVENT_STRUCT.size:
                raise RuntimeError(f"partial joystick event read: {len(packet)} bytes")

            _timestamp_ms, value, event_type, number = JS_EVENT_STRUCT.unpack(packet)
            event_type &= ~JS_EVENT_INIT
            if event_type == JS_EVENT_AXIS and number < len(axes):
                axes[number] = _axis_value(value)
                _publish(writer, axes, buttons)
            elif event_type == JS_EVENT_BUTTON and number < len(buttons):
                buttons[number] = value != 0
                _publish(writer, axes, buttons)
    finally:
        os.close(fd)
        writer.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
