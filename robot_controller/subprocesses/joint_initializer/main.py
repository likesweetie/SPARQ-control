from __future__ import annotations

import argparse
import os
import signal
import time
from pathlib import Path

import numpy as np

from robot_controller.core.config import load_robot_controller_config
from robot_controller.subprocesses.task_controller.policy_runner import (
    action_offset,
    load_policies,
    load_yaml,
    project_root,
    resolve_policy_config_dir,
)
from robot_controller.shm.aux_command import AuxCommandShm, mask_to_buttons
from robot_controller.shm.control_command import ControlCommandShm, ControlTarget
from robot_controller.shm.robot_state import RobotStateShm


RUNNING = True


def _float_env(name: str) -> float | None:
    value = os.environ.get(name)
    if value is None:
        return None
    try:
        return float(value)
    except ValueError as exc:
        raise SystemExit(f"{name} must be a float, got {value!r}") from exc


def _handle_signal(signum: int, _frame) -> None:
    global RUNNING
    print(f"[task_controller] signal {signum}, shutting down", flush=True)
    RUNNING = False


def parse_args() -> argparse.Namespace:
    control_hz_default = _float_env("TASK_CONTROL_HZ")
    rate_log_interval_s_default = _float_env("TASK_RATE_LOG_INTERVAL_S")
    parser = argparse.ArgumentParser(description="QHRR Python task controller")
    parser.add_argument("--controller-config", default=os.environ.get("ROBOT_CONTROLLER_CONFIG", "config/app_config/robot_controller.yaml"))
    parser.add_argument("--robot-name", default=os.environ.get("ROBOT_NAME"))
    parser.add_argument("--project-root", default=os.environ.get("QHRR_PROJECT_ROOT", "."))
    parser.add_argument("--policy-config-dir", default=os.environ.get("POLICY_CONFIG_DIR"))
    parser.add_argument("--control-hz", type=float, default=control_hz_default, required=control_hz_default is None)
    parser.add_argument("--rate-log-interval-s", type=float, default=rate_log_interval_s_default)
    return parser.parse_args()


def _controller_config_path(root: Path, value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else root / path


def _sleep_until_next_tick(tick_start: float, period_s: float) -> None:
    elapsed_s = time.monotonic() - tick_start
    if elapsed_s < period_s:
        time.sleep(period_s - elapsed_s)


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    root = project_root(args.project_root)
    controller_config = load_robot_controller_config(_controller_config_path(root, args.controller_config))
    robot_name = args.robot_name or controller_config.platform.robot.name
    robot_assets = controller_config.platform.robots[robot_name]
    policy_config_dir = resolve_policy_config_dir(
        root,
        args.policy_config_dir or robot_assets.policy_config_dir,
        robot_name,
    )
    active_policy = load_policies(root, policy_config_dir)[0]
    pd_config = load_yaml(active_policy.directory / "pd_config.yaml")
    kp = float(pd_config["kp"])
    kd = float(pd_config["kd"])

    can_ids = [int(can_id) for can_id in controller_config.can.motors.can_ids]

    control_state_reader = RobotStateShm.open_reader(controller_config.shm.control_state.name)
    aux_reader = AuxCommandShm.open_reader(controller_config.shm.aux_command.name)
    control_command_writer = ControlCommandShm.open_writer(controller_config.shm.mit_command.name)
    print(
        f"[task_controller] control={controller_config.shm.control_state.name} "
        f"aux={controller_config.shm.aux_command.name} control_cmd={controller_config.shm.mit_command.name}",
        flush=True,
    )

    period_s = 1.0 / args.control_hz
    if args.rate_log_interval_s is not None and args.rate_log_interval_s < 0.0:
        raise ValueError("--rate-log-interval-s must be >= 0")
    print(
        f"[task_controller] target_policy_output_hz={args.control_hz:.3f} "
        f"rate_log_interval_s={args.rate_log_interval_s}",
        flush=True,
    )
    try:
        print("[task_controller] waiting for control_state", flush=True)
        while RUNNING:
            control_state = control_state_reader.read_relaxed()
            if int(control_state.timestamp_ns) != 0:
                print("[task_controller] control_state received", flush=True)
                break
            time.sleep(period_s)

        published_count = 0
        last_rate_report_t = time.monotonic()
        last_rate_report_count = 0
        while RUNNING:
            tick_start = time.monotonic()

            control_state = control_state_reader.read_relaxed()
            if int(control_state.timestamp_ns) == 0:
                _sleep_until_next_tick(tick_start, period_s)
                continue

            aux_state = aux_reader.read_relaxed()
            lin_vel = [float(value) for value in aux_state.lin_vel_target]
            ang_vel_cmd = [float(value) for value in aux_state.ang_vel_target]
            buttons = mask_to_buttons(int(aux_state.button_mask))

            actuators = {
                int(item.can_id): item
                for item in control_state.actuators[: int(control_state.actuator_count)]
            }
            dof_pos = np.asarray(
                [float(actuators[can_id].position_rad) for can_id in can_ids],
                dtype=np.float64,
            )
            dof_vel = np.asarray(
                [float(actuators[can_id].velocity_rad_s) for can_id in can_ids],
                dtype=np.float64,
            )
            imu = control_state.imu
            quat_xyzw = imu.quat_xyzw
            quat = [
                float(quat_xyzw[3]),
                float(quat_xyzw[0]),
                float(quat_xyzw[1]),
                float(quat_xyzw[2]),
            ]
            gyro = np.asarray(
                [float(value) for value in imu.angular_velocity_rad_s],
                dtype=np.float64,
            )

            mode = bool(buttons.get("a_button", False))
            active_policy.set_state(dof_pos, dof_vel, quat, gyro)
            active_policy.set_commands(float(lin_vel[0]), float(lin_vel[1]), float(ang_vel_cmd[2]), mode)
            q_target = active_policy.compute_action() + action_offset(active_policy, robot_name, dof_pos, mode)

            control_command_writer.write_targets(
                [
                    ControlTarget(
                        can_id=can_id,
                        q=float(q_target[index]),
                        dq=0.0,
                        kp=kp,
                        kd=kd,
                        tau=0.0,
                    )
                    for index, can_id in enumerate(can_ids)
                ]
            )
            published_count += 1

            now = time.monotonic()
            if args.rate_log_interval_s and now - last_rate_report_t >= args.rate_log_interval_s:
                dt_s = now - last_rate_report_t
                delta_count = published_count - last_rate_report_count
                actual_hz = delta_count / dt_s
                print(
                    f"[task_controller] policy_output_rate_hz={actual_hz:.2f} "
                    f"published={published_count} target_hz={args.control_hz:.2f}",
                    flush=True,
                )
                last_rate_report_t = now
                last_rate_report_count = published_count

            elapsed_s = now - tick_start
            if elapsed_s >= period_s:
                print(f"[task_controller] loop overrun: {elapsed_s:.6f}s", flush=True)
            _sleep_until_next_tick(tick_start, period_s)
    finally:
        control_state_reader.close()
        aux_reader.close()
        control_command_writer.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
