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
from robot_controller.shm.real_feedback import RealFeedbackShm
from robot_controller.shm.real_imu import RealImuShm
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

    # qhrr_control_state's actuator/IMU feedback is Path1 (RobotController's
    # own CAN daemon), whose CAN interface is vcan0 with no real device
    # attached -- so it never updates and dof_pos/dof_vel/quat/gyro would
    # silently stay frozen forever (confirmed empirically: obs was frozen
    # for a full session). Real joint feedback comes from feedback_bridge
    # (sparq_can's live fb block); real IMU comes from imu_bridge (the
    # WitMotion serial IMU via imu_serial_cpp, converted to quat/rad-s).
    real_feedback_shm_name = os.environ.get("REAL_FEEDBACK_SHM_NAME", "qhrr_real_feedback")
    real_imu_shm_name = os.environ.get("REAL_IMU_SHM_NAME", "qhrr_real_imu")
    control_state_reader = RobotStateShm.open_reader(controller_config.shm.control_state.name)
    aux_reader = AuxCommandShm.open_reader(controller_config.shm.aux_command.name)
    control_command_writer = ControlCommandShm.open_writer(controller_config.shm.mit_command.name)
    print(f"[task_controller] waiting for real_feedback shm: {real_feedback_shm_name}", flush=True)
    real_feedback_reader = None
    while RUNNING and real_feedback_reader is None:
        try:
            real_feedback_reader = RealFeedbackShm.open_reader(real_feedback_shm_name)
        except FileNotFoundError:
            time.sleep(0.1)
    if real_feedback_reader is None:
        return 0
    print(f"[task_controller] waiting for real_imu shm: {real_imu_shm_name}", flush=True)
    real_imu_reader = None
    while RUNNING and real_imu_reader is None:
        try:
            real_imu_reader = RealImuShm.open_reader(real_imu_shm_name)
        except FileNotFoundError:
            time.sleep(0.1)
    if real_imu_reader is None:
        real_feedback_reader.close()
        return 0
    print(
        f"[task_controller] control={controller_config.shm.control_state.name} "
        f"aux={controller_config.shm.aux_command.name} control_cmd={controller_config.shm.mit_command.name} "
        f"real_feedback={real_feedback_shm_name} real_imu={real_imu_shm_name}",
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

            real_feedback = real_feedback_reader.read_relaxed()
            real_motors = {
                int(m.can_id): m
                for m in real_feedback.motors[: int(real_feedback.num_motors)]
            }
            dof_pos = np.asarray(
                [float(real_motors[can_id].pos) for can_id in can_ids],
                dtype=np.float32,
            )
            dof_vel = np.asarray(
                [float(real_motors[can_id].vel) for can_id in can_ids],
                dtype=np.float32,
            )
            real_imu = real_imu_reader.read_relaxed()
            quat = [float(value) for value in real_imu.quat_wxyz]
            gyro = np.asarray(
                [float(value) for value in real_imu.ang_vel_rad_s],
                dtype=np.float32,
            )

            mode = bool(buttons.get("a_button", False))
            active_policy.set_state(dof_pos, dof_vel, quat, gyro)
            active_policy.set_commands(float(lin_vel[0]), float(lin_vel[1]), float(ang_vel_cmd[2]), mode)
            q_target = (active_policy.compute_action()*mode) + action_offset(active_policy, robot_name, dof_pos, mode)

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
        real_feedback_reader.close()
        real_imu_reader.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
