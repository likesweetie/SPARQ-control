#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from robot_controller.core.platform_config import (
    load_platform_config,
    load_yaml_mapping,
    resolve_config_path,
)


PROJECT_ROOT = Path(__file__).resolve().parent


def require_path(path: Path, description: str) -> Path:
    resolved = (PROJECT_ROOT / path).resolve() if not path.is_absolute() else path
    if not resolved.exists():
        raise FileNotFoundError(f"{description} not found: {resolved}")
    return resolved


def build_env(
    args: argparse.Namespace,
    *,
    robot_name: str,
    model_config_path: Path,
    policy_config_dir: Path,
    pd_config_path: Path,
) -> dict[str, str]:
    env = dict(os.environ)
    env["ROBOT_NAME"] = robot_name
    env["QHRR_PROJECT_ROOT"] = str(PROJECT_ROOT)
    env["POLICY_CONFIG_DIR"] = str(require_path(policy_config_dir, "policy config directory"))
    env["PD_CONFIG_PATH"] = str(require_path(pd_config_path, "PD config"))
    env["MUJOCO_CAN_CONFIG"] = str(model_config_path)
    env["GLFW_PLATFORM"] = "wayland"

    library_paths: list[str] = []

    wsl_lib_dir = Path("/usr/lib/wsl/lib")
    if wsl_lib_dir.exists():
        library_paths.append(str(wsl_lib_dir))

        env["GALLIUM_DRIVER"] = "d3d12"
        env["MESA_LOADER_DRIVER_OVERRIDE"] = "d3d12"
        env["LIBGL_ALWAYS_SOFTWARE"] = "0"
        env["MUJOCO_GL"] = "glfw"

        # 혹시 기존 환경에서 간섭할 수 있는 값 제거
        env.pop("LIBGL_ALWAYS_INDIRECT", None)

    library_paths.extend(
        [
            str(require_path(Path("third_party/mujoco/lib"), "MuJoCo library directory")),
            str(require_path(Path("third_party/onnxruntime/lib"), "ONNX Runtime library directory")),
        ]
    )

    existing_ld_path = env.get("LD_LIBRARY_PATH")
    if existing_ld_path:
        library_paths.append(existing_ld_path)

    env["LD_LIBRARY_PATH"] = ":".join(library_paths)
    return env


def spawn(name: str, command: list[str], env: dict[str, str]) -> subprocess.Popen:
    print(f"[mujoco-launch] starting {name}: {' '.join(command)}", flush=True)
    print(f"[mujoco-launch] LD_LIBRARY_PATH={env.get('LD_LIBRARY_PATH', '')}", flush=True)
    print(f"[mujoco-launch] GALLIUM_DRIVER={env.get('GALLIUM_DRIVER', '')}", flush=True)
    print(f"[mujoco-launch] MESA_LOADER_DRIVER_OVERRIDE={env.get('MESA_LOADER_DRIVER_OVERRIDE', '')}", flush=True)
    print(f"[mujoco-launch] MUJOCO_GL={env.get('MUJOCO_GL', '')}", flush=True)
    return subprocess.Popen(command, cwd=PROJECT_ROOT, env=env, start_new_session=True)


def run_checked(command: list[str]) -> None:
    print(f"[mujoco-launch] running: {' '.join(command)}", flush=True)
    subprocess.run(command, cwd=PROJECT_ROOT, check=True)


def build_mujoco_simulate(build_dir: Path) -> Path:
    source_dir = require_path(Path("."), "MuJoCo CMake source directory")
    resolved_build_dir = (PROJECT_ROOT / build_dir).resolve() if not build_dir.is_absolute() else build_dir
    run_checked(["cmake", "-S", str(source_dir), "-B", str(resolved_build_dir)])
    run_checked(["cmake", "--build", str(resolved_build_dir), "--target", "mujoco_simulate"])
    return require_path(resolved_build_dir / "mujoco_simulate", "mujoco_simulate binary")


def terminate(processes: list[subprocess.Popen], timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    for process in processes:
        if process.poll() is None:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)

    for process in processes:
        remaining = max(0.0, deadline - time.monotonic())
        try:
            process.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait(timeout=timeout_s)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Launch QHRR MuJoCo simulation from the repository root.")
    parser.add_argument("--config", type=Path, default=Path("config/app_config/mujoco.yaml"))
    parser.add_argument("--robot")
    parser.add_argument("--build-dir", type=Path, default=Path("build/mujoco"))
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--shutdown-timeout-s", type=float, default=2.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    mujoco_config_path = require_path(Path(args.config), "MuJoCo app config")
    mujoco_raw = load_yaml_mapping(mujoco_config_path)
    if "platform_config" not in mujoco_raw:
        raise KeyError(f"platform_config is required in {mujoco_config_path}")
    platform_config_path = resolve_config_path(
        mujoco_config_path,
        str(mujoco_raw["platform_config"]),
        "platform_config",
    )
    platform = load_platform_config(platform_config_path)
    robot_name = args.robot or platform.robot.name
    if robot_name not in platform.robots:
        raise KeyError(f"Robot '{robot_name}' is not defined in {platform_config_path}")
    robot = platform.robots[robot_name]
    build_dir = Path(args.build_dir)
    if args.skip_build:
        build_dir = require_path(build_dir, "build directory")
        mujoco_simulate = require_path(build_dir / "mujoco_simulate", "mujoco_simulate binary")
    else:
        mujoco_simulate = build_mujoco_simulate(build_dir)
    model_path = require_path(Path(robot.model_path), "MuJoCo model")
    env = build_env(
        args,
        robot_name=robot_name,
        model_config_path=mujoco_config_path,
        policy_config_dir=Path(robot.policy_config_dir),
        pd_config_path=Path(robot.pd_config_path),
    )
    processes: list[subprocess.Popen] = []
    try:
        processes.append(spawn("mujoco_simulate", [str(mujoco_simulate), str(model_path)], env))
        while processes[0].poll() is None:
            time.sleep(0.1)
        return int(processes[0].returncode or 0)
    except KeyboardInterrupt:
        print("[mujoco-launch] signal received, shutting down", flush=True)
        return 130
    finally:
        terminate(processes, timeout_s=float(args.shutdown_timeout_s))


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"[mujoco-launch] fatal: {exc}", file=sys.stderr)
        raise SystemExit(1)
