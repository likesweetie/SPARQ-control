from __future__ import annotations

import argparse
import os
from pathlib import Path


IMU_BRIDGE_DIR = Path(__file__).resolve().parent
DEFAULT_BINARY = IMU_BRIDGE_DIR / "imu_bridge"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the imu_bridge C++ subprocess")
    parser.add_argument(
        "--binary",
        type=Path,
        default=DEFAULT_BINARY,
        help="Path to the compiled imu_bridge executable",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    binary = args.binary.resolve()
    if not binary.exists():
        raise FileNotFoundError(f"imu_bridge binary not found: {binary}")
    if not os.access(binary, os.X_OK):
        raise PermissionError(f"imu_bridge binary is not executable: {binary}")
    os.chdir(binary.parent)
    os.execv(str(binary), [str(binary)])


if __name__ == "__main__":
    main()
