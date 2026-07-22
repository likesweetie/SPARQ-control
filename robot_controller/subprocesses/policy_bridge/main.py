from __future__ import annotations

import argparse
import os
from pathlib import Path


POLICY_BRIDGE_DIR = Path(__file__).resolve().parent
DEFAULT_BINARY = POLICY_BRIDGE_DIR / "policy_bridge"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the policy_bridge C++ subprocess")
    parser.add_argument(
        "--binary",
        type=Path,
        default=DEFAULT_BINARY,
        help="Path to the compiled policy_bridge executable",
    )
    parser.add_argument(
        "--enable-motor-output",
        action="store_true",
        help=(
            "Forwarded to the policy_bridge binary. Without this flag the bridge "
            "always writes Kp=Kd=0 to the real motors regardless of policy output."
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    binary = args.binary.resolve()
    if not binary.exists():
        raise FileNotFoundError(f"policy_bridge binary not found: {binary}")
    if not os.access(binary, os.X_OK):
        raise PermissionError(f"policy_bridge binary is not executable: {binary}")
    os.chdir(binary.parent)
    argv = [str(binary)]
    if args.enable_motor_output:
        argv.append("--enable-motor-output")
    os.execv(str(binary), argv)


if __name__ == "__main__":
    main()
