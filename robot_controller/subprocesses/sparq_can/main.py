from __future__ import annotations

import argparse
import os
from pathlib import Path


SPARQ_CAN_DIR = Path(__file__).resolve().parent
DEFAULT_BINARY = SPARQ_CAN_DIR / "SPARQ_CAN"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the SPARQ CAN C subprocess")
    parser.add_argument(
        "--binary",
        type=Path,
        default=DEFAULT_BINARY,
        help="Path to the compiled SPARQ_CAN executable",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    binary = args.binary.resolve()
    if not binary.exists():
        raise FileNotFoundError(f"SPARQ CAN binary not found: {binary}")
    if not os.access(binary, os.X_OK):
        raise PermissionError(f"SPARQ CAN binary is not executable: {binary}")
    os.chdir(binary.parent)
    os.execv(str(binary), [str(binary)])


if __name__ == "__main__":
    main()
