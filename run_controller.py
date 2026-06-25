#!/usr/bin/env python3
from __future__ import annotations

import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent
SPARQ_CAN_DIR = PROJECT_ROOT / "robot_controller" / "subprocesses" / "sparq_can"


def build_sparq_can() -> None:
    if not SPARQ_CAN_DIR.is_dir():
        raise FileNotFoundError(f"SPARQ CAN subprocess directory not found: {SPARQ_CAN_DIR}")
    subprocess.run(["make"], cwd=SPARQ_CAN_DIR, check=True)


def main() -> int:
    build_sparq_can()
    from robot_controller.main import main as robot_controller_main

    robot_controller_main()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except subprocess.CalledProcessError as exc:
        print(f"[run_controller] SPARQ CAN build failed: {exc}", file=sys.stderr)
        raise SystemExit(exc.returncode)
