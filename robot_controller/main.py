from __future__ import annotations

import argparse
import signal
from pathlib import Path

from .core.config import load_robot_controller_config
from .controller import RobotController


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "app_config" / "robot_controller.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="QHRR RobotController runtime")
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help="RobotController YAML config path",
    )
    parser.add_argument(
        "--estop-ok",
        action="store_true",
        help="Declare that the hardware E-stop path was checked before startup.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = load_robot_controller_config(args.config)
    controller = RobotController(config)

    def handle_signal(signum: int, _frame: object) -> None:
        print(f"[robot_controller] signal {signum}, shutting down")
        controller.request_stop()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        controller.start()
        controller.run()
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()
