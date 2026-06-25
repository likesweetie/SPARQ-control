from __future__ import annotations

from robot_controller.telemetry.robot_snapshot import RobotSnapshot
from robot_controller.telemetry.shm_state_publisher import ShmStatePublisher


class DashboardPublisher:
    def __init__(self, shm_name: str, publish_hz: float) -> None:
        self.publisher = ShmStatePublisher(shm_name, publish_hz)

    def close(self) -> None:
        self.publisher.close()

    def publish(self, snapshot: RobotSnapshot, *, force: bool = False) -> None:
        self.publisher.publish(snapshot, force=force)
