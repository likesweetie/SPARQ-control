from __future__ import annotations

from dataclasses import dataclass

from .actuators import QHRR0ActuatorSpec, actuator_specs_from_platform
from .imu import QHRR0ImuSpec, imu_spec_from_platform


@dataclass(frozen=True)
class QHRR0RobotSpec:
    actuators: tuple[QHRR0ActuatorSpec, ...]
    imu: QHRR0ImuSpec

    @property
    def can_ids(self) -> list[int]:
        return [spec.can_id for spec in self.actuators]


def robot_spec_from_platform(platform) -> QHRR0RobotSpec:
    return QHRR0RobotSpec(
        actuators=actuator_specs_from_platform(platform),
        imu=imu_spec_from_platform(platform),
    )
