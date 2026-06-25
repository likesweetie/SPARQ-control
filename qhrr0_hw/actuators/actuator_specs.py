from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class QHRR0ActuatorSpec:
    name: str
    can_id: int
    joint_index: int
    joint_name: str
    sign: float
    offset_rad: float
    enabled: bool


def actuator_specs_from_platform(platform) -> tuple[QHRR0ActuatorSpec, ...]:
    specs: list[QHRR0ActuatorSpec] = []
    for index, actuator in enumerate(platform.actuators):
        if not bool(actuator.enabled):
            continue
        specs.append(
            QHRR0ActuatorSpec(
                name=str(actuator.name),
                can_id=int(actuator.can_id),
                joint_index=index,
                joint_name=str(actuator.mujoco_joint),
                sign=float(actuator.sign),
                offset_rad=float(actuator.offset_rad),
                enabled=bool(actuator.enabled),
            )
        )
    return tuple(specs)


def can_ids(specs: Iterable[QHRR0ActuatorSpec]) -> list[int]:
    return [int(spec.can_id) for spec in specs]

