from __future__ import annotations

from .actuators import QHRR0ActuatorSpec


def can_id_order(specs: tuple[QHRR0ActuatorSpec, ...]) -> list[int]:
    return [spec.can_id for spec in specs]


def joint_names(specs: tuple[QHRR0ActuatorSpec, ...]) -> list[str]:
    return [spec.joint_name for spec in specs]

