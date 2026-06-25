from __future__ import annotations

from .actuators import QHRR0ActuatorSpec


def apply_joint_calibration(position_rad: float, spec: QHRR0ActuatorSpec) -> float:
    return float(spec.sign) * float(position_rad) + float(spec.offset_rad)


def remove_joint_calibration(position_rad: float, spec: QHRR0ActuatorSpec) -> float:
    return (float(position_rad) - float(spec.offset_rad)) / float(spec.sign)

