"""
Actuator state definitions.

This module contains product-independent state containers for CAN-based
actuators. Vendor-specific raw payload details should remain in protocol
implementations, not in this state layer.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class ActuatorState:
    """
    Latest actuator feedback state.

    Internal unit convention:
    - position: rad
    - velocity: rad/s
    - torque: Nm
    - current: A
    - voltage: V
    - temperature: degC

    Most fields are optional because many actuator protocols return partial
    feedback packets.
    """

    position_rad: float | None = None
    velocity_rad_s: float | None = None
    torque_nm: float | None = None
    current_a: float | None = None

    temperature_c: float | None = None
    voltage_v: float | None = None

    fault_code: int | None = None
    is_enabled: bool | None = None
    mode: str | None = None

    last_feedback_t: float = 0.0

    # Optional vendor-specific decoded fields.
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class ActuatorCommand:
    """
    Generic actuator command container.

    Vendor-specific protocol classes may support only a subset of these fields.
    Unsupported command fields should be rejected in the protocol layer.
    """

    position_rad: float | None = None
    velocity_rad_s: float | None = None
    torque_nm: float | None = None

    kp: float | None = None
    kd: float | None = None

    current_a: float | None = None
    mode: str | None = None


@dataclass
class ActuatorLimits:
    """
    Optional actuator limits used by higher-level managers.

    This class is not enforced by ActuatorDriver by default. Enforcement can be
    added in a safety layer, controller layer, or product-specific driver.
    """

    position_min_rad: float | None = None
    position_max_rad: float | None = None
    velocity_max_rad_s: float | None = None
    torque_max_nm: float | None = None
    current_max_a: float | None = None