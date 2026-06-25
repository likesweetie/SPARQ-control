from __future__ import annotations

from hal.hardware.can.actuator.driver import ActuatorDriver

from .actuator_specs import QHRR0ActuatorSpec
from .dongilc_protocol import SPGActuatorProtocol, SPGMITConfig


def create_spg_actuator_driver(
    spec: QHRR0ActuatorSpec,
    *,
    mit_config: SPGMITConfig,
    feedback_timeout_s: float,
    feedback_speed_is_motor_side: bool,
    iq_count_to_amp: float | None,
) -> ActuatorDriver:
    return ActuatorDriver(
        name=spec.name,
        protocol=SPGActuatorProtocol(
            command_id=spec.can_id,
            feedback_id=spec.can_id,
            mit_config=mit_config,
            expose_single_turn_position=True,
            feedback_speed_is_motor_side=feedback_speed_is_motor_side,
            iq_count_to_amp=iq_count_to_amp,
        ),
        feedback_timeout=feedback_timeout_s,
    )
