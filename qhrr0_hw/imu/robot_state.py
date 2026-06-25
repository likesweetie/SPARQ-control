
from dataclasses import dataclass
from typing import Optional
from hal.hardware.can.imu.state import IMUState

@dataclass
class RobotPoseState(IMUState):
    projected_gravity_b: tuple[float, float, float] | None = None

    last_quat_t: float = 0.0
    last_gyro_t: float = 0.0