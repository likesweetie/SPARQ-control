#ifndef POLICY_BRIDGE_CONFIG_H
#define POLICY_BRIDGE_CONFIG_H

#include "SPARQ_config.h"

#include <cstdint>

// Path1 (platform.yaml actuators) and Path2 (SHM_MOTOR_INDEX_* in
// SPARQ_config.h) use the same CAN ID numbering scheme but group legs in a
// different order (Path1: FL,FR,RL,RR / Path2: LF,LR,RF,RR). Mapping must
// always go through the CAN ID, never a positional array index, or the
// LEFT_REAR/RIGHT_FRONT joints get silently transposed.
inline int can_id_to_shm_index(uint32_t can_id) {
    switch (can_id) {
        case CAN_ID_LEFT_FRONT_HIP_ROLL:    return SHM_MOTOR_INDEX_LEFT_FRONT_HIP_ROLL;
        case CAN_ID_LEFT_FRONT_HIP_PITCH:   return SHM_MOTOR_INDEX_LEFT_FRONT_HIP_PITCH;
        case CAN_ID_LEFT_FRONT_KNEE_PITCH:  return SHM_MOTOR_INDEX_LEFT_FRONT_KNEE_PITCH;
        case CAN_ID_LEFT_REAR_HIP_ROLL:     return SHM_MOTOR_INDEX_LEFT_REAR_HIP_ROLL;
        case CAN_ID_LEFT_REAR_HIP_PITCH:    return SHM_MOTOR_INDEX_LEFT_REAR_HIP_PITCH;
        case CAN_ID_LEFT_REAR_KNEE_PITCH:   return SHM_MOTOR_INDEX_LEFT_REAR_KNEE_PITCH;
        case CAN_ID_RIGHT_FRONT_HIP_ROLL:   return SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_ROLL;
        case CAN_ID_RIGHT_FRONT_HIP_PITCH:  return SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_PITCH;
        case CAN_ID_RIGHT_FRONT_KNEE_PITCH: return SHM_MOTOR_INDEX_RIGHT_FRONT_KNEE_PITCH;
        case CAN_ID_RIGHT_REAR_HIP_ROLL:    return SHM_MOTOR_INDEX_RIGHT_REAR_HIP_ROLL;
        case CAN_ID_RIGHT_REAR_HIP_PITCH:   return SHM_MOTOR_INDEX_RIGHT_REAR_HIP_PITCH;
        case CAN_ID_RIGHT_REAR_KNEE_PITCH:  return SHM_MOTOR_INDEX_RIGHT_REAR_KNEE_PITCH;
        default:                            return -1;  // unknown CAN ID: caller must reject the tick, never guess
    }
}

constexpr int POLICY_BRIDGE_MOTOR_NUM = 12;

// RS02 gain ranges (RobstrideMotor.hpp MotorConstants<RS02>). The downstream
// write_updated_operation_frame() clamps silently to the same range; the
// bridge clamps+logs here so an out-of-range gain is visible where it enters
// the system, not two hops later.
constexpr double POLICY_BRIDGE_KP_MAX = 500.0;
constexpr double POLICY_BRIDGE_KD_MAX = 5.0;

// task_controller publishes qhrr_mit_command at TASK_CONTROL_HZ=50.0
// (processes.yaml), i.e. a fresh timestamp at least every 20ms. 100ms is 5x
// that period: generous enough for scheduling jitter, tight enough to catch
// a dead/stuck producer quickly. Deliberately not reusing
// robot_controller.yaml's safety.damping_timeout_s=1.0 -- that governs a
// different failure mode (CAN feedback going stale on Path1), not this
// bridge's producer-liveness check.
constexpr double POLICY_BRIDGE_MIT_STALE_TIMEOUT_S = 0.1;
constexpr double POLICY_BRIDGE_AUX_STALE_TIMEOUT_S = 0.1;

constexpr long POLICY_BRIDGE_LOOP_PERIOD_NS = 2'000'000;  // 1kHz, matches sparq_can's own control cadence

#endif  // POLICY_BRIDGE_CONFIG_H
