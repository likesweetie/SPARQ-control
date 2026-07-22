#ifndef FEEDBACK_BRIDGE_CONFIG_H
#define FEEDBACK_BRIDGE_CONFIG_H

#include "SPARQ_config.h"

#include <cstdint>

// Reverse of policy_bridge's can_id_to_shm_index: SHM_MOTOR_INDEX_* -> CAN ID,
// so published feedback is tagged by CAN ID (self-documenting, matches
// platform.yaml's numbering) rather than a positional index that differs
// between Path1 (FL,FR,RL,RR) and Path2 (LF,LR,RF,RR) grouping order.
inline uint32_t shm_index_to_can_id(int shm_index) {
    switch (shm_index) {
        case SHM_MOTOR_INDEX_LEFT_FRONT_HIP_ROLL:    return CAN_ID_LEFT_FRONT_HIP_ROLL;
        case SHM_MOTOR_INDEX_LEFT_FRONT_HIP_PITCH:   return CAN_ID_LEFT_FRONT_HIP_PITCH;
        case SHM_MOTOR_INDEX_LEFT_FRONT_KNEE_PITCH:  return CAN_ID_LEFT_FRONT_KNEE_PITCH;
        case SHM_MOTOR_INDEX_LEFT_REAR_HIP_ROLL:     return CAN_ID_LEFT_REAR_HIP_ROLL;
        case SHM_MOTOR_INDEX_LEFT_REAR_HIP_PITCH:    return CAN_ID_LEFT_REAR_HIP_PITCH;
        case SHM_MOTOR_INDEX_LEFT_REAR_KNEE_PITCH:   return CAN_ID_LEFT_REAR_KNEE_PITCH;
        case SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_ROLL:   return CAN_ID_RIGHT_FRONT_HIP_ROLL;
        case SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_PITCH:  return CAN_ID_RIGHT_FRONT_HIP_PITCH;
        case SHM_MOTOR_INDEX_RIGHT_FRONT_KNEE_PITCH: return CAN_ID_RIGHT_FRONT_KNEE_PITCH;
        case SHM_MOTOR_INDEX_RIGHT_REAR_HIP_ROLL:    return CAN_ID_RIGHT_REAR_HIP_ROLL;
        case SHM_MOTOR_INDEX_RIGHT_REAR_HIP_PITCH:   return CAN_ID_RIGHT_REAR_HIP_PITCH;
        case SHM_MOTOR_INDEX_RIGHT_REAR_KNEE_PITCH:  return CAN_ID_RIGHT_REAR_KNEE_PITCH;
        default:                                     return 0;  // unreachable for shm_index in [0,11]
    }
}

constexpr int FEEDBACK_BRIDGE_MOTOR_NUM = 12;
constexpr long FEEDBACK_BRIDGE_LOOP_PERIOD_NS = 2'000'000;  // 1kHz, matches sparq_can's own cadence

#endif  // FEEDBACK_BRIDGE_CONFIG_H
