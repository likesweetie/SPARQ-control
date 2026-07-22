#ifndef QHRR_SHM_MIRROR_HPP
#define QHRR_SHM_MIRROR_HPP

#include <cstddef>
#include <cstdint>

// Packed mirrors of the Python ctypes structs that task_controller/aux_reader
// publish over POSIX shared memory. Layout must track:
//   robot_controller/shm/control_command.py (ControlCommandC, ControlTargetC)
//   robot_controller/shm/aux_command.py     (AuxCommandC)
// Both Python structs use `_pack_ = 1`; if either changes, the static_asserts
// below will fail the build and must be updated together with this file.

#pragma pack(push, 1)

struct ControlTargetMirror {
    uint32_t can_id;
    float q;
    float dq;
    float kp;
    float kd;
    float tau;
};

struct ControlCommandMirror {
    uint64_t timestamp_ns;
    uint32_t num_targets;
    ControlTargetMirror targets[12];
};

struct AuxCommandMirror {
    uint64_t timestamp_ns;
    float lin_vel_target[3];
    float ang_vel_target[3];
    uint32_t button_mask;
};

#pragma pack(pop)

// Sizes/offsets confirmed against the live Python structs with
// ctypes.sizeof(...) / ControlCommandC.targets.offset before writing this file.
static_assert(sizeof(ControlTargetMirror) == 24, "ControlTargetC layout drift");
static_assert(sizeof(ControlCommandMirror) == 300, "ControlCommandC layout drift");
static_assert(offsetof(ControlCommandMirror, targets) == 12, "ControlCommandC.targets offset drift");
static_assert(sizeof(AuxCommandMirror) == 36, "AuxCommandC layout drift");

// BUTTON_FIELDS order in robot_controller/shm/aux_command.py:
// a,b,x,y,lb,rb,back,start,guide,l3,r3 (bit index = position in this list).
constexpr uint32_t AUX_BUTTON_BIT_RB = 1u << 5;

#endif  // QHRR_SHM_MIRROR_HPP
