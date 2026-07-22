#ifndef REAL_FEEDBACK_SHM_HPP
#define REAL_FEEDBACK_SHM_HPP

#include <cstddef>
#include <cstdint>

// New POSIX shm segment `/qhrr_real_feedback`, published by feedback_bridge
// from sparq_can's live Path2 (Robstride) motor feedback (Control_Shm::fb),
// keyed by CAN ID. This struct is authored fresh here -- there is no
// pre-existing Python struct to mirror, so the Python side
// (robot_controller/shm/real_feedback.py) is written to match this layout
// field-for-field. If either side changes, update both together.

#pragma pack(push, 1)

struct RealFeedbackTarget {
    uint32_t can_id;
    float pos;
    float vel;
    float torque;
    float temp;
};

struct RealFeedbackMirror {
    uint64_t timestamp_ns;
    uint32_t num_motors;
    RealFeedbackTarget motors[12];
};

#pragma pack(pop)

static_assert(sizeof(RealFeedbackTarget) == 20, "RealFeedbackTarget layout");
static_assert(sizeof(RealFeedbackMirror) == 12 + 20 * 12, "RealFeedbackMirror layout");
static_assert(offsetof(RealFeedbackMirror, motors) == 12, "RealFeedbackMirror.motors offset");

#endif  // REAL_FEEDBACK_SHM_HPP
