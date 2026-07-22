#ifndef REAL_IMU_SHM_HPP
#define REAL_IMU_SHM_HPP

#include <cstddef>
#include <cstdint>

// New POSIX shm segment `/qhrr_real_imu`, published by imu_bridge from the
// WitMotion serial IMU (via imu_serial_cpp's /witmotion_imu), converted to
// the quat(wxyz)/angular-velocity(rad/s) form policy_runner.py expects.
// Authored fresh here -- robot_controller/shm/real_imu.py is written to
// match this layout field-for-field; update both together if it changes.

#pragma pack(push, 1)

struct RealImuMirror {
    uint64_t timestamp_ns;
    float quat_wxyz[4];
    float ang_vel_rad_s[3];
};

#pragma pack(pop)

static_assert(sizeof(RealImuMirror) == 8 + 16 + 12, "RealImuMirror layout");
static_assert(offsetof(RealImuMirror, quat_wxyz) == 8, "RealImuMirror.quat_wxyz offset");
static_assert(offsetof(RealImuMirror, ang_vel_rad_s) == 24, "RealImuMirror.ang_vel_rad_s offset");

#endif  // REAL_IMU_SHM_HPP
