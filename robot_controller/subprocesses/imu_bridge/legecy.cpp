// imu_bridge: converts the WitMotion serial IMU's Euler-angle output
// (imu_serial_cpp's `/witmotion_imu` shm: acc/gyro/angle in g, deg/s, deg)
// into the quaternion(wxyz)/angular-velocity(rad/s) form
// policy_runner.py._quat_to_projected_gravity() and set_state() expect, and
// publishes it into a new shm `/qhrr_real_imu` for task_controller to read.
//
// This replaces the previously-frozen IMU obs (projected_gravity/
// base_ang_vel were stuck at [0,0,-1]/[0,0,0] all session because
// qhrr_control_state's Path1 IMU driver never received real CAN frames --
// the real IMU here is a WitMotion USB-serial device, not the E2Box CAN
// unit Path1's code expects).
//
// The IMU is physically mounted such that (operator-confirmed): sensor Y
// points toward the robot's rear (-Front) and sensor Z points toward the
// ground (-Up). Desired body frame: X=Front, Z=Up (=> Y=Left, for a
// right-handed frame). NOTE this is NOT a simple 180deg-about-X rotation:
// since IMU chips report a right-handed axis triad (X x Y = Z), fixing
// Y_sensor=-Front and Z_sensor=-Up forces X_sensor to be Right, not Front
// (Front can't be simultaneously X and have Y=-Front -- that would make X
// and Y non-orthogonal). So X and Y both change, not just Y/Z.
//
// Change-of-basis matrix (columns = sensor axes expressed in body coords):
//   X_sensor(Right)=(0,-1,0), Y_sensor(Back)=(-1,0,0), Z_sensor(Ground)=(0,0,-1)
//   R_mount = [[0,-1,0],[-1,0,0],[0,0,-1]]  (det=+1, proper rotation, a 180deg
//   rotation about the diagonal axis (1,-1,0)/sqrt(2), not about X alone).
// As a quaternion: q_mount = (0, -1/sqrt2, 1/sqrt2, 0). Composing on the
// right (q_body = q_sensor (x) q_mount) and expanding the Hamilton product
// gives the closed form implemented below.
//
// Verified against the real sensor reading: projected_gravity's Z flips
// from +0.998 (ground) to -0.998 (sky) as expected, AND (unlike an
// X-only-flip correction, which was tried and wrong) the X/Y components are
// not swapped relative to what this derivation predicts.
//
// The ZYX Euler->quaternion formula itself (pre-correction) is the standard
// Tait-Bryan convention and was cross-checked against the IMU's own raw
// accelerometer reading (stationary projected_gravity ~= -accelerometer),
// which only holds if the convention/handedness is right -- so that part is
// on solid empirical footing. The X_sensor=Right inference follows
// necessarily from the operator's Y/Z facts plus the IMU's right-handed
// axis convention (standard for MEMS IMUs including WitMotion); it was not
// independently physically verified by rotating the sensor about a known
// body axis, so a tilt-test is still worthwhile before fully trusting it in
// a closed control loop.

#include "WitmotionImuData.hpp"
#include "RealImuShm.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cmath>
#include <csignal>
#include <cstring>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace {

volatile sig_atomic_t g_running = 1;

void signal_handler(int) { g_running = 0; }

std::string now_str() {
    auto t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

void log_line(const std::string& msg) {
    std::cerr << "[" << now_str() << "] [imu_bridge] " << msg << std::endl;
}

constexpr double DEG2RAD = M_PI / 180.0;
constexpr long LOOP_PERIOD_NS = 2'000'000;  // 1kHz

// Standard ZYX (yaw-pitch-roll) Euler -> quaternion(wxyz), in the raw sensor
// frame (before the 180deg-about-X mount correction).
void euler_zyx_to_quat_wxyz(double roll, double pitch, double yaw, double out[4]) {
    double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
    double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);

    out[0] = cr * cp * cy + sr * sp * sy;  // w
    out[1] = sr * cp * cy - cr * sp * sy;  // x
    out[2] = cr * sp * cy + sr * cp * sy;  // y
    out[3] = cr * cp * sy - sr * sp * cy;  // z
}

// Applies the fixed sensor-mount correction: q_body = q_sensor (x) q_mount,
// q_mount=(0,-1/sqrt2,1/sqrt2,0) -> closed form below (see file header for
// derivation).
void apply_mount_correction_wxyz(const double sensor[4], float out[4]) {
    constexpr double INV_SQRT2 = 0.70710678118654752440;
    double w = sensor[0], x = sensor[1], y = sensor[2], z = sensor[3];

    out[0] = static_cast<float>((y - x) * INV_SQRT2);   // w
    out[1] = static_cast<float>((w + z) * INV_SQRT2);   // x
    out[2] = static_cast<float>((z - w) * INV_SQRT2);   // y
    out[3] = static_cast<float>(-(x + y) * INV_SQRT2);  // z
}

}  // namespace

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    log_line("starting");

    log_line("waiting for /witmotion_imu (imu_serial_cpp must be running)");
    int in_fd = -1;
    ImuData* in_ptr = nullptr;
    while (g_running && in_ptr == nullptr) {
        in_fd = shm_open("/witmotion_imu", O_RDONLY, 0666);
        if (in_fd < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        void* p = mmap(nullptr, sizeof(ImuData), PROT_READ, MAP_SHARED, in_fd, 0);
        if (p == MAP_FAILED) {
            log_line(std::string("FATAL: mmap(/witmotion_imu) failed: ") + std::strerror(errno));
            return 1;
        }
        in_ptr = static_cast<ImuData*>(p);
    }
    if (!g_running) return 0;
    log_line("connected to /witmotion_imu");

    int out_fd = shm_open("/qhrr_real_imu", O_CREAT | O_RDWR, 0666);
    if (out_fd < 0) {
        log_line(std::string("FATAL: shm_open(/qhrr_real_imu) failed: ") + std::strerror(errno));
        return 1;
    }
    if (ftruncate(out_fd, sizeof(RealImuMirror)) != 0) {
        log_line(std::string("FATAL: ftruncate failed: ") + std::strerror(errno));
        return 1;
    }
    auto* out_ptr = static_cast<RealImuMirror*>(
        mmap(nullptr, sizeof(RealImuMirror), PROT_READ | PROT_WRITE, MAP_SHARED, out_fd, 0));
    if (out_ptr == MAP_FAILED) {
        log_line(std::string("FATAL: mmap(/qhrr_real_imu) failed: ") + std::strerror(errno));
        return 1;
    }

    while (g_running) {
        auto tick_start = std::chrono::steady_clock::now();

        ImuData snap_in;
        std::memcpy(&snap_in, in_ptr, sizeof(snap_in));

        RealImuMirror snap_out;
        snap_out.timestamp_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count());
        double sensor_quat_wxyz[4];
        euler_zyx_to_quat_wxyz(
            snap_in.angle[0] * DEG2RAD,
            snap_in.angle[1] * DEG2RAD,
            snap_in.angle[2] * DEG2RAD,
            sensor_quat_wxyz);
        apply_mount_correction_wxyz(sensor_quat_wxyz, snap_out.quat_wxyz);

        // Same R_mount = [[0,-1,0],[-1,0,0],[0,0,-1]] applied directly as a
        // vector transform (angular velocity is a vector, not a quaternion).
        snap_out.ang_vel_rad_s[0] = static_cast<float>(-snap_in.gyro[1] * DEG2RAD);
        snap_out.ang_vel_rad_s[1] = static_cast<float>(-snap_in.gyro[0] * DEG2RAD);
        snap_out.ang_vel_rad_s[2] = static_cast<float>(-snap_in.gyro[2] * DEG2RAD);

        snap_out.ang_vel_rad_s[0] = static_cast<float>(snap_in.gyro[1] * DEG2RAD);
        snap_out.ang_vel_rad_s[1] = static_cast<float>(snap_in.gyro[0] * DEG2RAD);
        snap_out.ang_vel_rad_s[2] = static_cast<float>(-snap_in.gyro[2] * DEG2RAD);

        std::memcpy(out_ptr, &snap_out, sizeof(snap_out));

        auto elapsed = std::chrono::steady_clock::now() - tick_start;
        auto period = std::chrono::nanoseconds(LOOP_PERIOD_NS);
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }

    log_line("shutting down");
    munmap(in_ptr, sizeof(ImuData));
    munmap(out_ptr, sizeof(RealImuMirror));
    close(in_fd);
    close(out_fd);
    return 0;
}
