// imu_bridge: converts the WitMotion serial IMU's Euler-angle output
// into quaternion(wxyz)/angular-velocity(rad/s), applies the sensor mount
// correction, and publishes the result to /qhrr_real_imu.

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

void signal_handler(int)
{
    g_running = 0;
}

std::string now_str()
{
    const auto t = std::time(nullptr);

    std::tm tm{};
    localtime_r(&t, &tm);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");

    return oss.str();
}

void log_line(const std::string& msg)
{
    std::cerr
        << "[" << now_str() << "] "
        << "[imu_bridge] "
        << msg
        << std::endl;
}

constexpr double DEG2RAD = M_PI / 180.0;

// 2 ms = 500 Hz
constexpr long LOOP_PERIOD_NS = 2'000'000;

// 약 24 Hz
constexpr long PRINT_PERIOD_NS = 41'666'667;


// Standard ZYX (yaw-pitch-roll) Euler -> quaternion(wxyz),
// in the raw sensor frame.
void euler_zyx_to_quat_wxyz(
    double roll,
    double pitch,
    double yaw,
    double out[4])
{
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);

    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);

    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    out[0] = cr * cp * cy + sr * sp * sy;  // w
    out[1] = sr * cp * cy - cr * sp * sy;  // x
    out[2] = cr * sp * cy + sr * cp * sy;  // y
    out[3] = cr * cp * sy - sr * sp * cy;  // z
}


// q_body = q_sensor × q_mount
//
// q_mount = (0, -1/sqrt(2), 1/sqrt(2), 0)
void apply_mount_correction_wxyz(
    const double sensor[4],
    float out[4])
{
    constexpr double INV_SQRT2 =
        0.70710678118654752440;

    const double w = sensor[0];
    const double x = sensor[1];
    const double y = sensor[2];
    const double z = sensor[3];

    out[0] =
        static_cast<float>((y - x) * INV_SQRT2);

    out[1] =
        static_cast<float>((w + z) * INV_SQRT2);

    out[2] =
        static_cast<float>((z - w) * INV_SQRT2);

    out[3] =
        static_cast<float>(-(x + y) * INV_SQRT2);
}


// 변환 완료된 body-frame quaternion으로부터
// world gravity [0, 0, -1]을 body frame에 투영합니다.
//
// 정상 직립 자세에서는:
//
//     projected_gravity ≈ [0, 0, -1]
//
void quat_to_projected_gravity(
    const float quat_wxyz[4],
    float projected_gravity[3])
{
    double w = static_cast<double>(quat_wxyz[0]);
    double x = static_cast<double>(quat_wxyz[1]);
    double y = static_cast<double>(quat_wxyz[2]);
    double z = static_cast<double>(quat_wxyz[3]);

    const double norm =
        std::sqrt(
            w * w +
            x * x +
            y * y +
            z * z);

    if (norm < 1.0e-12) {
        projected_gravity[0] = 0.0F;
        projected_gravity[1] = 0.0F;
        projected_gravity[2] = -1.0F;
        return;
    }

    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;

    // q^-1 × [0, 0, -1] × q
    projected_gravity[0] =
        static_cast<float>(
            2.0 * (w * y - x * z));

    projected_gravity[1] =
        static_cast<float>(
            -2.0 * (w * x + y * z));

    projected_gravity[2] =
        static_cast<float>(
            2.0 * (x * x + y * y) - 1.0);
}


void print_imu_status(const RealImuMirror& imu)
{
    float projected_gravity[3];

    quat_to_projected_gravity(
        imu.quat_wxyz,
        projected_gravity);

    // \r       : 현재 줄의 처음으로 이동
    // \033[2K  : 현재 줄 전체 지우기
    std::cout
        << "\r\033[2K"
        << std::fixed
        << std::setprecision(5)

        << "ang_vel [rad/s] "
        << "x=" << std::setw(9) << imu.ang_vel_rad_s[0]
        << "  y=" << std::setw(9) << imu.ang_vel_rad_s[1]
        << "  z=" << std::setw(9) << imu.ang_vel_rad_s[2]

        << "    |    "

        << "gravity "
        << "x=" << std::setw(9) << projected_gravity[0]
        << "  y=" << std::setw(9) << projected_gravity[1]
        << "  z=" << std::setw(9) << projected_gravity[2]

        << std::flush;
}

}  // namespace


int main()
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    log_line("starting");

    log_line(
        "waiting for /witmotion_imu "
        "(imu_serial_cpp must be running)");

    int in_fd = -1;
    ImuData* in_ptr = nullptr;

    while (g_running && in_ptr == nullptr) {
        in_fd = shm_open(
            "/witmotion_imu",
            O_RDONLY,
            0666);

        if (in_fd < 0) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(200));

            continue;
        }

        void* p = mmap(
            nullptr,
            sizeof(ImuData),
            PROT_READ,
            MAP_SHARED,
            in_fd,
            0);

        if (p == MAP_FAILED) {
            log_line(
                std::string(
                    "FATAL: mmap(/witmotion_imu) failed: ") +
                std::strerror(errno));

            return 1;
        }

        in_ptr = static_cast<ImuData*>(p);
    }

    if (!g_running) {
        return 0;
    }

    log_line("connected to /witmotion_imu");

    int out_fd = shm_open(
        "/qhrr_real_imu",
        O_CREAT | O_RDWR,
        0666);

    if (out_fd < 0) {
        log_line(
            std::string(
                "FATAL: shm_open(/qhrr_real_imu) failed: ") +
            std::strerror(errno));

        return 1;
    }

    if (ftruncate(
            out_fd,
            sizeof(RealImuMirror)) != 0) {

        log_line(
            std::string("FATAL: ftruncate failed: ") +
            std::strerror(errno));

        return 1;
    }

    auto* out_ptr =
        static_cast<RealImuMirror*>(
            mmap(
                nullptr,
                sizeof(RealImuMirror),
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                out_fd,
                0));

    if (out_ptr == MAP_FAILED) {
        log_line(
            std::string(
                "FATAL: mmap(/qhrr_real_imu) failed: ") +
            std::strerror(errno));

        return 1;
    }

    const auto loop_period =
        std::chrono::nanoseconds(
            LOOP_PERIOD_NS);

    const auto print_period =
        std::chrono::nanoseconds(
            PRINT_PERIOD_NS);

    auto last_print =
        std::chrono::steady_clock::now();

    while (g_running) {
        const auto tick_start =
            std::chrono::steady_clock::now();

        ImuData snap_in{};

        std::memcpy(
            &snap_in,
            in_ptr,
            sizeof(snap_in));

        RealImuMirror snap_out{};

        snap_out.timestamp_ns =
            static_cast<uint64_t>(
                std::chrono::duration_cast<
                    std::chrono::nanoseconds>(
                    std::chrono::system_clock::now()
                        .time_since_epoch())
                    .count());

        double sensor_quat_wxyz[4];

        euler_zyx_to_quat_wxyz(
            snap_in.angle[0] * DEG2RAD,
            snap_in.angle[1] * DEG2RAD,
            snap_in.angle[2] * DEG2RAD,
            sensor_quat_wxyz);

        // 장착 방향을 적용한 body-frame quaternion
        apply_mount_correction_wxyz(
            sensor_quat_wxyz,
            snap_out.quat_wxyz);

        // R_mount =
        // [[ 0, -1,  0],
        //  [-1,  0,  0],
        //  [ 0,  0, -1]]
        snap_out.ang_vel_rad_s[0] =
            static_cast<float>(
                -snap_in.gyro[1] * DEG2RAD);

        snap_out.ang_vel_rad_s[1] =
            static_cast<float>(
                -snap_in.gyro[0] * DEG2RAD);

        snap_out.ang_vel_rad_s[2] =
            static_cast<float>(
                -snap_in.gyro[2] * DEG2RAD);

        std::memcpy(
            out_ptr,
            &snap_out,
            sizeof(snap_out));

        const auto now =
            std::chrono::steady_clock::now();

        // 변환 완료된 body-frame quaternion 기준으로
        // projected gravity를 약 24 Hz로 출력합니다.
        if (now - last_print >= print_period) {
            print_imu_status(snap_out);

            last_print += print_period;

            if (now - last_print >= print_period) {
                last_print = now;
            }
        }

        const auto elapsed =
            std::chrono::steady_clock::now() -
            tick_start;

        if (elapsed < loop_period) {
            std::this_thread::sleep_for(
                loop_period - elapsed);
        }
    }

    std::cout << '\n';

    log_line("shutting down");

    munmap(
        in_ptr,
        sizeof(ImuData));

    munmap(
        out_ptr,
        sizeof(RealImuMirror));

    close(in_fd);
    close(out_fd);

    return 0;
}
