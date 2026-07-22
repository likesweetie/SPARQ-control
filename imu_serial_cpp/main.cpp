// LORD / MicroStrain 3DM-GX5-25 실시간 IMU 리더
//
// 수신:
//   - Filter Attitude Quaternion: w, x, y, z
//   - Filter Compensated Angular Rate: x, y, z [rad/s]
//
// 출력 공유메모리:
//   /qhrr_real_imu
//
// 사용법:
//   gx5_imu
//   gx5_imu /dev/ttyACM0
//   gx5_imu /dev/ttyACM0 115200
//   gx5_imu /dev/ttyACM0 115200 100
//
// 인자:
//   1: serial port
//   2: baud rate
//   3: filter output rate [Hz]
//
// 종료:
//   Ctrl+C

#include <microstrain/connections/serial/serial_connection.hpp>
#include <mip/mip_all.hpp>

#include "shm.hpp"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <type_traits>
#include <vector>
#include <cmath>

#include <dirent.h>
#include <time.h>

namespace {

// signal handler에서는 이 값만 변경합니다.
volatile sig_atomic_t g_running = 1;

void signal_handler(int)
{
    g_running = 0;
}

/*
 * task_controller가 읽을 최종 IMU 공유메모리 형식입니다.
 *
 * 기존 imu_bridge가 만들던 /qhrr_real_imu 형식과 같은 용도입니다.
 * 이 구조체 정의는 consumer 쪽에서도 정확히 같아야 합니다.
 */
struct ImuData {
    // CLOCK_MONOTONIC 기준 실제 GX5 샘플 수신 시각
    std::uint64_t timestamp_ns = 0;

    // Quaternion: w, x, y, z
    float quat_wxyz[4] = {
        1.0F, 0.0F, 0.0F, 0.0F
    };

    // Vehicle frame angular velocity: x, y, z [rad/s]
    float ang_vel_rad_s[3] = {
        0.0F, 0.0F, 0.0F
    };
};

static_assert(std::is_standard_layout_v<ImuData>);
static_assert(std::is_trivially_copyable_v<ImuData>);

std::uint64_t monotonic_now_ns()
{
    timespec ts{};

    if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0;
    }

    return static_cast<std::uint64_t>(ts.tv_sec) *
               1'000'000'000ULL +
           static_cast<std::uint64_t>(ts.tv_nsec);
}

/*
 * /dev/ttyACM*을 먼저 찾고, 없으면 /dev/ttyUSB*을 찾습니다.
 *
 * 3DM-GX5-25의 USB 연결은 일반적으로 ttyACM으로 나타납니다.
 * 실기에서는 /dev/serial/by-id/... 경로를 직접 주는 편이 더 안전합니다.
 */
std::vector<std::string> enumerate_ports()
{
    std::vector<std::string> acm_ports;
    std::vector<std::string> usb_ports;

    DIR* dir = ::opendir("/dev");
    if (dir == nullptr) {
        return {};
    }

    while (dirent* entry = ::readdir(dir)) {
        const std::string name(entry->d_name);

        if (name.rfind("ttyACM", 0) == 0) {
            acm_ports.emplace_back("/dev/" + name);
        } else if (name.rfind("ttyUSB", 0) == 0) {
            usb_ports.emplace_back("/dev/" + name);
        }
    }

    ::closedir(dir);

    std::sort(acm_ports.begin(), acm_ports.end());
    std::sort(usb_ports.begin(), usb_ports.end());

    acm_ports.insert(
        acm_ports.end(),
        usb_ports.begin(),
        usb_ports.end());

    return acm_ports;
}

std::string find_port()
{
    const auto ports = enumerate_ports();

    if (ports.empty()) {
        return {};
    }

    return ports.front();
}

bool command_ok(
    const mip::CmdResult& result,
    const char* operation)
{
    if (result.isAck()) {
        return true;
    }

    std::cerr
        << operation
        << " 실패: result="
        << result.name()
        << " ("
        << static_cast<int>(result.value)
        << ")\n";

    return false;
}

/*
 * Filter stream에 포함할 필드:
 *
 * 0x82, 0x11 : Filter Timestamp
 * 0x82, 0x10 : Filter Status
 * 0x82, 0x03 : Attitude Quaternion
 * 0x82, 0x0E : Compensated Angular Rate
 */
bool configure_filter_stream(
    mip::Interface& device,
    std::uint16_t requested_rate_hz)
{
    std::uint16_t filter_base_rate = 0;

    if (!command_ok(
            mip::commands_3dm::filterGetBaseRate(
                device,
                &filter_base_rate),
            "filterGetBaseRate")) {
        return false;
    }

    if (filter_base_rate == 0) {
        std::cerr << "Filter base rate가 0 Hz입니다.\n";
        return false;
    }

    if (requested_rate_hz == 0 ||
        requested_rate_hz > filter_base_rate) {

        std::cerr
            << "잘못된 출력 주기: "
            << requested_rate_hz
            << " Hz, 유효 범위: 1~"
            << filter_base_rate
            << " Hz\n";

        return false;
    }

    /*
     * 실제 출력률:
     *
     *     actual_rate = base_rate / decimation
     */
    const std::uint16_t decimation =
        std::max<std::uint16_t>(
            1,
            static_cast<std::uint16_t>(
                filter_base_rate / requested_rate_hz));

    const double actual_rate_hz =
        static_cast<double>(filter_base_rate) /
        static_cast<double>(decimation);

    const mip::DescriptorRate descriptors[] = {
        {
            mip::data_filter::Timestamp::FIELD_DESCRIPTOR,
            decimation
        },
        {
            mip::data_filter::Status::FIELD_DESCRIPTOR,
            decimation
        },
        {
            mip::data_filter::AttitudeQuaternion::FIELD_DESCRIPTOR,
            decimation
        },
        {
            mip::data_filter::CompAngularRate::FIELD_DESCRIPTOR,
            decimation
        }
    };


        const mip::DescriptorRate sensor_descriptors[] = {
        {
            mip::data_sensor::CompQuaternion::FIELD_DESCRIPTOR,
            1  // base rate 그대로
        },
        {
            mip::data_sensor::ScaledGyro::FIELD_DESCRIPTOR,
            1
        }
    };

    mip::commands_3dm::writeImuMessageFormat(
        device,
        static_cast<std::uint8_t>(
            std::size(sensor_descriptors)),
        sensor_descriptors);


    const auto result =
        mip::commands_3dm::writeFilterMessageFormat(
            device,
            static_cast<std::uint8_t>(
                sizeof(descriptors) / sizeof(descriptors[0])),
            descriptors);

    if (!command_ok(result, "writeFilterMessageFormat")) {
        return false;
    }

    std::cout
        << "Filter base rate : "
        << filter_base_rate
        << " Hz\n"
        << "Decimation      : "
        << decimation
        << "\n"
        << "Actual rate     : "
        << actual_rate_hz
        << " Hz\n";

    return true;
}

bool initialize_device(
    mip::Interface& device,
    std::uint16_t sample_rate_hz)
{
    // 통신 확인
    if (!command_ok(
            mip::commands_base::ping(device),
            "ping")) {
        return false;
    }

    /*
     * 설정 도중 기존 streaming traffic을 정지시킵니다.
     */
    if (!command_ok(
            mip::commands_base::setIdle(device),
            "setIdle")) {
        return false;
    }

    if (!configure_filter_stream(
            device,
            sample_rate_hz)) {
        return false;
    }

    /*
     * 전원 인가 후 filter가 자동으로 초기화되도록 합니다.
     */
    if (!command_ok(
            mip::commands_filter::writeAutoInitControl(
                device,
                1),
            "writeAutoInitControl")) {
        return false;
    }

    /*
     * 변경된 filter 설정을 적용합니다.
     */
    if (!command_ok(
            mip::commands_filter::reset(device),
            "filter reset")) {
        return false;
    }

    return true;
}
// Quaternion(wxyz)으로 world gravity [0, 0, -1]을 body frame에 투영합니다.
// 정상 직립 자세에서는 projected_gravity = [0, 0, -1]입니다.
static void quat_to_projected_gravity(
    const float quat_wxyz[4],
    float projected_gravity[3])
{
    double w = static_cast<double>(quat_wxyz[0]);
    double x = static_cast<double>(quat_wxyz[1]);
    double y = static_cast<double>(quat_wxyz[2]);
    double z = static_cast<double>(quat_wxyz[3]);

    // 수치 오차에 대비하여 정규화
    const double norm =
        std::sqrt(w * w + x * x + y * y + z * z);

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

    // q^-1 * [0, 0, -1] * q
    projected_gravity[0] =
        static_cast<float>(2.0 * (w * y - x * z));

    projected_gravity[1] =
        static_cast<float>(-2.0 * (w * x + y * z));

    projected_gravity[2] =
        static_cast<float>(2.0 * (x * x + y * y) - 1.0);
}
void render(
    const ImuData& data,
    const mip::data_filter::Status& status,
    std::uint64_t sample_count,
    const std::string& port,
    std::uint32_t baud,
    double actual_age_ms)
{
    float projected_gravity[3];

    quat_to_projected_gravity(
        data.quat_wxyz,
        projected_gravity);

    const double gravity_norm =
        std::sqrt(
            static_cast<double>(projected_gravity[0]) *
                projected_gravity[0] +
            static_cast<double>(projected_gravity[1]) *
                projected_gravity[1] +
            static_cast<double>(projected_gravity[2]) *
                projected_gravity[2]);

    std::printf("\033[H\033[J");

    std::printf(
        "===== LORD 3DM-GX5-25 IMU 실시간 =====\n");

    std::printf(
        "port=%s  baud=%u\n",
        port.c_str(),
        baud);

    std::printf(
        "filter_state=%d  samples=%llu  age=%.3f ms\n\n",
        static_cast<int>(status.filter_state),
        static_cast<unsigned long long>(sample_count),
        actual_age_ms);

    std::printf(
        "Quaternion (wxyz)\n"
        "  w:%+10.6f  x:%+10.6f  "
        "y:%+10.6f  z:%+10.6f\n\n",
        data.quat_wxyz[0],
        data.quat_wxyz[1],
        data.quat_wxyz[2],
        data.quat_wxyz[3]);

    std::printf(
        "Angular velocity (rad/s)\n"
        "  x:%+10.6f  y:%+10.6f  z:%+10.6f\n\n",
        data.ang_vel_rad_s[0],
        data.ang_vel_rad_s[1],
        data.ang_vel_rad_s[2]);

    std::printf(
        "Projected gravity (body frame)\n"
        "  x:%+10.6f  y:%+10.6f  z:%+10.6f"
        "  norm:%8.6f\n",
        projected_gravity[0],
        projected_gravity[1],
        projected_gravity[2],
        gravity_norm);

    std::fflush(stdout);
}

}  // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::string port;
    std::uint32_t baud = 115200;
    std::uint16_t sample_rate_hz = 500;

    if (argc > 1) {
        port = argv[1];
    }

    if (argc > 2) {
        baud = static_cast<std::uint32_t>(
            std::strtoul(argv[2], nullptr, 10));
    }

    if (argc > 3) {
        sample_rate_hz = static_cast<std::uint16_t>(
            std::strtoul(argv[3], nullptr, 10));
    }

    if (port.empty()) {
        port = find_port();

        if (port.empty()) {
            std::cerr
                << "시리얼 포트를 찾지 못했습니다.\n"
                << "예:\n"
                << "  gx5_imu /dev/ttyACM0 115200 100\n";

            return EXIT_FAILURE;
        }
    }

    std::cout
        << "LORD 3DM-GX5-25 연결 시도: "
        << port
        << " @ "
        << baud
        << " bps\n";

    /*
     * USB 연결도 운영체제에서는 가상 serial port로 보입니다.
     */
    microstrain::connections::SerialConnection connection(
        port.c_str(),
        baud);

    if (!connection.connect()) {
        std::cerr
            << "장치 연결 실패: "
            << port
            << "\n"
            << "권한 확인:\n"
            << "  sudo usermod -aG dialout $USER\n"
            << "이후 로그아웃/로그인이 필요합니다.\n";

        return EXIT_FAILURE;
    }

    /*
     * MIP device interface.
     *
     * 두 번째 인자:
     *   baud rate에 따라 계산한 command timeout
     *
     * 세 번째 인자:
     *   command reply timeout [ms]
     */
    mip::Interface device(
        &connection,
        mip::C::mip_timeout_from_baudrate(baud),
        2000);

    if (!initialize_device(
            device,
            sample_rate_hz)) {

        connection.disconnect();
        return EXIT_FAILURE;
    }

    /*
     * MIP SDK가 여기에 최신 field를 deserialize합니다.
     */
    mip::data_filter::Timestamp filter_timestamp{};

    // mip::data_filter::AttitudeQuaternion filter_quaternion{};
    // mip::data_filter::CompAngularRate filter_angular_rate{};

    mip::data_filter::Status filter_status{};
    mip::data_sensor::CompQuaternion filter_quaternion{};
    mip::data_sensor::ScaledGyro filter_angular_rate{};

    mip::DispatchHandler handlers[2];

    device.registerExtractor(
        handlers[0],
        &filter_timestamp);

    device.registerExtractor(
        handlers[1],
        &filter_status);
    
    mip::DispatchHandler sensor_handlers[2];

    device.registerExtractor(
        sensor_handlers[0],
        &filter_quaternion);

    device.registerExtractor(
        sensor_handlers[1],
        &filter_angular_rate);
        

    /*
     * 장치를 idle 상태에서 다시 streaming 상태로 전환합니다.
     */
    if (!command_ok(
            mip::commands_base::resume(device),
            "resume")) {

        connection.disconnect();
        return EXIT_FAILURE;
    }

    /*
     * 이 프로그램이 최종 quaternion/rad/s 형식을 직접 게시하므로
     * 기존 imu_bridge는 동시에 실행하지 않습니다.
     */
    SharedMemory<ImuData> shm(
        "/qhrr_real_imu",
        true);

    ImuData output{};

    bool have_timestamp = false;
    double last_tow = 0.0;
    std::uint16_t last_week = 0;

    std::uint64_t sample_count = 0;

    auto last_render =
        std::chrono::steady_clock::now();

    std::printf("\033[2J");

    while (g_running) {
        /*
         * 최대 10 ms 동안 입력을 기다립니다.
         *
         * MIP 데이터가 들어오면 즉시 파싱되고 extractor가 갱신됩니다.
         * 10 ms가 매번 강제로 추가되는 것은 아닙니다.
         */
        device.update(10);

        /*
         * Device timestamp가 변한 경우에만 새 센서 샘플로 판단합니다.
         * 이전 값을 반복해서 새 timestamp로 게시하지 않습니다.
         */
        const bool timestamp_changed =
            !have_timestamp ||
            filter_timestamp.week_number != last_week ||
            filter_timestamp.tow != last_tow;

        if (timestamp_changed) {
            have_timestamp = true;
            last_week = filter_timestamp.week_number;
            last_tow = filter_timestamp.tow;

            const bool filter_valid =
                filter_status.filter_state ==
                mip::data_filter::FilterMode::
                    GX5_RUN_SOLUTION_VALID;


                output.timestamp_ns =
                    monotonic_now_ns();

                //NED→NWU conversion  q_nwu_from_ned = [0, 1, 0, 0]  // wxyz
                output.quat_wxyz[0] = -filter_quaternion.q[1];
                output.quat_wxyz[1] =  filter_quaternion.q[0];
                output.quat_wxyz[2] = -filter_quaternion.q[3];
                output.quat_wxyz[3] =  filter_quaternion.q[2];
                // output.quat_wxyz[0] =
                //     filter_quaternion.q[0];
                
                // output.quat_wxyz[1] =
                //     filter_quaternion.q[1];

                // output.quat_wxyz[2] =
                //     filter_quaternion.q[2];

                // output.quat_wxyz[3] =
                //     filter_quaternion.q[3];

                output.ang_vel_rad_s[0] =
                    filter_angular_rate.scaled_gyro[0];

                output.ang_vel_rad_s[1] =
                    filter_angular_rate.scaled_gyro[1];

                output.ang_vel_rad_s[2] =
                    filter_angular_rate.scaled_gyro[2];

                shm.write(output);
                ++sample_count;
        }

        const auto now =
            std::chrono::steady_clock::now();

        if (now - last_render >=
            std::chrono::milliseconds(100)) {

            const std::uint64_t now_ns =
                monotonic_now_ns();

            double age_ms = 0.0;

            if (output.timestamp_ns != 0 &&
                now_ns >= output.timestamp_ns) {

                age_ms =
                    static_cast<double>(
                        now_ns - output.timestamp_ns) *
                    1.0e-6;
            }

            render(
                output,
                filter_status,
                sample_count,
                port,
                baud,
                age_ms);

            last_render = now;
        }
    }

    std::cout << "\n종료 중...\n";

    /*
     * 장치 streaming 정지.
     * 종료 과정이므로 실패해도 프로세스는 계속 종료합니다.
     */
    mip::commands_base::setIdle(device);

    connection.disconnect();

    return EXIT_SUCCESS;
}