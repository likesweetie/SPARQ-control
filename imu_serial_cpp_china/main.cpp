// WitMotion Serial IMU 실시간 리더 (C++ / Linux POSIX Serial)
//
// USB 시리얼(CH340/CP2102)로 연결된 WitMotion IMU 센서의 데이터를
// BLE 프로토콜(0x55 0x61, 20바이트) 형식으로 수신하여
// 가속도/각속도/각도를 터미널에서 실시간 갱신.
//
// 사용법:
//   imu_serial                       : /dev/ttyUSB* 자동 탐색 후 연결
//   imu_serial /dev/ttyUSB0          : 포트 직접 지정
//   imu_serial /dev/ttyUSB0 115200   : 포트 + 보드레이트 지정
//
// 권한 오류 시: sudo usermod -aG dialout $USER  (재로그인 필요)
// 종료: Ctrl + C

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <csignal>
#include <dirent.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>

#include "shm.hpp"

// ---- 패킷 타입 ----
static constexpr uint8_t HDR       = 0x55;
static constexpr uint8_t TYPE_BLE  = 0x61;  // BLE 올인원 (20바이트)
static constexpr uint8_t TYPE_ACC  = 0x51;  // 표준 가속도 (11바이트)
static constexpr uint8_t TYPE_GYRO = 0x52;  // 표준 각속도
static constexpr uint8_t TYPE_ANG  = 0x53;  // 표준 각도

struct ImuData {
    double acc[3]   = {0, 0, 0}; // g
    double gyro[3]  = {0, 0, 0}; // deg/s
    double angle[3] = {0, 0, 0}; // deg (roll, pitch, yaw)
};

static ImuData g_data;
static std::atomic<bool> g_running{true};
static const char* g_proto = "감지 중...";

// 리틀엔디언 2바이트 -> 부호있는 16비트 정수
static int16_t s16(uint8_t lo, uint8_t hi) {
    return static_cast<int16_t>(static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8));
}

// BLE 프로토콜 패킷 파싱 (0x55 0x61, 20바이트)
static bool parse_ble(const uint8_t* p) {
    g_data.acc[0]   = s16(p[2],  p[3])  / 32768.0 * 16.0;
    g_data.acc[1]   = s16(p[4],  p[5])  / 32768.0 * 16.0;
    g_data.acc[2]   = s16(p[6],  p[7])  / 32768.0 * 16.0;
    g_data.gyro[0]  = s16(p[8],  p[9])  / 32768.0 * 2000.0;
    g_data.gyro[1]  = s16(p[10], p[11]) / 32768.0 * 2000.0;
    g_data.gyro[2]  = s16(p[12], p[13]) / 32768.0 * 2000.0;
    g_data.angle[0] = s16(p[14], p[15]) / 32768.0 * 180.0;
    g_data.angle[1] = s16(p[16], p[17]) / 32768.0 * 180.0;
    g_data.angle[2] = s16(p[18], p[19]) / 32768.0 * 180.0;
    g_proto = "BLE (0x55 0x61, 20B)";
    return true;
}

// 표준 Serial 프로토콜 패킷 파싱 (0x55 + 타입, 11바이트)
static bool parse_serial(const uint8_t* p) {
    uint8_t cksum = 0;
    for (int i = 0; i < 10; ++i) cksum += p[i];
    if (cksum != p[10]) return false;

    int16_t x = s16(p[2], p[3]);
    int16_t y = s16(p[4], p[5]);
    int16_t z = s16(p[6], p[7]);

    switch (p[1]) {
        case TYPE_ACC:
            g_data.acc[0] = x / 32768.0 * 16.0;
            g_data.acc[1] = y / 32768.0 * 16.0;
            g_data.acc[2] = z / 32768.0 * 16.0;
            break;
        case TYPE_GYRO:
            g_data.gyro[0] = x / 32768.0 * 2000.0;
            g_data.gyro[1] = y / 32768.0 * 2000.0;
            g_data.gyro[2] = z / 32768.0 * 2000.0;
            break;
        case TYPE_ANG:
            g_data.angle[0] = x / 32768.0 * 180.0;
            g_data.angle[1] = y / 32768.0 * 180.0;
            g_data.angle[2] = z / 32768.0 * 180.0;
            break;
        default:
            return false;
    }
    g_proto = "Serial (0x55 0x5x, 11B)";
    return true;
}

static constexpr double DEG2RAD = M_PI / 180.0;

// WitMotion ZYX Euler angle을 기준으로
// world gravity [0, 0, -1]을 sensor frame으로 투영합니다.
//
// 출력:
//   sensor가 수평이면 approximately [0, 0, -1]
static void euler_to_projected_gravity(
    const double angle_deg[3],
    double projected_gravity[3])
{
    const double roll  = angle_deg[0] * DEG2RAD;
    const double pitch = angle_deg[1] * DEG2RAD;

    const double sin_roll  = std::sin(roll);
    const double cos_roll  = std::cos(roll);
    const double sin_pitch = std::sin(pitch);
    const double cos_pitch = std::cos(pitch);

    // R_body_to_world^T * [0, 0, -1]
    projected_gravity[0] = sin_pitch;
    projected_gravity[1] = -sin_roll * cos_pitch;
    projected_gravity[2] = -cos_roll * cos_pitch;
}

static void render()
{
    double projected_gravity[3];

    euler_to_projected_gravity(
        g_data.angle,
        projected_gravity);

    const double gravity_norm =
        std::sqrt(
            projected_gravity[0] * projected_gravity[0] +
            projected_gravity[1] * projected_gravity[1] +
            projected_gravity[2] * projected_gravity[2]);

    std::printf("\033[H\033[J");

    std::printf(
        "===== WitMotion Serial IMU 실시간 (Ctrl+C 종료) =====\n");

    std::printf(
        "프로토콜: %s\n\n",
        g_proto);

    std::printf(
        "가속도  (g)     "
        "X:%+8.3f  Y:%+8.3f  Z:%+8.3f\n",
        g_data.acc[0],
        g_data.acc[1],
        g_data.acc[2]);

    std::printf(
        "각속도  (deg/s) "
        "X:%+8.2f  Y:%+8.2f  Z:%+8.2f\n",
        g_data.gyro[0],
        g_data.gyro[1],
        g_data.gyro[2]);

    std::printf(
        "각도    (deg)   "
        "Roll:%+8.2f  Pitch:%+8.2f  Yaw:%+8.2f\n\n",
        g_data.angle[0],
        g_data.angle[1],
        g_data.angle[2]);

    std::printf(
        "Projected gravity (sensor frame)\n"
        "  X:%+10.6f  Y:%+10.6f  Z:%+10.6f  norm:%8.6f\n",
        projected_gravity[0],
        projected_gravity[1],
        projected_gravity[2],
        gravity_norm);

    std::fflush(stdout);
}

// /dev/ttyUSB*, /dev/ttyACM* 포트 목록 반환 (번호 순 정렬)
static std::vector<std::string> enumerate_ports() {
    std::vector<std::string> ports;
    DIR* dir = opendir("/dev");
    if (!dir) return ports;
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string name(entry->d_name);
        if (name.find("ttyUSB") == 0 || name.find("ttyACM") == 0) {
            ports.push_back("/dev/" + name);
        }
    }
    closedir(dir);
    std::sort(ports.begin(), ports.end());
    return ports;
}

static std::string find_port() {
    auto ports = enumerate_ports();
    return ports.empty() ? "" : ports[0];
}

static speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B115200;
    }
}

static int open_serial(const std::string& port, int baud) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    // 블로킹 모드로 전환
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return -1;
    }

    speed_t spd = baud_to_speed(baud);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    // raw 모드 (8N1), 흐름 제어 없음
    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;   // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;  // 하드웨어 흐름 제어 비활성

    // read 타임아웃: 100ms (VTIME 단위 = 100ms)
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

static void sig_handler(int) {
    g_running.store(false);
}

int main(int argc, char** argv) {
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    std::string port;
    int baud = 115200;

    if (argc > 1) port = argv[1];
    if (argc > 2) baud = std::atoi(argv[2]);

    if (port.empty()) {
        port = find_port();
        if (port.empty()) {
            std::cout << "시리얼 포트를 찾지 못했어요. 장치를 연결했는지 확인하거나 포트를 직접 지정하세요.\n";
            std::cout << "예: imu_serial /dev/ttyUSB0\n";
            return 1;
        }
    }

    std::printf("포트 %s @ %d bps 로 연결 시도...\n", port.c_str(), baud);
    int fd = open_serial(port, baud);
    if (fd < 0) {
        std::perror("포트 열기 실패");
        std::cout << "권한 오류라면: sudo usermod -aG dialout $USER 후 재로그인\n";
        return 1;
    }

    SharedMemory<ImuData> shm("/witmotion_imu", true);

    std::printf("\033[2J");

    std::vector<uint8_t> buf;
    buf.reserve(4096);
    uint8_t tmp[256];
    auto last_render = std::chrono::steady_clock::now();

    while (g_running.load()) {
        ssize_t n = read(fd, tmp, sizeof(tmp));
        if (n > 0) {
            buf.insert(buf.end(), tmp, tmp + n);
        } else if (n < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // 버퍼에서 패킷 파싱
        while (buf.size() >= 11) {
            if (buf[0] != HDR) {
                buf.erase(buf.begin());
                continue;
            }

            if (buf[1] == TYPE_BLE) {
                if (buf.size() < 20) break;
                parse_ble(buf.data());
                buf.erase(buf.begin(), buf.begin() + 20);
            } else if (buf[1] == TYPE_ACC || buf[1] == TYPE_GYRO ||
                       buf[1] == TYPE_ANG) {
                parse_serial(buf.data());
                buf.erase(buf.begin(), buf.begin() + 11);
            } else {
                buf.erase(buf.begin());
            }
        }

        shm.write(g_data);

        if (buf.size() > 2048) {
            buf.erase(buf.begin(), buf.end() - 1024);
        }

        // ~20Hz로 렌더링 제한
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_render).count() >= 100) {
            render();
            last_render = now;
        }
    }

    std::printf("\n종료합니다.\n");
    close(fd);
    return 0;
}
