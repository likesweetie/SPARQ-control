#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <thread>

#include <csignal>

#include "../shm.hpp"

struct ImuData {
    double acc[3]   = {0, 0, 0}; // g
    double gyro[3]  = {0, 0, 0}; // deg/s
    double angle[3] = {0, 0, 0}; // deg (roll, pitch, yaw)
};

static std::atomic<bool> g_running{true};

static void sig_handler(int) { g_running.store(false); }

static void render(const ImuData& d) {
    std::printf("\033[H\033[J");
    std::printf("===== WitMotion SHM 리더 (Ctrl+C 종료) =====\n\n");
    std::printf("가속도  (g)     X:%+8.3f  Y:%+8.3f  Z:%+8.3f\n",
                d.acc[0], d.acc[1], d.acc[2]);
    std::printf("각속도  (deg/s) X:%+8.2f  Y:%+8.2f  Z:%+8.2f\n",
                d.gyro[0], d.gyro[1], d.gyro[2]);
    std::printf("각도    (deg)   Roll:%+8.2f  Pitch:%+8.2f  Yaw:%+8.2f\n",
                d.angle[0], d.angle[1], d.angle[2]);
    std::fflush(stdout);
}

int main() {
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    // 생산자가 아직 안 떴을 수 있으므로 연결될 때까지 재시도
    SharedMemory<ImuData>* shm = nullptr;
    while (g_running.load() && shm == nullptr) {
        try {
            shm = new SharedMemory<ImuData>("/witmotion_imu", false);
        } catch (...) {
            std::printf("SHM 대기 중... (imu_serial 프로세스가 먼저 실행되어야 합니다)\r");
            std::fflush(stdout);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    if (!shm) return 0;

    std::printf("\033[2J");

    auto last_render = std::chrono::steady_clock::now();

    while (g_running.load()) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_render).count() >= 33) {
            render(shm->read());
            last_render = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::printf("\n종료합니다.\n");
    delete shm;
    return 0;
}
