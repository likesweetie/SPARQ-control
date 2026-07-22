// feedback_bridge: publishes sparq_can's live Path2 (Robstride) motor
// feedback (Control_Shm::fb) into a new POSIX shm `/qhrr_real_feedback`,
// keyed by CAN ID, at a steady 1kHz heartbeat cadence (timestamp always
// refreshed every tick, regardless of whether values changed -- learned
// from the aux_reader bug where event-only publishing let a staleness
// check false-trigger while a button was held steady).
//
// This is the missing counterpart discovered while debugging real-robot
// walking: task_controller reads dof_pos/dof_vel from Path1's
// qhrr_control_state, which only RobotController's own CAN daemon writes
// (Path1, vcan0, no real device attached) -- so those obs components were
// frozen at zero for the entire session. This process makes Path2's real
// feedback available under a new shm name; task_controller is patched
// separately to read from it.

#include "Sharemem.hpp"
#include "SPARQ_config.h"
#include "FeedbackBridgeConfig.h"
#include "RealFeedbackShm.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
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
    std::cerr << "[" << now_str() << "] [feedback_bridge] " << msg << std::endl;
}

}  // namespace

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    log_line("starting");

    // This process is the natural owner/creator of this brand-new segment --
    // unlike qhrr_mit_command (already created by RobotController's
    // ShmManager), nothing else creates /qhrr_real_feedback.
    int fd = shm_open("/qhrr_real_feedback", O_CREAT | O_RDWR, 0666);
    if (fd < 0) {
        log_line(std::string("FATAL: shm_open(/qhrr_real_feedback) failed: ") + std::strerror(errno));
        return 1;
    }
    if (ftruncate(fd, sizeof(RealFeedbackMirror)) != 0) {
        log_line(std::string("FATAL: ftruncate failed: ") + std::strerror(errno));
        return 1;
    }
    auto* out_ptr = static_cast<RealFeedbackMirror*>(
        mmap(nullptr, sizeof(RealFeedbackMirror), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
    if (out_ptr == MAP_FAILED) {
        log_line(std::string("FATAL: mmap failed: ") + std::strerror(errno));
        return 1;
    }

    Control_Shm<FEEDBACK_BRIDGE_MOTOR_NUM> ctrl_shm(13563267);
    auto* shm_ptr = ctrl_shm.get();

    while (g_running) {
        auto tick_start = std::chrono::steady_clock::now();

        Feedback_Param fb_buf[FEEDBACK_BRIDGE_MOTOR_NUM];
        bool have_fb = false;
        for (int attempt = 0; attempt < 3 && !have_fb; attempt++) {
            have_fb = shm_ptr->try_read_fb(fb_buf);
        }

        RealFeedbackMirror snap;
        snap.timestamp_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count());
        snap.num_motors = FEEDBACK_BRIDGE_MOTOR_NUM;
        for (int idx = 0; idx < FEEDBACK_BRIDGE_MOTOR_NUM; idx++) {
            snap.motors[idx].can_id = shm_index_to_can_id(idx);
            snap.motors[idx].pos = have_fb ? fb_buf[idx].pos : 0.0f;
            snap.motors[idx].vel = have_fb ? fb_buf[idx].vel : 0.0f;
            snap.motors[idx].torque = have_fb ? fb_buf[idx].torque : 0.0f;
            snap.motors[idx].temp = have_fb ? fb_buf[idx].temp : 0.0f;
        }
        // Always publish, even on a torn/failed fb read (with last-known-zero
        // defaults) -- the timestamp still advances every tick so this is a
        // true heartbeat; a consumer-side staleness check on timestamp_ns
        // catches this process dying, not a single missed fb read.
        std::memcpy(out_ptr, &snap, sizeof(snap));

        auto elapsed = std::chrono::steady_clock::now() - tick_start;
        auto period = std::chrono::nanoseconds(FEEDBACK_BRIDGE_LOOP_PERIOD_NS);
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }

    log_line("shutting down");
    munmap(out_ptr, sizeof(RealFeedbackMirror));
    close(fd);
    return 0;
}
