// policy_bridge: forwards task_controller's ONNX-policy output (published in
// the POSIX shm `qhrr_mit_command`) into the real Robstride motor control
// path (`sparq_can`'s SysV shm `Control_Shm<12>`, key 13563267).
//
// This is the first real writer of that shm's `ctrl` side -- previously
// nothing in the repo called write_ctrl(), so Kp/Kd always read 0.0 and the
// real robot never received policy commands. Safety gates here are the ONLY
// software protection on this path: there is no --hardware CLI gate anywhere
// else in this repo (see FALLBACK_POLICY.md and README's aspirational-but-
// unimplemented hardware mode flags).
//
// Fail-safe default: unless *all* of (a) --enable-motor-output was passed at
// launch, (b) the aux joystick's RB button is held every single cycle, (c)
// qhrr_mit_command is fresh, (d) qhrr_aux_command is fresh, and (e) the
// command batch is a complete, valid 12-motor set keyed by CAN ID -- this
// process writes Kp=Kd=0 every cycle. No silent fallback: every armed/
// disarmed transition and every rejected tick is logged.

#include "Sharemem.hpp"
#include "SPARQ_config.h"
#include "PolicyBridgeConfig.h"
#include "QhrrShmMirror.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <csignal>
#include <cstring>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
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
    std::cerr << "[" << now_str() << "] [policy_bridge] " << msg << std::endl;
}

std::string to_hex(uint32_t v) {
    std::ostringstream oss;
    oss << "0x" << std::hex << v;
    return oss.str();
}

template <typename T>
T* map_posix_shm_readonly(const char* name, int& fd_out) {
    int fd = shm_open(name, O_RDONLY, 0666);
    if (fd < 0) {
        log_line(std::string("FATAL: shm_open(") + name + ") failed: " + std::strerror(errno));
        std::exit(1);
    }
    void* p = mmap(nullptr, sizeof(T), PROT_READ, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) {
        log_line(std::string("FATAL: mmap(") + name + ") failed: " + std::strerror(errno));
        std::exit(1);
    }
    fd_out = fd;
    return static_cast<T*>(p);
}

// Validates a ControlCommandMirror snapshot and fills shm_index -> target
// mapping. Returns empty string if valid, else a human-readable reason.
std::string validate_batch(const ControlCommandMirror& snap, int (&shm_index_of_slot)[12]) {
    if (snap.num_targets != POLICY_BRIDGE_MOTOR_NUM) {
        return "num_targets=" + std::to_string(snap.num_targets) + " != " + std::to_string(POLICY_BRIDGE_MOTOR_NUM);
    }
    bool seen[POLICY_BRIDGE_MOTOR_NUM] = {false};
    for (int i = 0; i < POLICY_BRIDGE_MOTOR_NUM; i++) {
        int idx = can_id_to_shm_index(snap.targets[i].can_id);
        if (idx < 0) {
            return "unknown can_id=" + to_hex(snap.targets[i].can_id) + " at slot " + std::to_string(i);
        }
        if (seen[idx]) {
            return "duplicate can_id mapping to shm index " + std::to_string(idx);
        }
        seen[idx] = true;
        shm_index_of_slot[i] = idx;
    }
    for (int idx = 0; idx < POLICY_BRIDGE_MOTOR_NUM; idx++) {
        if (!seen[idx]) {
            return "missing required motor at shm index " + std::to_string(idx);
        }
    }
    return "";
}

}  // namespace

int main(int argc, char** argv) {
    bool enable_motor_output = false;
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--enable-motor-output") == 0) {
            enable_motor_output = true;
        }
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    log_line(std::string("starting, enable_motor_output=") + (enable_motor_output ? "true" : "false"));
    log_line("NOTE: Path2 (sparq_can update_Control_params) only reads ctrl_buf[...].pos/Kp/Kd -- "
              "vel/ffTorque written by this bridge are currently inert downstream.");

    int mit_fd = -1;
    int aux_fd = -1;
    auto* mit_ptr = map_posix_shm_readonly<ControlCommandMirror>("/qhrr_mit_command", mit_fd);
    auto* aux_ptr = map_posix_shm_readonly<AuxCommandMirror>("/qhrr_aux_command", aux_fd);

    Control_Shm<POLICY_BRIDGE_MOTOR_NUM> ctrl_shm(13563267);
    auto* shm_ptr = ctrl_shm.get();

    uint64_t last_mit_ts = 0;
    uint64_t last_aux_ts = 0;
    auto last_mit_change = std::chrono::steady_clock::now();
    auto last_aux_change = std::chrono::steady_clock::now();
    bool first_tick = true;

    std::string prev_reason;
    bool clamped_prev[POLICY_BRIDGE_MOTOR_NUM] = {false};

    while (g_running) {
        auto tick_start = std::chrono::steady_clock::now();

        ControlCommandMirror mit_snap;
        std::memcpy(&mit_snap, mit_ptr, sizeof(mit_snap));
        AuxCommandMirror aux_snap;
        std::memcpy(&aux_snap, aux_ptr, sizeof(aux_snap));

        auto now = std::chrono::steady_clock::now();

        if (first_tick || mit_snap.timestamp_ns != last_mit_ts) {
            last_mit_ts = mit_snap.timestamp_ns;
            last_mit_change = now;
        }
        if (first_tick || aux_snap.timestamp_ns != last_aux_ts) {
            last_aux_ts = aux_snap.timestamp_ns;
            last_aux_change = now;
        }
        first_tick = false;

        bool mit_fresh = std::chrono::duration<double>(now - last_mit_change).count() < POLICY_BRIDGE_MIT_STALE_TIMEOUT_S;
        bool aux_fresh = std::chrono::duration<double>(now - last_aux_change).count() < POLICY_BRIDGE_AUX_STALE_TIMEOUT_S;
        bool rb_held = (aux_snap.button_mask & AUX_BUTTON_BIT_RB) != 0;

        int shm_index_of_slot[POLICY_BRIDGE_MOTOR_NUM];
        std::string invalid_reason = validate_batch(mit_snap, shm_index_of_slot);
        bool batch_valid = invalid_reason.empty();

        bool armed = enable_motor_output && rb_held && mit_fresh && aux_fresh && batch_valid;

        std::string reason;
        if (!enable_motor_output) reason = "no --enable-motor-output flag";
        else if (!batch_valid) reason = "invalid batch: " + invalid_reason;
        else if (!mit_fresh) reason = "qhrr_mit_command stale";
        else if (!aux_fresh) reason = "qhrr_aux_command stale";
        else if (!rb_held) reason = "rb_button not held";
        else reason = "armed";

        if (reason != prev_reason) {
            log_line(std::string(armed ? "ARMED" : "DISARMED") + " reason=" + reason);
            prev_reason = reason;
        }

        Control_param ctrl_buf[POLICY_BRIDGE_MOTOR_NUM];

        if (armed) {
            for (int i = 0; i < POLICY_BRIDGE_MOTOR_NUM; i++) {
                const auto& t = mit_snap.targets[i];
                int idx = shm_index_of_slot[i];

                double kp = std::max(0.0, std::min(POLICY_BRIDGE_KP_MAX, static_cast<double>(t.kp)));
                double kd = std::max(0.0, std::min(POLICY_BRIDGE_KD_MAX, static_cast<double>(t.kd)));
                bool clamped = (kp != static_cast<double>(t.kp)) || (kd != static_cast<double>(t.kd));
                if (clamped != clamped_prev[idx]) {
                    log_line("clamp " + std::string(clamped ? "start" : "end") + " shm_index=" +
                             std::to_string(idx) + " raw_kp=" + std::to_string(t.kp) +
                             " raw_kd=" + std::to_string(t.kd) + " clamped_kp=" + std::to_string(kp) +
                             " clamped_kd=" + std::to_string(kd));
                    clamped_prev[idx] = clamped;
                }

                ctrl_buf[idx].pos = t.q;
                ctrl_buf[idx].vel = t.dq;
                ctrl_buf[idx].Kp = kp;
                ctrl_buf[idx].Kd = kd;
                ctrl_buf[idx].ffTorque = t.tau;
            }
        } else {
            Feedback_Param fb_buf[POLICY_BRIDGE_MOTOR_NUM];
            bool have_fb = false;
            for (int attempt = 0; attempt < 3 && !have_fb; attempt++) {
                have_fb = shm_ptr->try_read_fb(fb_buf);
            }
            for (int idx = 0; idx < POLICY_BRIDGE_MOTOR_NUM; idx++) {
                ctrl_buf[idx].pos = have_fb ? fb_buf[idx].pos : 0.0;
                ctrl_buf[idx].vel = 0.0;
                ctrl_buf[idx].Kp = 0.0;
                ctrl_buf[idx].Kd = 0.0;
                ctrl_buf[idx].ffTorque = 0.0;
                clamped_prev[idx] = false;
            }
        }

        shm_ptr->write_ctrl(ctrl_buf);

        auto elapsed = std::chrono::steady_clock::now() - tick_start;
        auto period = std::chrono::nanoseconds(POLICY_BRIDGE_LOOP_PERIOD_NS);
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }

    log_line("shutting down, writing final zero-gain ctrl");
    Control_param zero_buf[POLICY_BRIDGE_MOTOR_NUM];
    for (int idx = 0; idx < POLICY_BRIDGE_MOTOR_NUM; idx++) {
        zero_buf[idx].pos = 0.0;
        zero_buf[idx].vel = 0.0;
        zero_buf[idx].Kp = 0.0;
        zero_buf[idx].Kd = 0.0;
        zero_buf[idx].ffTorque = 0.0;
    }
    shm_ptr->write_ctrl(zero_buf);

    munmap(mit_ptr, sizeof(ControlCommandMirror));
    munmap(aux_ptr, sizeof(AuxCommandMirror));
    close(mit_fd);
    close(aux_fd);
    return 0;
}
