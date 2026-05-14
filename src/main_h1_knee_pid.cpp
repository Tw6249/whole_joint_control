#include "runtime_config.hpp"
#include "safety.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using LowCmdMsg = unitree_go::msg::dds_::LowCmd_;
using LowStateMsg = unitree_go::msg::dds_::LowState_;

namespace {

constexpr const char* kTopicLowCmd = "rt/lowcmd";
constexpr const char* kTopicLowState = "rt/lowstate";
constexpr float kPosStopF = 2.146E9f;
constexpr float kVelStopF = 16000.0f;

// Bring-up limits. Hard angle limits from YAML are still enforced every cycle.
constexpr double kSoftLimitMargin = 0.03;       // rad inside configured joint limits
constexpr double kMaxTargetStep = 1.50;         // rad from measured startup position
constexpr double kMaxMeasuredSpeed = 2.0;       // rad/s, trips if exceeded
constexpr double kMaxMeasuredJump = 0.10;       // rad between control samples
constexpr double kMaxControlDt = 0.010;         // s, trips on scheduler jitter/overrun
constexpr double kIntegralLimit = 1.0;          // rad s
constexpr double kMaxIntegralTorque = 4.0;      // N m contribution from I term
constexpr double kEmergencySeconds = 1.0;       // keep publishing hold after a trip
constexpr double kDefaultRefSpeed = 0.10;       // rad/s
constexpr double kDefaultKp = 16.0;             // N m / rad
constexpr double kDefaultKi = 0.0;              // disabled for first bring-up
constexpr double kDefaultKd = 2.0;              // N m s / rad
constexpr double kDefaultTauLimit = 6.0;        // N m
constexpr double kMaxRefSpeedLimit = 0.80;      // rad/s hard CLI cap
constexpr double kMaxKpLimit = 40.0;            // N m / rad hard CLI cap
constexpr double kMaxKiLimit = 5.0;             // N m / (rad s) hard CLI cap
constexpr double kMaxKdLimit = 5.0;             // N m s / rad hard CLI cap
constexpr double kMaxTauLimit = 18.0;           // N m hard CLI cap
constexpr double kMaxSineAmplitude = 0.80;      // rad hard CLI cap
constexpr double kMaxSineFrequency = 0.35;      // Hz hard CLI cap
constexpr double kTwoPi = 6.283185307179586;

std::atomic<bool> g_running{true};

std::uint64_t nowNs() {
    using namespace std::chrono;
    return static_cast<std::uint64_t>(
        duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count());
}

double nowSec() {
    return static_cast<double>(nowNs()) * 1e-9;
}

void signalHandler(int) {
    g_running.store(false, std::memory_order_release);
}

double clampd(double x, double lo, double hi) {
    return std::max(lo, std::min(x, hi));
}

std::uint32_t crc32Core(std::uint32_t* ptr, std::uint32_t len) {
    std::uint32_t xbit = 0;
    std::uint32_t data = 0;
    std::uint32_t crc = 0xFFFFFFFF;
    constexpr std::uint32_t polynomial = 0x04c11db7;

    for (std::uint32_t i = 0; i < len; ++i) {
        xbit = 1u << 31;
        data = ptr[i];
        for (std::uint32_t bits = 0; bits < 32; ++bits) {
            if (crc & 0x80000000) {
                crc <<= 1;
                crc ^= polynomial;
            } else {
                crc <<= 1;
            }
            if (data & xbit) {
                crc ^= polynomial;
            }
            xbit >>= 1;
        }
    }

    return crc;
}

struct AtomicRobotCache {
    std::array<std::atomic<double>, h1if::kMaxMotors> q;
    std::array<std::atomic<double>, h1if::kMaxMotors> dq;
    std::array<std::atomic<double>, h1if::kMaxMotors> tau_est;
    std::atomic<std::uint64_t> last_state_ns{0};

    AtomicRobotCache() {
        for (int i = 0; i < h1if::kMaxMotors; ++i) {
            q[i].store(0.0);
            dq[i].store(0.0);
            tau_est[i].store(0.0);
        }
    }
};

struct CliOptions {
    std::string config_path;
    double target_q = 0.0;
    double run_seconds = 8.0;
    double kp = kDefaultKp;
    double ki = kDefaultKi;
    double kd = kDefaultKd;
    double tau_limit = kDefaultTauLimit;
    double ref_speed = kDefaultRefSpeed;
    bool sine_mode = false;
    double sine_amplitude = 0.0;
    double sine_frequency = 0.0;
    bool armed = false;
};

class H1KneePidRuntime {
public:
    H1KneePidRuntime(h1if::RuntimeConfig cfg, const CliOptions& opts)
        : cfg_(std::move(cfg)),
          joint_id_(cfg_.controller.target_joint),
          target_q_(opts.target_q),
          run_seconds_(opts.run_seconds),
          kp_(opts.kp),
          ki_(opts.ki),
          kd_(opts.kd),
          tau_limit_(opts.tau_limit),
          ref_speed_(opts.ref_speed),
          sine_mode_(opts.sine_mode),
          sine_amplitude_(opts.sine_amplitude),
          sine_frequency_(opts.sine_frequency) {}

    void init() {
        initRealtimeMemory();

        unitree::robot::ChannelFactory::Instance()->Init(
            cfg_.domain_id,
            cfg_.network_interface);

        initLowCmd();

        lowcmd_pub_.reset(new unitree::robot::ChannelPublisher<LowCmdMsg>(kTopicLowCmd));
        lowcmd_pub_->InitChannel();

        lowstate_sub_.reset(new unitree::robot::ChannelSubscriber<LowStateMsg>(kTopicLowState));
        lowstate_sub_->InitChannel(
            std::bind(&H1KneePidRuntime::onLowState, this, std::placeholders::_1),
            1);

        tryReleaseMotionMode();
        waitForFirstState();

        h1if::RobotState initial = readRobotState(0, cfg_.control_dt);
        validateStartup(initial);

        start_q_ = initial.joint[joint_id_].q;
        ref_q_ = start_q_;
        last_q_ = start_q_;
        last_sample_t_ = initial.t;

        std::cout << std::fixed << std::setprecision(6)
                  << "h1_knee_pid armed\n"
                  << "joint_id=" << joint_id_
                  << " start_q=" << start_q_
                  << " target_q=" << target_q_
                  << " run_seconds=" << run_seconds_
                  << " mode=" << (sine_mode_ ? "sine" : "step") << "\n"
                  << "PID kp=" << kp_ << " ki=" << ki_ << " kd=" << kd_
                  << " tau_limit=" << tau_limit_
                  << " ref_speed_limit=" << ref_speed_;
        if (sine_mode_) {
            std::cout << " sine_center=" << target_q_
                      << " sine_amp=" << sine_amplitude_
                      << " sine_freq=" << sine_frequency_;
        }
        std::cout << "\n";
    }

    int run() {
        setThreadRealtime();

        std::uint64_t cycle = 0;
        const double run_start = nowSec();
        double last_t = run_start;

        while (g_running.load(std::memory_order_acquire)) {
            const auto loop_start = std::chrono::steady_clock::now();
            const double t = nowSec();
            const double dt = t - last_t;
            last_t = t;

            h1if::RobotState state = readRobotState(cycle, dt);
            h1if::RobotCommand command;
            h1if::ControllerDebug debug;

            const bool trip = checkTrip(state, dt);
            if (trip) {
                writeEmergencyHold(state);
                std::cerr << "PID stopped: " << fault_reason_ << "\n";
                publishEmergencyFor(kEmergencySeconds);
                return 3;
            }

            fillPidCommand(state, dt, command, debug);
            h1if::applySafety(state, command, debug, cfg_.safety);

            const std::uint32_t fatal_flags =
                debug.flags & ~static_cast<std::uint32_t>(h1if::kSafetyCommandSaturated);
            if (fatal_flags != 0) {
                fault_reason_ = "safety layer flagged command/state, flags=0x" +
                    std::to_string(debug.flags);
                writeEmergencyHold(state);
                std::cerr << "PID stopped: " << fault_reason_ << "\n";
                publishEmergencyFor(kEmergencySeconds);
                return 4;
            }
            if ((debug.flags & h1if::kSafetyCommandSaturated) != 0 && !warned_saturation_) {
                std::cerr << "Warning: non-fatal command saturation detected, flags=0x"
                          << std::hex << debug.flags << std::dec
                          << ". Continuing because knee state passed hard trip checks.\n";
                warned_saturation_ = true;
            }

            writeLowCmd(command);

            if ((cycle % 100u) == 0u) {
                const auto& j = state.joint[joint_id_];
                std::cout << std::fixed << std::setprecision(6)
                          << "t=" << (t - run_start)
                          << " q=" << j.q
                          << " dq=" << j.dq
                          << " ref=" << ref_q_
                          << " err=" << (ref_q_ - j.q)
                          << " integ=" << integral_
                          << " tau=" << last_tau_
                          << " age=" << state.lowstate_age << "\n";
            }

            if (sine_mode_) {
                if ((t - run_start) >= run_seconds_) {
                    break;
                }
            } else if ((t - run_start) >= run_seconds_ && std::abs(target_q_ - ref_q_) < 1e-3) {
                break;
            }

            ++cycle;
            std::this_thread::sleep_until(
                loop_start + std::chrono::duration<double>(cfg_.control_dt));
        }

        publishEmergencyFor(kEmergencySeconds);
        return 0;
    }

private:
    void initRealtimeMemory() {
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            std::cerr << "Warning: mlockall failed; continuing without page locking.\n";
        }
    }

    void setThreadRealtime() {
        sched_param param{};
        param.sched_priority = 80;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
            std::cerr << "Warning: failed to set SCHED_FIFO; continuing with normal scheduling.\n";
        }
    }

    void initLowCmd() {
        low_cmd_.head()[0] = 0xFE;
        low_cmd_.head()[1] = 0xEF;
        low_cmd_.level_flag() = 0xFF;
        low_cmd_.gpio() = 0;

        for (int i = 0; i < h1if::kMaxMotors; ++i) {
            low_cmd_.motor_cmd()[i].mode() = h1if::h1MotorMode(i);
            low_cmd_.motor_cmd()[i].q() = kPosStopF;
            low_cmd_.motor_cmd()[i].dq() = kVelStopF;
            low_cmd_.motor_cmd()[i].kp() = 0.0f;
            low_cmd_.motor_cmd()[i].kd() = 0.0f;
            low_cmd_.motor_cmd()[i].tau() = 0.0f;
        }
    }

    void onLowState(const void* message) {
        const auto* msg = static_cast<const LowStateMsg*>(message);
        for (int i = 0; i < h1if::kMaxMotors; ++i) {
            cache_.q[i].store(msg->motor_state()[i].q(), std::memory_order_relaxed);
            cache_.dq[i].store(msg->motor_state()[i].dq(), std::memory_order_relaxed);
            cache_.tau_est[i].store(msg->motor_state()[i].tau_est(), std::memory_order_relaxed);
        }
        cache_.last_state_ns.store(nowNs(), std::memory_order_release);
    }

    h1if::RobotState readRobotState(std::uint64_t cycle, double dt) const {
        h1if::RobotState state;
        state.cycle = cycle;
        state.t = nowSec();
        state.dt = dt;

        const std::uint64_t last_ns = cache_.last_state_ns.load(std::memory_order_acquire);
        state.state_valid = last_ns != 0;
        state.lowstate_age = state.state_valid
            ? static_cast<double>(nowNs() - last_ns) * 1e-9
            : 1e9;

        for (int i = 0; i < h1if::kMaxMotors; ++i) {
            state.joint[i].q = cache_.q[i].load(std::memory_order_relaxed);
            state.joint[i].dq = cache_.dq[i].load(std::memory_order_relaxed);
            state.joint[i].tau_est = cache_.tau_est[i].load(std::memory_order_relaxed);
        }

        return state;
    }

    void waitForFirstState() const {
        const auto start = nowNs();
        while (g_running.load(std::memory_order_acquire)) {
            if (cache_.last_state_ns.load(std::memory_order_acquire) != 0) {
                std::cout << "LowState received.\n";
                return;
            }
            if ((nowNs() - start) > static_cast<std::uint64_t>(5e9)) {
                throw std::runtime_error("no LowState received after 5 seconds");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        throw std::runtime_error("interrupted before first LowState");
    }

    void validateStartup(const h1if::RobotState& state) const {
        if (joint_id_ < 0 || joint_id_ >= h1if::kMaxMotors) {
            throw std::runtime_error("target joint is out of range");
        }

        const auto& lim = cfg_.safety.limit[joint_id_];
        const double q = state.joint[joint_id_].q;
        const double soft_min = lim.q_min + kSoftLimitMargin;
        const double soft_max = lim.q_max - kSoftLimitMargin;

        if (!state.state_valid || state.lowstate_age > cfg_.safety.lowstate_timeout) {
            throw std::runtime_error("LowState is missing or stale");
        }
        if (!std::isfinite(q) || !std::isfinite(state.joint[joint_id_].dq)) {
            throw std::runtime_error("initial knee state is non-finite");
        }
        if (q < lim.q_min || q > lim.q_max) {
            throw std::runtime_error("initial knee angle already exceeds configured hard limits");
        }
        if (target_q_ < soft_min || target_q_ > soft_max) {
            throw std::runtime_error("target angle must stay inside soft limits");
        }
        if (std::abs(target_q_ - q) > kMaxTargetStep) {
            throw std::runtime_error("target step is too large for first PID bring-up");
        }
        if (sine_mode_) {
            if (sine_amplitude_ <= 0.0 || sine_amplitude_ > kMaxSineAmplitude ||
                sine_frequency_ <= 0.0 || sine_frequency_ > kMaxSineFrequency) {
                throw std::runtime_error("sine amplitude/frequency outside hard bring-up limits");
            }
            if ((target_q_ - sine_amplitude_) < soft_min ||
                (target_q_ + sine_amplitude_) > soft_max) {
                throw std::runtime_error("full sine trajectory must stay inside soft limits");
            }
            if ((kTwoPi * sine_frequency_ * sine_amplitude_) > ref_speed_) {
                throw std::runtime_error("sine peak speed exceeds configured --speed cap");
            }
        }
    }

    bool checkTrip(const h1if::RobotState& state, double dt) {
        const auto& lim = cfg_.safety.limit[joint_id_];
        const auto& j = state.joint[joint_id_];

        if (!state.state_valid || state.lowstate_age > cfg_.safety.lowstate_timeout) {
            fault_reason_ = "LowState timeout";
            return true;
        }
        if (!std::isfinite(j.q) || !std::isfinite(j.dq) || !std::isfinite(j.tau_est)) {
            fault_reason_ = "non-finite measured knee state";
            return true;
        }
        if (j.q < lim.q_min || j.q > lim.q_max) {
            fault_reason_ = "measured knee angle exceeded configured hard limits";
            return true;
        }
        if (std::abs(j.dq) > kMaxMeasuredSpeed) {
            fault_reason_ = "measured knee speed exceeded jitter/speed limit";
            return true;
        }
        if (dt <= 0.0 || dt > kMaxControlDt) {
            fault_reason_ = "control loop jitter/overrun exceeded limit";
            return true;
        }
        if (last_sample_t_ > 0.0 && std::abs(j.q - last_q_) > kMaxMeasuredJump) {
            fault_reason_ = "measured knee angle jumped too far between samples";
            return true;
        }

        last_q_ = j.q;
        last_sample_t_ = state.t;
        return false;
    }

    void fillPidCommand(const h1if::RobotState& state, double dt, h1if::RobotCommand& command,
                        h1if::ControllerDebug& debug) {
        h1if::fillSafeHoldCommand(state, command, cfg_.safety);

        const auto& j = state.joint[joint_id_];
        double ref_dq = 0.0;
        if (sine_mode_ && sine_started_) {
            const double elapsed = state.t - sine_start_t_;
            const double omega = kTwoPi * sine_frequency_;
            ref_q_ = target_q_ + sine_amplitude_ * std::sin(omega * elapsed);
            ref_dq = sine_amplitude_ * omega * std::cos(omega * elapsed);
        } else {
            const double delta = target_q_ - ref_q_;
            const double max_step = ref_speed_ * dt;
            const double step = clampd(delta, -max_step, max_step);
            ref_q_ += step;
            ref_dq = dt > 0.0 ? step / dt : 0.0;
            if (sine_mode_ && std::abs(target_q_ - ref_q_) < 1e-4) {
                sine_started_ = true;
                sine_start_t_ = state.t;
                ref_q_ = target_q_;
                ref_dq = sine_amplitude_ * kTwoPi * sine_frequency_;
            }
        }

        const double e = ref_q_ - j.q;
        const double edot = ref_dq - j.dq;
        const double p_term = kp_ * e;
        const double d_term = kd_ * edot;
        const double integral_limit = ki_ > 0.0
            ? std::min(kIntegralLimit, kMaxIntegralTorque / ki_)
            : kIntegralLimit;
        const double next_integral = clampd(
            integral_ + e * dt,
            -integral_limit,
            integral_limit);
        const double next_tau = p_term + ki_ * next_integral + d_term;

        const bool tau_saturated_high = next_tau > tau_limit_;
        const bool tau_saturated_low = next_tau < -tau_limit_;
        const bool unwinding =
            (tau_saturated_high && e < 0.0) ||
            (tau_saturated_low && e > 0.0);
        if ((!tau_saturated_high && !tau_saturated_low) || unwinding) {
            integral_ = next_integral;
        }

        const double i_term = ki_ * integral_;
        last_tau_ = clampd(p_term + i_term + d_term, -tau_limit_, tau_limit_);

        auto& c = command.joint[joint_id_];
        c.mode = h1if::h1MotorMode(joint_id_);
        c.q = static_cast<float>(j.q);
        c.dq = 0.0f;
        c.kp = 0.0f;
        c.kd = 0.0f;
        c.tau = static_cast<float>(last_tau_);
        c.enable = true;

        debug.data[0] = ref_q_;
        debug.data[1] = ref_dq;
        debug.data[2] = j.q;
        debug.data[3] = j.dq;
        debug.data[4] = e;
        debug.data[5] = edot;
        debug.data[6] = integral_;
        debug.data[7] = last_tau_;
        debug.data[8] = p_term;
        debug.data[9] = i_term;
        debug.data[10] = d_term;
    }

    void writeLowCmd(const h1if::RobotCommand& command) {
        for (int i = 0; i < h1if::kMaxMotors; ++i) {
            const auto& c = command.joint[i];
            low_cmd_.motor_cmd()[i].mode() = c.mode;
            low_cmd_.motor_cmd()[i].q() = c.q;
            low_cmd_.motor_cmd()[i].dq() = c.dq;
            low_cmd_.motor_cmd()[i].kp() = c.kp;
            low_cmd_.motor_cmd()[i].kd() = c.kd;
            low_cmd_.motor_cmd()[i].tau() = c.tau;
        }

        low_cmd_.crc() = crc32Core(
            reinterpret_cast<std::uint32_t*>(&low_cmd_),
            (sizeof(LowCmdMsg) >> 2) - 1);
        lowcmd_pub_->Write(low_cmd_);
    }

    void writeEmergencyHold(const h1if::RobotState& state) {
        h1if::RobotCommand command;
        h1if::ControllerDebug debug;
        h1if::fillSafeHoldCommand(state, command, cfg_.safety);

        auto& c = command.joint[joint_id_];
        c.q = static_cast<float>(state.joint[joint_id_].q);
        c.dq = 0.0f;
        c.kp = 4.0f;
        c.kd = 2.0f;
        c.tau = 0.0f;

        h1if::applySafety(state, command, debug, cfg_.safety);
        writeLowCmd(command);
    }

    void publishEmergencyFor(double seconds) {
        const double start = nowSec();
        while ((nowSec() - start) < seconds) {
            h1if::RobotState state = readRobotState(0, cfg_.control_dt);
            writeEmergencyHold(state);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    void tryReleaseMotionMode() {
        unitree::robot::b2::MotionSwitcherClient client;
        client.SetTimeout(5.0f);
        client.Init();

        std::string robot_form;
        std::string motion_name;
        const int ret = client.CheckMode(robot_form, motion_name);
        if (ret != 0) {
            std::cerr << "MotionSwitcher CheckMode failed, ret=" << ret << "\n";
            return;
        }
        if (!motion_name.empty()) {
            std::cout << "Active motion service detected: " << motion_name << ". Releasing...\n";
            const int release_ret = client.ReleaseMode();
            if (release_ret != 0) {
                std::cerr << "ReleaseMode failed, ret=" << release_ret << "\n";
            }
        }
    }

    h1if::RuntimeConfig cfg_;
    int joint_id_ = 2;
    double target_q_ = 0.0;
    double run_seconds_ = 8.0;
    double start_q_ = 0.0;
    double ref_q_ = 0.0;
    double integral_ = 0.0;
    double last_tau_ = 0.0;
    double last_q_ = 0.0;
    double last_sample_t_ = 0.0;
    double kp_ = kDefaultKp;
    double ki_ = kDefaultKi;
    double kd_ = kDefaultKd;
    double tau_limit_ = kDefaultTauLimit;
    double ref_speed_ = kDefaultRefSpeed;
    bool sine_mode_ = false;
    bool sine_started_ = false;
    double sine_amplitude_ = 0.0;
    double sine_frequency_ = 0.0;
    double sine_start_t_ = 0.0;
    bool warned_saturation_ = false;
    std::string fault_reason_ = "none";
    AtomicRobotCache cache_;
    LowCmdMsg low_cmd_{};
    unitree::robot::ChannelPublisherPtr<LowCmdMsg> lowcmd_pub_;
    unitree::robot::ChannelSubscriberPtr<LowStateMsg> lowstate_sub_;
};

CliOptions parseArgs(int argc, char** argv) {
    if (argc < 4) {
        throw std::runtime_error("missing arguments");
    }

    CliOptions opts;
    opts.config_path = argv[1];
    opts.target_q = std::stod(argv[2]);
    opts.run_seconds = std::stod(argv[3]);
    for (int i = 4; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto readValue = [&](const std::string& name) -> double {
            if (i + 1 >= argc) {
                throw std::runtime_error(name + " requires a value");
            }
            ++i;
            return std::stod(argv[i]);
        };

        if (arg == "--arm") {
            opts.armed = true;
        } else if (arg == "--kp") {
            opts.kp = readValue(arg);
        } else if (arg == "--ki") {
            opts.ki = readValue(arg);
        } else if (arg == "--kd") {
            opts.kd = readValue(arg);
        } else if (arg == "--tau-limit") {
            opts.tau_limit = readValue(arg);
        } else if (arg == "--speed") {
            opts.ref_speed = readValue(arg);
        } else if (arg == "--sine") {
            opts.sine_mode = true;
        } else if (arg == "--amp") {
            opts.sine_amplitude = readValue(arg);
        } else if (arg == "--freq") {
            opts.sine_frequency = readValue(arg);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }
    if (!opts.armed) {
        throw std::runtime_error("refusing to run without --arm");
    }
    if (opts.run_seconds <= 0.0 || opts.run_seconds > 120.0) {
        throw std::runtime_error("run_seconds must be in (0, 120]");
    }
    if (opts.kp < 0.0 || opts.kp > kMaxKpLimit ||
        opts.ki < 0.0 || opts.ki > kMaxKiLimit ||
        opts.kd < 0.0 || opts.kd > kMaxKdLimit ||
        opts.tau_limit <= 0.0 || opts.tau_limit > kMaxTauLimit ||
        opts.ref_speed <= 0.0 || opts.ref_speed > kMaxRefSpeedLimit) {
        throw std::runtime_error("PID option outside hard bring-up limits");
    }
    if (opts.sine_mode) {
        if (opts.sine_amplitude <= 0.0 || opts.sine_amplitude > kMaxSineAmplitude ||
            opts.sine_frequency <= 0.0 || opts.sine_frequency > kMaxSineFrequency) {
            throw std::runtime_error("sine option outside hard bring-up limits");
        }
    } else if (opts.sine_amplitude != 0.0 || opts.sine_frequency != 0.0) {
        throw std::runtime_error("--amp/--freq require --sine");
    }
    return opts;
}

void printUsage(const char* argv0) {
    std::cerr << "Usage:\n"
              << "  sudo " << argv0 << " <config.yaml> <target_q_rad> <run_seconds> --arm "
              << "[--kp K] [--ki K] [--kd K] [--tau-limit N_M] [--speed RAD_S]\n"
              << "  sudo " << argv0 << " <config.yaml> <center_q_rad> <run_seconds> --arm --sine "
              << "--amp RAD --freq HZ [--kp K] [--ki K] [--kd K] [--tau-limit N_M] [--speed RAD_S]\n\n"
              << "Example, stronger but still capped first bring-up:\n"
              << "  sudo " << argv0 << " config/h1_right_knee.yaml 0.55 8 --arm "
              << "--kp 30 --ki 1 --kd 3 --tau-limit 12 --speed 0.15\n\n"
              << "Example sine around 0.90 rad with 0.35 rad amplitude:\n"
              << "  sudo " << argv0 << " config/h1_right_knee.yaml 0.90 30 --arm --sine "
              << "--amp 0.35 --freq 0.08 --kp 30 --ki 1 --kd 3 --tau-limit 12 --speed 0.30\n\n"
              << "Safety constraints built into this first bring-up executable:\n"
              << "  target must be inside YAML joint limits by 0.03 rad\n"
              << "  target must be within 1.50 rad of the startup knee angle\n"
              << "  reference speed <= 0.80 rad/s, torque <= 18 N m\n"
              << "  kp <= 40, ki <= 5, kd <= 5, I torque contribution <= 4 N m\n"
              << "  sine amplitude <= 0.80 rad, frequency <= 0.35 Hz, full sine range inside soft limits\n"
              << "  trips on measured angle limit, speed > 2 rad/s, state jump, or loop overrun\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    try {
        CliOptions opts = parseArgs(argc, argv);
        h1if::RuntimeConfig cfg = h1if::loadRuntimeConfig(opts.config_path);
        H1KneePidRuntime runtime(std::move(cfg), opts);
        runtime.init();
        return runtime.run();
    } catch (const std::exception& ex) {
        std::cerr << "h1_knee_pid failed: " << ex.what() << "\n\n";
        printUsage(argv[0]);
        return 2;
    }
}
