#include "async_csv_logger.hpp"
#include "controller_factory.hpp"
#include "runtime_config.hpp"
#include "safety.hpp"

#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include <unitree/common/thread/thread.hpp>
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
constexpr double kPi = 3.14159265358979323846;

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

struct DirectRunOptions {
    double duration_s = 0.0;
    bool software_disturbance_enabled = false;
    std::vector<int> software_disturbance_joints;
    std::vector<double> software_disturbance_torques;
    double software_disturbance_start_s = 4.0;
    double software_disturbance_plateau_start_s = 4.2;
    double software_disturbance_plateau_end_s = 5.2;
    double software_disturbance_end_s = 5.4;
};

void printUsage(const char* argv0) {
    std::cerr << "Usage:\n"
              << "  " << argv0 << " <config.yaml> [--duration SEC] [--repeat ID] [--condition ID] "
              << "[--disturbance-target TARGET] [--disturbance-method METHOD]\n"
              << "      [--software-disturbance-joints IDS] [--software-disturbance-torques NM]\n"
              << "      [--software-disturbance-start SEC]\n"
              << "      [--software-disturbance-plateau-start SEC]\n"
              << "      [--software-disturbance-plateau-end SEC]\n"
              << "      [--software-disturbance-end SEC]\n\n"
              << "If --duration is omitted, the controller runs until Ctrl+C, SIGTERM, or a safety trip.\n"
              << "Software disturbance lists are comma-separated and are disabled unless joints/torques are set.\n"
              << "Real H1 config should set network_interface=enp3s0 and domain_id=0.\n"
              << "unitree_mujoco config should set network_interface=lo and domain_id=1.\n";
}

std::vector<int> parseIntList(const std::string& value, const std::string& label) {
    std::vector<int> result;
    std::stringstream ss(value);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (item.empty()) {
            throw std::runtime_error(label + " contains an empty item");
        }
        std::size_t used = 0;
        const int parsed = std::stoi(item, &used);
        if (used != item.size()) {
            throw std::runtime_error(label + " contains a non-integer item: " + item);
        }
        result.push_back(parsed);
    }
    return result;
}

std::vector<double> parseDoubleList(const std::string& value, const std::string& label) {
    std::vector<double> result;
    std::stringstream ss(value);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (item.empty()) {
            throw std::runtime_error(label + " contains an empty item");
        }
        std::size_t used = 0;
        const double parsed = std::stod(item, &used);
        if (used != item.size() || !std::isfinite(parsed)) {
            throw std::runtime_error(label + " contains an invalid number: " + item);
        }
        result.push_back(parsed);
    }
    return result;
}

double parseDoubleScalar(const std::string& value, const std::string& label) {
    std::size_t used = 0;
    const double parsed = std::stod(value, &used);
    if (used != value.size() || !std::isfinite(parsed)) {
        throw std::runtime_error(label + " must be a finite number");
    }
    return parsed;
}

std::string joinIntList(const std::vector<int>& values) {
    std::ostringstream out;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            out << ",";
        }
        out << values[i];
    }
    return out.str();
}

std::string joinDoubleList(const std::vector<double>& values) {
    std::ostringstream out;
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            out << ",";
        }
        out << values[i];
    }
    return out.str();
}

void validateDirectRunOptions(DirectRunOptions& opts) {
    const bool has_joints = !opts.software_disturbance_joints.empty();
    const bool has_torques = !opts.software_disturbance_torques.empty();
    if (has_joints != has_torques) {
        throw std::runtime_error(
            "--software-disturbance-joints and --software-disturbance-torques must be provided together");
    }
    opts.software_disturbance_enabled = has_joints && has_torques;
    if (!opts.software_disturbance_enabled) {
        return;
    }
    if (opts.software_disturbance_joints.size() != opts.software_disturbance_torques.size()) {
        throw std::runtime_error(
            "--software-disturbance-joints and --software-disturbance-torques must have the same length");
    }
    for (int joint_id : opts.software_disturbance_joints) {
        if (joint_id < 0 || joint_id >= h1if::kMaxMotors) {
            throw std::runtime_error("software disturbance joint id out of range: " + std::to_string(joint_id));
        }
    }
    const double t0 = opts.software_disturbance_start_s;
    const double t1 = opts.software_disturbance_plateau_start_s;
    const double t2 = opts.software_disturbance_plateau_end_s;
    const double t3 = opts.software_disturbance_end_s;
    if (!std::isfinite(t0) || !std::isfinite(t1) || !std::isfinite(t2) || !std::isfinite(t3) ||
        t0 < 0.0 || t1 < t0 || t2 < t1 || t3 <= t2) {
        throw std::runtime_error(
            "software disturbance timing must satisfy 0 <= start <= plateau-start <= plateau-end < end");
    }
    if (opts.duration_s > 0.0 && t3 > opts.duration_s) {
        throw std::runtime_error("--software-disturbance-end must not exceed --duration");
    }
}

DirectRunOptions applyCommandLineOptions(h1if::RuntimeConfig& cfg, int argc, char** argv) {
    DirectRunOptions opts;
    for (int i = 2; i < argc; ++i) {
        const std::string key = argv[i];
        if (i + 1 >= argc) {
            throw std::runtime_error("missing value for " + key);
        }
        const std::string value = argv[++i];
        if (key == "--duration") {
            opts.duration_s = parseDoubleScalar(value, key);
            if (!std::isfinite(opts.duration_s) || opts.duration_s <= 0.0) {
                throw std::runtime_error("--duration must be positive seconds");
            }
        } else if (key == "--repeat") {
            cfg.repeat_id = value;
        } else if (key == "--condition") {
            cfg.condition_id = value;
        } else if (key == "--disturbance-target") {
            cfg.disturbance_target = value;
        } else if (key == "--disturbance-method") {
            cfg.disturbance_method = value;
        } else if (key == "--experiment") {
            cfg.experiment_id = value;
        } else if (key == "--software-disturbance-joints") {
            opts.software_disturbance_joints = parseIntList(value, key);
        } else if (key == "--software-disturbance-torques") {
            opts.software_disturbance_torques = parseDoubleList(value, key);
        } else if (key == "--software-disturbance-start") {
            opts.software_disturbance_start_s = parseDoubleScalar(value, key);
        } else if (key == "--software-disturbance-plateau-start") {
            opts.software_disturbance_plateau_start_s = parseDoubleScalar(value, key);
        } else if (key == "--software-disturbance-plateau-end") {
            opts.software_disturbance_plateau_end_s = parseDoubleScalar(value, key);
        } else if (key == "--software-disturbance-end") {
            opts.software_disturbance_end_s = parseDoubleScalar(value, key);
        } else {
            throw std::runtime_error("unknown option: " + key);
        }
    }
    validateDirectRunOptions(opts);
    return opts;
}

void copyConfigSnapshot(const h1if::RuntimeConfig& cfg) {
    namespace fs = std::filesystem;
    if (cfg.config_path.empty() || cfg.log_path.empty()) {
        return;
    }
    std::error_code ec;
    const fs::path log_path(cfg.log_path);
    fs::create_directories(log_path.parent_path(), ec);
    if (ec) {
        std::cerr << "Warning: cannot create log directory " << log_path.parent_path()
                  << ": " << ec.message() << "\n";
        return;
    }
    const fs::path out_path = log_path.parent_path() / "input_config.yaml";
    fs::copy_file(cfg.config_path, out_path, fs::copy_options::overwrite_existing, ec);
    if (ec) {
        std::cerr << "Warning: cannot copy config snapshot to " << out_path << ": "
                  << ec.message() << "\n";
    }
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
    std::array<std::atomic<double>, 4> quat;
    std::array<std::atomic<double>, 3> gyro;
    std::array<std::atomic<double>, 3> acc;
    std::atomic<std::uint64_t> last_state_ns{0};

    AtomicRobotCache() {
        for (int i = 0; i < h1if::kMaxMotors; ++i) {
            q[i].store(0.0);
            dq[i].store(0.0);
            tau_est[i].store(0.0);
        }
        quat[0].store(1.0);
        for (int i = 1; i < 4; ++i) {
            quat[i].store(0.0);
        }
        for (int i = 0; i < 3; ++i) {
            gyro[i].store(0.0);
            acc[i].store(0.0);
        }
    }
};

class UnitreeH1DirectInterface {
public:
    UnitreeH1DirectInterface(h1if::RuntimeConfig cfg, DirectRunOptions opts)
        : cfg_(std::move(cfg)),
          opts_(std::move(opts)),
          controller_(h1if::createController(cfg_)),
          active_joints_(h1if::activeControllerJoints(cfg_)) {}

    ~UnitreeH1DirectInterface() {
        shutdown();
    }

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
            std::bind(&UnitreeH1DirectInterface::onLowState, this, std::placeholders::_1),
            1);

        tryReleaseMotionMode();

        if (!logger_.start(cfg_.log_path)) {
            std::cerr << "Warning: cannot open log file: " << cfg_.log_path << "\n";
        }

        std::cout << "Controller: " << controller_->name() << "\n";
        waitForFirstState();
        controller_->reset(readRobotState(0, cfg_.control_dt));
    }

    void run(double duration_s = 0.0) {
        setThreadRealtime();

        std::uint64_t cycle = 0;
        double last_t = nowSec();
        const double run_start_t = last_t;

        while (g_running.load(std::memory_order_acquire)) {
            const auto loop_start = std::chrono::steady_clock::now();
            const double t = nowSec();
            if (duration_s > 0.0 && t - run_start_t >= duration_s) {
                std::cout << "controller duration reached: " << duration_s << " s\n";
                break;
            }
            const double actual_dt = t - last_t;
            last_t = t;

            h1if::RobotState state = readRobotState(cycle, actual_dt);
            h1if::RobotCommand command;
            h1if::ControllerDebug debug;

            std::string trip_reason;
            if (checkMeasuredTrip(state, actual_dt, trip_reason)) {
                std::cerr << "controller stopped: " << trip_reason << "\n";
                sendSafeHold();
                break;
            }

            if (state.state_valid) {
                controller_->step(state, command, debug);
            } else {
                h1if::fillSafeHoldCommand(state, command, cfg_.safety);
            }

            const double elapsed_s = t - run_start_t;
            const double disturbance_scale = softwareDisturbanceScale(elapsed_s);
            const std::array<double, h1if::kMaxMotors> tau_dist =
                applySoftwareDisturbance(command, disturbance_scale);
            h1if::applySafety(state, command, debug, cfg_.safety);

            writeLowCmd(command);
            pushLog(state, command, debug, tau_dist, disturbance_scale);

            ++cycle;
            std::this_thread::sleep_until(
                loop_start + std::chrono::duration<double>(cfg_.control_dt));
        }

        shutdown();
    }

private:
    void shutdown() {
        if (shutdown_done_) {
            return;
        }
        shutdown_done_ = true;

        if (lowcmd_pub_) {
            sendSafeHold();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        logger_.stop();

        if (lowstate_sub_) {
            lowstate_sub_->CloseChannel();
            lowstate_sub_.reset();
        }
        if (lowcmd_pub_) {
            lowcmd_pub_->CloseChannel();
            lowcmd_pub_.reset();
        }
        unitree::robot::ChannelFactory::Instance()->Release();
    }

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
        for (int i = 0; i < 4; ++i) {
            cache_.quat[i].store(msg->imu_state().quaternion()[i], std::memory_order_relaxed);
        }
        for (int i = 0; i < 3; ++i) {
            cache_.gyro[i].store(msg->imu_state().gyroscope()[i], std::memory_order_relaxed);
            cache_.acc[i].store(msg->imu_state().accelerometer()[i], std::memory_order_relaxed);
        }
        cache_.last_state_ns.store(nowNs(), std::memory_order_release);
    }

    h1if::RobotState readRobotState(std::uint64_t cycle, double dt) {
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
        for (int i = 0; i < 4; ++i) {
            state.imu.quat[i] = cache_.quat[i].load(std::memory_order_relaxed);
        }
        for (int i = 0; i < 3; ++i) {
            state.imu.gyro[i] = cache_.gyro[i].load(std::memory_order_relaxed);
            state.imu.acc[i] = cache_.acc[i].load(std::memory_order_relaxed);
        }

        return state;
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

    void pushLog(const h1if::RobotState& state,
                 const h1if::RobotCommand& command,
                 const h1if::ControllerDebug& debug,
                 const std::array<double, h1if::kMaxMotors>& tau_dist,
                 double disturbance_scale) {
        for (int j : active_joints_) {
            h1if::LogSample sample;
            sample.experiment_id = cfg_.experiment_id;
            sample.condition_id = cfg_.condition_id;
            sample.repeat_id = cfg_.repeat_id;
            sample.disturbance_target = cfg_.disturbance_target;
            sample.disturbance_method = cfg_.disturbance_method;
            sample.config_path = cfg_.config_path;
            sample.log_path = cfg_.log_path;
            sample.cycle = state.cycle;
            sample.t = state.t;
            sample.dt = state.dt;
            sample.lowstate_age = state.lowstate_age;
            sample.joint_id = j;
            sample.measured = state.joint[j];
            sample.command = command.joint[j];
            sample.tau_dist = tau_dist[j];
            sample.disturbance_scale = disturbance_scale;
            sample.flags = debug.flags | debug.joint[j].flags;
            for (int i = 0; i < static_cast<int>(sample.debug.size()); ++i) {
                sample.debug[i] = debug.joint[j].data[i];
            }
            logger_.push(sample);
        }
    }

    double softwareDisturbanceScale(double elapsed_s) const {
        if (!opts_.software_disturbance_enabled) {
            return 0.0;
        }
        const double t0 = opts_.software_disturbance_start_s;
        const double t1 = opts_.software_disturbance_plateau_start_s;
        const double t2 = opts_.software_disturbance_plateau_end_s;
        const double t3 = opts_.software_disturbance_end_s;
        if (elapsed_s < t0 || elapsed_s >= t3) {
            return 0.0;
        }
        if (elapsed_s < t1) {
            if (t1 == t0) {
                return 1.0;
            }
            const double r = (elapsed_s - t0) / (t1 - t0);
            return 0.5 - 0.5 * std::cos(kPi * r);
        }
        if (elapsed_s <= t2) {
            return 1.0;
        }
        const double r = (elapsed_s - t2) / (t3 - t2);
        return 0.5 + 0.5 * std::cos(kPi * r);
    }

    std::array<double, h1if::kMaxMotors> applySoftwareDisturbance(
        h1if::RobotCommand& command,
        double disturbance_scale) const {
        std::array<double, h1if::kMaxMotors> tau_dist{};
        if (!opts_.software_disturbance_enabled || disturbance_scale == 0.0) {
            return tau_dist;
        }
        for (std::size_t i = 0; i < opts_.software_disturbance_joints.size(); ++i) {
            const int joint_id = opts_.software_disturbance_joints[i];
            const double tau = disturbance_scale * opts_.software_disturbance_torques[i];
            command.joint[joint_id].tau += static_cast<float>(tau);
            tau_dist[joint_id] = tau;
        }
        return tau_dist;
    }

    void sendSafeHold() {
        h1if::RobotCommand command;
        h1if::ControllerDebug debug;
        const h1if::RobotState state = readRobotState(0, cfg_.control_dt);
        h1if::fillSafeHoldCommand(state, command, cfg_.safety);
        h1if::applySafety(state, command, debug, cfg_.safety);
        writeLowCmd(command);
    }

    bool checkMeasuredTrip(const h1if::RobotState& state,
                           double dt,
                           std::string& reason) {
        if (!state.state_valid || state.lowstate_age > cfg_.safety.lowstate_timeout) {
            reason = "LowState timeout";
            return true;
        }
        if (dt <= 0.0 || dt > cfg_.safety.max_control_dt) {
            reason = "control loop jitter/overrun exceeded max_control_dt trip";
            return true;
        }

        for (int j : active_joints_) {
            const auto& lim = cfg_.safety.limit[j];
            const auto& target = state.joint[j];
            const std::string prefix = "joint " + std::to_string(j) + ": ";

            if (!std::isfinite(target.q) || !std::isfinite(target.dq) || !std::isfinite(target.tau_est)) {
                reason = prefix + "non-finite measured state";
                return true;
            }
            if (target.q < lim.q_min || target.q > lim.q_max) {
                reason = prefix + "angle exceeded configured limits";
                return true;
            }
            if (std::abs(target.dq) > cfg_.safety.measured_speed_trip) {
                reason = prefix + "speed exceeded measured_speed_trip trip";
                return true;
            }
            if (have_last_q_[j] && std::abs(target.q - last_q_[j]) > cfg_.safety.measured_jump_trip) {
                reason = prefix + "measured angle jump exceeded measured_jump_trip trip";
                return true;
            }

            last_q_[j] = target.q;
            have_last_q_[j] = true;
        }
        return false;
    }

    void waitForFirstState() {
        const auto start = nowNs();
        while (g_running.load(std::memory_order_acquire)) {
            if (cache_.last_state_ns.load(std::memory_order_acquire) != 0) {
                std::cout << "LowState received.\n";
                return;
            }
            if ((nowNs() - start) > static_cast<std::uint64_t>(5e9)) {
                std::cerr << "Warning: no LowState received after 5 seconds.\n";
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
    DirectRunOptions opts_;
    std::unique_ptr<h1if::IController> controller_;
    std::vector<int> active_joints_;
    std::array<double, h1if::kMaxMotors> last_q_{};
    std::array<bool, h1if::kMaxMotors> have_last_q_{};
    AtomicRobotCache cache_;
    LowCmdMsg low_cmd_{};
    unitree::robot::ChannelPublisherPtr<LowCmdMsg> lowcmd_pub_;
    unitree::robot::ChannelSubscriberPtr<LowStateMsg> lowstate_sub_;
    h1if::AsyncCsvLogger<> logger_;
    bool shutdown_done_ = false;
};

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    try {
        h1if::RuntimeConfig cfg = h1if::loadRuntimeConfig(argv[1]);
        const DirectRunOptions opts = applyCommandLineOptions(cfg, argc, argv);
        cfg.log_path = h1if::resolveLogPath(cfg);
        copyConfigSnapshot(cfg);

        std::cout << "WARNING: use only with H1 suspended, lying safely, or mechanically protected.\n"
                  << "WARNING: verify joint_id, motor mode, limits, and emergency stop before enabling.\n"
                  << "Log path: " << cfg.log_path << "\n"
                  << "Experiment: " << cfg.experiment_id
                  << " condition=" << cfg.condition_id
                  << " repeat=" << cfg.repeat_id << "\n"
                  << "Duration: "
                  << (opts.duration_s > 0.0 ? std::to_string(opts.duration_s) + " s" : "until stopped")
                  << "\n";
        if (opts.software_disturbance_enabled) {
            std::cout << "Software disturbance: joints="
                      << joinIntList(opts.software_disturbance_joints)
                      << " torques_Nm=" << joinDoubleList(opts.software_disturbance_torques)
                      << " window=" << opts.software_disturbance_start_s
                      << "," << opts.software_disturbance_plateau_start_s
                      << "," << opts.software_disturbance_plateau_end_s
                      << "," << opts.software_disturbance_end_s << "\n";
        }
        std::cout << "Press Enter to continue...\n";
        std::cin.get();

        UnitreeH1DirectInterface robot(std::move(cfg), opts);
        robot.init();
        robot.run(opts.duration_s);
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "h1_direct failed: " << ex.what() << "\n";
        return 2;
    }
}
