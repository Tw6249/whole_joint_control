#include "async_csv_logger.hpp"
#include "eid_controller.hpp"
#include "runtime_config.hpp"
#include "safety.hpp"

#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
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
constexpr double kMaxMeasuredSpeed = 8.0;
constexpr double kMaxMeasuredJump = 0.10;
constexpr double kMaxControlDt = 0.010;

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
    explicit UnitreeH1DirectInterface(h1if::RuntimeConfig cfg)
        : cfg_(std::move(cfg)),
          controller_(cfg_),
          active_joints_(h1if::activeEidJoints(cfg_)) {}

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

        std::cout << "Controller: " << controller_.name() << "\n";
        waitForFirstState();
        controller_.reset(readRobotState(0, cfg_.control_dt));
    }

    void run() {
        setThreadRealtime();

        std::uint64_t cycle = 0;
        double last_t = nowSec();

        while (g_running.load(std::memory_order_acquire)) {
            const auto loop_start = std::chrono::steady_clock::now();
            const double t = nowSec();
            const double actual_dt = t - last_t;
            last_t = t;

            h1if::RobotState state = readRobotState(cycle, actual_dt);
            h1if::RobotCommand command;
            h1if::ControllerDebug debug;

            std::string trip_reason;
            if (checkMeasuredTrip(state, actual_dt, trip_reason)) {
                std::cerr << "EID stopped: " << trip_reason << "\n";
                sendSafeHold();
                break;
            }

            if (state.state_valid) {
                controller_.step(state, command, debug);
            } else {
                h1if::fillSafeHoldCommand(state, command, cfg_.safety);
            }
            h1if::applySafety(state, command, debug, cfg_.safety);

            writeLowCmd(command);
            pushLog(state, command, debug);

            ++cycle;
            std::this_thread::sleep_until(
                loop_start + std::chrono::duration<double>(cfg_.control_dt));
        }

        sendSafeHold();
        logger_.stop();
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

    void pushLog(const h1if::RobotState& state, const h1if::RobotCommand& command, const h1if::ControllerDebug& debug) {
        for (int j : active_joints_) {
            h1if::LogSample sample;
            sample.cycle = state.cycle;
            sample.t = state.t;
            sample.dt = state.dt;
            sample.lowstate_age = state.lowstate_age;
            sample.joint_id = j;
            sample.measured = state.joint[j];
            sample.command = command.joint[j];
            sample.flags = debug.flags | debug.joint[j].flags;
            for (int i = 0; i < static_cast<int>(sample.debug.size()); ++i) {
                sample.debug[i] = debug.joint[j].data[i];
            }
            logger_.push(sample);
        }
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
        if (dt <= 0.0 || dt > kMaxControlDt) {
            reason = "control loop jitter/overrun exceeded trip";
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
            if (std::abs(target.dq) > kMaxMeasuredSpeed) {
                reason = prefix + "speed exceeded measured-speed trip";
                return true;
            }
            if (have_last_q_[j] && std::abs(target.q - last_q_[j]) > kMaxMeasuredJump) {
                reason = prefix + "measured angle jumped too far";
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
    h1if::EidMultiJointController controller_;
    std::vector<int> active_joints_;
    std::array<double, h1if::kMaxMotors> last_q_{};
    std::array<bool, h1if::kMaxMotors> have_last_q_{};
    AtomicRobotCache cache_;
    LowCmdMsg low_cmd_{};
    unitree::robot::ChannelPublisherPtr<LowCmdMsg> lowcmd_pub_;
    unitree::robot::ChannelSubscriberPtr<LowStateMsg> lowstate_sub_;
    h1if::AsyncCsvLogger<> logger_;
};

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage:\n"
                  << "  " << argv[0] << " <config.yaml>\n\n"
                  << "Real H1 config should set network_interface=enp3s0 and domain_id=0.\n"
                  << "unitree_mujoco config should set network_interface=lo and domain_id=1.\n";
        return 1;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    try {
        h1if::RuntimeConfig cfg = h1if::loadRuntimeConfig(argv[1]);
        cfg.log_path = h1if::resolveLogPath(cfg);

        std::cout << "WARNING: use only with H1 suspended, lying safely, or mechanically protected.\n"
                  << "WARNING: verify joint_id, motor mode, limits, and emergency stop before enabling.\n"
                  << "Press Enter to continue...\n";
        std::cin.get();

        UnitreeH1DirectInterface robot(std::move(cfg));
        robot.init();
        robot.run();
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "h1_direct failed: " << ex.what() << "\n";
        return 2;
    }
}
