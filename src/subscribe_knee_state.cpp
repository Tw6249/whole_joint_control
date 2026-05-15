#include "runtime_config.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <utility>

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using LowStateMsg = unitree_go::msg::dds_::LowState_;

namespace {

constexpr const char* kTopicLowState = "rt/lowstate";

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

struct AtomicJointSample {
    std::atomic<double> q{0.0};
    std::atomic<double> dq{0.0};
    std::atomic<double> tau_est{0.0};
    std::atomic<std::uint64_t> last_state_ns{0};
    std::atomic<std::uint64_t> sequence{0};
};

class KneeStateSubscriber {
public:
    explicit KneeStateSubscriber(h1if::RuntimeConfig cfg)
        : cfg_(std::move(cfg)),
          joint_id_(h1if::primaryEidJoint(cfg_)) {}

    void init() {
        unitree::robot::ChannelFactory::Instance()->Init(
            cfg_.domain_id,
            cfg_.network_interface);

        lowstate_sub_.reset(new unitree::robot::ChannelSubscriber<LowStateMsg>(kTopicLowState));
        lowstate_sub_->InitChannel(
            std::bind(&KneeStateSubscriber::onLowState, this, std::placeholders::_1),
            1);

        std::cout << "Listening to " << kTopicLowState
                  << " on interface " << cfg_.network_interface
                  << ", domain " << cfg_.domain_id << "\n"
                  << "Target joint: " << joint_id_ << " (primary eid_controllers joint)\n";
    }

    void run() {
        waitForFirstState();

        std::cout << "t_s,joint_id,q_rad,dq_rad_s,tau_est_nm,lowstate_age_s\n";
        while (g_running.load(std::memory_order_acquire)) {
            printLatestSample();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

private:
    void onLowState(const void* message) {
        const auto* msg = static_cast<const LowStateMsg*>(message);
        const auto& motor = msg->motor_state()[joint_id_];

        sample_.q.store(motor.q(), std::memory_order_relaxed);
        sample_.dq.store(motor.dq(), std::memory_order_relaxed);
        sample_.tau_est.store(motor.tau_est(), std::memory_order_relaxed);
        sample_.last_state_ns.store(nowNs(), std::memory_order_release);
        sample_.sequence.fetch_add(1, std::memory_order_release);
    }

    void waitForFirstState() const {
        const auto start = nowNs();
        while (g_running.load(std::memory_order_acquire)) {
            if (sample_.last_state_ns.load(std::memory_order_acquire) != 0) {
                return;
            }
            if ((nowNs() - start) > static_cast<std::uint64_t>(5e9)) {
                std::cerr << "Warning: no LowState received after 5 seconds.\n";
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void printLatestSample() const {
        const std::uint64_t last_ns = sample_.last_state_ns.load(std::memory_order_acquire);
        const double age = last_ns == 0
            ? 1e9
            : static_cast<double>(nowNs() - last_ns) * 1e-9;

        std::cout << std::fixed << std::setprecision(6)
                  << nowSec() << ','
                  << joint_id_ << ','
                  << sample_.q.load(std::memory_order_relaxed) << ','
                  << sample_.dq.load(std::memory_order_relaxed) << ','
                  << sample_.tau_est.load(std::memory_order_relaxed) << ','
                  << age << '\n';
    }

    h1if::RuntimeConfig cfg_;
    int joint_id_ = 2;
    AtomicJointSample sample_;
    unitree::robot::ChannelSubscriberPtr<LowStateMsg> lowstate_sub_;
};

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage:\n"
                  << "  " << argv[0] << " <config.yaml>\n\n"
                  << "The subscriber is read-only. It uses the primary eid_controllers joint from the YAML.\n";
        return 1;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    try {
        h1if::RuntimeConfig cfg = h1if::loadRuntimeConfig(argv[1]);
        KneeStateSubscriber subscriber(std::move(cfg));
        subscriber.init();
        subscriber.run();
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "h1_knee_state failed: " << ex.what() << "\n";
        return 2;
    }
}
