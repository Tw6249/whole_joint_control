#include "runtime_config.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <ctime>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

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

std::uint64_t wallNowNs() {
    using namespace std::chrono;
    return static_cast<std::uint64_t>(
        duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count());
}

double nsToSec(std::uint64_t ns) {
    return static_cast<double>(ns) * 1.0e-9;
}

std::uint64_t secToNs(double seconds) {
    return static_cast<std::uint64_t>(seconds * 1.0e9);
}

std::string formatLocalWallTime(std::uint64_t unix_ns) {
    const std::time_t seconds = static_cast<std::time_t>(unix_ns / 1000000000ULL);
    const std::uint64_t nanos = unix_ns % 1000000000ULL;
    std::tm tm{};
    localtime_r(&seconds, &tm);

    std::ostringstream out;
    out << std::put_time(&tm, "%Y-%m-%d %H:%M:%S")
        << '.' << std::setw(9) << std::setfill('0') << nanos << std::setfill(' ');
    return out.str();
}

void signalHandler(int) {
    g_running.store(false, std::memory_order_release);
}

std::vector<int> parseJointList(const std::string& value) {
    std::vector<int> joints;
    std::stringstream ss(value);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (item.empty()) {
            throw std::runtime_error("--joints contains an empty item");
        }
        std::size_t used = 0;
        const int joint = std::stoi(item, &used);
        if (used != item.size() || joint < 0 || joint >= h1if::kMaxMotors) {
            throw std::runtime_error("--joints item is out of range: " + item);
        }
        joints.push_back(joint);
    }
    if (joints.empty()) {
        throw std::runtime_error("--joints must contain at least one joint id");
    }
    return joints;
}

double parsePositiveDouble(const std::string& value, const std::string& label) {
    std::size_t used = 0;
    const double parsed = std::stod(value, &used);
    if (used != value.size() || !(parsed > 0.0) || !std::isfinite(parsed)) {
        throw std::runtime_error(label + " must be a positive finite number");
    }
    return parsed;
}

int parseIntScalar(const std::string& value, const std::string& label) {
    std::size_t used = 0;
    const int parsed = std::stoi(value, &used);
    if (used != value.size()) {
        throw std::runtime_error(label + " must be an integer");
    }
    return parsed;
}

std::string joinJoints(const std::vector<int>& joints) {
    std::ostringstream out;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        if (i > 0) {
            out << ",";
        }
        out << joints[i];
    }
    return out.str();
}

std::string jointName(const h1if::RuntimeConfig& cfg, int joint_id) {
    if (joint_id >= 0 && joint_id < h1if::kMaxMotors &&
        cfg.controller.joints[joint_id].has_value() &&
        !cfg.controller.joints[joint_id]->name.empty()) {
        return cfg.controller.joints[joint_id]->name;
    }
    switch (joint_id) {
        case 0: return "RightHipRoll";
        case 1: return "RightHipPitch";
        case 2: return "RightKnee";
        case 3: return "LeftHipRoll";
        case 4: return "LeftHipPitch";
        case 5: return "LeftKnee";
        case 6: return "WaistYaw";
        case 7: return "LeftShoulderPitch";
        case 8: return "LeftShoulderRoll";
        case 9: return "LeftShoulderYaw";
        case 10: return "LeftElbow";
        case 11: return "RightShoulderPitch";
        case 12: return "RightShoulderRoll";
        case 13: return "RightShoulderYaw";
        case 14: return "RightElbow";
        default: break;
    }
    return "joint_" + std::to_string(joint_id);
}

std::string defaultLogPath(const h1if::RuntimeConfig& cfg) {
    namespace fs = std::filesystem;
    fs::path resolved(h1if::resolveLogPath(cfg));
    const std::string stem = resolved.stem().string();
    const std::string ext = resolved.extension().empty() ? ".csv" : resolved.extension().string();
    return (resolved.parent_path() / (stem + "_native_torque" + ext)).string();
}

struct Options {
    std::string config_path;
    std::string out_path;
    std::vector<int> joints;
    double duration_s = 0.0;
    double sample_period_s = 0.0;
};

void printUsage(const char* argv0) {
    std::cerr << "Usage:\n"
              << "  " << argv0 << " <config.yaml> [--duration SEC] [--sample-period SEC] [--joints IDS] [--out CSV]\n"
              << "      [--iface IFACE] [--domain ID]\n\n"
              << "Examples:\n"
              << "  " << argv0 << " experiments/hip_knee/configs/h1_real_p3_selected_mpc_hip_knee_pd.yaml --duration 60 --joints 1,2\n"
              << "  " << argv0 << " experiments/hip_knee/configs/h1_real_p3_selected_mpc_hip_knee_pd.yaml --duration 60 --sample-period 0.01 --joints 1,2,4,5\n"
              << "  " << argv0 << " experiments/hip_knee/configs/h1_real_p3_selected_mpc_hip_knee_pd.yaml --duration 20 --out data/landing.csv\n\n"
              << "This program is read-only: it subscribes to " << kTopicLowState
              << " and never publishes rt/lowcmd.\n";
}

Options parseOptions(h1if::RuntimeConfig& cfg, int argc, char** argv) {
    if (argc < 2) {
        throw std::runtime_error("missing config path");
    }
    Options opts;
    opts.config_path = argv[1];

    for (int i = 2; i < argc; ++i) {
        const std::string key = argv[i];
        if (i + 1 >= argc) {
            throw std::runtime_error("missing value for " + key);
        }
        const std::string value = argv[++i];
        if (key == "--duration") {
            opts.duration_s = parsePositiveDouble(value, key);
        } else if (key == "--sample-period") {
            opts.sample_period_s = parsePositiveDouble(value, key);
        } else if (key == "--joints") {
            opts.joints = parseJointList(value);
        } else if (key == "--out") {
            opts.out_path = value;
        } else if (key == "--iface") {
            cfg.network_interface = value;
        } else if (key == "--domain") {
            cfg.domain_id = parseIntScalar(value, key);
        } else {
            throw std::runtime_error("unknown option: " + key);
        }
    }

    if (opts.joints.empty()) {
        opts.joints = h1if::activeControllerJoints(cfg);
    }
    if (opts.out_path.empty()) {
        opts.out_path = defaultLogPath(cfg);
    }
    return opts;
}

struct TorqueSample {
    std::uint64_t frame = 0;
    std::uint64_t host_ns = 0;
    std::uint64_t wall_unix_ns = 0;
    double t_rel_s = 0.0;
    double lowstate_dt_s = 0.0;
    double sample_dt_s = 0.0;
    int joint_id = 0;
    double q = 0.0;
    double dq = 0.0;
    double tau_est = 0.0;
    std::array<double, 4> imu_quat{1.0, 0.0, 0.0, 0.0};
    std::array<double, 3> imu_gyro{0.0, 0.0, 0.0};
    std::array<double, 3> imu_acc{0.0, 0.0, 0.0};
};

struct LatestLowState {
    bool valid = false;
    std::uint64_t state_host_ns = 0;
    double lowstate_dt_s = 0.0;
    std::array<double, h1if::kMaxMotors> q{};
    std::array<double, h1if::kMaxMotors> dq{};
    std::array<double, h1if::kMaxMotors> tau_est{};
    std::array<double, 4> imu_quat{1.0, 0.0, 0.0, 0.0};
    std::array<double, 3> imu_gyro{0.0, 0.0, 0.0};
    std::array<double, 3> imu_acc{0.0, 0.0, 0.0};
};

template <std::size_t Capacity = 262144>
class TorqueCsvLogger {
public:
    explicit TorqueCsvLogger(std::vector<std::string> names)
        : names_(std::move(names)) {}

    TorqueCsvLogger(const TorqueCsvLogger&) = delete;
    TorqueCsvLogger& operator=(const TorqueCsvLogger&) = delete;

    ~TorqueCsvLogger() {
        stop();
    }

    bool start(const std::string& path) {
        std::error_code ec;
        const auto parent = std::filesystem::path(path).parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent, ec);
        }
        out_.open(path, std::ios::out | std::ios::trunc);
        if (!out_) {
            return false;
        }
        writeHeader();
        running_.store(true, std::memory_order_release);
        worker_ = std::thread([this]() { writerLoop(); });
        return true;
    }

    void stop() {
        running_.store(false, std::memory_order_release);
        if (worker_.joinable()) {
            worker_.join();
        }
        drain();
        if (out_) {
            out_.flush();
            out_.close();
        }
    }

    bool push(const TorqueSample& sample) {
        const auto head = head_.load(std::memory_order_relaxed);
        const auto next = increment(head);
        if (next == tail_.load(std::memory_order_acquire)) {
            drops_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        (*buffer_)[head] = sample;
        head_.store(next, std::memory_order_release);
        return true;
    }

    std::uint64_t drops() const {
        return drops_.load(std::memory_order_relaxed);
    }

private:
    static std::size_t increment(std::size_t value) {
        return (value + 1) % Capacity;
    }

    bool pop(TorqueSample& sample) {
        const auto tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;
        }
        sample = (*buffer_)[tail];
        tail_.store(increment(tail), std::memory_order_release);
        return true;
    }

    void writerLoop() {
        while (running_.load(std::memory_order_acquire)) {
            drain();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    void drain() {
        TorqueSample sample;
        while (pop(sample)) {
            writeSample(sample);
        }
        if (out_) {
            out_.flush();
        }
    }

    void writeHeader() {
        out_ << "frame,host_ns,wall_unix_ns,wall_time_local,t_rel_s,lowstate_dt_s,sample_dt_s,joint_id,joint_name,"
             << "q,dq,tau_est,"
             << "imu_qw,imu_qx,imu_qy,imu_qz,"
             << "imu_gx,imu_gy,imu_gz,"
             << "imu_ax,imu_ay,imu_az\n";
    }

    void writeSample(const TorqueSample& s) {
        out_ << std::setprecision(17)
             << s.frame << ','
             << s.host_ns << ','
             << s.wall_unix_ns << ',';
        writeCsvCell(formatLocalWallTime(s.wall_unix_ns));
        out_ << ','
             << s.t_rel_s << ','
             << s.lowstate_dt_s << ','
             << s.sample_dt_s << ','
             << s.joint_id << ',';
        const std::string name = (s.joint_id >= 0 && s.joint_id < static_cast<int>(names_.size()))
            ? names_[s.joint_id]
            : "joint_" + std::to_string(s.joint_id);
        writeCsvCell(name);
        out_ << ','
             << s.q << ','
             << s.dq << ','
             << s.tau_est << ','
             << s.imu_quat[0] << ','
             << s.imu_quat[1] << ','
             << s.imu_quat[2] << ','
             << s.imu_quat[3] << ','
             << s.imu_gyro[0] << ','
             << s.imu_gyro[1] << ','
             << s.imu_gyro[2] << ','
             << s.imu_acc[0] << ','
             << s.imu_acc[1] << ','
             << s.imu_acc[2] << '\n';
    }

    void writeCsvCell(const std::string& value) {
        const bool quote = value.find_first_of(",\"\n\r") != std::string::npos;
        if (!quote) {
            out_ << value;
            return;
        }
        out_ << '"';
        for (char ch : value) {
            if (ch == '"') {
                out_ << "\"\"";
            } else {
                out_ << ch;
            }
        }
        out_ << '"';
    }

    std::vector<std::string> names_;
    std::unique_ptr<std::array<TorqueSample, Capacity>> buffer_{
        std::make_unique<std::array<TorqueSample, Capacity>>()};
    std::atomic<std::size_t> head_{0};
    std::atomic<std::size_t> tail_{0};
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> drops_{0};
    std::thread worker_;
    std::ofstream out_;
};

class LowStateTorqueRecorder {
public:
    LowStateTorqueRecorder(h1if::RuntimeConfig cfg, Options opts)
        : cfg_(std::move(cfg)),
          opts_(std::move(opts)),
          logger_(makeJointNames(cfg_)) {}

    ~LowStateTorqueRecorder() {
        shutdown();
    }

    void init() {
        if (!logger_.start(opts_.out_path)) {
            throw std::runtime_error("cannot open output CSV: " + opts_.out_path);
        }

        unitree::robot::ChannelFactory::Instance()->Init(
            cfg_.domain_id,
            cfg_.network_interface);

        lowstate_sub_.reset(new unitree::robot::ChannelSubscriber<LowStateMsg>(kTopicLowState));
        lowstate_sub_->InitChannel(
            std::bind(&LowStateTorqueRecorder::onLowState, this, std::placeholders::_1),
            1);

        if (opts_.sample_period_s > 0.0) {
            sampler_running_.store(true, std::memory_order_release);
            sampler_ = std::thread([this]() { sampleLoop(); });
        }
    }

    bool waitForFirstState(double timeout_s) const {
        const std::uint64_t start_ns = nowNs();
        while (g_running.load(std::memory_order_acquire)) {
            if (first_state_seen_.load(std::memory_order_acquire)) {
                return true;
            }
            if (nsToSec(nowNs() - start_ns) >= timeout_s) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    }

    void run() {
        const std::uint64_t run_start_ns = nowNs();
        while (g_running.load(std::memory_order_acquire)) {
            if (opts_.duration_s > 0.0 && nsToSec(nowNs() - run_start_ns) >= opts_.duration_s) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        shutdown();
    }

    std::uint64_t frames() const {
        return frame_.load(std::memory_order_relaxed);
    }

    std::uint64_t drops() const {
        return logger_.drops();
    }

private:
    static std::vector<std::string> makeJointNames(const h1if::RuntimeConfig& cfg) {
        std::vector<std::string> names(h1if::kMaxMotors);
        for (int i = 0; i < h1if::kMaxMotors; ++i) {
            names[i] = jointName(cfg, i);
        }
        return names;
    }

    void shutdown() {
        if (shutdown_done_) {
            return;
        }
        shutdown_done_ = true;
        if (lowstate_sub_) {
            lowstate_sub_->CloseChannel();
            lowstate_sub_.reset();
        }
        sampler_running_.store(false, std::memory_order_release);
        if (sampler_.joinable()) {
            sampler_.join();
        }
        unitree::robot::ChannelFactory::Instance()->Release();
        logger_.stop();
    }

    void onLowState(const void* message) {
        const auto* msg = static_cast<const LowStateMsg*>(message);
        const std::uint64_t host_ns = nowNs();
        const std::uint64_t wall_unix_ns = wallNowNs();
        const std::uint64_t previous_ns = last_state_ns_.exchange(host_ns, std::memory_order_acq_rel);
        const std::uint64_t start_ns = start_ns_.load(std::memory_order_acquire);
        if (start_ns == 0) {
            start_ns_.store(host_ns, std::memory_order_release);
        }

        std::array<double, 4> quat{};
        std::array<double, 3> gyro{};
        std::array<double, 3> acc{};
        for (int i = 0; i < 4; ++i) {
            quat[i] = msg->imu_state().quaternion()[i];
        }
        for (int i = 0; i < 3; ++i) {
            gyro[i] = msg->imu_state().gyroscope()[i];
            acc[i] = msg->imu_state().accelerometer()[i];
        }

        const std::uint64_t rel_base_ns = start_ns == 0 ? host_ns : start_ns;
        const double lowstate_dt = previous_ns == 0 ? 0.0 : nsToSec(host_ns - previous_ns);

        if (opts_.sample_period_s > 0.0) {
            LatestLowState latest;
            latest.valid = true;
            latest.state_host_ns = host_ns;
            latest.lowstate_dt_s = lowstate_dt;
            latest.imu_quat = quat;
            latest.imu_gyro = gyro;
            latest.imu_acc = acc;
            for (int i = 0; i < h1if::kMaxMotors; ++i) {
                const auto& motor = msg->motor_state()[i];
                latest.q[i] = motor.q();
                latest.dq[i] = motor.dq();
                latest.tau_est[i] = motor.tau_est();
            }
            {
                std::lock_guard<std::mutex> lock(latest_mutex_);
                latest_ = latest;
            }
            first_state_seen_.store(true, std::memory_order_release);
            return;
        }

        const std::uint64_t previous_sample_ns =
            last_sample_ns_.exchange(host_ns, std::memory_order_acq_rel);
        const std::uint64_t frame = frame_.fetch_add(1, std::memory_order_acq_rel);
        const double sample_dt = previous_sample_ns == 0 ? 0.0 : nsToSec(host_ns - previous_sample_ns);

        for (int joint_id : opts_.joints) {
            const auto& motor = msg->motor_state()[joint_id];
            TorqueSample sample;
            sample.frame = frame;
            sample.host_ns = host_ns;
            sample.wall_unix_ns = wall_unix_ns;
            sample.t_rel_s = nsToSec(host_ns - rel_base_ns);
            sample.lowstate_dt_s = lowstate_dt;
            sample.sample_dt_s = sample_dt;
            sample.joint_id = joint_id;
            sample.q = motor.q();
            sample.dq = motor.dq();
            sample.tau_est = motor.tau_est();
            sample.imu_quat = quat;
            sample.imu_gyro = gyro;
            sample.imu_acc = acc;
            logger_.push(sample);
        }

        first_state_seen_.store(true, std::memory_order_release);
    }

    void sampleLoop() {
        const std::uint64_t period_ns = secToNs(opts_.sample_period_s);
        std::uint64_t next_ns = nowNs();
        std::uint64_t previous_sample_ns = 0;

        while (sampler_running_.load(std::memory_order_acquire) &&
               g_running.load(std::memory_order_acquire)) {
            if (!first_state_seen_.load(std::memory_order_acquire)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                next_ns = nowNs();
                continue;
            }

            const std::uint64_t current_ns = nowNs();
            if (current_ns < next_ns) {
                const auto wait_ns = next_ns - current_ns;
                std::this_thread::sleep_for(std::chrono::nanoseconds(wait_ns));
                continue;
            }

            LatestLowState snapshot;
            {
                std::lock_guard<std::mutex> lock(latest_mutex_);
                snapshot = latest_;
            }

            if (snapshot.valid) {
                const std::uint64_t host_ns = nowNs();
                const std::uint64_t wall_unix_ns = wallNowNs();
                const std::uint64_t start_ns = start_ns_.load(std::memory_order_acquire);
                const std::uint64_t rel_base_ns = start_ns == 0 ? host_ns : start_ns;
                const std::uint64_t frame = frame_.fetch_add(1, std::memory_order_acq_rel);
                const double sample_dt = previous_sample_ns == 0
                    ? 0.0
                    : nsToSec(host_ns - previous_sample_ns);
                previous_sample_ns = host_ns;

                for (int joint_id : opts_.joints) {
                    TorqueSample sample;
                    sample.frame = frame;
                    sample.host_ns = host_ns;
                    sample.wall_unix_ns = wall_unix_ns;
                    sample.t_rel_s = nsToSec(host_ns - rel_base_ns);
                    sample.lowstate_dt_s = snapshot.lowstate_dt_s;
                    sample.sample_dt_s = sample_dt;
                    sample.joint_id = joint_id;
                    sample.q = snapshot.q[joint_id];
                    sample.dq = snapshot.dq[joint_id];
                    sample.tau_est = snapshot.tau_est[joint_id];
                    sample.imu_quat = snapshot.imu_quat;
                    sample.imu_gyro = snapshot.imu_gyro;
                    sample.imu_acc = snapshot.imu_acc;
                    logger_.push(sample);
                }
            }

            next_ns += period_ns;
            const std::uint64_t after_ns = nowNs();
            while (next_ns + period_ns < after_ns) {
                next_ns += period_ns;
            }
        }
    }

    h1if::RuntimeConfig cfg_;
    Options opts_;
    TorqueCsvLogger<> logger_;
    unitree::robot::ChannelSubscriberPtr<LowStateMsg> lowstate_sub_;
    std::mutex latest_mutex_;
    LatestLowState latest_;
    std::atomic<bool> sampler_running_{false};
    std::thread sampler_;
    std::atomic<std::uint64_t> start_ns_{0};
    std::atomic<std::uint64_t> last_state_ns_{0};
    std::atomic<std::uint64_t> last_sample_ns_{0};
    std::atomic<std::uint64_t> frame_{0};
    std::atomic<bool> first_state_seen_{false};
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
        Options opts = parseOptions(cfg, argc, argv);

        std::cout << "Read-only LowState torque logger\n"
                  << "Topic: " << kTopicLowState << "\n"
                  << "Interface: " << cfg.network_interface << "\n"
                  << "Domain: " << cfg.domain_id << "\n"
                  << "Joints: " << joinJoints(opts.joints) << "\n"
                  << "Output: " << opts.out_path << "\n"
                  << "Duration: "
                  << (opts.duration_s > 0.0 ? std::to_string(opts.duration_s) + " s" : "until Ctrl+C")
                  << "\n";

        LowStateTorqueRecorder recorder(std::move(cfg), std::move(opts));
        recorder.init();
        if (!recorder.waitForFirstState(5.0)) {
            std::cerr << "No LowState received after 5 seconds. Check interface/domain and robot network.\n";
            return 2;
        }
        std::cout << "LowState received; recording.\n";
        recorder.run();
        std::cout << "Recorded frames: " << recorder.frames()
                  << ", dropped samples: " << recorder.drops() << "\n";
        return recorder.drops() == 0 ? 0 : 3;
    } catch (const std::exception& ex) {
        std::cerr << "h1_torque_logger failed: " << ex.what() << "\n";
        return 2;
    }
}
