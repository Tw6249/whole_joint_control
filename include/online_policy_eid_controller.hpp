#pragma once

#include "eid_controller.hpp"
#include "runtime_config.hpp"

#include <torch/script.h>
#include <torch/torch.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace h1if {

constexpr std::uint32_t kOnlinePolicyWorkerFault = 1u << 16;

struct LstmDeployConfig {
    double control_dt = 0.10;
    double ang_vel_scale = 0.25;
    double dof_pos_scale = 1.0;
    double dof_vel_scale = 0.05;
    double action_scale = 0.25;
    std::array<double, 3> cmd_scale{2.0, 2.0, 0.25};
    int num_obs = 41;
    int num_actions = 10;
    std::vector<int> leg_joint2motor_idx;
    std::vector<double> default_angles;
};

struct OnlinePolicyOutput {
    std::array<double, kMaxMotors> q_ref{};
    std::array<double, kMaxMotors> q_ref_raw{};
    std::array<double, kMaxMotors> action{};
    std::array<bool, kMaxMotors> active{};
    bool updated = false;
    std::uint64_t update_count = 0;
    double policy_time_s = 0.0;
};

class LstmPolicyRuntime {
public:
    LstmPolicyRuntime(const OnlinePolicyConfig& policy_cfg,
                      const RuntimeConfig& runtime_cfg)
        : policy_cfg_(policy_cfg),
          deploy_cfg_(loadDeployConfig(resolvePath(policy_cfg.deploy_yaml, runtime_cfg.config_path))),
          model_path_(resolvePath(policy_cfg.model_path, runtime_cfg.config_path)) {
        if (deploy_cfg_.num_obs != 41 || deploy_cfg_.num_actions != 10) {
            throw std::runtime_error("online policy expects num_obs=41 and num_actions=10");
        }
        if (deploy_cfg_.leg_joint2motor_idx.size() != static_cast<std::size_t>(deploy_cfg_.num_actions) ||
            deploy_cfg_.default_angles.size() != static_cast<std::size_t>(deploy_cfg_.num_actions)) {
            throw std::runtime_error("online policy h1.yaml has inconsistent joint mapping/default angles");
        }

        torch::NoGradGuard no_grad;
        model_ = torch::jit::load(model_path_);
        model_.eval();

        last_action_.assign(static_cast<std::size_t>(deploy_cfg_.num_actions), 0.0f);
        resetMemory();
    }

    void resetRuntimeState() {
        if (!last_action_.empty()) {
            std::fill(last_action_.begin(), last_action_.end(), 0.0f);
        }
        last_output_.active.fill(false);
        last_output_.q_ref.fill(0.0);
        last_output_.q_ref_raw.fill(0.0);
        last_output_.action.fill(0.0);
        prev_q_ref_.fill(0.0);
        phase_ = 0.0;
        next_policy_time_s_ = 0.0;
        update_count_ = 0;
        initialized_ = false;
        have_prev_q_ref_ = false;
    }

    const LstmDeployConfig& deployConfig() const {
        return deploy_cfg_;
    }

    void resetMemory() {
        try {
            model_.get_method("reset_memory")({});
        } catch (const c10::Error&) {
        }
        resetRuntimeState();
    }

    OnlinePolicyOutput evaluateIfDue(const RobotState& state,
                                     const std::vector<int>& active_joints) {
        if (!initialized_) {
            initialized_ = true;
            t0_ = state.t;
        }
        const double t = state.t - t0_;

        if (update_count_ > 0 && t + 1.0e-12 < next_policy_time_s_) {
            last_output_.updated = false;
            last_output_.policy_time_s = std::max(0.0, t);
            return last_output_;
        }

        OnlinePolicyOutput output = evaluate(state, active_joints, std::max(0.0, t));
        output.updated = true;
        output.update_count = ++update_count_;
        output.policy_time_s = std::max(0.0, t);
        next_policy_time_s_ = std::max(0.0, t) + policyStep();
        last_output_ = output;
        return last_output_;
    }

    void primeReferenceFromState(const RobotState& state,
                                 const std::vector<int>& active_joints) {
        prev_q_ref_.fill(0.0);
        for (int joint_id : active_joints) {
            prev_q_ref_[static_cast<std::size_t>(joint_id)] = state.joint[joint_id].q;
        }
        have_prev_q_ref_ = true;
        prev_policy_t_ = 0.0;
    }

    OnlinePolicyOutput evaluateNow(const RobotState& state,
                                   const std::vector<int>& active_joints) {
        if (!initialized_) {
            initialized_ = true;
            t0_ = state.t;
        }
        OnlinePolicyOutput output = evaluate(state, active_joints, 0.0);
        output.updated = true;
        output.update_count = ++update_count_;
        output.policy_time_s = 0.0;
        next_policy_time_s_ = policyStep();
        last_output_ = output;
        return output;
    }

    OnlinePolicyOutput evaluateScheduled(const RobotState& state,
                                         const std::vector<int>& active_joints,
                                         double policy_time_s) {
        initialized_ = true;
        OnlinePolicyOutput output = evaluate(state, active_joints, std::max(0.0, policy_time_s));
        output.updated = true;
        output.update_count = ++update_count_;
        output.policy_time_s = std::max(0.0, policy_time_s);
        last_output_ = output;
        return last_output_;
    }

private:
    static std::string yamlValue(const std::string& path, const std::string& key) {
        std::ifstream in(path);
        if (!in) {
            throw std::runtime_error("cannot open policy deploy yaml: " + path);
        }
        std::string line;
        while (std::getline(in, line)) {
            line = stripComment(line);
            if (trim(line).empty()) {
                continue;
            }
            const auto pos = line.find(':');
            if (pos == std::string::npos) {
                continue;
            }
            const std::string found = trim(line.substr(0, pos));
            if (found != key) {
                continue;
            }

            std::string value = trim(line.substr(pos + 1));
            if (!value.empty() && value.front() == '[' && value.find(']') == std::string::npos) {
                std::string extra;
                while (std::getline(in, extra)) {
                    value += " " + trim(stripComment(extra));
                    if (extra.find(']') != std::string::npos) {
                        break;
                    }
                }
            }
            return value;
        }
        throw std::runtime_error("missing policy deploy yaml key: " + key);
    }

    static LstmDeployConfig loadDeployConfig(const std::string& path) {
        LstmDeployConfig cfg;
        cfg.control_dt = toDouble(yamlValue(path, "control_dt"));
        cfg.ang_vel_scale = toDouble(yamlValue(path, "ang_vel_scale"));
        cfg.dof_pos_scale = toDouble(yamlValue(path, "dof_pos_scale"));
        cfg.dof_vel_scale = toDouble(yamlValue(path, "dof_vel_scale"));
        cfg.action_scale = toDouble(yamlValue(path, "action_scale"));
        cfg.num_obs = toInt(yamlValue(path, "num_obs"));
        cfg.num_actions = toInt(yamlValue(path, "num_actions"));
        cfg.leg_joint2motor_idx = parseIntList(yamlValue(path, "leg_joint2motor_idx"));
        cfg.default_angles = parseDoubleList(yamlValue(path, "default_angles"));
        const std::vector<double> cmd_scale = parseDoubleList(yamlValue(path, "cmd_scale"));
        if (cmd_scale.size() == 3) {
            cfg.cmd_scale = {cmd_scale[0], cmd_scale[1], cmd_scale[2]};
        }
        return cfg;
    }

    static std::string resolvePath(const std::string& path, const std::string& config_path) {
        namespace fs = std::filesystem;
        fs::path p(path);
        if (p.is_absolute()) {
            return p.string();
        }
        if (fs::exists(p)) {
            return p.string();
        }
        const fs::path cfg_parent = fs::path(config_path).parent_path();
        const fs::path from_config = cfg_parent / p;
        if (fs::exists(from_config)) {
            return from_config.string();
        }
        return p.string();
    }

    double policyStep() const {
        return policy_cfg_.policy_dt > 0.0 ? policy_cfg_.policy_dt : deploy_cfg_.control_dt;
    }

    std::array<double, 3> projectedGravity(const RobotState& state) const {
        const double w0 = state.imu.quat[0];
        const double x0 = state.imu.quat[1];
        const double y0 = state.imu.quat[2];
        const double z0 = state.imu.quat[3];

        const double w = w0;
        const double x = -x0;
        const double y = -y0;
        const double z = -z0;
        const double x2 = x * x;
        const double y2 = y * y;
        const double z2 = z * z;
        const double w2 = w * w;
        const double xz = x * z;
        const double yz = y * z;
        const double wx = w * x;
        const double wy = w * y;

        return {
            -2.0 * (xz + wy),
            -2.0 * (yz - wx),
            -(w2 - x2 - y2 + z2),
        };
    }

    std::vector<float> makeObservation(const RobotState& state) {
        std::vector<float> obs(static_cast<std::size_t>(deploy_cfg_.num_obs), 0.0f);
        const auto gravity = projectedGravity(state);

        for (int i = 0; i < 3; ++i) {
            obs[static_cast<std::size_t>(i)] =
                static_cast<float>(state.imu.gyro[static_cast<std::size_t>(i)] * deploy_cfg_.ang_vel_scale);
            obs[static_cast<std::size_t>(3 + i)] = static_cast<float>(gravity[static_cast<std::size_t>(i)]);
            obs[static_cast<std::size_t>(6 + i)] =
                static_cast<float>(policy_cfg_.command[static_cast<std::size_t>(i)] *
                                   deploy_cfg_.cmd_scale[static_cast<std::size_t>(i)]);
        }

        for (int i = 0; i < deploy_cfg_.num_actions; ++i) {
            const int motor = deploy_cfg_.leg_joint2motor_idx[static_cast<std::size_t>(i)];
            if (motor < 0 || motor >= kMaxMotors) {
                throw std::runtime_error("leg_joint2motor_idx contains out-of-range motor id");
            }
            const double q = state.joint[motor].q;
            const double dq = state.joint[motor].dq;
            const double q_default = deploy_cfg_.default_angles[static_cast<std::size_t>(i)];
            obs[static_cast<std::size_t>(9 + i)] =
                static_cast<float>((q - q_default) * deploy_cfg_.dof_pos_scale);
            obs[static_cast<std::size_t>(19 + i)] =
                static_cast<float>(dq * deploy_cfg_.dof_vel_scale);
            obs[static_cast<std::size_t>(29 + i)] = last_action_[static_cast<std::size_t>(i)];
        }

        phase_ += policyStep() / policy_cfg_.phase_period_s;
        while (phase_ > 1.0) {
            phase_ -= 1.0;
        }
        obs[39] = static_cast<float>(phase_);
        obs[40] = static_cast<float>(1.0 - phase_);
        return obs;
    }

    OnlinePolicyOutput evaluate(const RobotState& state,
                                const std::vector<int>& active_joints,
                                double t) {
        torch::NoGradGuard no_grad;
        std::vector<float> obs = makeObservation(state);
        torch::Tensor obs_tensor = torch::from_blob(
                                       obs.data(),
                                       {1, deploy_cfg_.num_obs},
                                       torch::TensorOptions().dtype(torch::kFloat32))
                                       .clone();
        torch::Tensor output = model_.forward({obs_tensor}).toTensor().to(torch::kCPU).contiguous().view({-1});
        if (output.numel() < deploy_cfg_.num_actions) {
            throw std::runtime_error("online policy output has fewer than num_actions values");
        }
        const float* action_data = output.data_ptr<float>();

        OnlinePolicyOutput out;
        out.active.fill(false);
        out.q_ref.fill(0.0);
        out.q_ref_raw.fill(0.0);
        out.action.fill(0.0);

        for (int i = 0; i < deploy_cfg_.num_actions; ++i) {
            const int motor = deploy_cfg_.leg_joint2motor_idx[static_cast<std::size_t>(i)];
            if (std::find(active_joints.begin(), active_joints.end(), motor) == active_joints.end()) {
                continue;
            }

            double action = static_cast<double>(action_data[i]);
            if (!std::isfinite(action)) {
                throw std::runtime_error("online policy produced non-finite action");
            }
            action = clamp(action, -policy_cfg_.max_abs_action, policy_cfg_.max_abs_action);
            last_action_[static_cast<std::size_t>(i)] = static_cast<float>(action);

            const double q_default = deploy_cfg_.default_angles[static_cast<std::size_t>(i)];
            const double q_raw = q_default + deploy_cfg_.action_scale * action;
            out.action[static_cast<std::size_t>(motor)] = action;
            out.q_ref_raw[static_cast<std::size_t>(motor)] = q_raw;
            out.q_ref[static_cast<std::size_t>(motor)] = q_raw;
            out.active[static_cast<std::size_t>(motor)] = true;
        }

        applyReferenceSlew(out, t);
        return out;
    }

    void applyReferenceSlew(OnlinePolicyOutput& out, double t) {
        if (!have_prev_q_ref_ || policy_cfg_.q_ref_slew_rate <= 0.0) {
            for (int i = 0; i < kMaxMotors; ++i) {
                prev_q_ref_[static_cast<std::size_t>(i)] = out.q_ref[static_cast<std::size_t>(i)];
            }
            have_prev_q_ref_ = true;
            prev_policy_t_ = t;
            return;
        }

        const double dt = std::max(0.0, t - prev_policy_t_);
        const double max_delta = policy_cfg_.q_ref_slew_rate * dt;
        for (int i = 0; i < kMaxMotors; ++i) {
            if (!out.active[static_cast<std::size_t>(i)]) {
                continue;
            }
            const double prev = prev_q_ref_[static_cast<std::size_t>(i)];
            const double limited = clamp(out.q_ref[static_cast<std::size_t>(i)], prev - max_delta, prev + max_delta);
            out.q_ref[static_cast<std::size_t>(i)] = limited;
            prev_q_ref_[static_cast<std::size_t>(i)] = limited;
        }
        prev_policy_t_ = t;
    }

    static double clamp(double x, double lo, double hi) {
        return std::max(lo, std::min(x, hi));
    }

    OnlinePolicyConfig policy_cfg_;
    LstmDeployConfig deploy_cfg_;
    std::string model_path_;
    torch::jit::script::Module model_;
    std::vector<float> last_action_;
    OnlinePolicyOutput last_output_;
    std::array<double, kMaxMotors> prev_q_ref_{};
    bool have_prev_q_ref_ = false;
    bool initialized_ = false;
    double t0_ = 0.0;
    double phase_ = 0.0;
    double next_policy_time_s_ = 0.0;
    double prev_policy_t_ = 0.0;
    std::uint64_t update_count_ = 0;
};

class OnlinePolicyEidMultiJointController final : public IController {
public:
    explicit OnlinePolicyEidMultiJointController(RuntimeConfig cfg)
        : cfg_(std::move(cfg)),
          safety_(cfg_.safety),
          policy_(cfg_.online_policy, cfg_) {
        active_joints_ = activeControllerJoints(cfg_);
        for (int joint_id : active_joints_) {
            controllers_.emplace_back(*cfg_.controller.joints[joint_id]);
        }
    }

    ~OnlinePolicyEidMultiJointController() override {
        stopPolicyWorker();
    }

    std::string name() const override {
        std::ostringstream out;
        out << "OnlinePolicyEidMultiJointController(" << controllers_.size()
            << " joints, policy_dt=" << cfg_.online_policy.policy_dt << " s, async)";
        return out.str();
    }

    void reset(const RobotState& state) override {
        stopPolicyWorker();
        policy_.resetRuntimeState();
        policy_.primeReferenceFromState(state, active_joints_);
        for (auto& controller : controllers_) {
            controller.reset(state);
        }

        OnlinePolicyOutput hold;
        hold.active.fill(false);
        hold.q_ref.fill(0.0);
        hold.q_ref_raw.fill(0.0);
        hold.action.fill(0.0);
        hold.updated = true;
        hold.update_count = 0;
        hold.policy_time_s = 0.0;
        for (int joint_id : active_joints_) {
            hold.active[static_cast<std::size_t>(joint_id)] = true;
            hold.q_ref[static_cast<std::size_t>(joint_id)] = state.joint[joint_id].q;
            hold.q_ref_raw[static_cast<std::size_t>(joint_id)] = state.joint[joint_id].q;
        }

        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            current_policy_ = hold;
            policy_fault_.clear();
        }
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_state_ = state;
            latest_state_valid_ = state.state_valid;
        }
        last_seen_update_count_ = 0;
        startPolicyWorker();
    }

    void step(const RobotState& state, RobotCommand& command, ControllerDebug& debug) override {
        fillSafeHoldCommand(state, command, safety_);
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_state_ = state;
            latest_state_valid_ = state.state_valid;
        }

        OnlinePolicyOutput policy;
        std::string fault;
        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            policy = current_policy_;
            fault = policy_fault_;
        }
        if (!fault.empty()) {
            debug.flags |= kOnlinePolicyWorkerFault;
        }
        policy.updated = policy.update_count != last_seen_update_count_;

        for (auto& controller : controllers_) {
            const int joint_id = controller.jointId();
            if (!policy.active[static_cast<std::size_t>(joint_id)]) {
                debug.flags |= kOnlinePolicyWorkerFault;
                continue;
            }
            const double q_policy = policy.q_ref[static_cast<std::size_t>(joint_id)];
            const std::array<double, 3> preview{q_policy, q_policy, q_policy};
            controller.stepJointWithPreviewTargets(state, command, debug, preview);

            auto& jd = debug.joint[joint_id].data;
            jd[33] = policy.action[static_cast<std::size_t>(joint_id)];
            jd[34] = policy.q_ref_raw[static_cast<std::size_t>(joint_id)];
            jd[35] = q_policy;
            jd[36] = policy.updated ? 1.0 : 0.0;
            jd[37] = static_cast<double>(policy.update_count);
            jd[38] = policy.policy_time_s;
            jd[39] = cfg_.online_policy.policy_dt;
        }
        last_seen_update_count_ = policy.update_count;
    }

private:
    void startPolicyWorker() {
        worker_running_.store(true, std::memory_order_release);
        policy_worker_ = std::thread([this]() { policyWorkerLoop(); });
    }

    void stopPolicyWorker() {
        worker_running_.store(false, std::memory_order_release);
        if (policy_worker_.joinable()) {
            policy_worker_.join();
        }
    }

    void policyWorkerLoop() {
        using Clock = std::chrono::steady_clock;
        const auto period = std::chrono::duration<double>(cfg_.online_policy.policy_dt);
        auto next = Clock::now() + period;
        double policy_time_s = 0.0;

        while (worker_running_.load(std::memory_order_acquire)) {
            std::this_thread::sleep_until(next);
            if (!worker_running_.load(std::memory_order_acquire)) {
                break;
            }

            RobotState state;
            bool valid = false;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                state = latest_state_;
                valid = latest_state_valid_;
            }

            if (valid) {
                try {
                    policy_time_s += cfg_.online_policy.policy_dt;
                    const OnlinePolicyOutput output =
                        policy_.evaluateScheduled(state, active_joints_, policy_time_s);
                    {
                        std::lock_guard<std::mutex> lock(output_mutex_);
                        current_policy_ = output;
                        policy_fault_.clear();
                    }
                } catch (const std::exception& ex) {
                    std::lock_guard<std::mutex> lock(output_mutex_);
                    policy_fault_ = ex.what();
                } catch (...) {
                    std::lock_guard<std::mutex> lock(output_mutex_);
                    policy_fault_ = "unknown online policy worker failure";
                }
            }

            next += period;
            const auto now = Clock::now();
            if (next <= now) {
                next = now + period;
            }
        }
    }

    RuntimeConfig cfg_;
    SafetyConfig safety_;
    LstmPolicyRuntime policy_;
    std::vector<int> active_joints_;
    std::vector<EidJointController> controllers_;
    std::thread policy_worker_;
    std::atomic<bool> worker_running_{false};
    std::mutex state_mutex_;
    RobotState latest_state_;
    bool latest_state_valid_ = false;
    std::mutex output_mutex_;
    OnlinePolicyOutput current_policy_;
    std::string policy_fault_;
    std::uint64_t last_seen_update_count_ = 0;
};

}  // namespace h1if
