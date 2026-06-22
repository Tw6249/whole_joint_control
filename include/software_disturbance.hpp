#pragma once

#include "controller_interface.hpp"
#include "safety.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace h1if {

enum class SoftwareDisturbanceWaveform {
    Rectangular,
    SmoothRect,
};

struct SoftwareDisturbanceConfig {
    bool enabled = false;
    std::vector<int> joints;
    std::vector<double> torques;
    double start_s = 0.0;
    double end_s = 0.0;
    double ramp_s = 0.0;
    SoftwareDisturbanceWaveform waveform = SoftwareDisturbanceWaveform::SmoothRect;
};

struct SoftwareDisturbanceTelemetry {
    std::array<double, kMaxMotors> tau_controller{};
    std::array<double, kMaxMotors> tau_disturbance{};
    std::array<double, kMaxMotors> tau_before_limit{};
    std::array<double, kMaxMotors> tau_sent{};
    std::array<double, kMaxMotors> tau_limit{};
    std::array<std::uint8_t, kMaxMotors> saturation_flag{};
};

inline double commandTotalTorque(const RobotState& state, const JointCommand& command, int joint_id) {
    return static_cast<double>(command.kp) *
               (static_cast<double>(command.q) - state.joint[joint_id].q) +
           static_cast<double>(command.kd) *
               (static_cast<double>(command.dq) - state.joint[joint_id].dq) +
           static_cast<double>(command.tau);
}

inline double commandPdTerm(const RobotState& state, const JointCommand& command, int joint_id) {
    return static_cast<double>(command.kp) *
               (static_cast<double>(command.q) - state.joint[joint_id].q) +
           static_cast<double>(command.kd) *
               (static_cast<double>(command.dq) - state.joint[joint_id].dq);
}

inline double torqueLimitForJoint(const SafetyConfig& safety, int joint_id) {
    const double limit = static_cast<double>(safety.limit[joint_id].tau_max);
    if (!std::isfinite(limit) || limit < 0.0) {
        return 0.0;
    }
    return limit;
}

inline double clampToTorqueLimit(double tau, double limit) {
    if (limit <= 0.0) {
        return 0.0;
    }
    return std::max(-limit, std::min(tau, limit));
}

inline double softwareDisturbanceWindow(double t, const SoftwareDisturbanceConfig& cfg) {
    if (!cfg.enabled || cfg.end_s <= cfg.start_s || t < cfg.start_s || t > cfg.end_s) {
        return 0.0;
    }
    if (cfg.waveform == SoftwareDisturbanceWaveform::Rectangular || cfg.ramp_s <= 0.0) {
        return 1.0;
    }

    const double duration = cfg.end_s - cfg.start_s;
    const double ramp = std::min(cfg.ramp_s, 0.5 * duration);
    if (ramp <= 0.0) {
        return 1.0;
    }

    constexpr double pi = 3.14159265358979323846;
    if (t < cfg.start_s + ramp) {
        const double s = std::max(0.0, std::min((t - cfg.start_s) / ramp, 1.0));
        return 0.5 * (1.0 - std::cos(pi * s));
    }
    if (t > cfg.end_s - ramp) {
        const double s = std::max(0.0, std::min((cfg.end_s - t) / ramp, 1.0));
        return 0.5 * (1.0 - std::cos(pi * s));
    }
    return 1.0;
}

inline std::array<double, kMaxMotors> disturbanceTorqueByJoint(const SoftwareDisturbanceConfig& cfg) {
    std::array<double, kMaxMotors> out{};
    const std::size_t count = std::min(cfg.joints.size(), cfg.torques.size());
    for (std::size_t i = 0; i < count; ++i) {
        const int joint_id = cfg.joints[i];
        if (joint_id >= 0 && joint_id < kMaxMotors) {
            out[static_cast<std::size_t>(joint_id)] = cfg.torques[i];
        }
    }
    return out;
}

inline void initializeSoftwareDisturbanceTelemetry(
    const RobotState& state,
    const RobotCommand& command,
    const SafetyConfig& safety,
    SoftwareDisturbanceTelemetry& telemetry) {
    for (int i = 0; i < kMaxMotors; ++i) {
        const double total = commandTotalTorque(state, command.joint[i], i);
        telemetry.tau_controller[i] = total;
        telemetry.tau_disturbance[i] = 0.0;
        telemetry.tau_before_limit[i] = total;
        telemetry.tau_sent[i] = total;
        telemetry.tau_limit[i] = torqueLimitForJoint(safety, i);
        telemetry.saturation_flag[i] = 0;
    }
}

inline void applySoftwareDisturbance(
    double trial_time_s,
    const RobotState& state,
    const SoftwareDisturbanceConfig& cfg,
    const SafetyConfig& safety,
    RobotCommand& command,
    SoftwareDisturbanceTelemetry& telemetry) {
    initializeSoftwareDisturbanceTelemetry(state, command, safety, telemetry);
    if (!cfg.enabled || !state.state_valid) {
        return;
    }

    const double alpha = softwareDisturbanceWindow(trial_time_s, cfg);
    const auto tau_by_joint = disturbanceTorqueByJoint(cfg);
    for (int joint_id : cfg.joints) {
        if (joint_id < 0 || joint_id >= kMaxMotors) {
            continue;
        }
        const int j = joint_id;
        const double tau_controller = telemetry.tau_controller[j];
        const double tau_disturbance = tau_by_joint[static_cast<std::size_t>(j)] * alpha;
        const double tau_before_limit = tau_controller + tau_disturbance;
        const double limit = telemetry.tau_limit[j];
        const double tau_sent = clampToTorqueLimit(tau_before_limit, limit);
        const double pd_term = commandPdTerm(state, command.joint[j], j);

        command.joint[j].tau = static_cast<float>(tau_sent - pd_term);
        telemetry.tau_disturbance[j] = tau_disturbance;
        telemetry.tau_before_limit[j] = tau_before_limit;
        telemetry.tau_sent[j] = tau_sent;
        telemetry.saturation_flag[j] =
            std::abs(tau_sent - tau_before_limit) > 1.0e-9 ? 1 : 0;
    }
}

inline void finalizeSoftwareDisturbanceTelemetry(
    const RobotState& state,
    const RobotCommand& command,
    SoftwareDisturbanceTelemetry& telemetry) {
    for (int i = 0; i < kMaxMotors; ++i) {
        const double final_total = commandTotalTorque(state, command.joint[i], i);
        telemetry.tau_sent[i] = final_total;
        if (std::abs(final_total - telemetry.tau_before_limit[i]) > 1.0e-6) {
            telemetry.saturation_flag[i] = 1;
        }
    }
}

}  // namespace h1if
