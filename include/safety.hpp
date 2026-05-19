#pragma once

#include "controller_interface.hpp"

#include <algorithm>
#include <cmath>

namespace h1if {

enum SafetyFlags : std::uint32_t {
    kSafetyLowStateTimeout = 1u << 0,
    kSafetyNonFiniteCommand = 1u << 1,
    kSafetyCommandSaturated = 1u << 2,
    kSafetyInvalidState = 1u << 3
};

// Limits for uncontrolled joints default to zero torque/velocity/gain.
// Controlled joints must have their limits set via YAML joint_limits section.
struct JointLimit {
    float q_min = -3.14f;
    float q_max = 3.14f;
    float dq_max = 0.0f;
    float tau_max = 0.0f;
    float kp_max = 0.0f;
    float kd_max = 0.0f;
};

struct SafetyConfig {
    std::array<JointLimit, kMaxMotors> limit{};
    float hold_kp = 10.0f;
    float hold_kd = 1.0f;

    // Timeout (seconds) for LowState messages. If no valid state is received
    // within this window, the controller trips into safe-hold.
    double lowstate_timeout = 0.05;

    // Hard trip: measured joint speed (rad/s) above this value immediately
    // triggers safe-hold. Set to a value larger than any physically possible
    // speed to disable this check.
    double measured_speed_trip = 8.0;

    // Hard trip: measured joint position jump (rad) between two consecutive
    // cycles above this value triggers safe-hold. Guards against sensor
    // glitches or communication corruption.
    double measured_jump_trip = 0.10;

    // Hard trip: control-loop interval (seconds) above this value triggers
    // safe-hold. Protects against scheduler jitter or thread starvation.
    double max_control_dt = 0.010;
};

inline std::uint8_t h1MotorMode(int joint_id) {
    switch (joint_id) {
        case 12:
        case 13:
        case 14:
        case 15:
        case 16:
        case 17:
        case 18:
        case 19:
            return 0x01;
        default:
            return 0x0A;
    }
}

inline float clampf(float x, float lo, float hi) {
    return std::max(lo, std::min(x, hi));
}

inline bool finiteState(const RobotState& state) {
    for (const auto& joint : state.joint) {
        if (!std::isfinite(joint.q) || !std::isfinite(joint.dq) || !std::isfinite(joint.tau_est)) {
            return false;
        }
    }
    return true;
}

inline bool finiteCommand(const JointCommand& c) {
    return std::isfinite(c.q) &&
           std::isfinite(c.dq) &&
           std::isfinite(c.kp) &&
           std::isfinite(c.kd) &&
           std::isfinite(c.tau);
}

inline void fillSafeHoldCommand(const RobotState& state, RobotCommand& cmd, const SafetyConfig& cfg) {
    for (int i = 0; i < kMaxMotors; ++i) {
        cmd.joint[i].mode = h1MotorMode(i);
        cmd.joint[i].q = std::isfinite(state.joint[i].q) ? static_cast<float>(state.joint[i].q) : 0.0f;
        cmd.joint[i].dq = 0.0f;
        cmd.joint[i].kp = cfg.hold_kp;
        cmd.joint[i].kd = cfg.hold_kd;
        cmd.joint[i].tau = 0.0f;
        cmd.joint[i].enable = true;
    }
}

inline void applySafety(const RobotState& state, RobotCommand& cmd, ControllerDebug& debug, const SafetyConfig& cfg) {
    bool unsafe = false;

    if (!state.state_valid) {
        unsafe = true;
        debug.flags |= kSafetyInvalidState;
    }

    if (state.lowstate_age > cfg.lowstate_timeout) {
        unsafe = true;
        debug.flags |= kSafetyLowStateTimeout;
    }

    if (!finiteState(state)) {
        unsafe = true;
        debug.flags |= kSafetyInvalidState;
    }

    for (int i = 0; i < kMaxMotors; ++i) {
        auto& c = cmd.joint[i];
        const auto& lim = cfg.limit[i];

        if (!finiteCommand(c)) {
            unsafe = true;
            debug.flags |= kSafetyNonFiniteCommand;
            debug.joint[i].flags |= kSafetyNonFiniteCommand;
            continue;
        }

        const JointCommand before = c;
        c.mode = h1MotorMode(i);
        c.q = clampf(c.q, lim.q_min, lim.q_max);
        c.dq = clampf(c.dq, -lim.dq_max, lim.dq_max);
        c.kp = clampf(c.kp, 0.0f, lim.kp_max);
        c.kd = clampf(c.kd, 0.0f, lim.kd_max);
        c.tau = clampf(c.tau, -lim.tau_max, lim.tau_max);

        if (before.q != c.q || before.dq != c.dq || before.kp != c.kp ||
            before.kd != c.kd || before.tau != c.tau) {
            debug.flags |= kSafetyCommandSaturated;
            debug.joint[i].flags |= kSafetyCommandSaturated;
        }
    }

    if (unsafe) {
        fillSafeHoldCommand(state, cmd, cfg);
    }
}

}  // namespace h1if
