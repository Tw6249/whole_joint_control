#pragma once

#include "safety.hpp"

#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace h1if {

struct EidControllerConfig {
    int target_joint = 2;
    double kp = 260.0;
    double kd = 18.0;
    double observer_gain_q = 0.9;
    double observer_gain_dq = 1.1;
    double filter_alpha = 0.3;
    double control_dt = 0.002;
    double policy_reference_dt = 0.05;
    double ref_center = 0.75;
    double ref_amplitude = 0.75;
    double ref_frequency = 0.05;
    double ref_phase = -1.5707963267948966;
    double startup_ramp_duration = 4.0;
    double eid_tau_limit = 12.0;
    double eid_tau_slew_rate = 60.0;
    double torque_safe_kp = 0.0;
    double torque_safe_kd = 0.0;
    double inverse_q_weight = 0.0;
    double inverse_dq_weight = 0.0;
};

struct PlantModelConfig {
    double Jeff = 0.238;
    double b = 1.0;
    double gravityA = 4.2835;
    double gravityB = 0.0;
    double tau0 = -0.2711;
    double q_min = -0.26;
    double q_max = 2.05;
    double tau_max = 80.0;
};

struct RuntimeConfig {
    std::string robot = "H1";
    std::string network_interface = "enp3s0";
    int domain_id = 0;
    double control_dt = 0.002;
    double mock_duration = 5.0;
    SafetyConfig safety;
    EidControllerConfig controller;
    PlantModelConfig plant;
    std::string log_path = "h1_mock_log.csv";
};

inline std::string trim(std::string s) {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

inline std::string stripComment(const std::string& s) {
    const auto pos = s.find('#');
    return pos == std::string::npos ? s : s.substr(0, pos);
}

inline bool parseKeyValue(const std::string& line, std::string& key, std::string& value) {
    const auto pos = line.find(':');
    if (pos == std::string::npos) {
        return false;
    }
    key = trim(line.substr(0, pos));
    value = trim(line.substr(pos + 1));
    if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
        value = value.substr(1, value.size() - 2);
    }
    return !key.empty();
}

inline int indentation(const std::string& line) {
    int count = 0;
    for (char ch : line) {
        if (ch == ' ') {
            ++count;
        } else {
            break;
        }
    }
    return count;
}

inline double toDouble(const std::string& value) {
    return std::stod(value);
}

inline int toInt(const std::string& value) {
    return std::stoi(value, nullptr, 0);
}

inline void validateRuntimeConfig(const RuntimeConfig& cfg) {
    const auto finite = [](double v) {
        return std::isfinite(v);
    };

    if (cfg.control_dt <= 0.0 || !finite(cfg.control_dt)) {
        throw std::runtime_error("control_dt must be positive and finite");
    }
    if (cfg.controller.target_joint < 0 || cfg.controller.target_joint >= kMaxMotors) {
        throw std::runtime_error("controller.target_joint is out of range");
    }
    if (cfg.controller.filter_alpha < 0.0 || cfg.controller.filter_alpha > 1.0 ||
        !finite(cfg.controller.filter_alpha)) {
        throw std::runtime_error("controller.filter_alpha must be in [0, 1]");
    }
    if (cfg.controller.inverse_q_weight < 0.0 || cfg.controller.inverse_dq_weight < 0.0) {
        throw std::runtime_error("controller inverse weights must be non-negative");
    }
    if (cfg.controller.policy_reference_dt <= 0.0 || !finite(cfg.controller.policy_reference_dt)) {
        throw std::runtime_error("controller.policy_reference_dt must be positive and finite");
    }
    if (cfg.controller.startup_ramp_duration < 0.0 ||
        cfg.controller.eid_tau_limit <= 0.0 ||
        cfg.controller.eid_tau_slew_rate <= 0.0 ||
        !finite(cfg.controller.startup_ramp_duration) ||
        !finite(cfg.controller.eid_tau_limit) ||
        !finite(cfg.controller.eid_tau_slew_rate)) {
        throw std::runtime_error("controller EID ramp/torque limits must be positive and finite");
    }
    if (cfg.plant.Jeff <= 0.0 || cfg.plant.tau_max <= 0.0 || cfg.plant.q_min >= cfg.plant.q_max) {
        throw std::runtime_error("plant model has invalid inertia, torque, or position limits");
    }

    for (int i = 0; i < kMaxMotors; ++i) {
        const auto& lim = cfg.safety.limit[i];
        if (!std::isfinite(lim.q_min) || !std::isfinite(lim.q_max) ||
            !std::isfinite(lim.dq_max) || !std::isfinite(lim.tau_max) ||
            !std::isfinite(lim.kp_max) || !std::isfinite(lim.kd_max)) {
            throw std::runtime_error("joint limit contains non-finite value");
        }
        if (lim.q_min > lim.q_max || lim.dq_max < 0.0f || lim.tau_max < 0.0f ||
            lim.kp_max < 0.0f || lim.kd_max < 0.0f) {
            throw std::runtime_error("joint limit has invalid range");
        }
    }
}

inline RuntimeConfig loadRuntimeConfig(const std::string& path) {
    RuntimeConfig cfg;
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("Cannot open config file: " + path);
    }

    std::string section;
    int current_joint = -1;
    std::string line;

    while (std::getline(in, line)) {
        line = stripComment(line);
        if (trim(line).empty()) {
            continue;
        }

        const int indent = indentation(line);
        std::string key;
        std::string value;
        if (!parseKeyValue(line, key, value)) {
            continue;
        }

        if (indent == 0 && value.empty()) {
            section = key;
            current_joint = -1;
            continue;
        }

        if (indent == 2 && section == "joint_limits" && value.empty()) {
            current_joint = toInt(key);
            continue;
        }

        if (indent == 0) {
            if (key == "robot") cfg.robot = value;
            else if (key == "domain_id") cfg.domain_id = toInt(value);
            else if (key == "network_interface") cfg.network_interface = value;
            else if (key == "control_dt") cfg.control_dt = toDouble(value);
            else if (key == "mock_duration") cfg.mock_duration = toDouble(value);
            else if (key == "log_path") cfg.log_path = value;
            continue;
        }

        if (section == "safe_hold") {
            if (key == "kp") cfg.safety.hold_kp = static_cast<float>(toDouble(value));
            else if (key == "kd") cfg.safety.hold_kd = static_cast<float>(toDouble(value));
            else if (key == "lowstate_timeout") cfg.safety.lowstate_timeout = toDouble(value);
        } else if (section == "controller") {
            if (key == "target_joint") cfg.controller.target_joint = toInt(value);
            else if (key == "kp") cfg.controller.kp = toDouble(value);
            else if (key == "kd") cfg.controller.kd = toDouble(value);
            else if (key == "observer_gain_q") cfg.controller.observer_gain_q = toDouble(value);
            else if (key == "observer_gain_dq") cfg.controller.observer_gain_dq = toDouble(value);
            else if (key == "filter_alpha") cfg.controller.filter_alpha = toDouble(value);
            else if (key == "policy_reference_dt") cfg.controller.policy_reference_dt = toDouble(value);
            else if (key == "ref_center") cfg.controller.ref_center = toDouble(value);
            else if (key == "ref_amplitude") cfg.controller.ref_amplitude = toDouble(value);
            else if (key == "ref_frequency") cfg.controller.ref_frequency = toDouble(value);
            else if (key == "ref_phase") cfg.controller.ref_phase = toDouble(value);
            else if (key == "startup_ramp_duration") cfg.controller.startup_ramp_duration = toDouble(value);
            else if (key == "eid_tau_limit") cfg.controller.eid_tau_limit = toDouble(value);
            else if (key == "eid_tau_slew_rate") cfg.controller.eid_tau_slew_rate = toDouble(value);
            else if (key == "torque_safe_kp") cfg.controller.torque_safe_kp = toDouble(value);
            else if (key == "torque_safe_kd") cfg.controller.torque_safe_kd = toDouble(value);
            else if (key == "inverse_q_weight") cfg.controller.inverse_q_weight = toDouble(value);
            else if (key == "inverse_dq_weight") cfg.controller.inverse_dq_weight = toDouble(value);
        } else if (section == "plant") {
            if (key == "Jeff") cfg.plant.Jeff = toDouble(value);
            else if (key == "b") cfg.plant.b = toDouble(value);
            else if (key == "gravityA") cfg.plant.gravityA = toDouble(value);
            else if (key == "gravityB") cfg.plant.gravityB = toDouble(value);
            else if (key == "tau0") cfg.plant.tau0 = toDouble(value);
            else if (key == "q_min") cfg.plant.q_min = toDouble(value);
            else if (key == "q_max") cfg.plant.q_max = toDouble(value);
            else if (key == "tau_max") cfg.plant.tau_max = toDouble(value);
        } else if (section == "joint_limits" && current_joint >= 0 && current_joint < kMaxMotors) {
            auto& lim = cfg.safety.limit[current_joint];
            if (key == "q_min") lim.q_min = static_cast<float>(toDouble(value));
            else if (key == "q_max") lim.q_max = static_cast<float>(toDouble(value));
            else if (key == "dq_max") lim.dq_max = static_cast<float>(toDouble(value));
            else if (key == "tau_max") lim.tau_max = static_cast<float>(toDouble(value));
            else if (key == "kp_max") lim.kp_max = static_cast<float>(toDouble(value));
            else if (key == "kd_max") lim.kd_max = static_cast<float>(toDouble(value));
        }
    }

    cfg.controller.control_dt = cfg.control_dt;
    validateRuntimeConfig(cfg);
    return cfg;
}

}  // namespace h1if
