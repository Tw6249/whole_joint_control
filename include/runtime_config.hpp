#pragma once

#include "reference_trajectory.hpp"
#include "safety.hpp"

#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace h1if {

enum class ReferenceMode {
    OpenLoop,
    ClosedLoop,
};

struct EidControllerConfig {
    int target_joint = 2;
    double kp = 260.0;
    double kd = 18.0;
    double observer_gain_q = 0.9;
    double observer_gain_dq = 1.1;
    double filter_alpha = 0.3;
    ReferenceMode reference_mode = ReferenceMode::OpenLoop;
    ReferenceSignal reference_signal = ReferenceSignal::Sine;
    double control_dt = 0.002;
    double policy_reference_dt = 0.05;
    double closed_loop_reference_tau = 0.05;
    double ref_center = 0.75;
    double ref_amplitude = 0.75;
    double ref_frequency = 0.05;
    double ref_phase = -1.5707963267948966;
    double ref_step_time = 1.0;
    double startup_ramp_duration = 4.0;
    double eid_tau_limit = 0.0;
    double eid_tau_slew_rate = 0.0;
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
    double q_min = 0.0;
    double q_max = 0.0;
    double tau_max = 0.0;
};

struct JointEidConfig {
    std::string name;
    EidControllerConfig controller;
    PlantModelConfig plant;
    bool has_plant = false;
    bool enabled = true;
};

struct RuntimeConfig {
    std::string robot = "H1";
    std::string network_interface = "enp3s0";
    int domain_id = 0;
    double control_dt = 0.002;
    double mock_duration = 5.0;
    SafetyConfig safety;
    EidControllerConfig eid_defaults;
    std::array<std::optional<JointEidConfig>, kMaxMotors> eid_controllers{};
    std::string log_path = "data/h1_mock_log.csv";
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

inline bool toBool(const std::string& value) {
    std::string token = trim(value);
    for (char& ch : token) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        if (ch == '-') {
            ch = '_';
        }
    }
    if (token == "true" || token == "1" || token == "yes" || token == "on") {
        return true;
    }
    if (token == "false" || token == "0" || token == "no" || token == "off") {
        return false;
    }
    throw std::runtime_error("boolean value must be true or false");
}

inline std::string normalizeToken(std::string value) {
    for (char& ch : value) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        if (ch == '-') {
            ch = '_';
        }
    }
    return value;
}

inline ReferenceMode parseReferenceMode(const std::string& value) {
    const std::string token = normalizeToken(trim(value));
    if (token == "open_loop" || token == "openloop" || token == "open") {
        return ReferenceMode::OpenLoop;
    }
    if (token == "closed_loop" || token == "closedloop" || token == "closed") {
        return ReferenceMode::ClosedLoop;
    }
    throw std::runtime_error("reference_mode must be open_loop or closed_loop");
}

inline ReferenceSignal parseReferenceSignal(const std::string& value) {
    const std::string token = normalizeToken(trim(value));
    if (token == "sine" || token == "sin" || token == "smooth_sine" || token == "smoothsine") {
        return ReferenceSignal::Sine;
    }
    if (token == "step") {
        return ReferenceSignal::Step;
    }
    throw std::runtime_error("reference_signal must be sine or step");
}

inline void parseEidControllerField(EidControllerConfig& cfg,
                                    const std::string& key,
                                    const std::string& value) {
    if (key == "kp") cfg.kp = toDouble(value);
    else if (key == "kd") cfg.kd = toDouble(value);
    else if (key == "observer_gain_q") cfg.observer_gain_q = toDouble(value);
    else if (key == "observer_gain_dq") cfg.observer_gain_dq = toDouble(value);
    else if (key == "filter_alpha") cfg.filter_alpha = toDouble(value);
    else if (key == "reference_mode") cfg.reference_mode = parseReferenceMode(value);
    else if (key == "reference_signal") cfg.reference_signal = parseReferenceSignal(value);
    else if (key == "policy_reference_dt") cfg.policy_reference_dt = toDouble(value);
    else if (key == "closed_loop_reference_tau") cfg.closed_loop_reference_tau = toDouble(value);
    else if (key == "ref_center") cfg.ref_center = toDouble(value);
    else if (key == "ref_amplitude") cfg.ref_amplitude = toDouble(value);
    else if (key == "ref_frequency") cfg.ref_frequency = toDouble(value);
    else if (key == "ref_phase") cfg.ref_phase = toDouble(value);
    else if (key == "ref_step_time") cfg.ref_step_time = toDouble(value);
    else if (key == "startup_ramp_duration") cfg.startup_ramp_duration = toDouble(value);
    else if (key == "eid_tau_limit") cfg.eid_tau_limit = toDouble(value);
    else if (key == "eid_tau_slew_rate") cfg.eid_tau_slew_rate = toDouble(value);
    else if (key == "torque_safe_kp") cfg.torque_safe_kp = toDouble(value);
    else if (key == "torque_safe_kd") cfg.torque_safe_kd = toDouble(value);
    else if (key == "inverse_q_weight") cfg.inverse_q_weight = toDouble(value);
    else if (key == "inverse_dq_weight") cfg.inverse_dq_weight = toDouble(value);
}

inline void parsePlantField(PlantModelConfig& plant,
                            const std::string& key,
                            const std::string& value) {
    if (key == "Jeff") plant.Jeff = toDouble(value);
    else if (key == "b") plant.b = toDouble(value);
    else if (key == "gravityA") plant.gravityA = toDouble(value);
    else if (key == "gravityB") plant.gravityB = toDouble(value);
    else if (key == "tau0") plant.tau0 = toDouble(value);
    else if (key == "q_min") plant.q_min = toDouble(value);
    else if (key == "q_max") plant.q_max = toDouble(value);
    else if (key == "tau_max") plant.tau_max = toDouble(value);
}

inline std::vector<int> activeEidJoints(const RuntimeConfig& cfg) {
    std::vector<int> joints;
    for (int i = 0; i < kMaxMotors; ++i) {
        if (cfg.eid_controllers[i].has_value() && cfg.eid_controllers[i]->enabled) {
            joints.push_back(i);
        }
    }
    return joints;
}

inline int primaryEidJoint(const RuntimeConfig& cfg) {
    if (cfg.eid_controllers[2].has_value() && cfg.eid_controllers[2]->enabled) {
        return 2;
    }
    const auto joints = activeEidJoints(cfg);
    if (joints.empty()) {
        throw std::runtime_error("eid_controllers must contain at least one active joint");
    }
    return joints.front();
}

inline void validateEidControllerConfig(const EidControllerConfig& c, const std::string& prefix) {
    const auto finite = [](double v) {
        return std::isfinite(v);
    };

    if (c.target_joint < 0 || c.target_joint >= kMaxMotors) {
        throw std::runtime_error(prefix + ".target_joint is out of range");
    }
    if (c.filter_alpha < 0.0 || c.filter_alpha > 1.0 || !finite(c.filter_alpha)) {
        throw std::runtime_error(prefix + ".filter_alpha must be in [0, 1]");
    }
    if (c.inverse_q_weight < 0.0 || c.inverse_dq_weight < 0.0) {
        throw std::runtime_error(prefix + " inverse weights must be non-negative");
    }
    if (c.policy_reference_dt <= 0.0 || !finite(c.policy_reference_dt)) {
        throw std::runtime_error(prefix + ".policy_reference_dt must be positive and finite");
    }
    if (c.ref_step_time < 0.0 || !finite(c.ref_step_time)) {
        throw std::runtime_error(prefix + ".ref_step_time must be non-negative and finite");
    }
    if (c.closed_loop_reference_tau <= 0.0 || !finite(c.closed_loop_reference_tau)) {
        throw std::runtime_error(prefix + ".closed_loop_reference_tau must be positive and finite");
    }
    if (c.startup_ramp_duration < 0.0 ||
        c.eid_tau_limit <= 0.0 ||
        c.eid_tau_slew_rate < 0.0 ||
        !finite(c.startup_ramp_duration) ||
        !finite(c.eid_tau_limit) ||
        !finite(c.eid_tau_slew_rate)) {
        throw std::runtime_error(prefix + " eid_tau_limit must be > 0 (set in YAML), eid_tau_slew_rate must be >= 0");
    }
    if (!finite(c.kp) || !finite(c.kd) ||
        !finite(c.observer_gain_q) || !finite(c.observer_gain_dq) ||
        !finite(c.ref_center) || !finite(c.ref_amplitude) ||
        !finite(c.ref_frequency) || !finite(c.ref_phase) ||
        !finite(c.torque_safe_kp) || !finite(c.torque_safe_kd)) {
        throw std::runtime_error(prefix + " contains non-finite controller value");
    }
}

inline void validatePlantConfig(const PlantModelConfig& plant, const std::string& prefix) {
    if (plant.Jeff <= 0.0 || plant.tau_max <= 0.0 || plant.q_min >= plant.q_max) {
        throw std::runtime_error(prefix + " has invalid inertia, torque, or position limits");
    }
    if (!std::isfinite(plant.Jeff) || !std::isfinite(plant.b) ||
        !std::isfinite(plant.gravityA) || !std::isfinite(plant.gravityB) ||
        !std::isfinite(plant.tau0) || !std::isfinite(plant.q_min) ||
        !std::isfinite(plant.q_max) || !std::isfinite(plant.tau_max)) {
        throw std::runtime_error(prefix + " contains non-finite plant value");
    }
}

inline void validateRuntimeConfig(const RuntimeConfig& cfg) {
    const auto finite = [](double v) {
        return std::isfinite(v);
    };

    if (cfg.control_dt <= 0.0 || !finite(cfg.control_dt)) {
        throw std::runtime_error("control_dt must be positive and finite");
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

    const auto active = activeEidJoints(cfg);
    if (active.empty()) {
        throw std::runtime_error("eid_controllers must contain at least one active joint");
    }

    for (int joint_id : active) {
        if (joint_id == 9) {
            throw std::runtime_error("eid_controllers must not include NotUsedJoint index 9");
        }

        const auto& jc = *cfg.eid_controllers[joint_id];
        if (!jc.has_plant) {
            throw std::runtime_error("eid_controllers." + std::to_string(joint_id) + ".plant is required");
        }

        const std::string prefix = "eid_controllers." + std::to_string(joint_id);
        if (jc.controller.target_joint != joint_id) {
            throw std::runtime_error(prefix + ".target_joint must match its map key");
        }
        validateEidControllerConfig(jc.controller, prefix);
        validatePlantConfig(jc.plant, prefix + ".plant");

        const auto& lim = cfg.safety.limit[joint_id];
        if (std::abs(jc.controller.eid_tau_limit) > static_cast<double>(lim.tau_max) + 1.0e-6) {
            throw std::runtime_error(prefix + ".eid_tau_limit exceeds joint_limits tau_max");
        }
    }
}

inline std::string resolveLogPath(const RuntimeConfig& cfg) {
    namespace fs = std::filesystem;
    const auto now = std::chrono::system_clock::now();
    const auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm;
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);

    fs::path p(cfg.log_path);
    return (p.parent_path() / buf / p.filename()).string();
}

inline RuntimeConfig loadRuntimeConfig(const std::string& path) {
    RuntimeConfig cfg;
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("Cannot open config file: " + path);
    }

    std::string section;
    int current_limit_joint = -1;
    int current_eid_joint = -1;
    bool current_eid_plant = false;
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
            if (key == "controller" || key == "plant") {
                throw std::runtime_error("legacy controller/plant YAML sections are no longer supported");
            }
            section = key;
            current_limit_joint = -1;
            current_eid_joint = -1;
            current_eid_plant = false;
            continue;
        }

        if (indent == 2 && section == "joint_limits" && value.empty()) {
            current_limit_joint = toInt(key);
            continue;
        }

        if (indent == 2 && section == "eid_controllers" && value.empty()) {
            current_eid_joint = toInt(key);
            if (current_eid_joint < 0 || current_eid_joint >= kMaxMotors) {
                throw std::runtime_error("eid_controllers joint id is out of range");
            }
            JointEidConfig joint_cfg;
            joint_cfg.controller = cfg.eid_defaults;
            joint_cfg.controller.control_dt = cfg.control_dt;
            joint_cfg.controller.target_joint = current_eid_joint;
            cfg.eid_controllers[current_eid_joint] = joint_cfg;
            current_eid_plant = false;
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
        } else if (section == "eid_defaults") {
            parseEidControllerField(cfg.eid_defaults, key, value);
        } else if (section == "eid_controllers" &&
                   current_eid_joint >= 0 &&
                   cfg.eid_controllers[current_eid_joint].has_value()) {
            auto& joint_cfg = *cfg.eid_controllers[current_eid_joint];
            if (indent == 4 && key == "plant" && value.empty()) {
                current_eid_plant = true;
                joint_cfg.has_plant = true;
                continue;
            }
            if (indent == 4) {
                current_eid_plant = false;
                if (key == "name") {
                    joint_cfg.name = value;
                } else if (key == "enabled") {
                    joint_cfg.enabled = toBool(value);
                } else {
                    parseEidControllerField(joint_cfg.controller, key, value);
                }
            } else if (indent == 6 && current_eid_plant) {
                parsePlantField(joint_cfg.plant, key, value);
            }
        } else if (section == "joint_limits" &&
                   current_limit_joint >= 0 &&
                   current_limit_joint < kMaxMotors) {
            auto& lim = cfg.safety.limit[current_limit_joint];
            if (key == "q_min") lim.q_min = static_cast<float>(toDouble(value));
            else if (key == "q_max") lim.q_max = static_cast<float>(toDouble(value));
            else if (key == "dq_max") lim.dq_max = static_cast<float>(toDouble(value));
            else if (key == "tau_max") lim.tau_max = static_cast<float>(toDouble(value));
            else if (key == "kp_max") lim.kp_max = static_cast<float>(toDouble(value));
            else if (key == "kd_max") lim.kd_max = static_cast<float>(toDouble(value));
        }
    }

    cfg.eid_defaults.control_dt = cfg.control_dt;
    for (int i = 0; i < kMaxMotors; ++i) {
        if (cfg.eid_controllers[i].has_value()) {
            cfg.eid_controllers[i]->controller.control_dt = cfg.control_dt;
            cfg.eid_controllers[i]->controller.target_joint = i;
        }
    }
    validateRuntimeConfig(cfg);
    return cfg;
}

}  // namespace h1if
