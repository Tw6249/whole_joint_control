#pragma once

#include "reference_trajectory.hpp"
#include "safety.hpp"

#include <algorithm>
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

enum class ControllerKind {
    Eid,
    PositionPd,
};

struct ControllerParams {
    int target_joint = 2;
    double kp = 260.0;
    double kd = 18.0;
    double observer_gain_q = 0.9;
    double observer_gain_dq = 1.1;
    double ku_q = 0.0;
    double ku_dq = 0.0;
    double filter_alpha = 0.3;
    PolicyInterpolation policy_interpolation = PolicyInterpolation::OpenLoop;
    PolicySource policy_source = PolicySource::Sine;
    double control_dt = 0.002;
    double policy_dt = 0.05;
    double policy_center = 0.75;
    double policy_amplitude = 0.75;
    double policy_frequency_hz = 0.05;
    double policy_phase_rad = -1.5707963267948966;
    double policy_step_time_s = 1.0;
    double policy_max_velocity = 0.0;
    int policy_reference_points = 4;
    double startup_blend_duration_s = 4.0;
    double tau_limit = 0.0;
    double tau_slew_rate = 0.0;
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

struct JointControllerConfig {
    std::string name;
    ControllerParams controller;
    PlantModelConfig plant;
    bool has_plant = false;
    bool enabled = true;
};

struct ControllerRuntimeConfig {
    ControllerKind kind = ControllerKind::Eid;
    ControllerParams defaults;
    std::array<std::optional<JointControllerConfig>, kMaxMotors> joints{};
};

struct RuntimeConfig {
    std::string robot = "H1";
    std::string network_interface = "enp3s0";
    int domain_id = 0;
    double control_dt = 0.002;
    double mock_duration = 5.0;
    SafetyConfig safety;
    ControllerRuntimeConfig controller;
    std::string log_path = "data/h1_mock_log.csv";
    std::string config_path;
    std::string experiment_id;
    std::string condition_id;
    std::string repeat_id;
    std::string disturbance_target;
    std::string disturbance_method;
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

inline std::vector<int> parseIntList(std::string value) {
    value = trim(value);
    if (value.size() >= 2 && value.front() == '[' && value.back() == ']') {
        value = value.substr(1, value.size() - 2);
    }

    std::vector<int> result;
    std::stringstream ss(value);
    std::string item;
    while (std::getline(ss, item, ',')) {
        item = trim(item);
        if (!item.empty()) {
            result.push_back(toInt(item));
        }
    }
    return result;
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

inline ControllerKind parseControllerKind(const std::string& value) {
    const std::string token = normalizeToken(trim(value));
    if (token == "eid") {
        return ControllerKind::Eid;
    }
    if (token == "position_pd" || token == "positionpd" || token == "pd") {
        return ControllerKind::PositionPd;
    }
    throw std::runtime_error("controller.kind must be eid or position_pd");
}

inline std::string controllerKindName(ControllerKind kind) {
    switch (kind) {
        case ControllerKind::Eid:
            return "eid";
        case ControllerKind::PositionPd:
            return "position_pd";
    }
    return "unknown";
}

inline PolicyInterpolation parsePolicyInterpolation(const std::string& value) {
    const std::string token = normalizeToken(trim(value));
    if (token == "open_loop" || token == "openloop" || token == "open") {
        return PolicyInterpolation::OpenLoop;
    }
    if (token == "closed_loop" || token == "closedloop" || token == "closed") {
        return PolicyInterpolation::ClosedLoop;
    }
    if (token == "preview_mpc" || token == "previewmpc" || token == "mpc") {
        return PolicyInterpolation::PreviewMpc;
    }
    throw std::runtime_error("policy_interpolation must be open_loop, closed_loop, or preview_mpc");
}

inline PolicySource parsePolicySource(const std::string& value) {
    const std::string token = normalizeToken(trim(value));
    if (token == "hold") {
        return PolicySource::Hold;
    }
    if (token == "sine" || token == "sin" || token == "smooth_sine" || token == "smoothsine") {
        return PolicySource::Sine;
    }
    if (token == "step") {
        return PolicySource::Step;
    }
    throw std::runtime_error("policy_source must be hold, sine, or step");
}

inline void parseControllerParamField(ControllerParams& cfg,
                                      const std::string& key,
                                      const std::string& value) {
    static const std::array<const char*, 8> kRemovedFields{
        "reference_mode",
        "reference_signal",
        "policy_reference_dt",
        "closed_loop_reference_tau",
        "ref_center",
        "ref_amplitude",
        "ref_frequency",
        "ref_phase",
    };
    for (const char* removed : kRemovedFields) {
        if (key == removed) {
            throw std::runtime_error("removed policy field '" + key + "'; use policy_* names");
        }
    }
    if (key == "ref_step_time" || key == "startup_ramp_duration") {
        throw std::runtime_error("removed policy field '" + key + "'; use policy_step_time_s/startup_blend_duration_s");
    }
    if (key == "eid_tau_limit") {
        throw std::runtime_error("removed controller field 'eid_tau_limit'; use tau_limit");
    }
    if (key == "eid_tau_slew_rate") {
        throw std::runtime_error("removed controller field 'eid_tau_slew_rate'; use tau_slew_rate");
    }
    if (key == "kp") cfg.kp = toDouble(value);
    else if (key == "kd") cfg.kd = toDouble(value);
    else if (key == "observer_gain_q") cfg.observer_gain_q = toDouble(value);
    else if (key == "observer_gain_dq") cfg.observer_gain_dq = toDouble(value);
    else if (key == "ku_q") cfg.ku_q = toDouble(value);
    else if (key == "ku_dq") cfg.ku_dq = toDouble(value);
    else if (key == "filter_alpha") cfg.filter_alpha = toDouble(value);
    else if (key == "policy_interpolation") cfg.policy_interpolation = parsePolicyInterpolation(value);
    else if (key == "policy_source") cfg.policy_source = parsePolicySource(value);
    else if (key == "policy_dt") cfg.policy_dt = toDouble(value);
    else if (key == "policy_center") cfg.policy_center = toDouble(value);
    else if (key == "policy_amplitude") cfg.policy_amplitude = toDouble(value);
    else if (key == "policy_frequency_hz") cfg.policy_frequency_hz = toDouble(value);
    else if (key == "policy_phase_rad") cfg.policy_phase_rad = toDouble(value);
    else if (key == "policy_step_time_s") cfg.policy_step_time_s = toDouble(value);
    else if (key == "policy_reference_points") cfg.policy_reference_points = toInt(value);
    else if (key == "startup_blend_duration_s") cfg.startup_blend_duration_s = toDouble(value);
    else if (key == "tau_limit") cfg.tau_limit = toDouble(value);
    else if (key == "tau_slew_rate") cfg.tau_slew_rate = toDouble(value);
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

inline std::vector<int> activeControllerJoints(const RuntimeConfig& cfg) {
    std::vector<int> joints;
    for (int i = 0; i < kMaxMotors; ++i) {
        if (cfg.controller.joints[i].has_value() && cfg.controller.joints[i]->enabled) {
            joints.push_back(i);
        }
    }
    return joints;
}

inline int primaryControllerJoint(const RuntimeConfig& cfg) {
    if (cfg.controller.joints[2].has_value() && cfg.controller.joints[2]->enabled) {
        return 2;
    }
    const auto joints = activeControllerJoints(cfg);
    if (joints.empty()) {
        throw std::runtime_error("controller.joints must contain at least one active joint");
    }
    return joints.front();
}

inline PolicyReferenceConfig makePolicyReferenceConfig(const ControllerParams& cfg,
                                                       const PlantModelConfig* model = nullptr) {
    PolicyReferenceConfig ref;
    ref.interpolation = cfg.policy_interpolation;
    ref.source = cfg.policy_source;
    ref.policy_dt = cfg.policy_dt;
    ref.step_time_s = cfg.policy_step_time_s;
    ref.max_velocity = cfg.policy_max_velocity;
    ref.reference_points = cfg.policy_reference_points;

    if (model != nullptr && model->q_max > model->q_min) {
        const double q_min = model->q_min;
        const double q_max = model->q_max;
        const double default_center = 0.5 * (q_min + q_max);
        const double default_amplitude = 0.5 * (q_max - q_min);
        ref.center = std::isfinite(cfg.policy_center)
                         ? std::max(q_min, std::min(cfg.policy_center, q_max))
                         : default_center;

        const double requested_amplitude =
            std::isfinite(cfg.policy_amplitude) ? cfg.policy_amplitude : default_amplitude;
        if (cfg.policy_source == PolicySource::Step) {
            ref.amplitude = std::max(q_min - ref.center, std::min(requested_amplitude, q_max - ref.center));
        } else {
            const double max_amplitude = std::min(ref.center - q_min, q_max - ref.center);
            ref.amplitude = std::max(0.0, std::min(std::abs(requested_amplitude), max_amplitude));
        }
    } else {
        ref.center = cfg.policy_center;
        ref.amplitude = cfg.policy_amplitude;
    }

    ref.frequency_hz = std::isfinite(cfg.policy_frequency_hz) && cfg.policy_frequency_hz > 0.0
                            ? cfg.policy_frequency_hz
                            : 0.05;
    ref.phase_rad = std::isfinite(cfg.policy_phase_rad) ? cfg.policy_phase_rad : -1.57079632679489661923;
    return ref;
}

inline void validateCommonControllerConfig(const ControllerParams& c, const std::string& prefix) {
    const auto finite = [](double v) {
        return std::isfinite(v);
    };

    if (c.target_joint < 0 || c.target_joint >= kMaxMotors) {
        throw std::runtime_error(prefix + ".target_joint is out of range");
    }
    if (c.kp < 0.0 || c.kd < 0.0 || !finite(c.kp) || !finite(c.kd)) {
        throw std::runtime_error(prefix + ".kp/.kd must be non-negative and finite");
    }
    if (c.policy_dt <= 0.0 || !finite(c.policy_dt)) {
        throw std::runtime_error(prefix + ".policy_dt must be positive and finite");
    }
    if (c.policy_step_time_s < 0.0 || !finite(c.policy_step_time_s)) {
        throw std::runtime_error(prefix + ".policy_step_time_s must be non-negative and finite");
    }
    if (!finite(c.policy_center) || !finite(c.policy_amplitude) ||
        !finite(c.policy_frequency_hz) || !finite(c.policy_phase_rad)) {
        throw std::runtime_error(prefix + " contains non-finite policy value");
    }
    if (c.policy_reference_points != 1 &&
        c.policy_reference_points != 2 &&
        c.policy_reference_points != 3 &&
        c.policy_reference_points != 4) {
        throw std::runtime_error(prefix + ".policy_reference_points must be 1, 2, 3, or 4");
    }
}

inline void validateEidControllerConfig(const ControllerParams& c, const std::string& prefix) {
    const auto finite = [](double v) {
        return std::isfinite(v);
    };

    validateCommonControllerConfig(c, prefix);
    if (c.filter_alpha < 0.0 || c.filter_alpha > 1.0 || !finite(c.filter_alpha)) {
        throw std::runtime_error(prefix + ".filter_alpha must be in [0, 1]");
    }
    if (c.inverse_q_weight < 0.0 || c.inverse_dq_weight < 0.0) {
        throw std::runtime_error(prefix + " inverse weights must be non-negative");
    }
    if (c.startup_blend_duration_s < 0.0 ||
        c.tau_limit <= 0.0 ||
        c.tau_slew_rate < 0.0 ||
        !finite(c.startup_blend_duration_s) ||
        !finite(c.tau_limit) ||
        !finite(c.tau_slew_rate)) {
        throw std::runtime_error(prefix + " tau_limit must be > 0 (set in YAML), tau_slew_rate must be >= 0");
    }
    if (!finite(c.observer_gain_q) || !finite(c.observer_gain_dq) ||
        !finite(c.ku_q) || !finite(c.ku_dq) ||
        !finite(c.torque_safe_kp) || !finite(c.torque_safe_kd)) {
        throw std::runtime_error(prefix + " contains non-finite EID value");
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

    const auto active = activeControllerJoints(cfg);
    if (active.empty()) {
        throw std::runtime_error("controller.joints must contain at least one active joint");
    }

    for (int joint_id : active) {
        if (joint_id == 9) {
            throw std::runtime_error("controller.joints must not include NotUsedJoint index 9");
        }

        const auto& jc = *cfg.controller.joints[joint_id];
        const std::string prefix = "controller.joints." + std::to_string(joint_id);
        if (jc.controller.target_joint != joint_id) {
            throw std::runtime_error(prefix + ".target_joint must match its map key");
        }

        const auto& lim = cfg.safety.limit[joint_id];
        if (cfg.controller.kind == ControllerKind::Eid) {
            if (!jc.has_plant) {
                throw std::runtime_error(prefix + ".plant is required for controller.kind=eid");
            }
            validateEidControllerConfig(jc.controller, prefix);
            validatePlantConfig(jc.plant, prefix + ".plant");

            if (std::abs(jc.controller.tau_limit) > static_cast<double>(lim.tau_max) + 1.0e-6) {
                throw std::runtime_error(prefix + ".tau_limit exceeds joint_limits tau_max");
            }
        } else {
            validateCommonControllerConfig(jc.controller, prefix);
            if (jc.has_plant) {
                validatePlantConfig(jc.plant, prefix + ".plant");
            }
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

    struct ParamAssignment {
        std::string key;
        std::string value;
    };

    struct ControllerGroup {
        std::string name;
        std::vector<int> joints;
        std::vector<ParamAssignment> params;
    };

    std::string section;
    std::string controller_scope;
    int current_limit_joint = -1;
    int current_controller_joint = -1;
    int current_controller_group = -1;
    bool current_joint_plant = false;
    std::vector<ControllerGroup> controller_groups;
    std::array<std::vector<ParamAssignment>, kMaxMotors> joint_param_overrides{};
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
            if (key == "eid_defaults" || key == "eid_controllers") {
                throw std::runtime_error("legacy " + key + " YAML section is no longer supported; use controller.defaults/controller.joints");
            }
            if (key == "plant") {
                throw std::runtime_error("legacy plant YAML section is no longer supported; use controller.joints.<id>.plant");
            }
            section = key;
            controller_scope.clear();
            current_limit_joint = -1;
            current_controller_joint = -1;
            current_controller_group = -1;
            current_joint_plant = false;
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

        if (indent == 2 && section == "joint_limits" && value.empty()) {
            current_limit_joint = toInt(key);
            continue;
        }

        if (section == "safe_hold") {
            if (key == "kp") cfg.safety.hold_kp = static_cast<float>(toDouble(value));
            else if (key == "kd") cfg.safety.hold_kd = static_cast<float>(toDouble(value));
            else if (key == "lowstate_timeout") cfg.safety.lowstate_timeout = toDouble(value);
            else if (key == "measured_speed_trip") cfg.safety.measured_speed_trip = toDouble(value);
            else if (key == "measured_jump_trip") cfg.safety.measured_jump_trip = toDouble(value);
            else if (key == "max_control_dt") cfg.safety.max_control_dt = toDouble(value);
        } else if (section == "experiment") {
            if (key == "id") cfg.experiment_id = value;
            else if (key == "condition") cfg.condition_id = value;
            else if (key == "repeat") cfg.repeat_id = value;
            else if (key == "disturbance_target") cfg.disturbance_target = value;
            else if (key == "disturbance_method") cfg.disturbance_method = value;
        } else if (section == "controller") {
            if (indent == 2) {
                current_controller_joint = -1;
                current_controller_group = -1;
                current_joint_plant = false;
                if (key == "kind") {
                    cfg.controller.kind = parseControllerKind(value);
                } else if (key == "defaults" && value.empty()) {
                    controller_scope = "defaults";
                } else if (key == "groups" && value.empty()) {
                    controller_scope = "groups";
                } else if (key == "joints" && value.empty()) {
                    controller_scope = "joints";
                }
                continue;
            }

            if (indent == 4 && controller_scope == "defaults") {
                parseControllerParamField(cfg.controller.defaults, key, value);
            } else if (indent == 4 && controller_scope == "groups" && value.empty()) {
                ControllerGroup group;
                group.name = key;
                controller_groups.push_back(group);
                current_controller_group = static_cast<int>(controller_groups.size()) - 1;
            } else if (indent == 6 && controller_scope == "groups" && current_controller_group >= 0) {
                auto& group = controller_groups[static_cast<std::size_t>(current_controller_group)];
                if (key == "joints") {
                    group.joints = parseIntList(value);
                    for (int joint_id : group.joints) {
                        if (joint_id < 0 || joint_id >= kMaxMotors) {
                            throw std::runtime_error("controller.groups." + group.name + ".joints contains out-of-range joint id");
                        }
                    }
                } else {
                    group.params.push_back({key, value});
                }
            } else if (indent == 4 && controller_scope == "joints" && value.empty()) {
                current_controller_joint = toInt(key);
                if (current_controller_joint < 0 || current_controller_joint >= kMaxMotors) {
                    throw std::runtime_error("controller.joints joint id is out of range");
                }
                JointControllerConfig joint_cfg;
                joint_cfg.controller = cfg.controller.defaults;
                joint_cfg.controller.control_dt = cfg.control_dt;
                joint_cfg.controller.target_joint = current_controller_joint;
                cfg.controller.joints[current_controller_joint] = joint_cfg;
                current_joint_plant = false;
            } else if (indent == 6 && controller_scope == "joints" &&
                       current_controller_joint >= 0 &&
                       cfg.controller.joints[current_controller_joint].has_value()) {
                auto& joint_cfg = *cfg.controller.joints[current_controller_joint];
                if (key == "plant" && value.empty()) {
                    current_joint_plant = true;
                    joint_cfg.has_plant = true;
                    continue;
                }
                current_joint_plant = false;
                if (key == "name") {
                    joint_cfg.name = value;
                } else if (key == "enabled") {
                    joint_cfg.enabled = toBool(value);
                } else {
                    joint_param_overrides[static_cast<std::size_t>(current_controller_joint)].push_back({key, value});
                    parseControllerParamField(joint_cfg.controller, key, value);
                }
            } else if (indent == 8 && controller_scope == "joints" && current_joint_plant &&
                       current_controller_joint >= 0 &&
                       cfg.controller.joints[current_controller_joint].has_value()) {
                parsePlantField(cfg.controller.joints[current_controller_joint]->plant, key, value);
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

    cfg.config_path = path;
    cfg.controller.defaults.control_dt = cfg.control_dt;
    for (int i = 0; i < kMaxMotors; ++i) {
        if (cfg.controller.joints[i].has_value()) {
            ControllerParams merged = cfg.controller.defaults;
            for (const auto& group : controller_groups) {
                if (std::find(group.joints.begin(), group.joints.end(), i) == group.joints.end()) {
                    continue;
                }
                for (const auto& param : group.params) {
                    parseControllerParamField(merged, param.key, param.value);
                }
            }
            for (const auto& param : joint_param_overrides[static_cast<std::size_t>(i)]) {
                parseControllerParamField(merged, param.key, param.value);
            }
            merged.control_dt = cfg.control_dt;
            merged.target_joint = i;
            merged.policy_max_velocity = static_cast<double>(cfg.safety.limit[i].dq_max);
            cfg.controller.joints[i]->controller = merged;
        }
    }
    validateRuntimeConfig(cfg);
    return cfg;
}

}  // namespace h1if
