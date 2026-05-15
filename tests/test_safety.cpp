#include "eid_controller.hpp"
#include "runtime_config.hpp"
#include "safety.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>

namespace {

h1if::RobotState validState() {
    h1if::RobotState state;
    state.state_valid = true;
    state.lowstate_age = 0.001;
    for (int i = 0; i < h1if::kMaxMotors; ++i) {
        state.joint[i].q = 0.5;
        state.joint[i].dq = 0.0;
        state.joint[i].tau_est = 0.0;
    }
    return state;
}

std::filesystem::path writeTempConfig(const std::string& name, const std::string& text) {
    const auto path = std::filesystem::temp_directory_path() / name;
    std::ofstream out(path);
    out << text;
    return path;
}

}  // namespace

int main() {
    h1if::SafetyConfig cfg;
    cfg.limit[2].q_min = -0.2f;
    cfg.limit[2].q_max = 2.0f;
    cfg.limit[2].dq_max = 4.0f;
    cfg.limit[2].tau_max = 80.0f;
    cfg.limit[2].kp_max = 100.0f;
    cfg.limit[2].kd_max = 10.0f;

    {
        auto state = validState();
        h1if::RobotCommand cmd;
        h1if::ControllerDebug debug;
        h1if::fillSafeHoldCommand(state, cmd, cfg);
        cmd.joint[2].q = 10.0f;
        cmd.joint[2].dq = 20.0f;
        cmd.joint[2].kp = 200.0f;
        cmd.joint[2].kd = 20.0f;
        cmd.joint[2].tau = 120.0f;
        h1if::applySafety(state, cmd, debug, cfg);

        assert(cmd.joint[2].q == cfg.limit[2].q_max);
        assert(cmd.joint[2].dq == cfg.limit[2].dq_max);
        assert(cmd.joint[2].kp == cfg.limit[2].kp_max);
        assert(cmd.joint[2].kd == cfg.limit[2].kd_max);
        assert(cmd.joint[2].tau == cfg.limit[2].tau_max);
        assert((debug.flags & h1if::kSafetyCommandSaturated) != 0);
    }

    {
        auto state = validState();
        h1if::RobotCommand cmd;
        h1if::ControllerDebug debug;
        h1if::fillSafeHoldCommand(state, cmd, cfg);
        cmd.joint[2].tau = std::numeric_limits<float>::quiet_NaN();
        h1if::applySafety(state, cmd, debug, cfg);

        assert((debug.flags & h1if::kSafetyNonFiniteCommand) != 0);
        assert(cmd.joint[2].q == static_cast<float>(state.joint[2].q));
        assert(cmd.joint[2].tau == 0.0f);
    }

    {
        auto state = validState();
        state.lowstate_age = 1.0;
        h1if::RobotCommand cmd;
        h1if::ControllerDebug debug;
        h1if::fillSafeHoldCommand(state, cmd, cfg);
        cmd.joint[2].tau = 20.0f;
        h1if::applySafety(state, cmd, debug, cfg);

        assert((debug.flags & h1if::kSafetyLowStateTimeout) != 0);
        assert(cmd.joint[2].tau == 0.0f);
    }

    assert(h1if::h1MotorMode(2) == 0x0A);
    assert(h1if::h1MotorMode(7) == 0x0A);
    assert(h1if::h1MotorMode(9) == 0x0A);
    assert(h1if::h1MotorMode(10) == 0x0A);
    assert(h1if::h1MotorMode(11) == 0x0A);
    assert(h1if::h1MotorMode(12) == 0x01);

    assert(h1if::parseReferenceMode("open_loop") == h1if::ReferenceMode::OpenLoop);
    assert(h1if::parseReferenceMode("closed-loop") == h1if::ReferenceMode::ClosedLoop);
    assert(h1if::parseReferenceSignal("sine") == h1if::ReferenceSignal::Sine);
    assert(h1if::parseReferenceSignal("step") == h1if::ReferenceSignal::Step);

    {
        h1if::ReferenceTrajectoryConfig step_cfg;
        step_cfg.signal = h1if::ReferenceSignal::Step;
        step_cfg.policy_dt = 0.05;
        step_cfg.center = 0.5;
        step_cfg.amplitude = -0.2;
        step_cfg.step_time = 0.10;
        h1if::SmoothSineReferenceTrajectory step_ref(step_cfg);
        const auto before = step_ref.sample(0.02, 0.002);
        const auto after = step_ref.sample(0.20, 0.002);
        assert(std::abs(before.now.q - 0.5) < 1.0e-9);
        assert(std::abs(after.now.q - 0.3) < 1.0e-9);
    }

    const std::string valid_multi_config = R"YAML(
robot: H1
domain_id: 0
network_interface: lo
control_dt: 0.002
mock_duration: 0.02
log_path: data/test_multi.csv

safe_hold:
  kp: 10.0
  kd: 1.0
  lowstate_timeout: 0.05

eid_defaults:
  kp: 40.0
  kd: 6.0
  observer_gain_q: 0.25
  observer_gain_dq: 0.25
  filter_alpha: 0.5
  reference_mode: open_loop
  reference_signal: sine
  policy_reference_dt: 0.05
  closed_loop_reference_tau: 0.05
  ref_step_time: 0.25
  startup_ramp_duration: 0.0
  eid_tau_slew_rate: 60.0
  torque_safe_kp: 0.0
  torque_safe_kd: 0.8
  inverse_q_weight: 0.0
  inverse_dq_weight: 0.0

eid_controllers:
  2:
    name: RightKnee
    kp: 51.0
    kd: 7.0
    observer_gain_q: 0.31
    observer_gain_dq: 0.32
    filter_alpha: 0.61
    inverse_q_weight: 0.03
    inverse_dq_weight: 0.04
    reference_signal: step
    ref_center: 0.8
    ref_amplitude: -0.1
    ref_frequency: 0.1
    ref_phase: 0.0
    ref_step_time: 0.12
    eid_tau_limit: 8.0
    plant:
      Jeff: 0.238
      b: 1.0
      gravityA: 4.2835
      gravityB: 0.0
      tau0: -0.2711
      q_min: -0.26
      q_max: 2.05
      tau_max: 8.0
  5:
    name: LeftKnee
    kp: 52.0
    kd: 7.5
    observer_gain_q: 0.41
    observer_gain_dq: 0.42
    filter_alpha: 0.62
    inverse_q_weight: 0.05
    inverse_dq_weight: 0.06
    reference_signal: sine
    ref_center: 0.6
    ref_amplitude: 0.1
    ref_frequency: 0.1
    ref_phase: 0.0
    eid_tau_limit: 6.0
    plant:
      Jeff: 0.3
      b: 1.2
      gravityA: 3.0
      gravityB: 0.0
      tau0: 0.1
      q_min: -0.26
      q_max: 2.05
      tau_max: 6.0
  6:
    name: WaistYaw
    kp: 30.0
    kd: 4.0
    observer_gain_q: 0.25
    observer_gain_dq: 0.25
    filter_alpha: 0.5
    reference_signal: step
    ref_center: 0.0
    ref_amplitude: 0.1
    ref_frequency: 0.1
    ref_phase: 0.0
    ref_step_time: 0.0
    eid_tau_limit: 5.0
    plant:
      Jeff: 0.8
      b: 1.0
      gravityA: 0.0
      gravityB: 0.0
      tau0: 0.0
      q_min: -2.35
      q_max: 2.35
      tau_max: 5.0
  12:
    name: RightShoulderPitch
    enabled: false

joint_limits:
  2:
    q_min: -0.26
    q_max: 2.05
    dq_max: 14.0
    tau_max: 8.0
    kp_max: 120.0
    kd_max: 5.0
  5:
    q_min: -0.26
    q_max: 2.05
    dq_max: 14.0
    tau_max: 6.0
    kp_max: 120.0
    kd_max: 5.0
  6:
    q_min: -2.35
    q_max: 2.35
    dq_max: 23.0
    tau_max: 5.0
    kp_max: 100.0
    kd_max: 5.0
)YAML";

    const auto valid_path = writeTempConfig("h1if_valid_multi.yaml", valid_multi_config);
    h1if::RuntimeConfig runtime_cfg = h1if::loadRuntimeConfig(valid_path.string());
    const auto active = h1if::activeEidJoints(runtime_cfg);
    assert(active.size() == 3);
    assert(active[0] == 2);
    assert(active[1] == 5);
    assert(active[2] == 6);
    assert(h1if::primaryEidJoint(runtime_cfg) == 2);
    assert(runtime_cfg.eid_controllers[6]->enabled);
    assert(!runtime_cfg.eid_controllers[12]->enabled);
    assert(runtime_cfg.eid_controllers[2]->controller.kp == 51.0);
    assert(runtime_cfg.eid_controllers[2]->controller.kd == 7.0);
    assert(runtime_cfg.eid_controllers[2]->controller.observer_gain_q == 0.31);
    assert(runtime_cfg.eid_controllers[2]->controller.observer_gain_dq == 0.32);
    assert(runtime_cfg.eid_controllers[2]->controller.filter_alpha == 0.61);
    assert(runtime_cfg.eid_controllers[2]->controller.inverse_q_weight == 0.03);
    assert(runtime_cfg.eid_controllers[2]->controller.inverse_dq_weight == 0.04);
    assert(runtime_cfg.eid_controllers[2]->controller.reference_signal == h1if::ReferenceSignal::Step);
    assert(runtime_cfg.eid_controllers[2]->controller.ref_step_time == 0.12);
    assert(runtime_cfg.eid_controllers[5]->controller.kp == 52.0);
    assert(runtime_cfg.eid_controllers[5]->controller.kd == 7.5);
    assert(runtime_cfg.eid_controllers[5]->controller.observer_gain_q == 0.41);
    assert(runtime_cfg.eid_controllers[5]->controller.observer_gain_dq == 0.42);
    assert(runtime_cfg.eid_controllers[5]->controller.filter_alpha == 0.62);
    assert(runtime_cfg.eid_controllers[5]->controller.inverse_q_weight == 0.05);
    assert(runtime_cfg.eid_controllers[5]->controller.inverse_dq_weight == 0.06);
    assert(runtime_cfg.eid_controllers[5]->controller.reference_signal == h1if::ReferenceSignal::Sine);

    {
        auto state = validState();
        state.t = 0.0;
        state.dt = runtime_cfg.control_dt;
        h1if::RobotCommand cmd;
        h1if::ControllerDebug debug;
        h1if::EidMultiJointController controller(runtime_cfg);
        controller.reset(state);
        controller.step(state, cmd, debug);
        h1if::applySafety(state, cmd, debug, runtime_cfg.safety);

        assert(std::abs(cmd.joint[2].tau) > 0.0f);
        assert(std::abs(cmd.joint[5].tau) > 0.0f);
        assert(std::abs(cmd.joint[6].tau) > 0.0f);
        assert(cmd.joint[12].tau == 0.0f);
        assert(cmd.joint[4].tau == 0.0f);
        assert(cmd.joint[4].q == static_cast<float>(state.joint[4].q));
        assert(debug.joint[2].data[0] != debug.joint[5].data[0]);
    }

    const auto legacy_path = writeTempConfig("h1if_legacy_config.yaml", R"YAML(
robot: H1
control_dt: 0.002
controller:
  target_joint: 2
plant:
  Jeff: 0.238
)YAML");
    bool rejected_legacy = false;
    try {
        (void)h1if::loadRuntimeConfig(legacy_path.string());
    } catch (const std::exception&) {
        rejected_legacy = true;
    }
    assert(rejected_legacy);

    const auto missing_plant_path = writeTempConfig("h1if_missing_plant.yaml", R"YAML(
robot: H1
control_dt: 0.002
eid_controllers:
  2:
    ref_center: 0.8
)YAML");
    bool rejected_missing_plant = false;
    try {
        (void)h1if::loadRuntimeConfig(missing_plant_path.string());
    } catch (const std::exception&) {
        rejected_missing_plant = true;
    }
    assert(rejected_missing_plant);

    const auto disabled_primary_path = writeTempConfig("h1if_disabled_primary.yaml", R"YAML(
robot: H1
control_dt: 0.002
eid_controllers:
  2:
    name: DisabledRightKnee
    enabled: false
  5:
    name: LeftKnee
    eid_tau_limit: 6.0
    plant:
      Jeff: 0.3
      b: 1.2
      gravityA: 3.0
      gravityB: 0.0
      tau0: 0.1
      q_min: -0.26
      q_max: 2.05
      tau_max: 6.0

joint_limits:
  5:
    q_min: -0.26
    q_max: 2.05
    dq_max: 14.0
    tau_max: 6.0
    kp_max: 120.0
    kd_max: 5.0
)YAML");
    h1if::RuntimeConfig disabled_primary_cfg = h1if::loadRuntimeConfig(disabled_primary_path.string());
    assert(h1if::primaryEidJoint(disabled_primary_cfg) == 5);

    const auto invalid_joint_path = writeTempConfig("h1if_invalid_joint.yaml", R"YAML(
robot: H1
control_dt: 0.002
eid_controllers:
  9:
    ref_center: 0.0
    ref_amplitude: 0.1
    ref_frequency: 0.1
    ref_phase: 0.0
    eid_tau_limit: 1.0
    plant:
      Jeff: 0.1
      b: 1.0
      gravityA: 0.0
      gravityB: 0.0
      tau0: 0.0
      q_min: -1.0
      q_max: 1.0
      tau_max: 1.0
)YAML");
    bool rejected_invalid_joint = false;
    try {
        (void)h1if::loadRuntimeConfig(invalid_joint_path.string());
    } catch (const std::exception&) {
        rejected_invalid_joint = true;
    }
    assert(rejected_invalid_joint);

    std::cout << "h1_safety_tests passed\n";
    return 0;
}
