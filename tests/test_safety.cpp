#include "controller_factory.hpp"
#include "runtime_config.hpp"
#include "safety.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

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

    assert(h1if::parsePolicyInterpolation("open_loop") == h1if::PolicyInterpolation::OpenLoop);
    assert(h1if::parsePolicyInterpolation("closed-loop") == h1if::PolicyInterpolation::ClosedLoop);
    assert(h1if::parsePolicyInterpolation("preview_mpc") == h1if::PolicyInterpolation::PreviewMpc);
    assert(h1if::parsePolicyInterpolation("preview_mpc_velocity") ==
           h1if::PolicyInterpolation::PreviewMpcVelocity);
    assert(h1if::parsePolicySource("hold") == h1if::PolicySource::Hold);
    assert(h1if::parsePolicySource("sine") == h1if::PolicySource::Sine);
    assert(h1if::parsePolicySource("step") == h1if::PolicySource::Step);

    {
        h1if::PolicyReferenceConfig step_cfg;
        step_cfg.source = h1if::PolicySource::Step;
        step_cfg.policy_dt = 0.05;
        step_cfg.center = 0.5;
        step_cfg.amplitude = -0.2;
        step_cfg.step_time_s = 0.10;
        step_cfg.reference_points = 1;
        h1if::PolicyReferenceInterpolator step_ref(step_cfg);
        const auto before = step_ref.sample(0.02, 0.002, 0.5, 0.0);
        const auto after = step_ref.sample(0.20, 0.002, 0.3, 0.0);
        assert(std::abs(before.now.q - 0.5) < 1.0e-9);
        assert(std::abs(after.now.q - 0.3) < 1.0e-9);
    }

    {
        h1if::PolicyReferenceConfig step_cfg;
        step_cfg.source = h1if::PolicySource::Step;
        step_cfg.policy_dt = 0.05;
        step_cfg.center = 0.5;
        step_cfg.amplitude = -0.2;
        step_cfg.step_time_s = 0.12;
        step_cfg.reference_points = 1;
        h1if::PolicyReferenceInterpolator step_ref(step_cfg);
        const auto before = step_ref.sample(0.148, 0.002, 0.5, 0.0);
        const auto entering = step_ref.sample(0.152, 0.002, 0.5, 0.0);
        const auto finished = step_ref.sample(0.202, 0.002, 0.3, 0.0);
        assert(std::abs(before.now.q - 0.5) < 1.0e-9);
        assert(std::abs(before.now.dq) < 1.0e-9);
        assert(entering.now.q < 0.5);
        assert(entering.now.q > 0.3);
        assert(entering.now.dq < 0.0);
        assert(std::isfinite(finished.now.q));
        assert(std::isfinite(finished.now.dq));
        assert(finished.now.q < 0.5);
    }

    {
        h1if::PolicyReferenceConfig sine_cfg;
        sine_cfg.source = h1if::PolicySource::Sine;
        sine_cfg.policy_dt = 0.20;
        sine_cfg.center = 0.1;
        sine_cfg.amplitude = 0.05;
        sine_cfg.frequency_hz = 0.2;
        sine_cfg.phase_rad = -1.5707963267948966;
        h1if::PolicyReferenceInterpolator sine_ref(sine_cfg);
        for (int i = 0; i <= 200; ++i) {
            const auto sample = sine_ref.sample(0.002 * i, 0.002, 0.1, 0.0);
            assert(std::isfinite(sample.now.q));
            assert(std::isfinite(sample.now.dq));
            assert(sample.now.q >= 0.03);
            assert(sample.now.q <= 0.17);
        }
    }

    {
        h1if::PolicyReferenceConfig cfg;
        cfg.interpolation = h1if::PolicyInterpolation::ClosedLoop;
        cfg.source = h1if::PolicySource::Step;
        cfg.policy_dt = 0.05;
        cfg.center = 0.0;
        cfg.amplitude = 1.0;
        cfg.step_time_s = 0.0;
        cfg.reference_points = 1;
        h1if::PolicyReferenceInterpolator ref(cfg);
        const auto first = ref.sample(0.051, 0.002, 0.2, 0.0);
        const auto same_segment = ref.sample(0.070, 0.002, 0.7, 0.0);
        const auto next_segment = ref.sample(0.101, 0.002, 0.7, 0.0);
        assert(first.now.q > 0.2);
        assert(same_segment.now.q < 0.7);
        assert(next_segment.now.q > 0.7);
    }

    {
        h1if::PolicyReferenceConfig cfg;
        cfg.source = h1if::PolicySource::Sine;
        cfg.policy_dt = 0.05;
        cfg.center = 0.5;
        cfg.amplitude = 0.1;
        cfg.frequency_hz = 0.4;
        cfg.phase_rad = -1.5707963267948966;

        cfg.reference_points = 1;
        h1if::PolicyReferenceInterpolator one_point(cfg);
        const auto one = one_point.sample(0.05, 0.002, 0.5, 0.0);
        assert(std::isfinite(one.now.dq));

        cfg.reference_points = 2;
        h1if::PolicyReferenceInterpolator two_point(cfg);
        const auto two = two_point.sample(0.05, 0.002, 0.5, 0.0);
        assert(std::isfinite(two.now.dq));

        cfg.reference_points = 4;
        h1if::PolicyReferenceInterpolator four_point(cfg);
        const auto four = four_point.sample(0.05, 0.002, 0.5, 0.0);
        assert(std::isfinite(four.now.dq));
        assert(std::abs(one.now.dq - two.now.dq) > 1.0e-12);
        assert(std::abs(two.now.dq - four.now.dq) > 1.0e-12);
    }

    {
        h1if::PolicyReferenceConfig cfg;
        cfg.interpolation = h1if::PolicyInterpolation::PreviewMpc;
        cfg.source = h1if::PolicySource::Sine;
        cfg.policy_dt = 0.05;
        cfg.center = 0.5;
        cfg.amplitude = 0.1;
        cfg.frequency_hz = 0.4;
        cfg.phase_rad = -1.5707963267948966;
        cfg.reference_points = 2;
        h1if::PolicyReferenceInterpolator preview_ref(cfg);
        bool threw = false;
        try {
            (void)preview_ref.sample(0.052, 0.002, 0.5, 0.0);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        assert(threw);
    }

    {
        h1if::PolicyReferenceConfig cfg;
        cfg.interpolation = h1if::PolicyInterpolation::PreviewMpc;
        cfg.source = h1if::PolicySource::Sine;
        cfg.policy_dt = 0.05;
        cfg.center = 0.5;
        cfg.amplitude = 0.1;
        cfg.frequency_hz = 0.4;
        cfg.phase_rad = -1.5707963267948966;
        cfg.reference_points = 3;
        h1if::PolicyReferenceInterpolator preview_ref(cfg);
        (void)preview_ref.sample(0.0, 0.002, 0.5, 0.0);
        (void)preview_ref.sample(0.05, 0.002, 0.5, 0.0);
        const auto boundary = preview_ref.sample(0.10, 0.002, 0.5, 0.0);
        const double expected = cfg.center + cfg.amplitude *
            std::sin(2.0 * 3.14159265358979323846 * cfg.frequency_hz * 0.05 + cfg.phase_rad);
        assert(std::isfinite(boundary.now.q));
        assert(std::isfinite(boundary.now.dq));
        assert(std::abs(boundary.now.q - expected) < 1.0e-8);
        const auto mid = preview_ref.sample(0.086, 0.002, 0.5, 0.0);
        assert(std::isfinite(mid.now.q));
        assert(std::isfinite(mid.now.dq));
        assert(std::isfinite(mid.now.ddq));
    }

    {
        h1if::PolicyReferenceConfig cfg;
        cfg.interpolation = h1if::PolicyInterpolation::PreviewMpcVelocity;
        cfg.source = h1if::PolicySource::Sine;
        cfg.policy_dt = 0.05;
        cfg.center = 0.5;
        cfg.amplitude = 0.1;
        cfg.frequency_hz = 0.4;
        cfg.phase_rad = -1.5707963267948966;
        cfg.reference_points = 4;
        h1if::PolicyReferenceInterpolator preview_ref(cfg);
        (void)preview_ref.sample(0.0, 0.002, 0.5, 0.0);
        (void)preview_ref.sample(0.05, 0.002, 0.5, 0.0);
        const auto boundary = preview_ref.sample(0.10, 0.002, 0.5, 0.0);
        const double expected = cfg.center + cfg.amplitude *
            std::sin(2.0 * 3.14159265358979323846 * cfg.frequency_hz * 0.05 + cfg.phase_rad);
        assert(std::isfinite(boundary.now.q));
        assert(std::isfinite(boundary.now.dq));
        assert(std::isfinite(boundary.now.ddq));
        assert(std::abs(boundary.now.q - expected) < 1.0e-8);
        const auto mid = preview_ref.sample(0.086, 0.002, 0.5, 0.0);
        assert(std::isfinite(mid.now.q));
        assert(std::isfinite(mid.now.dq));
        assert(std::isfinite(mid.now.ddq));
    }

    {
        h1if::PolicyReferenceConfig cfg;
        cfg.interpolation = h1if::PolicyInterpolation::PreviewMpcVelocity;
        cfg.source = h1if::PolicySource::Sine;
        cfg.policy_dt = 0.05;
        cfg.center = 0.5;
        cfg.amplitude = 0.1;
        cfg.frequency_hz = 0.4;
        cfg.phase_rad = -1.5707963267948966;
        cfg.reference_points = 3;
        h1if::PolicyReferenceInterpolator preview_ref(cfg);
        bool threw = false;
        try {
            (void)preview_ref.sample(0.052, 0.002, 0.5, 0.0);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        assert(threw);
    }

    {
        h1if::PolicyReferenceConfig cfg;
        cfg.interpolation = h1if::PolicyInterpolation::PreviewMpc;
        cfg.source = h1if::PolicySource::Sine;
        cfg.policy_dt = 0.05;
        cfg.center = 0.5;
        cfg.amplitude = 0.1;
        cfg.frequency_hz = 0.4;
        cfg.phase_rad = -1.5707963267948966;
        cfg.reference_points = 1;
        h1if::PolicyReferenceInterpolator preview_ref(cfg);
        bool threw = false;
        try {
            (void)preview_ref.sample(0.052, 0.002, 0.5, 0.0);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        assert(threw);
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

controller:
  kind: eid
  defaults:
    kp: 40.0
    kd: 6.0
    observer_gain_q: 0.25
    observer_gain_dq: 0.25
    filter_alpha: 0.5
    policy_interpolation: open_loop
    policy_source: sine
    policy_dt: 0.05
    policy_step_time_s: 0.25
    startup_blend_duration_s: 0.0
    tau_slew_rate: 60.0
    torque_safe_kp: 0.0
    torque_safe_kd: 0.8
    inverse_q_weight: 0.0
    inverse_dq_weight: 0.0
  groups:
    knees:
      joints: [2, 5]
      observer_gain_dq: 0.42
      filter_alpha: 0.62
      inverse_dq_weight: 0.06
    waist:
      joints: [6]
      kp: 30.0
      kd: 4.0
      policy_source: step
      policy_center: 0.0
      policy_amplitude: 0.1
      policy_step_time_s: 0.0
  joints:
    2:
      name: RightKnee
      kp: 51.0
      kd: 7.0
      observer_gain_q: 0.31
      observer_gain_dq: 0.32
      filter_alpha: 0.61
      inverse_q_weight: 0.03
      inverse_dq_weight: 0.04
      policy_source: step
      policy_center: 0.8
      policy_amplitude: -0.1
      policy_frequency_hz: 0.1
      policy_phase_rad: 0.0
      policy_step_time_s: 0.12
      tau_limit: 8.0
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
      inverse_q_weight: 0.05
      policy_source: sine
      policy_center: 0.6
      policy_amplitude: 0.1
      policy_frequency_hz: 0.1
      policy_phase_rad: 0.0
      tau_limit: 6.0
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
      observer_gain_q: 0.25
      observer_gain_dq: 0.25
      filter_alpha: 0.5
      policy_frequency_hz: 0.1
      policy_phase_rad: 0.0
      tau_limit: 5.0
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
    const auto active = h1if::activeControllerJoints(runtime_cfg);
    assert(active.size() == 3);
    assert(active[0] == 2);
    assert(active[1] == 5);
    assert(active[2] == 6);
    assert(h1if::primaryControllerJoint(runtime_cfg) == 2);
    assert(runtime_cfg.controller.kind == h1if::ControllerKind::Eid);
    assert(runtime_cfg.controller.joints[6]->enabled);
    assert(!runtime_cfg.controller.joints[12]->enabled);
    assert(runtime_cfg.controller.joints[2]->controller.kp == 51.0);
    assert(runtime_cfg.controller.joints[2]->controller.kd == 7.0);
    assert(runtime_cfg.controller.joints[2]->controller.observer_gain_q == 0.31);
    assert(runtime_cfg.controller.joints[2]->controller.observer_gain_dq == 0.32);
    assert(runtime_cfg.controller.joints[2]->controller.filter_alpha == 0.61);
    assert(runtime_cfg.controller.joints[2]->controller.inverse_q_weight == 0.03);
    assert(runtime_cfg.controller.joints[2]->controller.inverse_dq_weight == 0.04);
    assert(runtime_cfg.controller.joints[2]->controller.policy_source == h1if::PolicySource::Step);
    assert(runtime_cfg.controller.joints[2]->controller.policy_step_time_s == 0.12);
    assert(runtime_cfg.controller.joints[2]->controller.policy_reference_points == 4);
    assert(runtime_cfg.controller.joints[5]->controller.kp == 52.0);
    assert(runtime_cfg.controller.joints[5]->controller.kd == 7.5);
    assert(runtime_cfg.controller.joints[5]->controller.observer_gain_q == 0.41);
    assert(runtime_cfg.controller.joints[5]->controller.observer_gain_dq == 0.42);
    assert(runtime_cfg.controller.joints[5]->controller.filter_alpha == 0.62);
    assert(runtime_cfg.controller.joints[5]->controller.inverse_q_weight == 0.05);
    assert(runtime_cfg.controller.joints[5]->controller.inverse_dq_weight == 0.06);
    assert(runtime_cfg.controller.joints[5]->controller.policy_source == h1if::PolicySource::Sine);

    {
        auto state = validState();
        state.t = 0.0;
        state.dt = runtime_cfg.control_dt;
        h1if::RobotCommand cmd;
        h1if::ControllerDebug debug;
        const auto controller = h1if::createController(runtime_cfg);
        controller->reset(state);
        controller->step(state, cmd, debug);
        h1if::applySafety(state, cmd, debug, runtime_cfg.safety);

        assert(std::abs(cmd.joint[2].tau) > 0.0f);
        assert(std::abs(cmd.joint[5].tau) > 0.0f);
        assert(std::abs(cmd.joint[6].tau) > 0.0f);
        assert(cmd.joint[12].tau == 0.0f);
        assert(cmd.joint[4].tau == 0.0f);
        assert(cmd.joint[4].q == static_cast<float>(state.joint[4].q));
        assert(debug.joint[2].data[0] != debug.joint[5].data[0]);
    }

    const std::string position_pd_config = R"YAML(
robot: H1
control_dt: 0.002
controller:
  kind: position_pd
  defaults:
    kp: 18.0
    kd: 2.0
    policy_source: sine
    policy_dt: 0.05
    policy_center: 0.5
    policy_amplitude: 0.05
    policy_frequency_hz: 0.1
  joints:
    2:
      name: RightKnee
      enabled: true

joint_limits:
  2:
    q_min: -0.26
    q_max: 2.05
    dq_max: 14.0
    tau_max: 8.0
    kp_max: 120.0
    kd_max: 5.0
)YAML";
    const auto position_pd_path = writeTempConfig("h1if_position_pd.yaml", position_pd_config);
    h1if::RuntimeConfig pd_cfg = h1if::loadRuntimeConfig(position_pd_path.string());
    assert(pd_cfg.controller.kind == h1if::ControllerKind::PositionPd);
    assert(pd_cfg.controller.joints[2]->controller.policy_reference_points == 4);
    {
        auto state = validState();
        state.t = 0.0;
        state.dt = pd_cfg.control_dt;
        h1if::RobotCommand cmd;
        h1if::ControllerDebug debug;
        const auto controller = h1if::createController(pd_cfg);
        controller->reset(state);
        controller->step(state, cmd, debug);
        h1if::applySafety(state, cmd, debug, pd_cfg.safety);

        assert(cmd.joint[2].kp == 18.0f);
        assert(cmd.joint[2].kd == 2.0f);
        assert(cmd.joint[2].tau == 0.0f);
        assert(cmd.joint[2].enable);
        assert(std::isfinite(cmd.joint[2].q));
    }

    const std::string invalid_preview_config = R"YAML(
robot: H1
control_dt: 0.002
controller:
  kind: position_pd
  defaults:
    kp: 18.0
    kd: 2.0
    policy_interpolation: preview_mpc
    policy_reference_points: 1
    policy_source: sine
    policy_dt: 0.05
    policy_center: 0.5
    policy_amplitude: 0.05
    policy_frequency_hz: 0.1
  joints:
    2:
      name: RightKnee
      enabled: true

joint_limits:
  2:
    q_min: -0.26
    q_max: 2.05
    dq_max: 14.0
    tau_max: 8.0
    kp_max: 120.0
    kd_max: 5.0
)YAML";
    const auto invalid_preview_path = writeTempConfig("h1if_invalid_preview.yaml", invalid_preview_config);
    {
        bool threw = false;
        try {
            (void)h1if::loadRuntimeConfig(invalid_preview_path.string());
        } catch (const std::runtime_error&) {
            threw = true;
        }
        assert(threw);
    }

    const std::string selected_mpc_config = R"YAML(
robot: H1
control_dt: 0.002
controller:
  kind: position_pd
  defaults:
    kp: 18.0
    kd: 2.0
    policy_interpolation: preview_mpc
    policy_reference_points: 3
    policy_source: sine
    policy_dt: 0.05
    policy_center: 0.5
    policy_amplitude: 0.05
    policy_frequency_hz: 0.1
  joints:
    2:
      name: RightKnee
      enabled: true

joint_limits:
  2:
    q_min: -0.26
    q_max: 2.05
    dq_max: 14.0
    tau_max: 8.0
    kp_max: 120.0
    kd_max: 5.0
)YAML";
    const auto selected_mpc_path = writeTempConfig("h1if_selected_mpc.yaml", selected_mpc_config);
    h1if::RuntimeConfig selected_mpc_cfg = h1if::loadRuntimeConfig(selected_mpc_path.string());
    assert(selected_mpc_cfg.controller.joints[2]->controller.policy_interpolation ==
           h1if::PolicyInterpolation::PreviewMpc);
    assert(selected_mpc_cfg.controller.joints[2]->controller.policy_reference_points == 3);

    const std::string velocity_mpc_config = R"YAML(
robot: H1
control_dt: 0.002
controller:
  kind: position_pd
  defaults:
    kp: 18.0
    kd: 2.0
    policy_interpolation: preview_mpc_velocity
    policy_reference_points: 4
    policy_source: sine
    policy_dt: 0.05
    policy_center: 0.5
    policy_amplitude: 0.05
    policy_frequency_hz: 0.1
  joints:
    2:
      name: RightKnee
      enabled: true

joint_limits:
  2:
    q_min: -0.26
    q_max: 2.05
    dq_max: 14.0
    tau_max: 8.0
    kp_max: 120.0
    kd_max: 5.0
)YAML";
    const auto velocity_mpc_path = writeTempConfig("h1if_velocity_mpc.yaml", velocity_mpc_config);
    h1if::RuntimeConfig velocity_mpc_cfg = h1if::loadRuntimeConfig(velocity_mpc_path.string());
    assert(velocity_mpc_cfg.controller.joints[2]->controller.policy_interpolation ==
           h1if::PolicyInterpolation::PreviewMpcVelocity);
    assert(velocity_mpc_cfg.controller.joints[2]->controller.policy_reference_points == 4);

    const std::string invalid_velocity_mpc_config = R"YAML(
robot: H1
control_dt: 0.002
controller:
  kind: position_pd
  defaults:
    kp: 18.0
    kd: 2.0
    policy_interpolation: preview_mpc_velocity
    policy_reference_points: 3
    policy_source: sine
    policy_dt: 0.05
    policy_center: 0.5
    policy_amplitude: 0.05
    policy_frequency_hz: 0.1
  joints:
    2:
      name: RightKnee
      enabled: true

joint_limits:
  2:
    q_min: -0.26
    q_max: 2.05
    dq_max: 14.0
    tau_max: 8.0
    kp_max: 120.0
    kd_max: 5.0
)YAML";
    const auto invalid_velocity_mpc_path =
        writeTempConfig("h1if_invalid_velocity_mpc.yaml", invalid_velocity_mpc_config);
    {
        bool threw = false;
        try {
            (void)h1if::loadRuntimeConfig(invalid_velocity_mpc_path.string());
        } catch (const std::runtime_error&) {
            threw = true;
        }
        assert(threw);
    }

    const std::string removed_variant_config = R"YAML(
robot: H1
control_dt: 0.002
controller:
  kind: position_pd
  defaults:
    kp: 18.0
    kd: 2.0
    policy_interpolation: preview_mpc
    policy_mpc_variant: impossible_variant
    policy_reference_points: 3
    policy_source: sine
    policy_dt: 0.05
    policy_center: 0.5
    policy_amplitude: 0.05
    policy_frequency_hz: 0.1
  joints:
    2:
      name: RightKnee
      enabled: true

joint_limits:
  2:
    q_min: -0.26
    q_max: 2.05
    dq_max: 14.0
    tau_max: 8.0
    kp_max: 120.0
    kd_max: 5.0
)YAML";
    const auto removed_variant_path = writeTempConfig("h1if_removed_variant.yaml", removed_variant_config);
    {
        bool threw = false;
        try {
            (void)h1if::loadRuntimeConfig(removed_variant_path.string());
        } catch (const std::runtime_error&) {
            threw = true;
        }
        assert(threw);
    }

    const auto legacy_path = writeTempConfig("h1if_legacy_config.yaml", R"YAML(
robot: H1
control_dt: 0.002
eid_defaults:
  kp: 40.0
eid_controllers:
  2:
    tau_limit: 1.0
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
controller:
  kind: eid
  joints:
    2:
      policy_center: 0.8
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
controller:
  kind: eid
  joints:
    2:
      name: DisabledRightKnee
      enabled: false
    5:
      name: LeftKnee
      tau_limit: 6.0
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
    assert(h1if::primaryControllerJoint(disabled_primary_cfg) == 5);

    const auto invalid_joint_path = writeTempConfig("h1if_invalid_joint.yaml", R"YAML(
robot: H1
control_dt: 0.002
controller:
  kind: eid
  joints:
    9:
      policy_center: 0.0
      policy_amplitude: 0.1
      policy_frequency_hz: 0.1
      policy_phase_rad: 0.0
      tau_limit: 1.0
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
