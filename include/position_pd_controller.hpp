#pragma once

#include "controller_interface.hpp"
#include "reference_trajectory.hpp"
#include "runtime_config.hpp"
#include "safety.hpp"

#include <algorithm>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace h1if {

class PositionPdJointController final {
public:
    explicit PositionPdJointController(JointControllerConfig cfg)
        : cfg_(std::move(cfg)),
          reference_(makePolicyReferenceConfig(cfg_.controller, cfg_.has_plant ? &cfg_.plant : nullptr)) {}

    int jointId() const {
        return cfg_.controller.target_joint;
    }

    void reset(const RobotState& state) {
        const int j = jointId();
        t0_ = state.t;
        q_start_ = state.joint[j].q;
        dq_start_ = state.joint[j].dq;
        reference_.configure(makePolicyReferenceConfig(cfg_.controller, cfg_.has_plant ? &cfg_.plant : nullptr));
        reference_.reset();
        initialized_ = true;
    }

    void stepJoint(const RobotState& state, RobotCommand& command, ControllerDebug& debug) {
        if (!initialized_) {
            reset(state);
        }

        const int j = jointId();
        const double t = state.t - t0_;
        const JointReferencePair raw_ref =
            reference_.sample(t, cfg_.controller.control_dt, state.joint[j].q, state.joint[j].dq);
        const JointReferencePair ref = shapeStartupReference(raw_ref, t, cfg_.controller.control_dt);

        auto& c = command.joint[j];
        c.mode = h1MotorMode(j);
        c.q = static_cast<float>(ref.now.q);
        c.dq = static_cast<float>(ref.now.dq);
        c.kp = static_cast<float>(cfg_.controller.kp);
        c.kd = static_cast<float>(cfg_.controller.kd);
        c.tau = 0.0f;
        c.enable = true;

        auto& jd = debug.joint[j].data;
        jd[0] = ref.now.q;
        jd[1] = ref.now.dq;
        jd[2] = state.joint[j].q;
        jd[3] = state.joint[j].dq;
        jd[4] = ref.now.q - state.joint[j].q;
        jd[5] = ref.now.dq - state.joint[j].dq;
        jd[6] = c.kp;
        jd[7] = c.kd;
        jd[8] = c.tau;
        jd[9] = raw_ref.now.q;
        jd[10] = raw_ref.now.dq;
        jd[11] = raw_ref.now.q - state.joint[j].q;
        jd[12] = raw_ref.now.dq - state.joint[j].dq;

        for (int i = 0; i < 13 && i < kDebugSize; ++i) {
            debug.data[i] = jd[i];
        }
    }

private:
    JointReferencePair shapeStartupReference(const JointReferencePair& raw, double t, double dt) const {
        JointReferencePair shaped = raw;
        const double ramp = cfg_.controller.startup_blend_duration_s;
        if (ramp <= 1.0e-9 || t >= ramp) {
            return shaped;
        }

        const auto smooth = [ramp](double time, double& alpha, double& alpha_dot) {
            const double s = std::max(0.0, std::min(time / ramp, 1.0));
            alpha = s * s * (3.0 - 2.0 * s);
            alpha_dot = (6.0 * s - 6.0 * s * s) / ramp;
        };

        double a = 0.0;
        double adot = 0.0;
        smooth(t, a, adot);
        shaped.now.q = q_start_ + a * (raw.now.q - q_start_);
        shaped.now.dq = (1.0 - a) * dq_start_ + a * raw.now.dq + adot * (raw.now.q - q_start_);

        double an = 0.0;
        double adotn = 0.0;
        smooth(t + dt, an, adotn);
        shaped.next.q = q_start_ + an * (raw.next.q - q_start_);
        shaped.next.dq = (1.0 - an) * dq_start_ + an * raw.next.dq + adotn * (raw.next.q - q_start_);
        return shaped;
    }

    JointControllerConfig cfg_;
    bool initialized_ = false;
    double t0_ = 0.0;
    double q_start_ = 0.0;
    double dq_start_ = 0.0;
    PolicyReferenceInterpolator reference_;
};

class PositionPdMultiJointController final : public IController {
public:
    explicit PositionPdMultiJointController(RuntimeConfig cfg)
        : safety_(cfg.safety) {
        for (int joint_id : activeControllerJoints(cfg)) {
            controllers_.emplace_back(*cfg.controller.joints[joint_id]);
        }
    }

    std::string name() const override {
        std::ostringstream out;
        out << "PositionPdMultiJointController(" << controllers_.size() << " joints)";
        return out.str();
    }

    void reset(const RobotState& state) override {
        for (auto& controller : controllers_) {
            controller.reset(state);
        }
    }

    void step(const RobotState& state, RobotCommand& command, ControllerDebug& debug) override {
        fillSafeHoldCommand(state, command, safety_);
        for (auto& controller : controllers_) {
            controller.stepJoint(state, command, debug);
        }
    }

private:
    SafetyConfig safety_;
    std::vector<PositionPdJointController> controllers_;
};

}  // namespace h1if
