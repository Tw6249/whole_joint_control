#pragma once

#include "controller_interface.hpp"
#include "reference_trajectory.hpp"
#include "runtime_config.hpp"
#include "safety.hpp"

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
        const JointReferencePair ref =
            reference_.sample(t, cfg_.controller.control_dt, state.joint[j].q, state.joint[j].dq);

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

        for (int i = 0; i < 9 && i < kDebugSize; ++i) {
            debug.data[i] = jd[i];
        }
    }

private:
    JointControllerConfig cfg_;
    bool initialized_ = false;
    double t0_ = 0.0;
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
