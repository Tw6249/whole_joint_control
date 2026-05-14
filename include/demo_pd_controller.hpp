#pragma once

#include "controller_interface.hpp"
#include "reference_trajectory.hpp"
#include "safety.hpp"

namespace h1if {

class DemoSingleJointPD final : public IController {
public:
    explicit DemoSingleJointPD(int joint_id) : joint_id_(joint_id) {}

    std::string name() const override {
        return "DemoSingleJointPD";
    }

    void reset(const RobotState& state) override {
        q0_ = state.joint[joint_id_].q;
        t0_ = state.t;
        reference_.configure({0.05, q0_, 0.08, 0.10});
        reference_.reset();
    }

    void step(const RobotState& state, RobotCommand& command, ControllerDebug& debug) override {
        for (int i = 0; i < kMaxMotors; ++i) {
            command.joint[i].mode = h1MotorMode(i);
            command.joint[i].q = static_cast<float>(state.joint[i].q);
            command.joint[i].dq = 0.0f;
            command.joint[i].kp = 10.0f;
            command.joint[i].kd = 1.0f;
            command.joint[i].tau = 0.0f;
            command.joint[i].enable = true;
        }

        const double t = state.t - t0_;
        const JointReferencePair ref = reference_.sample(t, state.dt);
        const double q_ref = ref.now.q;
        const double dq_ref = ref.now.dq;

        auto& c = command.joint[joint_id_];
        c.mode = h1MotorMode(joint_id_);
        c.q = static_cast<float>(q_ref);
        c.dq = static_cast<float>(dq_ref);
        c.kp = 20.0f;
        c.kd = 2.0f;
        c.tau = 0.0f;

        debug.data[0] = q_ref;
        debug.data[1] = dq_ref;
        debug.data[2] = state.joint[joint_id_].q;
        debug.data[3] = state.joint[joint_id_].dq;
        debug.data[4] = q_ref - state.joint[joint_id_].q;
        debug.data[5] = dq_ref - state.joint[joint_id_].dq;
        debug.data[6] = c.kp;
        debug.data[7] = c.kd;
        debug.data[8] = c.tau;
    }

private:
    int joint_id_ = 2;
    double q0_ = 0.0;
    double t0_ = 0.0;
    SmoothSineReferenceTrajectory reference_;
};

}  // namespace h1if
