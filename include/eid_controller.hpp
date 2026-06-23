#pragma once

#include "controller_interface.hpp"
#include "reference_trajectory.hpp"
#include "runtime_config.hpp"
#include "safety.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace h1if {

class EidJointController final {
public:
    explicit EidJointController(JointControllerConfig cfg)
        : cfg_(std::move(cfg)),
          reference_(makePolicyReferenceConfig(cfg_.controller, &cfg_.plant)) {}

    int jointId() const {
        return cfg_.controller.target_joint;
    }

    const JointControllerConfig& config() const {
        return cfg_;
    }

    void reset(const RobotState& state) {
        const int j = jointId();
        t0_ = state.t;
        x_hat_q_ = state.joint[j].q;
        x_hat_dq_ = state.joint[j].dq;
        eta_q_ = 0.0;
        eta_dq_ = 0.0;
        eta_lpf_q_ = 0.0;
        eta_lpf_dq_ = 0.0;
        q_start_ = state.joint[j].q;
        dq_start_ = state.joint[j].dq;
        last_tau_ = 0.0;
        reference_.configure(makePolicyReferenceConfig(cfg_.controller, &cfg_.plant));
        reference_.reset();
        initialized_ = true;
    }

    void stepJoint(const RobotState& state, RobotCommand& command, ControllerDebug& debug) {
        if (!initialized_) {
            reset(state);
        }

        const int j = jointId();
        const double q = state.joint[j].q;
        const double dq = state.joint[j].dq;
        const double t = state.t - t0_;
        const double dt = cfg_.controller.control_dt;

        const JointReferencePair raw_ref = reference_.sample(t, dt, q, dq);
        const JointReferencePair ramped_ref = shapeStartupReference(raw_ref, t, dt);
        const JointReferencePair ref = ramped_ref;
        const StepResult result = controllerStep(q, dq, ref, dt);

        auto& c = command.joint[j];
        c.mode = h1MotorMode(j);
        c.q = static_cast<float>(q);
        c.dq = 0.0f;
        c.kp = static_cast<float>(cfg_.controller.torque_safe_kp);
        c.kd = static_cast<float>(cfg_.controller.torque_safe_kd);
        c.tau = static_cast<float>(result.u_t);
        c.enable = true;

        auto& jd = debug.joint[j].data;
        jd[0] = ref.now.q;
        jd[1] = ref.now.dq;
        jd[2] = q;
        jd[3] = dq;
        jd[4] = raw_ref.now.q - q;
        jd[5] = raw_ref.now.dq - dq;
        jd[6] = result.u_star;
        jd[7] = result.u_feedback;
        jd[8] = result.u_t;
        jd[9] = result.eta_q;
        jd[10] = result.eta_dq;
        jd[11] = result.x_hat_q;
        jd[12] = result.x_hat_dq;
        jd[13] = result.rho_q;
        jd[14] = result.rho_dq;
        jd[15] = result.x_bar_q;
        jd[16] = ref.next.q;
        jd[17] = ref.next.dq;
        jd[18] = result.x_bar_dq;
        jd[19] = result.r_d_q;
        jd[20] = result.r_d_dq;
        jd[21] = result.e_q;
        jd[22] = result.e_dq;
        jd[23] = result.observer_qacc;
        jd[24] = result.observer_tau_applied;
        jd[25] = result.u_raw;
        jd[26] = raw_ref.now.q;
        jd[27] = raw_ref.now.dq;
        jd[28] = raw_ref.now.q - q;
        jd[29] = raw_ref.now.dq - dq;
        jd[30] = ref.now.q - q;
        jd[31] = ref.now.dq - dq;
        jd[32] = result.eta_u;

        for (int i = 0; i < static_cast<int>(jd.size()) && i < kDebugSize; ++i) {
            debug.data[i] = jd[i];
        }
    }

private:
    struct ForwardStep {
        double q_next = 0.0;
        double dq_next = 0.0;
        double qacc = 0.0;
        double tau_applied = 0.0;
    };

    struct InverseResult {
        double u_star = 0.0;
        double rho_q = 0.0;
        double rho_dq = 0.0;
    };

    struct StepResult {
        double u_star = 0.0;
        double u_feedback = 0.0;
        double u_t = 0.0;
        double rho_q = 0.0;
        double rho_dq = 0.0;
        double eta_q = 0.0;
        double eta_dq = 0.0;
        double x_hat_q = 0.0;
        double x_hat_dq = 0.0;
        double x_bar_q = 0.0;
        double x_bar_dq = 0.0;
        double r_d_q = 0.0;
        double r_d_dq = 0.0;
        double e_q = 0.0;
        double e_dq = 0.0;
        double observer_qacc = 0.0;
        double observer_tau_applied = 0.0;
        double u_raw = 0.0;
        double eta_u = 0.0;
    };

    StepResult controllerStep(double q, double dq, const JointReferencePair& ref, double dt) {
        const auto& c = cfg_.controller;
        const double eta_q = eta_q_;
        const double eta_dq = eta_dq_;
        const double x_hat_q = x_hat_q_;
        const double x_hat_dq = x_hat_dq_;
        const double x_bar_q = x_hat_q + eta_q;
        const double x_bar_dq = x_hat_dq + eta_dq;

        const InverseResult inv =
            analyticInverseModel(
                ref.now.q,
                ref.now.dq,
                ref.next.q - ref.now.q,
                ref.next.dq - ref.now.dq,
                dt);

        const double eta_u = c.ku_q * eta_q + c.ku_dq * eta_dq;
        const double u_star_comp = inv.u_star - eta_u;

        const double den = c.kp * c.kp + c.kd * c.kd;
        const double w_q = den < 1.0e-12 ? 0.0 : c.kp / den;
        const double w_dq = den < 1.0e-12 ? 0.0 : c.kd / den;
        const double r_d_q = ref.now.q + w_q * u_star_comp;
        const double r_d_dq = ref.now.dq + w_dq * u_star_comp;

        const double e_q = r_d_q - x_bar_q;
        const double e_dq = r_d_dq - x_bar_dq;
        const double u_raw = c.kp * e_q + c.kd * e_dq;
        const double u_t = limitTorqueCommand(u_raw, dt);

        const ForwardStep pred = forwardModel(x_bar_q, x_bar_dq, u_t, dt);
        const double tilde_x_q = q - x_bar_q;
        const double tilde_x_dq = dq - x_bar_dq;
        const double eta_next_q =
            c.filter_alpha * c.observer_gain_q * tilde_x_q +
            (1.0 - c.filter_alpha) * eta_lpf_q_;
        const double eta_next_dq =
            c.filter_alpha * c.observer_gain_dq * tilde_x_dq +
            (1.0 - c.filter_alpha) * eta_lpf_dq_;

        StepResult out;
        out.u_star = inv.u_star;
        out.u_feedback = c.kp * (ref.now.q - x_bar_q) + c.kd * (ref.now.dq - x_bar_dq);
        out.u_t = u_t;
        out.rho_q = inv.rho_q;
        out.rho_dq = inv.rho_dq;
        out.eta_q = eta_q;
        out.eta_dq = eta_dq;
        out.x_hat_q = x_hat_q;
        out.x_hat_dq = x_hat_dq;
        out.x_bar_q = x_bar_q;
        out.x_bar_dq = x_bar_dq;
        out.r_d_q = r_d_q;
        out.r_d_dq = r_d_dq;
        out.e_q = e_q;
        out.e_dq = e_dq;
        out.observer_qacc = pred.qacc;
        out.observer_tau_applied = pred.tau_applied;
        out.u_raw = u_raw;
        out.eta_u = eta_u;

        x_hat_q_ = pred.q_next;
        x_hat_dq_ = pred.dq_next;
        eta_q_ = eta_next_q;
        eta_dq_ = eta_next_dq;
        eta_lpf_q_ = eta_next_q;
        eta_lpf_dq_ = eta_next_dq;
        return out;
    }

    JointReferencePair shapeStartupReference(const JointReferencePair& raw, double t, double dt) const {
        JointReferencePair shaped = raw;
        const double ramp = cfg_.controller.startup_blend_duration_s;
        if (ramp <= 1.0e-9 || t >= ramp) {
            return shaped;
        }

        const auto smooth = [ramp](double time, double& alpha, double& alpha_dot) {
            const double s = clamp(time / ramp, 0.0, 1.0);
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

    double limitTorqueCommand(double tau, double dt) {
        const auto& c = cfg_.controller;
        const double tau_limit = std::min(std::abs(c.tau_limit), cfg_.plant.tau_max);
        double limited = clamp(tau, -tau_limit, tau_limit);
        const double slew = c.tau_slew_rate;
        if (slew > 0.0) {
            const double max_delta = slew * std::max(dt, 0.0);
            limited = clamp(limited, last_tau_ - max_delta, last_tau_ + max_delta);
        }
        last_tau_ = limited;
        return limited;
    }

    InverseResult analyticInverseModel(double q,
                                       double dq,
                                       double delta_q,
                                       double delta_dq,
                                       double dt) const {
        const auto& c = cfg_.controller;
        const auto& model = cfg_.plant;
        const double q_target_next = q + delta_q;
        const double dq_target_next = dq + delta_dq;
        const double bias = model.b * dq + gravityTorque(q) + model.tau0;

        const double tau_from_q =
            bias + model.Jeff * ((q_target_next - q - dt * dq) / (dt * dt));
        const double tau_from_dq =
            bias + model.Jeff * ((dq_target_next - dq) / dt);

        const double q_weight =
            c.inverse_q_weight > 0.0 ? c.inverse_q_weight : 0.5 / (dt * dt);
        const double dq_weight =
            c.inverse_dq_weight > 0.0 ? c.inverse_dq_weight : 1.0;
        const double aq = dt * dt / model.Jeff;
        const double adq = dt / model.Jeff;
        const double den = q_weight * aq * aq + dq_weight * adq * adq;

        double u_star = 0.0;
        if (den >= 1.0e-12) {
            u_star = (q_weight * aq * aq * tau_from_q +
                      dq_weight * adq * adq * tau_from_dq) / den;
        }
        u_star = clamp(u_star, -model.tau_max, model.tau_max);

        const ForwardStep pred = forwardModel(q, dq, u_star, dt);

        InverseResult result;
        result.u_star = u_star;
        result.rho_q = pred.q_next - q_target_next;
        result.rho_dq = pred.dq_next - dq_target_next;
        return result;
    }

    ForwardStep forwardModel(double q, double dq, double tau_raw, double dt) const {
        const auto& model = cfg_.plant;
        const double tau = clamp(tau_raw, -model.tau_max, model.tau_max);
        const double qacc = (tau - model.b * dq - gravityTorque(q) - model.tau0) / model.Jeff;
        double dq_next = dq + dt * qacc;
        double q_next = q + dt * dq_next;

        if (q_next < model.q_min) {
            q_next = model.q_min;
            if (dq_next < 0.0) {
                dq_next = 0.0;
            }
        } else if (q_next > model.q_max) {
            q_next = model.q_max;
            if (dq_next > 0.0) {
                dq_next = 0.0;
            }
        }

        return {q_next, dq_next, qacc, tau};
    }

    double gravityTorque(double q) const {
        const auto& model = cfg_.plant;
        return model.gravityA * std::sin(q) + model.gravityB * std::cos(q);
    }

    static double clamp(double x, double lo, double hi) {
        return std::max(lo, std::min(x, hi));
    }

    JointControllerConfig cfg_;
    bool initialized_ = false;
    double t0_ = 0.0;
    double eta_q_ = 0.0;
    double eta_dq_ = 0.0;
    double eta_lpf_q_ = 0.0;
    double eta_lpf_dq_ = 0.0;
    double x_hat_q_ = 0.0;
    double x_hat_dq_ = 0.0;
    double q_start_ = 0.0;
    double dq_start_ = 0.0;
    double last_tau_ = 0.0;
    PolicyReferenceInterpolator reference_;
};

class EidMultiJointController final : public IController {
public:
    explicit EidMultiJointController(RuntimeConfig cfg)
        : safety_(cfg.safety) {
        for (int joint_id : activeControllerJoints(cfg)) {
            controllers_.emplace_back(*cfg.controller.joints[joint_id]);
        }
    }

    std::string name() const override {
        std::ostringstream out;
        out << "EidMultiJointController(" << controllers_.size() << " joints)";
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

    const std::vector<EidJointController>& jointControllers() const {
        return controllers_;
    }

private:
    SafetyConfig safety_;
    std::vector<EidJointController> controllers_;
};

}  // namespace h1if
