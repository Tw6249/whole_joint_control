#pragma once

#include "controller_interface.hpp"
#include "reference_trajectory.hpp"
#include "runtime_config.hpp"
#include "safety.hpp"

#include <algorithm>
#include <cmath>

namespace h1if {

class EidSingleJointController final : public IController {
public:
    EidSingleJointController(EidControllerConfig cfg, PlantModelConfig model)
        : cfg_(cfg), model_(model), reference_(makeReferenceConfig(cfg, model)) {}

    std::string name() const override {
        return "EidSingleJointController";
    }

    void reset(const RobotState& state) override {
        const int j = cfg_.target_joint;
        t0_ = state.t;
        x_hat_q_ = 0.0;
        x_hat_dq_ = 0.0;
        eta_q_ = state.joint[j].q;
        eta_dq_ = state.joint[j].dq;
        eta_lpf_q_ = 0.0;
        eta_lpf_dq_ = 0.0;
        q_start_ = state.joint[j].q;
        dq_start_ = state.joint[j].dq;
        last_tau_ = 0.0;
        reference_.configure(makeReferenceConfig(cfg_, model_));
        reference_.reset();
        initialized_ = true;
    }

    void step(const RobotState& state, RobotCommand& command, ControllerDebug& debug) override {
        if (!initialized_) {
            reset(state);
        }

        for (int i = 0; i < kMaxMotors; ++i) {
            command.joint[i].mode = h1MotorMode(i);
            command.joint[i].q = static_cast<float>(state.joint[i].q);
            command.joint[i].dq = 0.0f;
            command.joint[i].kp = static_cast<float>(cfg_.torque_safe_kp);
            command.joint[i].kd = static_cast<float>(cfg_.torque_safe_kd);
            command.joint[i].tau = 0.0f;
            command.joint[i].enable = true;
        }

        const int j = cfg_.target_joint;
        const double q = state.joint[j].q;
        const double dq = state.joint[j].dq;
        const double t = state.t - t0_;
        const double dt = cfg_.control_dt;

        const JointReferencePair raw_ref = reference_.sample(t, dt);
        const JointReferencePair ref = shapeStartupReference(raw_ref, t, dt);
        const StepResult result = controllerStep(q, dq, ref, dt);

        auto& c = command.joint[j];
        c.mode = h1MotorMode(j);
        c.q = static_cast<float>(q);
        c.dq = 0.0f;
        c.kp = static_cast<float>(cfg_.torque_safe_kp);
        c.kd = static_cast<float>(cfg_.torque_safe_kd);
        c.tau = static_cast<float>(result.u_t);

        debug.data[0] = ref.now.q;
        debug.data[1] = ref.now.dq;
        debug.data[2] = q;
        debug.data[3] = dq;
        debug.data[4] = ref.now.q - q;
        debug.data[5] = ref.now.dq - dq;
        debug.data[6] = result.u_star;
        debug.data[7] = result.u_feedback;
        debug.data[8] = result.u_t;
        debug.data[9] = result.eta_q;
        debug.data[10] = result.eta_dq;
        debug.data[11] = result.x_hat_q;
        debug.data[12] = result.x_hat_dq;
        debug.data[13] = result.rho_q;
        debug.data[14] = result.rho_dq;
        debug.data[15] = result.x_bar_q;
        debug.data[16] = ref.next.q;
        debug.data[17] = ref.next.dq;
        debug.data[18] = result.x_bar_dq;
        debug.data[19] = result.r_d_q;
        debug.data[20] = result.r_d_dq;
        debug.data[21] = result.e_q;
        debug.data[22] = result.e_dq;
        debug.data[23] = result.observer_qacc;
        debug.data[24] = result.observer_tau_applied;
        debug.data[25] = result.u_raw;
        debug.data[26] = raw_ref.now.q;
        debug.data[27] = raw_ref.now.dq;
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
    };

    StepResult controllerStep(double q, double dq, const JointReferencePair& ref, double dt) {
        const double eta_q = eta_q_;
        const double eta_dq = eta_dq_;
        const double x_hat_q = x_hat_q_;
        const double x_hat_dq = x_hat_dq_;
        const double x_bar_q = x_hat_q + eta_q;
        const double x_bar_dq = x_hat_dq + eta_dq;

        const double r_c_q_next = ref.next.q - eta_q;
        const double r_c_dq_next = ref.next.dq - eta_dq;
        const double delta_r_c_q = r_c_q_next - ref.now.q;
        const double delta_r_c_dq = r_c_dq_next - ref.now.dq;

        const InverseResult inv =
            analyticInverseModel(ref.now.q, ref.now.dq, delta_r_c_q, delta_r_c_dq, dt);

        const double den = cfg_.kp * cfg_.kp + cfg_.kd * cfg_.kd;
        const double w_q = den < 1.0e-12 ? 0.0 : cfg_.kp / den;
        const double w_dq = den < 1.0e-12 ? 0.0 : cfg_.kd / den;
        const double r_d_q = ref.now.q + w_q * inv.u_star;
        const double r_d_dq = ref.now.dq + w_dq * inv.u_star;

        const double e_q = r_d_q - x_bar_q;
        const double e_dq = r_d_dq - x_bar_dq;
        const double u_raw = cfg_.kp * e_q + cfg_.kd * e_dq;
        const double u_t = limitTorqueCommand(u_raw, dt);

        const ForwardStep pred = kneeForward(x_bar_q, x_bar_dq, u_t, dt);
        const double tilde_x_q = q - x_bar_q;
        const double tilde_x_dq = dq - x_bar_dq;
        const double eta_next_q =
            cfg_.filter_alpha * cfg_.observer_gain_q * tilde_x_q +
            (1.0 - cfg_.filter_alpha) * eta_lpf_q_;
        const double eta_next_dq =
            cfg_.filter_alpha * cfg_.observer_gain_dq * tilde_x_dq +
            (1.0 - cfg_.filter_alpha) * eta_lpf_dq_;

        StepResult out;
        out.u_star = inv.u_star;
        out.u_feedback = cfg_.kp * (ref.now.q - x_bar_q) + cfg_.kd * (ref.now.dq - x_bar_dq);
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
        const double ramp = cfg_.startup_ramp_duration;
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
        const double tau_limit = std::min(std::abs(cfg_.eid_tau_limit), model_.tau_max);
        double limited = clamp(tau, -tau_limit, tau_limit);
        const double max_delta = std::max(0.0, cfg_.eid_tau_slew_rate) * std::max(dt, 0.0);
        if (max_delta > 0.0) {
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
        const double q_target_next = q + delta_q;
        const double dq_target_next = dq + delta_dq;
        const double bias = model_.b * dq + gravityTorque(q) + model_.tau0;

        const double tau_from_q =
            bias + model_.Jeff * ((q_target_next - q - dt * dq) / (dt * dt));
        const double tau_from_dq =
            bias + model_.Jeff * ((dq_target_next - dq) / dt);

        const double q_weight =
            cfg_.inverse_q_weight > 0.0 ? cfg_.inverse_q_weight : 0.5 / (dt * dt);
        const double dq_weight =
            cfg_.inverse_dq_weight > 0.0 ? cfg_.inverse_dq_weight : 1.0;
        const double aq = dt * dt / model_.Jeff;
        const double adq = dt / model_.Jeff;
        const double den = q_weight * aq * aq + dq_weight * adq * adq;

        double u_star = 0.0;
        if (den >= 1.0e-12) {
            u_star = (q_weight * aq * aq * tau_from_q +
                      dq_weight * adq * adq * tau_from_dq) / den;
        }
        u_star = clamp(u_star, -model_.tau_max, model_.tau_max);

        const ForwardStep pred = kneeForward(q, dq, u_star, dt);

        InverseResult result;
        result.u_star = u_star;
        result.rho_q = pred.q_next - q_target_next;
        result.rho_dq = pred.dq_next - dq_target_next;
        return result;
    }

    ForwardStep kneeForward(double q, double dq, double tau_raw, double dt) const {
        const double tau = clamp(tau_raw, -model_.tau_max, model_.tau_max);
        const double qacc = (tau - model_.b * dq - gravityTorque(q) - model_.tau0) / model_.Jeff;
        double dq_next = dq + dt * qacc;
        double q_next = q + dt * dq_next;

        if (q_next < model_.q_min) {
            q_next = model_.q_min;
            if (dq_next < 0.0) {
                dq_next = 0.0;
            }
        } else if (q_next > model_.q_max) {
            q_next = model_.q_max;
            if (dq_next > 0.0) {
                dq_next = 0.0;
            }
        }

        return {q_next, dq_next, qacc, tau};
    }

    double gravityTorque(double q) const {
        return model_.gravityA * std::sin(q) + model_.gravityB * std::cos(q);
    }

    static ReferenceTrajectoryConfig makeReferenceConfig(const EidControllerConfig& cfg,
                                                         const PlantModelConfig& model) {
        constexpr double kRefMin = 0.0;
        constexpr double kRefMax = 1.5;
        constexpr double kMaxFrequency = 0.8;
        constexpr double kDefaultFrequency = 0.05;
        constexpr double kStartAtMinPhase = -1.57079632679489661923;

        const double q_min = std::max(kRefMin, model.q_min);
        const double q_max = std::min(kRefMax, model.q_max);
        const double default_center = 0.5 * (q_min + q_max);
        const double default_amplitude = 0.5 * (q_max - q_min);

        ReferenceTrajectoryConfig ref;
        ref.policy_dt = cfg.policy_reference_dt;
        if (q_max > q_min) {
            ref.center = std::isfinite(cfg.ref_center)
                             ? clamp(cfg.ref_center, q_min, q_max)
                             : default_center;

            const double requested_amplitude =
                (std::isfinite(cfg.ref_amplitude) && cfg.ref_amplitude > 0.0)
                    ? cfg.ref_amplitude
                    : default_amplitude;
            const double max_amplitude = std::min(ref.center - q_min, q_max - ref.center);
            ref.amplitude = clamp(requested_amplitude, 0.0, max_amplitude);
        } else {
            ref.center = clamp(cfg.ref_center, model.q_min, model.q_max);
            ref.amplitude = 0.0;
        }

        const double requested_frequency =
            (std::isfinite(cfg.ref_frequency) && cfg.ref_frequency > 0.0)
                ? cfg.ref_frequency
                : kDefaultFrequency;
        ref.frequency = std::min(requested_frequency, kMaxFrequency);
        ref.phase = std::isfinite(cfg.ref_phase) ? cfg.ref_phase : kStartAtMinPhase;
        return ref;
    }

    static double clamp(double x, double lo, double hi) {
        return std::max(lo, std::min(x, hi));
    }

    EidControllerConfig cfg_;
    PlantModelConfig model_;
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
    SmoothSineReferenceTrajectory reference_;
};

}  // namespace h1if
