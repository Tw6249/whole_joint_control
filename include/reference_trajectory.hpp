#pragma once

#include <ruckig/ruckig.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace h1if {

struct PolicyPoint {
    double q = 0.0;
    double dq = 0.0;
    double ddq = 0.0;
};

using JointReference = PolicyPoint;

struct JointReferencePair {
    JointReference now;
    JointReference next;
};

enum class PolicySource {
    Hold,
    Sine,
    Step,
};

enum class PolicyInterpolation {
    OpenLoop,
    ClosedLoop,
    Ruckig,
    RlSmoothed,
};

enum class RuckigTargetVelocity {
    Policy,
    Zero,
};

struct PolicyReferenceConfig {
    PolicyInterpolation interpolation = PolicyInterpolation::OpenLoop;
    PolicySource source = PolicySource::Sine;
    double policy_dt = 0.05;
    double center = 0.9;
    double amplitude = 0.08;
    double frequency_hz = 0.10;
    double phase_rad = 0.0;
    double step_time_s = 1.0;
    double max_velocity = 0.0;
    double max_acceleration = 0.0;
    double max_jerk = 0.0;
    double rl_velocity_alpha = 0.35;
    double rl_acceleration_alpha = 0.25;
    double rl_target_acceleration_blend = 0.5;
    RuckigTargetVelocity ruckig_target_velocity = RuckigTargetVelocity::Policy;
};

class PolicyReferenceInterpolator {
public:
    PolicyReferenceInterpolator() = default;
    explicit PolicyReferenceInterpolator(PolicyReferenceConfig cfg) : cfg_(cfg) {}

    void configure(PolicyReferenceConfig cfg) {
        cfg_ = cfg;
        reset();
    }

    void reset() {
        segment_ = {};
        rl_initialized_ = false;
        rl_last_policy_index_ = -1.0;
        rl_last_policy_q_ = 0.0;
        rl_target_velocity_ = 0.0;
        rl_target_acceleration_ = 0.0;
        rl_ref_position_ = 0.0;
        rl_ref_velocity_ = 0.0;
        rl_ref_acceleration_ = 0.0;
        ruckig_initialized_ = false;
        ruckig_position_ = 0.0;
        ruckig_velocity_ = 0.0;
        ruckig_acceleration_ = 0.0;
        ruckig_otg_.reset();
        ruckig_output_ = {};
    }

    JointReferencePair sample(double t, double dt, double q, double dq) {
        const double ts = std::max(dt, 1.0e-6);
        const double t_policy = std::max(cfg_.policy_dt, ts);
        const double t_now = std::max(t, 0.0);

        if (cfg_.interpolation == PolicyInterpolation::RlSmoothed) {
            return sampleRlSmoothed(t_now, ts, t_policy, q, dq);
        }
        if (cfg_.interpolation == PolicyInterpolation::Ruckig) {
            return sampleRuckig(t_now, ts, t_policy, q, dq);
        }

        JointReferencePair out;
        const double index_now = segmentIndex(t_now, t_policy);
        const double index_next = segmentIndex(t_now + ts, t_policy);
        ensureSegment(index_now, index_now * t_policy, t_policy, q, dq);
        out.now = evalSegment(segment_, t_now, t_policy);
        if (index_next == index_now) {
            out.next = evalSegment(segment_, t_now + ts, t_policy);
        } else {
            const SegmentState preview = makeSegment(index_next, index_next * t_policy, t_policy, q, dq);
            out.next = evalSegment(preview, t_now + ts, t_policy);
        }
        return out;
    }

private:
    struct SegmentState {
        bool initialized = false;
        double index = -1.0;
        double start_t = 0.0;
        PolicyPoint start;
        PolicyPoint target;
    };

    struct RuckigStep {
        JointReference ref;
        double ddq = 0.0;
    };

    static double segmentIndex(double t, double t_policy) {
        return std::floor((t + 1.0e-12) / t_policy);
    }

    JointReference evalSegment(const SegmentState& segment, double t, double t_policy) const {
        return evalQuinticReference(segment.start,
                                    segment.target,
                                    t_policy,
                                    clamp(t - segment.start_t, 0.0, t_policy));
    }

    void ensureSegment(double index, double start_t, double t_policy, double q, double dq) {
        if (segment_.initialized && index == segment_.index) {
            return;
        }
        segment_ = makeSegment(index, start_t, t_policy, q, dq);
    }

    SegmentState makeSegment(double index, double start_t, double t_policy, double q, double dq) const {
        SegmentState segment;
        segment.initialized = true;
        segment.index = index;
        segment.start_t = start_t;
        segment.target = positionOnlyPoint(start_t, t_policy);
        if (cfg_.interpolation == PolicyInterpolation::ClosedLoop) {
            segment.start = {q, dq, 0.0};
        } else if (index <= 0.0) {
            segment.start = positionOnlyPoint(0.0, t_policy);
        } else {
            segment.start = positionOnlyPoint((index - 1.0) * t_policy, t_policy);
        }
        return segment;
    }

    double policyPosition(double t) const {
        constexpr double pi = 3.14159265358979323846;
        t = std::max(t, 0.0);

        if (cfg_.source == PolicySource::Hold) {
            return cfg_.center;
        }
        if (cfg_.source == PolicySource::Sine) {
            const double omega = 2.0 * pi * cfg_.frequency_hz;
            const double arg = omega * t + cfg_.phase_rad;
            return cfg_.center + cfg_.amplitude * std::sin(arg);
        }
        if (cfg_.source == PolicySource::Step) {
            return t < cfg_.step_time_s ? cfg_.center : cfg_.center + cfg_.amplitude;
        }
        return cfg_.center;
    }

    PolicyPoint positionOnlyPoint(double t, double t_policy) const {
        const double q = policyPosition(t);
        if (t <= 0.0) {
            return {q, 0.0, 0.0};
        }
        const double prev_t = std::max(0.0, t - t_policy);
        const double dq = (q - policyPosition(prev_t)) / std::max(t - prev_t, 1.0e-6);
        return {q, dq, 0.0};
    }

    JointReferencePair sampleRlSmoothed(double t, double dt, double t_policy, double q, double dq) {
        validateRlSmoothedConfig();

        const double index_now = segmentIndex(t, t_policy);
        if (!rl_initialized_) {
            const PolicyPoint initial = positionOnlyPoint(index_now * t_policy, t_policy);
            rl_initialized_ = true;
            rl_last_policy_index_ = index_now;
            rl_last_policy_q_ = initial.q;
            rl_target_velocity_ = 0.0;
            rl_target_acceleration_ = 0.0;
            rl_ref_position_ = q;
            rl_ref_velocity_ = dq;
            rl_ref_acceleration_ = 0.0;
            segment_ = makeRlSegment(index_now, index_now * t_policy, t_policy);
        } else if (!segment_.initialized || index_now != rl_last_policy_index_) {
            segment_ = makeRlSegment(index_now, index_now * t_policy, t_policy);
        }

        double ddq_now = 0.0;
        JointReference now;
        evalQuintic(segment_.start.q,
                    segment_.start.dq,
                    segment_.start.ddq,
                    segment_.target.q,
                    segment_.target.dq,
                    segment_.target.ddq,
                    t_policy,
                    clamp(t - segment_.start_t, 0.0, t_policy),
                    now.q,
                    now.dq,
                    ddq_now);
        rl_ref_position_ = now.q;
        rl_ref_velocity_ = now.dq;
        rl_ref_acceleration_ = ddq_now;

        const double t_next = t + dt;
        const double index_next = segmentIndex(t_next, t_policy);
        SegmentState next_segment = segment_;
        if (index_next != index_now) {
            next_segment = makeRlPreviewSegment(index_next, index_next * t_policy, t_policy, now.q, now.dq, ddq_now);
        }

        double ddq_next = 0.0;
        JointReference next;
        evalQuintic(next_segment.start.q,
                    next_segment.start.dq,
                    next_segment.start.ddq,
                    next_segment.target.q,
                    next_segment.target.dq,
                    next_segment.target.ddq,
                    t_policy,
                    clamp(t_next - next_segment.start_t, 0.0, t_policy),
                    next.q,
                    next.dq,
                    ddq_next);
        return {now, next};
    }

    SegmentState makeRlSegment(double index, double start_t, double t_policy) {
        SegmentState segment = makeRlPreviewSegment(index,
                                                    start_t,
                                                    t_policy,
                                                    rl_ref_position_,
                                                    rl_ref_velocity_,
                                                    rl_ref_acceleration_);
        rl_last_policy_index_ = index;
        rl_last_policy_q_ = segment.target.q;
        rl_target_velocity_ = segment.target.dq;
        rl_target_acceleration_ = segment.target.ddq;
        return segment;
    }

    SegmentState makeRlPreviewSegment(double index,
                                      double start_t,
                                      double t_policy,
                                      double start_q,
                                      double start_dq,
                                      double start_ddq) const {
        SegmentState segment;
        segment.initialized = true;
        segment.index = index;
        segment.start_t = start_t;
        segment.start = {start_q, start_dq, start_ddq};

        const double target_q = policyPosition(start_t + t_policy);
        const double previous_policy_q = index <= 0.0 ? policyPosition(0.0) : rl_last_policy_q_;
        const double raw_velocity = (target_q - previous_policy_q) / t_policy;
        const double limited_velocity = clamp(raw_velocity, -cfg_.max_velocity, cfg_.max_velocity);
        const double target_velocity =
            lowpass(rl_target_velocity_, limited_velocity, cfg_.rl_velocity_alpha);
        const double raw_acceleration = (target_velocity - rl_target_velocity_) / t_policy;
        const double limited_acceleration = clamp(raw_acceleration, -cfg_.max_acceleration, cfg_.max_acceleration);
        const double filtered_acceleration =
            lowpass(rl_target_acceleration_, limited_acceleration, cfg_.rl_acceleration_alpha);
        const double blended_acceleration =
            (1.0 - cfg_.rl_target_acceleration_blend) * filtered_acceleration +
            cfg_.rl_target_acceleration_blend * start_ddq;

        segment.target = {
            target_q,
            target_velocity,
            clamp(blended_acceleration, -cfg_.max_acceleration, cfg_.max_acceleration),
        };
        return segment;
    }

    JointReferencePair sampleRuckig(double t, double dt, double t_policy, double q, double dq) {
        validateRuckigConfig();

        if (!ruckig_initialized_) {
            ruckig_position_ = q;
            ruckig_velocity_ = dq;
            ruckig_acceleration_ = 0.0;
            ruckig_initialized_ = true;
            ruckig_otg_.reset();
        }

        ruckig_otg_.delta_time = dt;
        const double target_time = (segmentIndex(t, t_policy) + 1.0) * t_policy;
        const double minimum_duration = std::max(target_time - t, dt);
        const RuckigStep now = stepRuckig(ruckig_otg_,
                                          ruckig_output_,
                                          target_time,
                                          t_policy,
                                          minimum_duration,
                                          ruckig_position_,
                                          ruckig_velocity_,
                                          ruckig_acceleration_);
        ruckig_position_ = now.ref.q;
        ruckig_velocity_ = now.ref.dq;
        ruckig_acceleration_ = now.ddq;

        ruckig::Ruckig<1> preview_otg{dt};
        ruckig::OutputParameter<1> preview_output;
        const double next_target_time = (segmentIndex(t + dt, t_policy) + 1.0) * t_policy;
        const double next_minimum_duration = std::max(next_target_time - (t + dt), dt);
        const RuckigStep next = stepRuckig(preview_otg,
                                           preview_output,
                                           next_target_time,
                                           t_policy,
                                           next_minimum_duration,
                                           ruckig_position_,
                                           ruckig_velocity_,
                                           ruckig_acceleration_);
        return {now.ref, next.ref};
    }

    RuckigStep stepRuckig(ruckig::Ruckig<1>& otg,
                          ruckig::OutputParameter<1>& output,
                          double target_time,
                          double t_policy,
                          double minimum_duration,
                          double q,
                          double dq,
                          double ddq) {
        ruckig::InputParameter<1> input;

        const PolicyPoint target = positionOnlyPoint(target_time, t_policy);
        if (std::abs(q - target.q) < 1.0e-12 &&
            std::abs(dq - target.dq) < 1.0e-12 &&
            std::abs(ddq) < 1.0e-12) {
            otg.reset();
            return {target, 0.0};
        }

        input.current_position = {q};
        input.current_velocity = {clamp(dq, -cfg_.max_velocity, cfg_.max_velocity)};
        input.current_acceleration = {clamp(ddq, -cfg_.max_acceleration, cfg_.max_acceleration)};
        input.target_position = {target.q};
        const double target_velocity =
            cfg_.ruckig_target_velocity == RuckigTargetVelocity::Zero ? 0.0 : target.dq;
        input.target_velocity = {clamp(target_velocity, -cfg_.max_velocity, cfg_.max_velocity)};
        input.target_acceleration = {0.0};
        input.max_velocity = {cfg_.max_velocity};
        input.max_acceleration = {cfg_.max_acceleration};
        input.max_jerk = {cfg_.max_jerk};
        input.minimum_duration = minimum_duration;

        const ruckig::Result result = otg.update(input, output);
        if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
            std::ostringstream msg;
            msg << "ruckig policy interpolation failed with result "
                << static_cast<int>(result)
                << " current=(" << q << ", " << dq << ", " << ddq << ")"
                << " target=(" << input.target_position[0] << ", "
                << input.target_velocity[0] << ", " << input.target_acceleration[0] << ")"
                << " limits=(" << cfg_.max_velocity << ", "
                << cfg_.max_acceleration << ", " << cfg_.max_jerk << ")";
            throw std::runtime_error(msg.str());
        }

        return {{output.new_position[0], output.new_velocity[0]}, output.new_acceleration[0]};
    }

    void validateRuckigConfig() const {
        if (cfg_.max_velocity <= 0.0 || cfg_.max_acceleration <= 0.0 || cfg_.max_jerk <= 0.0 ||
            !std::isfinite(cfg_.max_velocity) || !std::isfinite(cfg_.max_acceleration) ||
            !std::isfinite(cfg_.max_jerk)) {
            throw std::runtime_error("ruckig policy interpolation requires positive finite velocity, acceleration, and jerk limits");
        }
    }

    static JointReference evalQuinticReference(const PolicyPoint& start,
                                               const PolicyPoint& target,
                                               double t_total,
                                               double tau) {
        JointReference r;
        double ddq = 0.0;
        evalQuintic(start.q, start.dq, start.ddq, target.q, target.dq, target.ddq, t_total, tau, r.q, r.dq, ddq);
        return r;
    }

    void validateRlSmoothedConfig() const {
        if (cfg_.max_velocity <= 0.0 || cfg_.max_acceleration <= 0.0 ||
            !std::isfinite(cfg_.max_velocity) || !std::isfinite(cfg_.max_acceleration)) {
            throw std::runtime_error("rl_smoothed policy interpolation requires positive finite velocity and acceleration limits");
        }
        if (cfg_.rl_velocity_alpha < 0.0 || cfg_.rl_velocity_alpha > 1.0 ||
            cfg_.rl_acceleration_alpha < 0.0 || cfg_.rl_acceleration_alpha > 1.0 ||
            cfg_.rl_target_acceleration_blend < 0.0 || cfg_.rl_target_acceleration_blend > 1.0 ||
            !std::isfinite(cfg_.rl_velocity_alpha) ||
            !std::isfinite(cfg_.rl_acceleration_alpha) ||
            !std::isfinite(cfg_.rl_target_acceleration_blend)) {
            throw std::runtime_error("rl_smoothed alpha/blend parameters must be finite values in [0, 1]");
        }
    }

    static void evalQuintic(double q0,
                            double dq0,
                            double ddq0,
                            double q1,
                            double dq1,
                            double ddq1,
                            double t_total,
                            double tau,
                            double& q,
                            double& dq,
                            double& ddq) {
        const double a0 = q0;
        const double a1 = dq0;
        const double a2 = 0.5 * ddq0;
        const double t2 = t_total * t_total;
        const double t3 = t2 * t_total;

        const double b0 = q1 - (a0 + a1 * t_total + a2 * t2);
        const double b1 = dq1 - (a1 + 2.0 * a2 * t_total);
        const double b2 = ddq1 - 2.0 * a2;

        const double c0 = b0 / t3;
        const double c1 = b1 / t2;
        const double c2 = b2 / t_total;
        const double a3 = 10.0 * c0 - 4.0 * c1 + 0.5 * c2;
        const double a4 = (-15.0 * c0 + 7.0 * c1 - c2) / t_total;
        const double a5 = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / t2;

        tau = clamp(tau, 0.0, t_total);
        const double tau2 = tau * tau;
        const double tau3 = tau2 * tau;
        const double tau4 = tau3 * tau;
        const double tau5 = tau4 * tau;
        q = a0 + a1 * tau + a2 * tau2 + a3 * tau3 + a4 * tau4 + a5 * tau5;
        dq = a1 + 2.0 * a2 * tau + 3.0 * a3 * tau2 + 4.0 * a4 * tau3 +
             5.0 * a5 * tau4;
        ddq = 2.0 * a2 + 6.0 * a3 * tau + 12.0 * a4 * tau2 + 20.0 * a5 * tau3;
    }

    static double clamp(double x, double lo, double hi) {
        return std::max(lo, std::min(x, hi));
    }

    static double lowpass(double previous, double current, double alpha) {
        return previous + alpha * (current - previous);
    }

    PolicyReferenceConfig cfg_;
    SegmentState segment_;
    bool rl_initialized_ = false;
    double rl_last_policy_index_ = -1.0;
    double rl_last_policy_q_ = 0.0;
    double rl_target_velocity_ = 0.0;
    double rl_target_acceleration_ = 0.0;
    double rl_ref_position_ = 0.0;
    double rl_ref_velocity_ = 0.0;
    double rl_ref_acceleration_ = 0.0;
    bool ruckig_initialized_ = false;
    double ruckig_position_ = 0.0;
    double ruckig_velocity_ = 0.0;
    double ruckig_acceleration_ = 0.0;
    ruckig::Ruckig<1> ruckig_otg_{0.001};
    ruckig::OutputParameter<1> ruckig_output_;
};

}  // namespace h1if
