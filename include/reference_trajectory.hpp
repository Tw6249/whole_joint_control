#pragma once

#include <algorithm>
#include <cmath>

namespace h1if {

struct PolicyPoint {
    double q = 0.0;
    double dq = 0.0;
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
    }

    JointReferencePair sample(double t, double dt, double q, double dq) {
        const double ts = std::max(dt, 1.0e-6);
        const double t_policy = std::max(cfg_.policy_dt, ts);
        const double t_now = std::max(t, 0.0);

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
        segment.target = policyPoint(start_t);
        if (cfg_.interpolation == PolicyInterpolation::ClosedLoop) {
            segment.start = {q, dq};
        } else if (index <= 0.0) {
            segment.start = policyPoint(0.0);
        } else {
            segment.start = policyPoint((index - 1.0) * t_policy);
        }
        return segment;
    }

    PolicyPoint policyPoint(double t) const {
        constexpr double pi = 3.14159265358979323846;
        t = std::max(t, 0.0);

        if (cfg_.source == PolicySource::Hold) {
            return {cfg_.center, 0.0};
        }
        if (cfg_.source == PolicySource::Sine) {
            const double omega = 2.0 * pi * cfg_.frequency_hz;
            const double arg = omega * t + cfg_.phase_rad;
            return {
                cfg_.center + cfg_.amplitude * std::sin(arg),
                cfg_.amplitude * omega * std::cos(arg),
            };
        }
        if (cfg_.source == PolicySource::Step) {
            return {t < cfg_.step_time_s ? cfg_.center : cfg_.center + cfg_.amplitude, 0.0};
        }
        return {cfg_.center, 0.0};
    }

    static JointReference evalQuinticReference(const PolicyPoint& start,
                                               const PolicyPoint& target,
                                               double t_total,
                                               double tau) {
        JointReference r;
        double ddq = 0.0;
        evalQuintic(start.q, start.dq, 0.0, target.q, target.dq, 0.0, t_total, tau, r.q, r.dq, ddq);
        return r;
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

    PolicyReferenceConfig cfg_;
    SegmentState segment_;
};

}  // namespace h1if
