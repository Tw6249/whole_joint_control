#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace h1if {

struct JointReference {
    double q = 0.0;
    double dq = 0.0;
};

struct JointReferencePair {
    JointReference now;
    JointReference next;
};

struct ReferenceTrajectoryConfig {
    double policy_dt = 0.05;
    double center = 0.9;
    double amplitude = 0.08;
    double frequency = 0.10;
    double phase = 0.0;
};

class SmoothSineReferenceTrajectory {
public:
    SmoothSineReferenceTrajectory() = default;
    explicit SmoothSineReferenceTrajectory(ReferenceTrajectoryConfig cfg) : cfg_(cfg) {}

    void configure(ReferenceTrajectoryConfig cfg) {
        cfg_ = cfg;
    }

    void reset() {
        plan_ = {};
    }

    JointReferencePair sample(double t, double dt) {
        const double ts = std::max(dt, 1.0e-6);
        const double t_policy = std::max(cfg_.policy_dt, ts);

        JointReferencePair out;
        shapedPolicyReference(std::max(t, 0.0), ts, t_policy, out.now, out.next);
        return out;
    }

private:
    struct PlanState {
        static constexpr int kMaxNodes = 64;
        bool initialized = false;
        double last_segment = -1.0;
        double last_t = -1.0;
        double last_center = 0.0;
        double last_amplitude = 0.0;
        double last_frequency = 0.0;
        double last_phase = 0.0;
        double last_ts = -1.0;
        double last_t_policy = -1.0;
        int n_nodes = 2;
        double node_dt = 0.0;
        std::array<double, kMaxNodes> q_nodes{};
        std::array<double, kMaxNodes> dq_nodes{};
        std::array<double, kMaxNodes> ddq_nodes{};
    };

    void shapedPolicyReference(double t,
                               double ts,
                               double t_policy,
                               JointReference& now,
                               JointReference& next) {
        const double segment = std::floor((t + 1.0e-12) / t_policy);
        const bool params_changed =
            std::abs(cfg_.center - plan_.last_center) > 1.0e-12 ||
            std::abs(cfg_.amplitude - plan_.last_amplitude) > 1.0e-12 ||
            std::abs(cfg_.frequency - plan_.last_frequency) > 1.0e-12 ||
            std::abs(cfg_.phase - plan_.last_phase) > 1.0e-12 ||
            std::abs(ts - plan_.last_ts) > 1.0e-12 ||
            std::abs(t_policy - plan_.last_t_policy) > 1.0e-12;

        if (t < plan_.last_t - 0.5 * ts || params_changed) {
            plan_.initialized = false;
            plan_.last_segment = -1.0;
        }

        if (!plan_.initialized || segment != plan_.last_segment) {
            const double segment_start = segment * t_policy;
            double q0 = 0.0;
            double dq0 = 0.0;
            double ddq0 = 0.0;

            if (plan_.initialized && segment == plan_.last_segment + 1.0) {
                const int last = std::max(0, plan_.n_nodes - 1);
                q0 = plan_.q_nodes[last];
                dq0 = plan_.dq_nodes[last];
                ddq0 = plan_.ddq_nodes[last];
            } else {
                const JointReference raw0 = rawPolicyReference(segment_start, t_policy);
                q0 = raw0.q;
                dq0 = raw0.dq;
            }

            const JointReference target = rawPolicyReference(segment_start + t_policy, t_policy);
            buildQuinticPlanNodes(q0, dq0, ddq0, target.q, target.dq, 0.0, ts, t_policy);

            plan_.initialized = true;
            plan_.last_segment = segment;
            plan_.last_center = cfg_.center;
            plan_.last_amplitude = cfg_.amplitude;
            plan_.last_frequency = cfg_.frequency;
            plan_.last_phase = cfg_.phase;
            plan_.last_ts = ts;
            plan_.last_t_policy = t_policy;
        }

        const double tau = clamp(t - segment * t_policy, 0.0, t_policy);
        const double tau_next = std::min(tau + ts, t_policy);
        now = evalPlanNodes(tau);
        next = evalPlanNodes(tau_next);
        plan_.last_t = t;
    }

    void buildQuinticPlanNodes(double q0,
                               double dq0,
                               double ddq0,
                               double q1,
                               double dq1,
                               double ddq1,
                               double ts,
                               double t_policy) {
        constexpr int kMaxNodes = PlanState::kMaxNodes;
        const int rounded_nodes = static_cast<int>(std::llround(t_policy / ts)) + 1;
        plan_.n_nodes = std::min(kMaxNodes, std::max(2, rounded_nodes));
        plan_.node_dt = t_policy / static_cast<double>(plan_.n_nodes - 1);
        plan_.q_nodes.fill(0.0);
        plan_.dq_nodes.fill(0.0);
        plan_.ddq_nodes.fill(0.0);

        for (int i = 0; i < plan_.n_nodes; ++i) {
            const double tau = std::min(static_cast<double>(i) * plan_.node_dt, t_policy);
            evalQuintic(q0,
                        dq0,
                        ddq0,
                        q1,
                        dq1,
                        ddq1,
                        t_policy,
                        tau,
                        plan_.q_nodes[i],
                        plan_.dq_nodes[i],
                        plan_.ddq_nodes[i]);
        }
    }

    JointReference evalPlanNodes(double tau) const {
        JointReference r;
        if (plan_.n_nodes <= 1 || plan_.node_dt <= 0.0) {
            return r;
        }

        int idx = static_cast<int>(std::floor((tau + 1.0e-12) / plan_.node_dt));
        if (idx >= plan_.n_nodes - 1) {
            idx = plan_.n_nodes - 1;
            r.q = plan_.q_nodes[idx];
            r.dq = plan_.dq_nodes[idx];
            return r;
        }

        const double alpha =
            clamp((tau - static_cast<double>(idx) * plan_.node_dt) / plan_.node_dt,
                  0.0,
                  1.0);
        r.q = plan_.q_nodes[idx] + alpha * (plan_.q_nodes[idx + 1] - plan_.q_nodes[idx]);
        r.dq = plan_.dq_nodes[idx] + alpha * (plan_.dq_nodes[idx + 1] - plan_.dq_nodes[idx]);
        return r;
    }

    JointReference rawPolicyReference(double t, double t_policy) const {
        t = std::max(t, 0.0);
        const double policy_index = std::floor((t + 1.0e-12) / t_policy);
        const double t0 = policy_index * t_policy;
        const double q0 = policyPosition(t0);

        JointReference r;
        r.q = q0;
        r.dq = policyVelocity(t0);
        return r;
    }

    double policyPosition(double t) const {
        constexpr double pi = 3.14159265358979323846;
        return cfg_.center +
               cfg_.amplitude * std::sin(2.0 * pi * cfg_.frequency * t + cfg_.phase);
    }

    double policyVelocity(double t) const {
        constexpr double pi = 3.14159265358979323846;
        const double omega = 2.0 * pi * cfg_.frequency;
        return cfg_.amplitude * omega * std::cos(omega * t + cfg_.phase);
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

    ReferenceTrajectoryConfig cfg_;
    PlanState plan_;
};

}  // namespace h1if
