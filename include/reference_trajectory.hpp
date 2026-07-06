#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

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
    PreviewMpc,
    PreviewMpcVelocity,
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
    std::int32_t reference_points = 4;
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
        ensureSegment(index_now, index_now * t_policy, t_policy, ts, q, dq);
        out.now = evalSegment(segment_, t_now, t_policy);
        if (index_next == index_now) {
            out.next = evalSegment(segment_, t_now + ts, t_policy);
        } else {
            const SegmentState preview = makeSegment(index_next, index_next * t_policy, t_policy, ts, q, dq);
            out.next = evalSegment(preview, t_now + ts, t_policy);
        }
        return out;
    }

private:
    struct SegmentState {
        bool initialized = false;
        bool preview_mpc = false;
        double index = -1.0;
        double start_t = 0.0;
        double sample_dt = 0.0;
        PolicyPoint start;
        PolicyPoint target;
        std::vector<PolicyPoint> samples;
    };

    PolicyPoint previewMpcTargetPoint(double t, double t_policy) const {
        if (cfg_.reference_points != 3) {
            throw std::invalid_argument("preview_mpc requires exactly 3 policy_reference_points");
        }

        std::array<double, 3> q{};
        for (int i = 0; i < 3; ++i) {
            q[static_cast<std::size_t>(i)] = policyPosition(t + static_cast<double>(i) * t_policy);
        }

        const double dq = (-3.0 * q[0] + 4.0 * q[1] - q[2]) / (2.0 * t_policy);
        const double ddq = (q[0] - 2.0 * q[1] + q[2]) / (t_policy * t_policy);

        return {q[0], dq, ddq};
    }

    PolicyPoint previewMpcVelocityTargetPoint(double t, double t_policy) const {
        if (cfg_.reference_points != 4) {
            throw std::invalid_argument("preview_mpc_velocity requires exactly 4 policy_reference_points");
        }

        std::array<double, 4> q{};
        for (int i = 0; i < 4; ++i) {
            q[static_cast<std::size_t>(i)] = policyPosition(t + static_cast<double>(i) * t_policy);
        }

        const double dq0 = (q[1] - q[0]) / t_policy;
        const double dq1 = (q[2] - q[1]) / t_policy;
        const double ddq0 = (dq1 - dq0) / t_policy;
        return {q[0], dq0, ddq0};
    }

    static double segmentIndex(double t, double t_policy) {
        return std::floor((t + 1.0e-12) / t_policy);
    }

    JointReference evalSegment(const SegmentState& segment, double t, double t_policy) const {
        if (segment.preview_mpc && !segment.samples.empty()) {
            return evalPreviewSamples(segment, t);
        }
        const double tau = clamp(t - segment.start_t, 0.0, t_policy);
        return evalQuinticReference(segment.start, segment.target, t_policy, tau);
    }

    void ensureSegment(double index, double start_t, double t_policy, double dt, double q, double dq) {
        if (segment_.initialized && index == segment_.index) {
            return;
        }
        segment_ = makeSegment(index, start_t, t_policy, dt, q, dq);
    }

    SegmentState makeSegment(double index, double start_t, double t_policy, double dt, double q, double dq) const {
        SegmentState segment;
        segment.initialized = true;
        segment.index = index;
        segment.start_t = start_t;
        segment.target = policyPoint(start_t, t_policy);
        if (cfg_.interpolation == PolicyInterpolation::PreviewMpc) {
            makePreviewMpcSegment(segment, t_policy, dt);
            return segment;
        }
        if (cfg_.interpolation == PolicyInterpolation::PreviewMpcVelocity) {
            makePreviewMpcVelocitySegment(segment, t_policy, dt);
            return segment;
        }
        if (cfg_.interpolation == PolicyInterpolation::ClosedLoop) {
            segment.start = {q, dq, 0.0};
        } else if (index <= 0.0) {
            segment.start = policyPoint(0.0, t_policy);
        } else {
            segment.start = policyPoint((index - 1.0) * t_policy, t_policy);
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

    PolicyPoint policyPoint(double t, double t_policy) const {
        const double q = policyPosition(t);
        if (cfg_.reference_points <= 1) {
            return {q, 0.0, 0.0};
        }
        if (cfg_.reference_points == 2) {
            const double q_next = policyPosition(t + t_policy);
            return {q, (q_next - q) / t_policy, 0.0};
        }
        if (cfg_.reference_points >= 4) {
            const double q1 = policyPosition(t + t_policy);
            const double q2 = policyPosition(t + 2.0 * t_policy);
            const double q3 = policyPosition(t + 3.0 * t_policy);
            const double dq = (-11.0 * q + 18.0 * q1 - 9.0 * q2 + 2.0 * q3) / (6.0 * t_policy);
            return {q, dq, 0.0};
        }
        return {q, 0.0, 0.0};
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

    void makePreviewMpcSegment(SegmentState& segment, double t_policy, double control_dt) const {
        const int policy_steps = std::max(1, static_cast<int>(std::llround(
                                                 t_policy / std::max(control_dt, 1.0e-6))));
        const double dt = t_policy / static_cast<double>(policy_steps);
        segment.preview_mpc = true;
        segment.sample_dt = dt;
        segment.samples.clear();
        const PolicyPoint target = previewMpcTargetPoint(segment.start_t, t_policy);

        if (segment.index <= 0.0) {
            segment.start = {policyPosition(0.0), 0.0, 0.0};
        } else if (segment_.initialized && segment_.preview_mpc &&
                   std::abs(segment_.index + 1.0 - segment.index) < 0.5 &&
                   !segment_.samples.empty()) {
            segment.start = segment_.samples.back();
        } else {
            segment.start = previewMpcTargetPoint((segment.index - 1.0) * t_policy, t_policy);
        }

        std::vector<double> targets;
        targets.reserve(3);
        for (int i = 0; i < 3; ++i) {
            targets.push_back(policyPosition(segment.start_t + static_cast<double>(i) * t_policy));
        }
        const bool solved = solveSoftPreviewNoTerminalMpc(segment.start, targets, policy_steps, dt, segment.samples);

        if (!solved) {
            fillQuinticFallback(segment, t_policy, policy_steps, target);
        }
        segment.target = segment.samples.empty() ? segment.start : segment.samples.back();
    }

    void makePreviewMpcVelocitySegment(SegmentState& segment, double t_policy, double control_dt) const {
        const int policy_steps = std::max(1, static_cast<int>(std::llround(
                                                 t_policy / std::max(control_dt, 1.0e-6))));
        const double dt = t_policy / static_cast<double>(policy_steps);
        segment.preview_mpc = true;
        segment.sample_dt = dt;
        segment.samples.clear();
        const PolicyPoint target = previewMpcVelocityTargetPoint(segment.start_t, t_policy);

        if (segment.index <= 0.0) {
            segment.start = {policyPosition(0.0), 0.0, 0.0};
        } else if (segment_.initialized && segment_.preview_mpc &&
                   std::abs(segment_.index + 1.0 - segment.index) < 0.5 &&
                   !segment_.samples.empty()) {
            segment.start = segment_.samples.back();
        } else {
            segment.start = previewMpcVelocityTargetPoint((segment.index - 1.0) * t_policy, t_policy);
        }

        std::array<double, 4> q{};
        for (int i = 0; i < 4; ++i) {
            q[static_cast<std::size_t>(i)] =
                policyPosition(segment.start_t + static_cast<double>(i) * t_policy);
        }
        std::vector<double> targets_q{q[0], q[1], q[2], q[3]};
        std::vector<double> targets_dq;
        targets_dq.reserve(3);
        for (int i = 0; i < 3; ++i) {
            targets_dq.push_back((q[static_cast<std::size_t>(i + 1)] -
                                  q[static_cast<std::size_t>(i)]) /
                                 t_policy);
        }

        const bool solved = solveSoftPreviewVelocityMpc(
            segment.start, targets_q, targets_dq, policy_steps, dt, segment.samples);

        if (!solved) {
            fillQuinticFallback(segment, t_policy, policy_steps, target);
        }
        segment.target = segment.samples.empty() ? segment.start : segment.samples.back();
    }

    static JointReference evalPreviewSamples(const SegmentState& segment, double t) {
        if (segment.samples.size() == 1 || segment.sample_dt <= 0.0) {
            return segment.samples.front();
        }
        const double tau = std::max(0.0, t - segment.start_t);
        const double raw = tau / segment.sample_dt;
        const int lo = static_cast<int>(std::floor(raw));
        if (lo <= 0) {
            return segment.samples.front();
        }
        const int last = static_cast<int>(segment.samples.size()) - 1;
        if (lo >= last) {
            return segment.samples.back();
        }
        const double alpha = raw - static_cast<double>(lo);
        const auto& a = segment.samples[static_cast<std::size_t>(lo)];
        const auto& b = segment.samples[static_cast<std::size_t>(lo + 1)];
        return {
            (1.0 - alpha) * a.q + alpha * b.q,
            (1.0 - alpha) * a.dq + alpha * b.dq,
            (1.0 - alpha) * a.ddq + alpha * b.ddq,
        };
    }

    static bool solveSoftPreviewNoTerminalMpc(const PolicyPoint& start,
                                              const std::vector<double>& preview_q,
                                              int policy_steps,
                                              double dt,
                                              std::vector<PolicyPoint>& first_segment) {
        if (preview_q.empty() || policy_steps <= 0 || dt <= 0.0) {
            return false;
        }

        constexpr double w_preview_q = 2.0e7;
        constexpr double w_path_v = 3.0e-3;
        constexpr double w_path_a = 8.0e-5;
        constexpr double w_jerk = 2.0e-9;
        constexpr double w_ridge = 1.0e-10;

        const int horizon_steps = policy_steps * static_cast<int>(preview_q.size());
        if (horizon_steps <= 0 || horizon_steps > 240) {
            return false;
        }

        std::vector<std::vector<double>> aq(horizon_steps, std::vector<double>(horizon_steps, 0.0));
        std::vector<std::vector<double>> av(horizon_steps, std::vector<double>(horizon_steps, 0.0));
        std::vector<std::vector<double>> aa(horizon_steps, std::vector<double>(horizon_steps, 0.0));
        for (int k = 1; k <= horizon_steps; ++k) {
            for (int i = 0; i < k; ++i) {
                const double r = static_cast<double>(k - i);
                aa[static_cast<std::size_t>(k - 1)][static_cast<std::size_t>(i)] = dt;
                av[static_cast<std::size_t>(k - 1)][static_cast<std::size_t>(i)] =
                    0.5 * dt * dt * (r * r - (r - 1.0) * (r - 1.0));
                aq[static_cast<std::size_t>(k - 1)][static_cast<std::size_t>(i)] =
                    (dt * dt * dt / 6.0) * (r * r * r - (r - 1.0) * (r - 1.0) * (r - 1.0));
            }
        }

        std::vector<double> q_base(horizon_steps, 0.0);
        std::vector<double> dq_base(horizon_steps, 0.0);
        std::vector<double> ddq_base(horizon_steps, start.ddq);
        for (int k = 1; k <= horizon_steps; ++k) {
            const double tk = dt * static_cast<double>(k);
            q_base[static_cast<std::size_t>(k - 1)] = start.q + start.dq * tk + 0.5 * start.ddq * tk * tk;
            dq_base[static_cast<std::size_t>(k - 1)] = start.dq + start.ddq * tk;
        }

        std::vector<std::vector<double>> q_mat(horizon_steps, std::vector<double>(horizon_steps, 0.0));
        std::vector<double> c_vec(horizon_steps, 0.0);
        for (int i = 0; i < horizon_steps; ++i) {
            for (int j = 0; j < horizon_steps; ++j) {
                double qij = 0.0;
                for (int k = 0; k < horizon_steps; ++k) {
                    qij += w_path_v * av[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] *
                           av[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)];
                    qij += w_path_a * aa[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] *
                           aa[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)];
                }
                if (i == j) {
                    qij += w_jerk + w_ridge;
                }
                q_mat[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = qij;
            }

            double ci = 0.0;
            for (int k = 0; k < horizon_steps; ++k) {
                ci += w_path_v * av[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] *
                      dq_base[static_cast<std::size_t>(k)];
                ci += w_path_a * aa[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] *
                      ddq_base[static_cast<std::size_t>(k)];
            }
            c_vec[static_cast<std::size_t>(i)] = ci;
        }

        for (std::size_t p = 1; p < preview_q.size(); ++p) {
            const int row_index = static_cast<int>(p + 1) * policy_steps - 1;
            const auto& row = aq[static_cast<std::size_t>(row_index)];
            const double err = q_base[static_cast<std::size_t>(row_index)] - preview_q[p];
            for (int i = 0; i < horizon_steps; ++i) {
                c_vec[static_cast<std::size_t>(i)] += w_preview_q * row[static_cast<std::size_t>(i)] * err;
                for (int j = 0; j < horizon_steps; ++j) {
                    q_mat[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] +=
                        w_preview_q * row[static_cast<std::size_t>(i)] * row[static_cast<std::size_t>(j)];
                }
            }
        }

        const auto& ce = aq[static_cast<std::size_t>(policy_steps - 1)];
        const double be = preview_q.front() - q_base[static_cast<std::size_t>(policy_steps - 1)];
        const int n = horizon_steps + 1;
        std::vector<std::vector<double>> kkt(n, std::vector<double>(n, 0.0));
        std::vector<double> rhs(n, 0.0);
        for (int i = 0; i < horizon_steps; ++i) {
            for (int j = 0; j < horizon_steps; ++j) {
                kkt[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] =
                    q_mat[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
            }
            kkt[static_cast<std::size_t>(i)][static_cast<std::size_t>(horizon_steps)] =
                ce[static_cast<std::size_t>(i)];
            kkt[static_cast<std::size_t>(horizon_steps)][static_cast<std::size_t>(i)] =
                ce[static_cast<std::size_t>(i)];
            rhs[static_cast<std::size_t>(i)] = -c_vec[static_cast<std::size_t>(i)];
        }
        rhs[static_cast<std::size_t>(horizon_steps)] = be;

        std::vector<double> sol;
        if (!solveLinearSystem(kkt, rhs, sol)) {
            return false;
        }

        first_segment.clear();
        first_segment.reserve(static_cast<std::size_t>(policy_steps + 1));
        first_segment.push_back(start);
        for (int k = 0; k < policy_steps; ++k) {
            double q = q_base[static_cast<std::size_t>(k)];
            double dq = dq_base[static_cast<std::size_t>(k)];
            double ddq = ddq_base[static_cast<std::size_t>(k)];
            for (int i = 0; i < horizon_steps; ++i) {
                const double j = sol[static_cast<std::size_t>(i)];
                q += aq[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] * j;
                dq += av[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] * j;
                ddq += aa[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] * j;
            }
            if (!std::isfinite(q) || !std::isfinite(dq) || !std::isfinite(ddq)) {
                return false;
            }
            first_segment.push_back({q, dq, ddq});
        }
        return true;
    }

    static bool solveSoftPreviewVelocityMpc(const PolicyPoint& start,
                                            const std::vector<double>& preview_q,
                                            const std::vector<double>& preview_dq,
                                            int policy_steps,
                                            double dt,
                                            std::vector<PolicyPoint>& first_segment) {
        if (preview_q.size() != 4 || preview_dq.size() != 3 || policy_steps <= 0 || dt <= 0.0) {
            return false;
        }

        constexpr double w_preview_q = 2.0e7;
        constexpr double w_preview_v = 1.0;
        constexpr double w_path_v = 3.0e-3;
        constexpr double w_path_a = 8.0e-5;
        constexpr double w_jerk = 2.0e-9;
        constexpr double w_ridge = 1.0e-10;

        constexpr int optimized_policy_points = 3;
        const int horizon_steps = policy_steps * optimized_policy_points;
        if (horizon_steps <= 0 || horizon_steps > 240) {
            return false;
        }

        std::vector<std::vector<double>> aq(horizon_steps, std::vector<double>(horizon_steps, 0.0));
        std::vector<std::vector<double>> av(horizon_steps, std::vector<double>(horizon_steps, 0.0));
        std::vector<std::vector<double>> aa(horizon_steps, std::vector<double>(horizon_steps, 0.0));
        for (int k = 1; k <= horizon_steps; ++k) {
            for (int i = 0; i < k; ++i) {
                const double r = static_cast<double>(k - i);
                aa[static_cast<std::size_t>(k - 1)][static_cast<std::size_t>(i)] = dt;
                av[static_cast<std::size_t>(k - 1)][static_cast<std::size_t>(i)] =
                    0.5 * dt * dt * (r * r - (r - 1.0) * (r - 1.0));
                aq[static_cast<std::size_t>(k - 1)][static_cast<std::size_t>(i)] =
                    (dt * dt * dt / 6.0) * (r * r * r - (r - 1.0) * (r - 1.0) * (r - 1.0));
            }
        }

        std::vector<double> q_base(horizon_steps, 0.0);
        std::vector<double> dq_base(horizon_steps, 0.0);
        std::vector<double> ddq_base(horizon_steps, start.ddq);
        for (int k = 1; k <= horizon_steps; ++k) {
            const double tk = dt * static_cast<double>(k);
            q_base[static_cast<std::size_t>(k - 1)] = start.q + start.dq * tk + 0.5 * start.ddq * tk * tk;
            dq_base[static_cast<std::size_t>(k - 1)] = start.dq + start.ddq * tk;
        }

        std::vector<std::vector<double>> q_mat(horizon_steps, std::vector<double>(horizon_steps, 0.0));
        std::vector<double> c_vec(horizon_steps, 0.0);
        for (int i = 0; i < horizon_steps; ++i) {
            for (int j = 0; j < horizon_steps; ++j) {
                double qij = 0.0;
                for (int k = 0; k < horizon_steps; ++k) {
                    qij += w_path_v * av[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] *
                           av[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)];
                    qij += w_path_a * aa[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] *
                           aa[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)];
                }
                if (i == j) {
                    qij += w_jerk + w_ridge;
                }
                q_mat[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = qij;
            }

            double ci = 0.0;
            for (int k = 0; k < horizon_steps; ++k) {
                ci += w_path_v * av[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] *
                      dq_base[static_cast<std::size_t>(k)];
                ci += w_path_a * aa[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] *
                      ddq_base[static_cast<std::size_t>(k)];
            }
            c_vec[static_cast<std::size_t>(i)] = ci;
        }

        for (std::size_t p = 1; p < 3; ++p) {
            const int row_index = static_cast<int>(p + 1) * policy_steps - 1;
            const auto& row = aq[static_cast<std::size_t>(row_index)];
            const double err = q_base[static_cast<std::size_t>(row_index)] - preview_q[p];
            for (int i = 0; i < horizon_steps; ++i) {
                c_vec[static_cast<std::size_t>(i)] += w_preview_q * row[static_cast<std::size_t>(i)] * err;
                for (int j = 0; j < horizon_steps; ++j) {
                    q_mat[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] +=
                        w_preview_q * row[static_cast<std::size_t>(i)] * row[static_cast<std::size_t>(j)];
                }
            }
        }

        for (std::size_t p = 0; p < 3; ++p) {
            const int row_index = static_cast<int>(p + 1) * policy_steps - 1;
            const auto& row = av[static_cast<std::size_t>(row_index)];
            const double err = dq_base[static_cast<std::size_t>(row_index)] - preview_dq[p];
            for (int i = 0; i < horizon_steps; ++i) {
                c_vec[static_cast<std::size_t>(i)] += w_preview_v * row[static_cast<std::size_t>(i)] * err;
                for (int j = 0; j < horizon_steps; ++j) {
                    q_mat[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] +=
                        w_preview_v * row[static_cast<std::size_t>(i)] * row[static_cast<std::size_t>(j)];
                }
            }
        }

        const auto& ce = aq[static_cast<std::size_t>(policy_steps - 1)];
        const double be = preview_q.front() - q_base[static_cast<std::size_t>(policy_steps - 1)];
        const int n = horizon_steps + 1;
        std::vector<std::vector<double>> kkt(n, std::vector<double>(n, 0.0));
        std::vector<double> rhs(n, 0.0);
        for (int i = 0; i < horizon_steps; ++i) {
            for (int j = 0; j < horizon_steps; ++j) {
                kkt[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] =
                    q_mat[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
            }
            kkt[static_cast<std::size_t>(i)][static_cast<std::size_t>(horizon_steps)] =
                ce[static_cast<std::size_t>(i)];
            kkt[static_cast<std::size_t>(horizon_steps)][static_cast<std::size_t>(i)] =
                ce[static_cast<std::size_t>(i)];
            rhs[static_cast<std::size_t>(i)] = -c_vec[static_cast<std::size_t>(i)];
        }
        rhs[static_cast<std::size_t>(horizon_steps)] = be;

        std::vector<double> sol;
        if (!solveLinearSystem(kkt, rhs, sol)) {
            return false;
        }

        first_segment.clear();
        first_segment.reserve(static_cast<std::size_t>(policy_steps + 1));
        first_segment.push_back(start);
        for (int k = 0; k < policy_steps; ++k) {
            double q = q_base[static_cast<std::size_t>(k)];
            double dq = dq_base[static_cast<std::size_t>(k)];
            double ddq = ddq_base[static_cast<std::size_t>(k)];
            for (int i = 0; i < horizon_steps; ++i) {
                const double j = sol[static_cast<std::size_t>(i)];
                q += aq[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] * j;
                dq += av[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] * j;
                ddq += aa[static_cast<std::size_t>(k)][static_cast<std::size_t>(i)] * j;
            }
            if (!std::isfinite(q) || !std::isfinite(dq) || !std::isfinite(ddq)) {
                return false;
            }
            first_segment.push_back({q, dq, ddq});
        }
        return true;
    }

    static bool solveLinearSystem(std::vector<std::vector<double>> a,
                                  std::vector<double> b,
                                  std::vector<double>& x) {
        const int n = static_cast<int>(b.size());
        if (n == 0 || static_cast<int>(a.size()) != n) {
            return false;
        }
        for (int col = 0; col < n; ++col) {
            int pivot = col;
            double pivot_abs = std::abs(a[static_cast<std::size_t>(col)][static_cast<std::size_t>(col)]);
            for (int row = col + 1; row < n; ++row) {
                const double value = std::abs(a[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)]);
                if (value > pivot_abs) {
                    pivot = row;
                    pivot_abs = value;
                }
            }
            if (pivot_abs < 1.0e-14 || !std::isfinite(pivot_abs)) {
                return false;
            }
            if (pivot != col) {
                std::swap(a[static_cast<std::size_t>(pivot)], a[static_cast<std::size_t>(col)]);
                std::swap(b[static_cast<std::size_t>(pivot)], b[static_cast<std::size_t>(col)]);
            }
            const double diag = a[static_cast<std::size_t>(col)][static_cast<std::size_t>(col)];
            for (int j = col; j < n; ++j) {
                a[static_cast<std::size_t>(col)][static_cast<std::size_t>(j)] /= diag;
            }
            b[static_cast<std::size_t>(col)] /= diag;
            for (int row = 0; row < n; ++row) {
                if (row == col) {
                    continue;
                }
                const double factor = a[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)];
                if (std::abs(factor) < 1.0e-18) {
                    continue;
                }
                for (int j = col; j < n; ++j) {
                    a[static_cast<std::size_t>(row)][static_cast<std::size_t>(j)] -=
                        factor * a[static_cast<std::size_t>(col)][static_cast<std::size_t>(j)];
                }
                b[static_cast<std::size_t>(row)] -= factor * b[static_cast<std::size_t>(col)];
            }
        }
        x = std::move(b);
        return true;
    }

    static void fillQuinticFallback(SegmentState& segment,
                                    double t_policy,
                                    int policy_steps,
                                    const PolicyPoint& target) {
        segment.samples.clear();
        segment.samples.reserve(static_cast<std::size_t>(policy_steps + 1));
        for (int i = 0; i <= policy_steps; ++i) {
            const double tau = t_policy * static_cast<double>(i) / static_cast<double>(policy_steps);
            segment.samples.push_back(evalQuinticReference(segment.start, target, t_policy, tau));
        }
    }

    PolicyReferenceConfig cfg_;
    SegmentState segment_;
};

}  // namespace h1if
