function out = build_simulink_final_model(stopTime, outputCsv, makePlot)
%BUILD_SIMULINK_FINAL_MODEL Run a MATLAB equivalent of simulink_final.slx.
%
% This is an algorithmic/control-loop equivalent, not an SLX payload writer.
% It mirrors the saved simulink_final.slx topology, including the explicit
% Unit Delay memories in the plant, forward observer, eta low-pass filter,
% and eta output path.
%
% The MATLAB Function chart bodies below match those in simulink_final.slx.

if nargin < 1 || isempty(stopTime)
    stopTime = 5.0;
end
if nargin < 2 || strlength(string(outputCsv)) == 0
    outputCsv = "simulink_final_m_equivalent_rollout.csv";
end
if nargin < 3 || isempty(makePlot)
    makePlot = true;
end

packageDir = fileparts(mfilename("fullpath"));
if strlength(packageDir) == 0
    packageDir = pwd;
end
addpath(fullfile(packageDir, "mujoco_knee"));

% Model workspace values from simulink_final.slx.
eid_Ko = [0.9 0.0; 0.0 1.1];
eid_Kpd = [260.0 18.0];
eid_control_dt = 0.002;
eid_eta_alpha = 0.9;
eid_policy_reference_dt = 0.1;

% Mask parameter values on Reference_Trajectory in simulink_final.slx.
eid_reference_mode_id = 8;
eid_ref_center = 0.9;
eid_ref_amplitude = 0.5;
eid_ref_frequency = 0.8;
eid_shaper_track_time = 0.06;
eid_shaper_velocity_track_time = 0.03;
eid_shaper_v_max = 1.2;
eid_shaper_a_max = 20.0;
eid_shaper_j_max = 500.0;

initial_x = [0.9; 0.0];
Ts = eid_control_dt;
nSamples = floor(double(stopTime) / Ts + 1.0e-12) + 1;

t = zeros(nSamples, 1);
q = zeros(nSamples, 1);
dq = zeros(nSamples, 1);
r_star_q = zeros(nSamples, 1);
r_star_dq = zeros(nSamples, 1);
r_star_next_q = zeros(nSamples, 1);
r_star_next_dq = zeros(nSamples, 1);
eta_q = zeros(nSamples, 1);
eta_dq = zeros(nSamples, 1);
eta_filter_next_q = zeros(nSamples, 1);
eta_filter_next_dq = zeros(nSamples, 1);
x_hat_q = zeros(nSamples, 1);
x_hat_dq = zeros(nSamples, 1);
x_bar_q = zeros(nSamples, 1);
x_bar_dq = zeros(nSamples, 1);
x_bar_next_q = zeros(nSamples, 1);
x_bar_next_dq = zeros(nSamples, 1);
x_tilde_q = zeros(nSamples, 1);
x_tilde_dq = zeros(nSamples, 1);
r_d_q = zeros(nSamples, 1);
r_d_dq = zeros(nSamples, 1);
e_q_t = zeros(nSamples, 1);
e_dq_t = zeros(nSamples, 1);
u_t = zeros(nSamples, 1);
d_in_t = zeros(nSamples, 1);
u_plant_raw_t = zeros(nSamples, 1);
u_plant_t = zeros(nSamples, 1);
qacc = zeros(nSamples, 1);
q_next = zeros(nSamples, 1);
dq_next = zeros(nSamples, 1);
x_hat_next_q = zeros(nSamples, 1);
x_hat_next_dq = zeros(nSamples, 1);
observer_qacc = zeros(nSamples, 1);
observer_tau_applied = zeros(nSamples, 1);
u_star = zeros(nSamples, 1);

x_mem = initial_x;
% Unit Delay initial conditions from simulink_final.slx:
% Plant_State_Memory=[0.9;0], Forward_Observer memory=[0;0],
% EID output memory=[0.9;0], eta low-pass memory=[0;0].
x_hat_mem = [0.0; 0.0];
eta_output_mem = initial_x;
eta_lpf_mem = [0.0; 0.0];
disturbance_state = uint32(0);
K_dagger = eid_Kpd' / (eid_Kpd * eid_Kpd');

xmlPath = fullfile(packageDir, "h1_single_knee", "h1_single_knee.xml");
plant = mujocoKneeCreate(xmlPath, Ts);
cleanupObj = onCleanup(@() mujocoKneeClose(plant));
[plant, ~, ~] = mujocoKneeReset(plant, initial_x(1), initial_x(2));

for k = 1:nSamples
    tk = (k - 1) * Ts;
    t(k) = tk;

    [r_star, r_star_next] = reference_trajectory( ...
        tk, ...
        eid_control_dt, ...
        eid_policy_reference_dt, ...
        eid_reference_mode_id, ...
        eid_ref_center, ...
        eid_ref_amplitude, ...
        eid_ref_frequency, ...
        eid_shaper_track_time, ...
        eid_shaper_velocity_track_time, ...
        eid_shaper_v_max, ...
        eid_shaper_a_max, ...
        eid_shaper_j_max);

    eta = eta_output_mem;
    x_hat = x_hat_mem;
    x_bar = x_hat + eta;

    r_c_next = r_star_next - eta;
    delta_r_c = r_c_next - r_star;
    u_star_k = analytic_inverse_model(r_star, delta_r_c, Ts);
    r_d = r_star + K_dagger * u_star_k;

    e = r_d - x_bar;
    u = eid_Kpd * e;

    [x_hat_next, observer_qacc_k, observer_tau_k] = joint_dynamics(x_bar, u, Ts);
    tilde_x = x_mem - x_bar;
    Ko_tilde_x = eid_Ko * tilde_x;
    eta_filter_next = eid_eta_alpha * Ko_tilde_x + (1.0 - eid_eta_alpha) * eta_lpf_mem;
    x_bar_next = x_hat_next + eta_filter_next;

    [disturbance_state, d_in] = simulink_input_disturbance_step(tk, disturbance_state);
    u_plant_raw = u + d_in;
    plantStep = mujocoKneeStep(plant, u_plant_raw);
    plant_x_next = [plantStep.q_next; plantStep.dq_next];

    q(k) = x_mem(1);
    dq(k) = x_mem(2);
    r_star_q(k) = r_star(1);
    r_star_dq(k) = r_star(2);
    r_star_next_q(k) = r_star_next(1);
    r_star_next_dq(k) = r_star_next(2);
    eta_q(k) = eta(1);
    eta_dq(k) = eta(2);
    eta_filter_next_q(k) = eta_filter_next(1);
    eta_filter_next_dq(k) = eta_filter_next(2);
    x_hat_q(k) = x_hat(1);
    x_hat_dq(k) = x_hat(2);
    x_bar_q(k) = x_bar(1);
    x_bar_dq(k) = x_bar(2);
    x_bar_next_q(k) = x_bar_next(1);
    x_bar_next_dq(k) = x_bar_next(2);
    x_tilde_q(k) = tilde_x(1);
    x_tilde_dq(k) = tilde_x(2);
    r_d_q(k) = r_d(1);
    r_d_dq(k) = r_d(2);
    e_q_t(k) = e(1);
    e_dq_t(k) = e(2);
    u_t(k) = u;
    d_in_t(k) = d_in;
    u_plant_raw_t(k) = u_plant_raw;
    u_plant_t(k) = plantStep.tau_applied;
    qacc(k) = plantStep.qacc;
    q_next(k) = plant_x_next(1);
    dq_next(k) = plant_x_next(2);
    x_hat_next_q(k) = x_hat_next(1);
    x_hat_next_dq(k) = x_hat_next(2);
    observer_qacc(k) = observer_qacc_k;
    observer_tau_applied(k) = observer_tau_k;
    u_star(k) = u_star_k;

    x_mem = plant_x_next;
    x_hat_mem = x_hat_next;
    eta_output_mem = eta_filter_next;
    eta_lpf_mem = eta_filter_next;
end

rows = table( ...
    t, q, dq, q_next, dq_next, qacc, ...
    r_star_q, r_star_dq, r_star_next_q, r_star_next_dq, ...
    x_hat_q, x_hat_dq, x_hat_next_q, x_hat_next_dq, ...
    eta_q, eta_dq, eta_filter_next_q, eta_filter_next_dq, ...
    x_bar_q, x_bar_dq, x_bar_next_q, x_bar_next_dq, ...
    x_tilde_q, x_tilde_dq, ...
    r_d_q, r_d_dq, e_q_t, e_dq_t, u_star, u_t, ...
    d_in_t, u_plant_raw_t, u_plant_t, observer_qacc, observer_tau_applied);

out = struct();
out.rows = rows;
out.u_t = timeseries(u_t, t, "Name", "u_t");
out.e_q_t = timeseries(e_q_t, t, "Name", "e_q_t");
out.e_dq_t = timeseries(e_dq_t, t, "Name", "e_dq_t");
out.params = struct( ...
    "eid_Ko", eid_Ko, ...
    "eid_Kpd", eid_Kpd, ...
    "eid_control_dt", eid_control_dt, ...
    "eid_eta_alpha", eid_eta_alpha, ...
    "eid_policy_reference_dt", eid_policy_reference_dt, ...
    "eid_reference_mode_id", eid_reference_mode_id, ...
    "eid_ref_center", eid_ref_center, ...
    "eid_ref_amplitude", eid_ref_amplitude, ...
    "eid_ref_frequency", eid_ref_frequency, ...
    "eid_shaper_track_time", eid_shaper_track_time, ...
    "eid_shaper_velocity_track_time", eid_shaper_velocity_track_time, ...
    "eid_shaper_v_max", eid_shaper_v_max, ...
    "eid_shaper_a_max", eid_shaper_a_max, ...
    "eid_shaper_j_max", eid_shaper_j_max);

if strlength(string(outputCsv)) > 0
    writetable(rows, fullfile(packageDir, outputCsv));
end

if makePlot
    plotSimulinkFinalMEquivalent(rows);
end

fprintf("MATLAB equivalent rollout finished. StopTime=%g, samples=%d\n", stopTime, nSamples);
if strlength(string(outputCsv)) > 0
    fprintf("Wrote CSV: %s\n", fullfile(packageDir, outputCsv));
end

end

function plotSimulinkFinalMEquivalent(rows)

fig = figure("Name", "simulink_final MATLAB equivalent closed loop", "Color", "w");
tl = tiledlayout(fig, 4, 1, ...
    "TileSpacing", "compact", ...
    "Padding", "compact");
title(tl, "simulink_final MATLAB equivalent closed loop");

nexttile;
plot(rows.t, rows.r_star_q, "--", "LineWidth", 1.1);
hold on;
plot(rows.t, rows.q, "LineWidth", 1.2);
grid on;
ylabel("q [rad]");
legend("r star q", "q", "Location", "best");

nexttile;
plot(rows.t, rows.r_star_dq, "--", "LineWidth", 1.1);
hold on;
plot(rows.t, rows.dq, "LineWidth", 1.2);
grid on;
ylabel("dq [rad/s]");
legend("r star dq", "dq", "Location", "best");

nexttile;
plot(rows.t, rows.u_t, "LineWidth", 1.1);
hold on;
plot(rows.t, rows.u_plant_t, "LineWidth", 1.2);
plot(rows.t, rows.d_in_t, "LineWidth", 1.0);
grid on;
ylabel("torque [Nm]");
legend("u", "u plant", "d in", "Location", "best");

nexttile;
plot(rows.t, rows.e_q_t, "LineWidth", 1.1);
hold on;
plot(rows.t, rows.e_dq_t, "LineWidth", 1.1);
plot(rows.t, rows.eta_q, "LineWidth", 1.0);
plot(rows.t, rows.eta_dq, "LineWidth", 1.0);
grid on;
xlabel("time [s]");
ylabel("error / eta");
legend("e q", "e dq", "eta q", "eta dq", "Location", "best");

end

function [state, d_in_t] = simulink_input_disturbance_step(t, state)
% Same logic as the input_disturbance MATLAB Function chart, with explicit
% state so each top-level run starts from the Simulink initial seed.
d_in = 100.0;
d_start = 2.0;
d_end = 4.0;
if d_in == 0.0 || t < d_start || t >= d_end
    d_in_t = 0.0;
    return;
end
state_double = mod(1664525.0 * double(state) + 1013904223.0, 4294967296.0);
state = uint32(state_double);
u01 = state_double / 4294967296.0;
random_signed = 2.0 * u01 - 1.0;
d_in_t = abs(d_in) * random_signed;
end

function [r_star, r_star_next] = reference_trajectory(t, eid_control_dt, eid_policy_reference_dt, eid_reference_mode_id, eid_ref_center, eid_ref_amplitude, eid_ref_frequency, eid_shaper_track_time, eid_shaper_velocity_track_time, eid_shaper_v_max, eid_shaper_a_max, eid_shaper_j_max)
% Standalone reference generator with plan nodes derived from Tpolicy/Ts.

Ts = max(eid_control_dt, 1.0e-6);
Tpolicy = max(eid_policy_reference_dt, Ts);
modeId = eid_reference_mode_id;
center = eid_ref_center;
amplitude = eid_ref_amplitude;
frequency = eid_ref_frequency;
trackTime = eid_shaper_track_time;
velocityTrackTime = eid_shaper_velocity_track_time;
vMax = eid_shaper_v_max;
aMax = eid_shaper_a_max;
jMax = eid_shaper_j_max;

if modeId == 0.0
    [q, dq] = raw_policy_reference(t, Tpolicy, center, amplitude, frequency);
    [q_next, dq_next] = raw_policy_reference(t + Ts, Tpolicy, center, amplitude, frequency);
else
    [q, dq, q_next, dq_next] = shaped_policy_reference(t, Ts, Tpolicy, modeId, center, amplitude, frequency, trackTime, velocityTrackTime, vMax, aMax, jMax);
end

r_star = [q; dq];
r_star_next = [q_next; dq_next];
end

function [q, dq, q_next, dq_next] = shaped_policy_reference(t, Ts, Tpolicy, modeId, center, amplitude, frequency, trackTime, velocityTrackTime, vMax, aMax, jMax)
MAX_NODES = 64;
persistent initialized lastSegment lastT lastMode lastCenter lastAmplitude lastFrequency lastTs lastTpolicy
persistent qNodes dqNodes ddqNodes nNodes nodeDt

if isempty(initialized)
    initialized = false;
    lastSegment = -1.0;
    lastT = -1.0;
    lastMode = -999.0;
    lastCenter = 0.0;
    lastAmplitude = 0.0;
    lastFrequency = 0.0;
    lastTs = -1.0;
    lastTpolicy = -1.0;
    qNodes = zeros(MAX_NODES, 1);
    dqNodes = zeros(MAX_NODES, 1);
    ddqNodes = zeros(MAX_NODES, 1);
    nNodes = 2;
    nodeDt = Tpolicy;
end

t = max(t, 0.0);
segment = floor((t + 1.0e-12) / Tpolicy);
paramsChanged = local_abs(modeId - lastMode) > 0.5 || ...
    local_abs(center - lastCenter) > 1.0e-12 || ...
    local_abs(amplitude - lastAmplitude) > 1.0e-12 || ...
    local_abs(frequency - lastFrequency) > 1.0e-12 || ...
    local_abs(Ts - lastTs) > 1.0e-12 || ...
    local_abs(Tpolicy - lastTpolicy) > 1.0e-12;

if t < lastT - 0.5*Ts || paramsChanged
    initialized = false;
    lastSegment = -1.0;
end

if ~initialized || segment ~= lastSegment
    segmentStart = segment * Tpolicy;
    if initialized && segment == lastSegment + 1.0
        q0 = qNodes(nNodes);
        dq0 = dqNodes(nNodes);
        ddq0 = ddqNodes(nNodes);
    else
        [q0, dq0] = raw_policy_reference(segmentStart, Tpolicy, center, amplitude, frequency);
        ddq0 = 0.0;
    end

    [qTarget, dqTarget] = raw_policy_reference(segmentStart + Tpolicy, Tpolicy, center, amplitude, frequency);
    ddqTarget = 0.0;
    [qNodes, dqNodes, ddqNodes, nNodes, nodeDt] = build_plan_nodes(q0, dq0, ddq0, qTarget, dqTarget, ddqTarget, Ts, Tpolicy, modeId, trackTime, velocityTrackTime, vMax, aMax, jMax);

    initialized = true;
    lastSegment = segment;
    lastMode = modeId;
    lastCenter = center;
    lastAmplitude = amplitude;
    lastFrequency = frequency;
    lastTs = Ts;
    lastTpolicy = Tpolicy;
end

tau = min(max(t - segment*Tpolicy, 0.0), Tpolicy);
tauNext = min(tau + Ts, Tpolicy);
[q, dq] = eval_nodes(qNodes, dqNodes, nNodes, tau, nodeDt);
[q_next, dq_next] = eval_nodes(qNodes, dqNodes, nNodes, tauNext, nodeDt);
lastT = t;
end

function [qNodes, dqNodes, ddqNodes, nNodes, nodeDt] = build_plan_nodes(q0, dq0, ddq0, qTarget, dqTarget, ddqTarget, Ts, Tpolicy, modeId, trackTime, velocityTrackTime, vMax, aMax, jMax)
MAX_NODES = 64;
nNodes = min(MAX_NODES, max(2, round(Tpolicy / Ts) + 1));
nodeDt = Tpolicy / double(nNodes - 1);
qNodes = zeros(MAX_NODES, 1);
dqNodes = zeros(MAX_NODES, 1);
ddqNodes = zeros(MAX_NODES, 1);

if modeId == 2.0
    for i = 1:nNodes
        tau = min((i - 1) * nodeDt, Tpolicy);
        [qNodes(i), dqNodes(i), ddqNodes(i)] = eval_quintic(q0, dq0, ddq0, qTarget, dqTarget, ddqTarget, Tpolicy, tau);
    end
elseif modeId == 7.0
    for i = 1:nNodes
        tau = min((i - 1) * nodeDt, Tpolicy);
        [qNodes(i), dqNodes(i), ddqNodes(i)] = eval_quintic_position_only(q0, dq0, ddq0, qTarget, Tpolicy, tau);
    end
elseif modeId == 3.0
    for i = 1:nNodes
        tau = min((i - 1) * nodeDt, Tpolicy);
        [qNodes(i), dqNodes(i), ddqNodes(i)] = eval_cubic(q0, dq0, qTarget, dqTarget, Tpolicy, tau);
    end
elseif modeId == 8.0
    for i = 1:nNodes
        tau = min((i - 1) * nodeDt, Tpolicy);
        [qNodes(i), dqNodes(i), ddqNodes(i)] = eval_cubic_position_only(q0, dq0, qTarget, Tpolicy, tau);
    end
elseif modeId == 5.0 || modeId == 9.0
    useTerminalVelocity = modeId == 5.0;
    [qNodes, dqNodes, ddqNodes] = build_scurve_nodes(q0, dq0, ddq0, qTarget, dqTarget, ddqTarget, nodeDt, Tpolicy, nNodes, MAX_NODES, useTerminalVelocity, vMax, aMax, jMax);
elseif modeId == 6.0 || modeId == 10.0
    useTerminalVelocity = modeId == 6.0;
    [qNodes, dqNodes, ddqNodes] = build_mpc_nodes(q0, dq0, ddq0, qTarget, dqTarget, ddqTarget, nodeDt, nNodes, MAX_NODES, useTerminalVelocity, vMax, aMax, jMax);
elseif modeId == 4.0
    [qNodes, dqNodes, ddqNodes] = build_jerk_remaining_nodes(q0, dq0, ddq0, qTarget, dqTarget, nodeDt, nNodes, MAX_NODES, trackTime, velocityTrackTime, vMax, aMax, jMax);
else
    [qNodes, dqNodes, ddqNodes] = build_jerk_nodes(q0, dq0, ddq0, qTarget, dqTarget, nodeDt, nNodes, MAX_NODES, trackTime, velocityTrackTime, vMax, aMax, jMax);
end
end

function [qNodes, dqNodes, ddqNodes] = build_jerk_nodes(q0, dq0, ddq0, qTarget, dqTarget, Ts, nNodes, MAX_NODES, trackTime, velocityTrackTime, vMax, aMax, jMax)
qNodes = zeros(MAX_NODES, 1);
dqNodes = zeros(MAX_NODES, 1);
ddqNodes = zeros(MAX_NODES, 1);
q = clamp(q0, -0.26, 2.05);
dq = clamp(dq0, -vMax, vMax);
ddq = clamp(ddq0, -aMax, aMax);
qNodes(1) = q;
dqNodes(1) = dq;
ddqNodes(1) = ddq;
for i = 2:nNodes
    vRef = (qTarget - q) / max(trackTime, Ts);
    vRef = clamp(vRef, -vMax, vMax);
    aRef = (vRef - dq) / max(velocityTrackTime, Ts);
    aRef = clamp(aRef, -aMax, aMax);
    jerk = clamp((aRef - ddq) / Ts, -jMax, jMax);
    [q, dq, ddq] = integrate_one_step(q, dq, ddq, jerk, Ts, vMax, aMax);
    qNodes(i) = q;
    dqNodes(i) = dq;
    ddqNodes(i) = ddq;
end
end

function [qNodes, dqNodes, ddqNodes] = build_jerk_remaining_nodes(q0, dq0, ddq0, qTarget, dqTarget, Ts, nNodes, MAX_NODES, trackTime, velocityTrackTime, vMax, aMax, jMax)
qNodes = zeros(MAX_NODES, 1);
dqNodes = zeros(MAX_NODES, 1);
ddqNodes = zeros(MAX_NODES, 1);
q = clamp(q0, -0.26, 2.05);
dq = clamp(dq0, -vMax, vMax);
ddq = clamp(ddq0, -aMax, aMax);
qNodes(1) = q;
dqNodes(1) = dq;
ddqNodes(1) = ddq;
for i = 2:nNodes
    remaining = max((nNodes - i + 1) * Ts, Ts);
    vRef = (qTarget - q) / max(remaining, trackTime);
    vRef = clamp(vRef, -vMax, vMax);
    aRef = (vRef - dq) / max(min(velocityTrackTime, remaining), Ts);
    aRef = clamp(aRef, -aMax, aMax);
    jerk = clamp((aRef - ddq) / Ts, -jMax, jMax);
    [q, dq, ddq] = integrate_one_step(q, dq, ddq, jerk, Ts, vMax, aMax);
    qNodes(i) = q;
    dqNodes(i) = dq;
    ddqNodes(i) = ddq;
end
end

function [qNodes, dqNodes, ddqNodes] = build_scurve_nodes(q0, dq0, ddq0, qTarget, dqTarget, ddqTarget, Ts, Tpolicy, nNodes, MAX_NODES, useTerminalVelocity, vMax, aMax, jMax)
qNodes = zeros(MAX_NODES, 1);
dqNodes = zeros(MAX_NODES, 1);
ddqNodes = zeros(MAX_NODES, 1);
q = clamp(q0, -0.26, 2.05);
dq = clamp(dq0, -vMax, vMax);
ddq = clamp(ddq0, -aMax, aMax);
qNodes(1) = q;
dqNodes(1) = dq;
ddqNodes(1) = ddq;
for i = 2:nNodes
    remaining = max(Tpolicy - (i - 2)*Ts, Ts);
    if useTerminalVelocity
        [~, ~, localDdq] = eval_quintic(q, dq, ddq, qTarget, dqTarget, ddqTarget, remaining, Ts);
    else
        [~, ~, localDdq] = eval_quintic_position_only(q, dq, ddq, qTarget, remaining, Ts);
    end
    jerk = clamp((localDdq - ddq) / Ts, -jMax, jMax);
    [q, dq, ddq] = integrate_one_step(q, dq, ddq, jerk, Ts, vMax, aMax);
    qNodes(i) = q;
    dqNodes(i) = dq;
    ddqNodes(i) = ddq;
end
end

function [qNodes, dqNodes, ddqNodes] = build_mpc_nodes(q0, dq0, ddq0, qTarget, dqTarget, ddqTarget, Ts, nNodes, MAX_NODES, useTerminalVelocity, vMax, aMax, jMax)
MAX_HORIZON = 63;
N = min(MAX_HORIZON, max(1, nNodes - 1));
qBase = zeros(MAX_HORIZON, 1);
dqBase = zeros(MAX_HORIZON, 1);
ddqBase = zeros(MAX_HORIZON, 1);
QJ = zeros(MAX_HORIZON, MAX_HORIZON);
DqJ = zeros(MAX_HORIZON, MAX_HORIZON);
DdqJ = zeros(MAX_HORIZON, MAX_HORIZON);
for i = 1:N
    qBase(i) = q0 + i*Ts*dq0 + Ts*Ts*(i*(i + 1)/2.0)*ddq0;
    dqBase(i) = dq0 + i*Ts*ddq0;
    ddqBase(i) = ddq0;
    for m = 1:i
        r = i - m + 1;
        QJ(i, m) = Ts^3 * (r * (r + 1) / 2.0);
        DqJ(i, m) = Ts^2 * r;
        DdqJ(i, m) = Ts;
    end
end

wq = 1.0;
wdq = 0.05;
wddq = 0.005;
wTq = 5000.0;
wTdq = 50.0;
wTddq = 5.0;
wj = 1.0e-6;

H = zeros(MAX_HORIZON, MAX_HORIZON);
f = zeros(MAX_HORIZON, 1);
for i = 1:N
    for j = 1:N
        H(i, j) = 2.0 * wj * double(i == j);
        if useTerminalVelocity
            H(i, j) = H(i, j) + 2.0 * ( ...
                wTq * QJ(N, i) * QJ(N, j) + ...
                wTdq * DqJ(N, i) * DqJ(N, j) + ...
                wTddq * DdqJ(N, i) * DdqJ(N, j));
        else
            H(i, j) = H(i, j) + 2.0 * wTq * QJ(N, i) * QJ(N, j);
        end
        for k = 1:N
            if useTerminalVelocity
                H(i, j) = H(i, j) + 2.0 * ( ...
                    wq * QJ(k, i) * QJ(k, j) + ...
                    wdq * DqJ(k, i) * DqJ(k, j) + ...
                    wddq * DdqJ(k, i) * DdqJ(k, j));
            else
                H(i, j) = H(i, j) + 2.0 * wq * QJ(k, i) * QJ(k, j);
            end
        end
    end
end

for i = 1:N
    if useTerminalVelocity
        f(i) = f(i) + 2.0 * ( ...
            wTq * QJ(N, i) * (qBase(N) - qTarget) + ...
            wTdq * DqJ(N, i) * (dqBase(N) - dqTarget) + ...
            wTddq * DdqJ(N, i) * (ddqBase(N) - ddqTarget));
    else
        f(i) = f(i) + 2.0 * wTq * QJ(N, i) * (qBase(N) - qTarget);
    end
    for k = 1:N
        if useTerminalVelocity
            f(i) = f(i) + 2.0 * ( ...
                wq * QJ(k, i) * (qBase(k) - qTarget) + ...
                wdq * DqJ(k, i) * (dqBase(k) - dqTarget) + ...
                wddq * DdqJ(k, i) * (ddqBase(k) - ddqTarget));
        else
            f(i) = f(i) + 2.0 * wq * QJ(k, i) * (qBase(k) - qTarget);
        end
    end
end

jSeq = solve_spd(H, -f, N, MAX_HORIZON);
for i = 1:N
    jSeq(i) = clamp(jSeq(i), -jMax, jMax);
end
[qNodes, dqNodes, ddqNodes] = integrate_jerk_sequence(q0, dq0, ddq0, jSeq, Ts, nNodes, MAX_NODES, vMax, aMax);
end

function [qNodes, dqNodes, ddqNodes] = integrate_jerk_sequence(q0, dq0, ddq0, jSeq, Ts, nNodes, MAX_NODES, vMax, aMax)
qNodes = zeros(MAX_NODES, 1);
dqNodes = zeros(MAX_NODES, 1);
ddqNodes = zeros(MAX_NODES, 1);
q = clamp(q0, -0.26, 2.05);
dq = clamp(dq0, -vMax, vMax);
ddq = clamp(ddq0, -aMax, aMax);
qNodes(1) = q;
dqNodes(1) = dq;
ddqNodes(1) = ddq;
for i = 2:nNodes
    [q, dq, ddq] = integrate_one_step(q, dq, ddq, jSeq(i - 1), Ts, vMax, aMax);
    qNodes(i) = q;
    dqNodes(i) = dq;
    ddqNodes(i) = ddq;
end
end

function x = solve_spd(H, b, N, MAX_HORIZON)
A = zeros(MAX_HORIZON, MAX_HORIZON);
rhs = zeros(MAX_HORIZON, 1);
x = zeros(MAX_HORIZON, 1);
for i = 1:MAX_HORIZON
    A(i, i) = 1.0;
end
for i = 1:N
    rhs(i) = b(i);
    for j = 1:N
        A(i, j) = H(i, j);
    end
    A(i, i) = A(i, i) + 1.0e-9;
end
x = A \ rhs;
for i = 1:N
    if ~isfinite(x(i))
        x = zeros(MAX_HORIZON, 1);
        return;
    end
end
end

function [q, dq, ddq] = integrate_one_step(q, dq, ddq, jerk, Ts, vMax, aMax)
ddq = clamp(ddq + jerk*Ts, -aMax, aMax);
dq = clamp(dq + ddq*Ts, -vMax, vMax);
q = q + dq*Ts;
[q, dq] = enforce_position_limit(q, dq);
end

function [q, dq] = raw_policy_reference(t, Tpolicy, center, amplitude, frequency)
t = max(t, 0.0);
policyIndex = floor((t + 1.0e-12) / Tpolicy);
tPolicy = policyIndex * Tpolicy;
q0 = policy_position(tPolicy, center, amplitude, frequency);
q1 = policy_position(tPolicy + Tpolicy, center, amplitude, frequency);
q = q0;
dq = (q1 - q0) / Tpolicy;
end

function q = policy_position(t, center, amplitude, frequency)
q = center + amplitude * sin(2.0*pi*frequency*t);
end

function [q, dq] = eval_nodes(qNodes, dqNodes, nNodes, tau, nodeDt)
idx = floor((tau + 1.0e-12) / nodeDt) + 1;
if idx >= nNodes
    q = qNodes(nNodes);
    dq = dqNodes(nNodes);
else
    alpha = (tau - (idx - 1)*nodeDt) / nodeDt;
    alpha = clamp(alpha, 0.0, 1.0);
    q = qNodes(idx) + alpha*(qNodes(idx + 1) - qNodes(idx));
    dq = dqNodes(idx) + alpha*(dqNodes(idx + 1) - dqNodes(idx));
end
end

function [q, dq, ddq] = eval_cubic(q0, dq0, q1, dq1, T, tau)
T2 = T*T;
T3 = T2*T;
a0 = q0;
a1 = dq0;
a2 = 3.0*(q1 - q0)/T2 - (2.0*dq0 + dq1)/T;
a3 = -2.0*(q1 - q0)/T3 + (dq0 + dq1)/T2;
q = a0 + a1*tau + a2*tau*tau + a3*tau*tau*tau;
dq = a1 + 2.0*a2*tau + 3.0*a3*tau*tau;
ddq = 2.0*a2 + 6.0*a3*tau;
end

function [q, dq, ddq] = eval_cubic_position_only(q0, dq0, q1, T, tau)
T2 = T*T;
T3 = T2*T;
d = q1 - q0 - dq0*T;
a0 = q0;
a1 = dq0;
a2 = 1.5*d/T2;
a3 = -0.5*d/T3;
q = a0 + a1*tau + a2*tau*tau + a3*tau*tau*tau;
dq = a1 + 2.0*a2*tau + 3.0*a3*tau*tau;
ddq = 2.0*a2 + 6.0*a3*tau;
end

function [q, dq, ddq] = eval_quintic(q0, dq0, ddq0, q1, dq1, ddq1, T, tau)
a0 = q0;
a1 = dq0;
a2 = 0.5*ddq0;
T2 = T*T;
T3 = T2*T;
T4 = T3*T;
T5 = T4*T;
M = [T3, T4, T5; 3.0*T2, 4.0*T3, 5.0*T4; 6.0*T, 12.0*T2, 20.0*T3];
b = [q1 - (a0 + a1*T + a2*T2); dq1 - (a1 + 2.0*a2*T); ddq1 - 2.0*a2];
x = M \ b;
a3 = x(1);
a4 = x(2);
a5 = x(3);
tau2 = tau*tau;
tau3 = tau2*tau;
tau4 = tau3*tau;
tau5 = tau4*tau;
q = a0 + a1*tau + a2*tau2 + a3*tau3 + a4*tau4 + a5*tau5;
dq = a1 + 2.0*a2*tau + 3.0*a3*tau2 + 4.0*a4*tau3 + 5.0*a5*tau4;
ddq = 2.0*a2 + 6.0*a3*tau + 12.0*a4*tau2 + 20.0*a5*tau3;
end

function [q, dq, ddq] = eval_quintic_position_only(q0, dq0, ddq0, q1, T, tau)
a0 = q0;
a1 = dq0;
a2 = 0.5*ddq0;
T2 = T*T;
T3 = T2*T;
T4 = T3*T;
T5 = T4*T;
d = q1 - (a0 + a1*T + a2*T2);
a3 = (5.0/3.0)*d/T3;
a4 = -(5.0/6.0)*d/T4;
a5 = (1.0/6.0)*d/T5;
tau2 = tau*tau;
tau3 = tau2*tau;
tau4 = tau3*tau;
tau5 = tau4*tau;
q = a0 + a1*tau + a2*tau2 + a3*tau3 + a4*tau4 + a5*tau5;
dq = a1 + 2.0*a2*tau + 3.0*a3*tau2 + 4.0*a4*tau3 + 5.0*a5*tau4;
ddq = 2.0*a2 + 6.0*a3*tau + 12.0*a4*tau2 + 20.0*a5*tau3;
end

function [q, dq] = enforce_position_limit(q, dq)
if q < -0.26
    q = -0.26;
    if dq < 0.0
        dq = 0.0;
    end
elseif q > 2.05
    q = 2.05;
    if dq > 0.0
        dq = 0.0;
    end
end
end

function y = clamp(x, lo, hi)
y = min(max(x, lo), hi);
end

function y = local_abs(x)
if x < 0.0
    y = -x;
else
    y = x;
end
end


function u_star = analytic_inverse_model(r_star, delta_r_c, eid_control_dt)
% Analytic inverse model consistent with semi-implicit Euler joint dynamics.
Ts = eid_control_dt;
J = 0.238;
b = 1.0;
A = 4.2835;
B = 0.0;
tau0 = -0.2711;
tauLimit = 300.0;
useTorqueLimit = 1;
inverse_q_weight = 0.5 / (Ts * Ts);
inverse_dq_weight = 1.0;
q = r_star(1);
dq = r_star(2);
q_target_next = q + delta_r_c(1);
dq_target_next = dq + delta_r_c(2);
bias = b * dq + A * sin(q) + B * cos(q) + tau0;
tau_from_q = bias + J * ((q_target_next - q - Ts * dq) / (Ts * Ts));
tau_from_dq = bias + J * ((dq_target_next - dq) / Ts);
Aq = Ts * Ts / J;
Adq = Ts / J;
den = inverse_q_weight * Aq * Aq + inverse_dq_weight * Adq * Adq;
if den < 1e-12
    u_star = 0.0;
else
    u_star = (inverse_q_weight * Aq * Aq * tau_from_q + inverse_dq_weight * Adq * Adq * tau_from_dq) / den;
end
if useTorqueLimit ~= 0
    if u_star > tauLimit
        u_star = tauLimit;
    elseif u_star < -tauLimit
        u_star = -tauLimit;
    end
end
end

function [x_next, qacc, tau_applied] = joint_dynamics(x, tau_raw, eid_control_dt)
% Semi-implicit Euler single-joint dynamics consistent with eid_contro.m.
Ts = eid_control_dt;
J = 0.238;
b = 1.0;
A = 4.2835;
B = 0.0;
tau0 = -0.2711;
tauLimit = 300.0;
useTorqueLimit = 1;
qMin = -0.26;
qMax = 2.05;
usePositionLimit = 1;
q = x(1);
dq = x(2);
tau_applied = tau_raw;
if useTorqueLimit ~= 0
    if tau_applied > tauLimit
        tau_applied = tauLimit;
    elseif tau_applied < -tauLimit
        tau_applied = -tauLimit;
    end
end
qacc = (tau_applied - b * dq - A * sin(q) - B * cos(q) - tau0) / J;
dq_next = dq + Ts * qacc;
q_next = q + Ts * dq_next;
if usePositionLimit ~= 0
    if q_next < qMin
        q_next = qMin;
        if dq_next < 0
            dq_next = 0.0;
        end
    elseif q_next > qMax
        q_next = qMax;
        if dq_next > 0
            dq_next = 0.0;
        end
    end
end
x_next = [q_next; dq_next];
end
