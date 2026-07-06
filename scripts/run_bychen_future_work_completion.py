#!/usr/bin/env python3
"""Complete selected future-work analyses for the right hip/knee EID report.

MuJoCo is used for candidate-level O4+U1 versus O5+U1 simulation.
Local identified discrete models are used for controller ablations and
residual-definition numerical studies that are not exposed as stepper switches.
"""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml
from scipy import signal

from report_paths import analysis_report_dir, repo_relpath
from run_bychen_mujoco_deep_analysis import fit_identified_plants


ROOT = Path(__file__).resolve().parents[1]
HIP = 1
KNEE = 2
JOINTS = [HIP, KNEE]
JOINT_LABELS = {HIP: "Hip", KNEE: "Knee"}
NOMINAL_OBSERVER_Q = 0.8
NOMINAL_OBSERVER_DQ = 0.2
U1_KU_Q = 6.0
U1_KU_DQ = 0.5
KP = 120.0
KD = 5.0
ALPHA = 0.85


@dataclass(frozen=True)
class MujocoCandidate:
    case: str
    label: str
    hip_so: float
    knee_so: float


@dataclass(frozen=True)
class AblationSpec:
    key: str
    label: str
    feedback_x: float
    feedback_xhat: float
    center_eta: float
    input_eta: float


MUJOCO_CANDIDATES = [
    MujocoCandidate("O4U1", "O4+U1", 1.00, 1.00),
    MujocoCandidate("O5U1", "O5+U1", 1.25, 1.25),
]

ABLATIONS = [
    AblationSpec("pd", "PD / inverse only", 1.0, 0.0, 0.0, 0.0),
    AblationSpec("center_only", "center feedback only", 0.0, 1.0, 1.0, 0.0),
    AblationSpec("input_only", "input compensation only", 1.0, 0.0, 0.0, 1.0),
    AblationSpec("full_eid", "full EID", 0.0, 1.0, 1.0, 1.0),
]

COLORS = {
    "O4+U1": "#0072B2",
    "O5+U1": "#D55E00",
    "PD / inverse only": "#7A7A7A",
    "center feedback only": "#009E73",
    "input compensation only": "#E69F00",
    "full EID": "#D55E00",
}


def configure_matplotlib() -> None:
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "DejaVu Serif"],
        "font.size": 8.5,
        "axes.labelsize": 8.5,
        "xtick.labelsize": 7.5,
        "ytick.labelsize": 7.5,
        "legend.fontsize": 7.5,
        "figure.dpi": 300,
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.grid": True,
        "grid.alpha": 0.18,
        "grid.linewidth": 0.5,
    })


def yaml_joint(cfg: dict[str, Any], joint_id: int) -> dict[str, Any]:
    joints = cfg["controller"]["joints"]
    if joint_id in joints:
        return joints[joint_id]
    return joints[str(joint_id)]


def build_mujoco_config(base_config: Path, out_path: Path, candidate: MujocoCandidate) -> None:
    cfg = yaml.safe_load(base_config.read_text(encoding="utf-8"))
    cfg.setdefault("experiment", {})
    cfg["experiment"]["id"] = "P2D_MuJoCo_FutureWork"
    cfg["experiment"]["condition"] = f"future_work_{candidate.case}_{candidate.label}"
    cfg["experiment"]["disturbance_method"] = "mujoco_input_torque_half_cosine"
    cfg["experiment"]["disturbance_target"] = "RightHipPitch,RightKnee"

    defaults = cfg["controller"]["defaults"]
    defaults["observer_gain_q"] = float(NOMINAL_OBSERVER_Q)
    defaults["observer_gain_dq"] = float(NOMINAL_OBSERVER_DQ)
    defaults["ku_q"] = float(U1_KU_Q)
    defaults["ku_dq"] = float(U1_KU_DQ)

    for joint_id, so in [(HIP, candidate.hip_so), (KNEE, candidate.knee_so)]:
        jc = yaml_joint(cfg, joint_id)
        jc["enabled"] = True
        jc["observer_gain_q"] = float(NOMINAL_OBSERVER_Q * so)
        jc["observer_gain_dq"] = float(NOMINAL_OBSERVER_DQ * so)
        jc["ku_q"] = float(U1_KU_Q)
        jc["ku_dq"] = float(U1_KU_DQ)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True), encoding="utf-8")


def run_mujoco_case(config: Path, run_dir: Path, args: argparse.Namespace) -> bool:
    log_path = run_dir / "mujoco_closed_loop_log.csv"
    if args.skip_existing and log_path.exists():
        return True
    cmd = [
        sys.executable,
        "scripts/run_mujoco.py",
        "--scene", str(args.scene),
        "--config", str(config),
        "--stepper", str(args.stepper),
        "--out-dir", str(run_dir),
        "--duration", f"{args.duration:g}",
        "--dt", f"{args.dt:g}",
        "--disturbance-joints", args.disturbance_joints,
        "--disturbance-torques", args.disturbance_torques,
        "--disturbance-start", f"{args.disturbance_start:g}",
        "--disturbance-plateau-start", f"{args.disturbance_plateau_start:g}",
        "--disturbance-plateau-end", f"{args.disturbance_plateau_end:g}",
        "--disturbance-end", f"{args.disturbance_end:g}",
        "--log-every-step",
        "--export-summary",
    ]
    try:
        subprocess.run(cmd, cwd=ROOT, check=True)
        return True
    except subprocess.CalledProcessError as exc:
        run_dir.mkdir(parents=True, exist_ok=True)
        (run_dir / "run_error.txt").write_text(str(exc), encoding="utf-8")
        return False


def rms(x: np.ndarray) -> float:
    if len(x) == 0:
        return float("nan")
    return float(np.sqrt(np.mean(np.square(x))))


def window_mask(t: np.ndarray, start: float, stop: float) -> np.ndarray:
    return (t >= start) & (t < stop)


def load_mujoco_arrays(log_path: Path) -> dict[str, np.ndarray]:
    df = pd.read_csv(log_path)
    df = df[df["joint_id"].isin(JOINTS)].copy()
    pivot = df.pivot_table(
        index="t",
        columns="joint_id",
        values=[
            "q_ref_shaped", "q_actual", "dq_actual", "tau_total",
            "tau_applied", "tau_dist", "eta_u", "flags", "joint_flags",
        ],
        aggfunc="first",
    ).dropna()
    t = pivot.index.to_numpy(dtype=float)
    out: dict[str, np.ndarray] = {"t": t}
    for joint_id, suffix in [(HIP, "h"), (KNEE, "k")]:
        for col in ["q_ref_shaped", "q_actual", "dq_actual", "tau_total", "tau_applied", "tau_dist", "eta_u"]:
            out[f"{col}_{suffix}"] = pivot[(col, joint_id)].to_numpy(dtype=float)
    out["flags_h"] = pivot[("flags", HIP)].to_numpy(dtype=float)
    out["flags_k"] = pivot[("flags", KNEE)].to_numpy(dtype=float)
    out["joint_flags_h"] = pivot[("joint_flags", HIP)].to_numpy(dtype=float)
    out["joint_flags_k"] = pivot[("joint_flags", KNEE)].to_numpy(dtype=float)
    out["e_h"] = out["q_ref_shaped_h"] - out["q_actual_h"]
    out["e_k"] = out["q_ref_shaped_k"] - out["q_actual_k"]
    out["e_coord"] = out["e_h"] - out["e_k"]
    return out


def mujoco_metrics(arr: dict[str, np.ndarray], candidate: MujocoCandidate, args: argparse.Namespace) -> dict[str, Any]:
    t = arr["t"]
    dt = float(np.median(np.diff(t))) if len(t) > 1 else args.dt
    dw = window_mask(t, args.disturbance_start, args.disturbance_end)
    rw = window_mask(t, args.disturbance_end, min(args.duration, args.disturbance_end + 2.0))
    flags = np.concatenate([arr["flags_h"], arr["flags_k"], arr["joint_flags_h"], arr["joint_flags_k"]]).astype(np.int64)

    def delta_rms(key: str, mask: np.ndarray) -> float:
        y = arr[key][mask]
        return rms(np.diff(y)) if len(y) > 1 else float("nan")

    row: dict[str, Any] = {
        "case": candidate.case,
        "label": candidate.label,
        "hip_so": candidate.hip_so,
        "knee_so": candidate.knee_so,
        "hip_observer_gain_q": candidate.hip_so * NOMINAL_OBSERVER_Q,
        "knee_observer_gain_q": candidate.knee_so * NOMINAL_OBSERVER_Q,
        "ku_q": U1_KU_Q,
        "ku_dq": U1_KU_DQ,
        "dt_log_s": dt,
        "disturbance_samples": int(np.count_nonzero(dw)),
        "recovery_samples": int(np.count_nonzero(rw)),
        "hip_rmse": rms(arr["e_h"][dw]),
        "knee_rmse": rms(arr["e_k"][dw]),
        "coord_rmse": rms(arr["e_coord"][dw]),
        "hip_recovery_rmse": rms(arr["e_h"][rw]),
        "knee_recovery_rmse": rms(arr["e_k"][rw]),
        "coord_recovery_rmse": rms(arr["e_coord"][rw]),
        "hip_delta_tau_total_rms": delta_rms("tau_total_h", dw),
        "knee_delta_tau_total_rms": delta_rms("tau_total_k", dw),
        "hip_tau_total_rms": rms(arr["tau_total_h"][dw]),
        "knee_tau_total_rms": rms(arr["tau_total_k"][dw]),
        "hip_eta_u_rms": rms(arr["eta_u_h"][dw]),
        "knee_eta_u_rms": rms(arr["eta_u_k"][dw]),
        "combined_flags": int(np.bitwise_or.reduce(flags)) if len(flags) else 0,
    }
    return row


def welch_psd(x: np.ndarray, fs: float) -> tuple[np.ndarray, np.ndarray]:
    y = np.asarray(x, dtype=float)
    y = y - np.nanmean(y)
    y = np.nan_to_num(y)
    nperseg = min(512, len(y))
    noverlap = nperseg // 2
    return signal.welch(y, fs=fs, window="hann", nperseg=nperseg, noverlap=noverlap, scaling="density")


def band_power(freq: np.ndarray, psd: np.ndarray, lo: float, hi: float) -> float:
    mask = (freq >= lo) & (freq < hi)
    if np.count_nonzero(mask) < 2:
        return float("nan")
    return float(np.trapezoid(psd[mask], freq[mask]))


def analyze_mujoco_frequency(cases: dict[str, dict[str, np.ndarray]], args: argparse.Namespace) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    for label, arr in cases.items():
        t = arr["t"]
        fs = 1.0 / float(np.median(np.diff(t)))
        dw = window_mask(t, args.disturbance_start, args.disturbance_end)
        for signal_name, key in [
            ("hip_tau_total", "tau_total_h"),
            ("knee_tau_total", "tau_total_k"),
            ("hip_delta_tau_total", "tau_total_h"),
            ("knee_delta_tau_total", "tau_total_k"),
            ("hip_eta_u", "eta_u_h"),
            ("knee_eta_u", "eta_u_k"),
            ("coord_error", "e_coord"),
        ]:
            x = arr[key][dw]
            if "delta" in signal_name:
                x = np.r_[0.0, np.diff(x)]
            freq, psd = welch_psd(x, fs)
            usable = (freq >= 0.5) & (freq <= 100.0)
            idx = np.where(usable)[0][int(np.argmax(psd[usable]))] if np.any(usable) else 0
            rows.append({
                "case": label,
                "signal": signal_name,
                "peak_freq_hz_0p5_100": float(freq[idx]),
                "peak_psd": float(psd[idx]),
                "bandpower_0p5_3hz": band_power(freq, psd, 0.5, 3.0),
                "bandpower_3_15hz": band_power(freq, psd, 3.0, 15.0),
                "bandpower_15_80hz": band_power(freq, psd, 15.0, 80.0),
            })
    return pd.DataFrame(rows)


def disturbance_profile(t: np.ndarray, args: argparse.Namespace, amp: float) -> np.ndarray:
    y = np.zeros_like(t)
    t0, t1, t2, t3 = (
        args.disturbance_start,
        args.disturbance_plateau_start,
        args.disturbance_plateau_end,
        args.disturbance_end,
    )
    rise = (t >= t0) & (t < t1)
    plateau = (t >= t1) & (t <= t2)
    fall = (t > t2) & (t < t3)
    if np.any(rise):
        r = (t[rise] - t0) / max(t1 - t0, 1.0e-12)
        y[rise] = 0.5 - 0.5 * np.cos(np.pi * r)
    y[plateau] = 1.0
    if np.any(fall):
        r = (t[fall] - t2) / max(t3 - t2, 1.0e-12)
        y[fall] = 0.5 + 0.5 * np.cos(np.pi * r)
    return amp * y


def control_coefficients(k: np.ndarray, ku: np.ndarray, spec: AblationSpec) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    ux = -spec.feedback_x * k
    uxhat = -spec.feedback_xhat * k
    ueta = -spec.center_eta * k - spec.input_eta * ku
    return ux, uxhat, ueta


def generalized_augmented_matrix(
    a: np.ndarray,
    b: np.ndarray,
    k: np.ndarray,
    ku: np.ndarray,
    ko: np.ndarray,
    alpha: float,
    residual_lambda: float,
    delay_steps: int,
    spec: AblationSpec,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    nx = 6 + 2 * delay_steps
    acl = np.zeros((nx, nx), dtype=float)
    bd = np.zeros((nx, 1), dtype=float)
    bv = np.zeros((nx, 2), dtype=float)
    ux, uxhat, ueta = control_coefficients(k, ku, spec)

    acl[0:2, 0:2] = a + b @ ux
    acl[0:2, 2:4] = b @ uxhat
    acl[0:2, 4:6] = b @ ueta
    bd[0:2, 0:1] = b

    # The nominal predictor propagates the feedback center.  For the current
    # full EID structure this reduces to xhat+ = A(xhat + eta) + B u, matching
    # augmented_mats_identified() in run_bychen_mujoco_deep_analysis.py.
    acl[2:4, 0:2] = spec.feedback_x * a + b @ ux
    acl[2:4, 2:4] = spec.feedback_xhat * a + b @ uxhat
    acl[2:4, 4:6] = spec.center_eta * a + b @ ueta

    if delay_steps == 0:
        acl[4:6, 0:2] = alpha * ko
    else:
        z_m = 6 + 2 * (delay_steps - 1)
        acl[4:6, z_m:z_m + 2] = alpha * ko
    acl[4:6, 2:4] = -alpha * ko
    acl[4:6, 4:6] = (1.0 - alpha) * np.eye(2) - alpha * residual_lambda * ko
    bv[4:6, :] = alpha * ko

    if delay_steps > 0:
        acl[6:8, 0:2] = np.eye(2)
        for i in range(1, delay_steps):
            dst = 6 + 2 * i
            src = 6 + 2 * (i - 1)
            acl[dst:dst + 2, src:src + 2] = np.eye(2)

    c_error = np.zeros((1, nx), dtype=float)
    c_error[0, 0] = -1.0
    c_u = np.zeros((1, nx), dtype=float)
    c_u[0, 0:2] = ux
    c_u[0, 2:4] = uxhat
    c_u[0, 4:6] = ueta
    return acl, bd, bv, c_error, c_u


def simulate_linear_joint(
    a: np.ndarray,
    b: np.ndarray,
    k: np.ndarray,
    ku: np.ndarray,
    ko: np.ndarray,
    disturbance: np.ndarray,
    alpha: float,
    residual_lambda: float,
    spec: AblationSpec,
    delay_steps: int = 0,
) -> dict[str, np.ndarray]:
    acl, bd, _, _, c_u = generalized_augmented_matrix(
        a, b, k, ku, ko, alpha, residual_lambda, delay_steps, spec
    )
    x = np.zeros(acl.shape[0], dtype=float)
    q_error = np.zeros_like(disturbance)
    u = np.zeros_like(disturbance)
    eta_u = np.zeros_like(disturbance)
    for i, d in enumerate(disturbance):
        q_error[i] = -x[0]
        u[i] = float((c_u @ x).squeeze())
        eta_u[i] = float(spec.input_eta * (ku @ x[4:6]).squeeze())
        x = acl @ x + bd[:, 0] * d
    return {"q_error": q_error, "u": u, "eta_u": eta_u}


def numerical_metrics(
    t: np.ndarray,
    hip: dict[str, np.ndarray],
    knee: dict[str, np.ndarray],
    args: argparse.Namespace,
) -> dict[str, float]:
    dw = window_mask(t, args.disturbance_start, args.disturbance_end)
    rw = window_mask(t, args.disturbance_end, min(args.duration, args.disturbance_end + 2.0))
    coord = hip["q_error"] - knee["q_error"]
    return {
        "hip_rmse": rms(hip["q_error"][dw]),
        "knee_rmse": rms(knee["q_error"][dw]),
        "coord_rmse": rms(coord[dw]),
        "hip_recovery_rmse": rms(hip["q_error"][rw]),
        "knee_recovery_rmse": rms(knee["q_error"][rw]),
        "coord_recovery_rmse": rms(coord[rw]),
        "hip_u_rms": rms(hip["u"][dw]),
        "knee_u_rms": rms(knee["u"][dw]),
        "hip_delta_u_rms": rms(np.diff(hip["u"][dw])),
        "knee_delta_u_rms": rms(np.diff(knee["u"][dw])),
        "hip_eta_u_rms": rms(hip["eta_u"][dw]),
        "knee_eta_u_rms": rms(knee["eta_u"][dw]),
        "coord_peak_abs_error": float(np.max(np.abs(coord[dw]))),
    }


def freq_response_mag(
    a: np.ndarray,
    b: np.ndarray,
    c: np.ndarray,
    freqs: np.ndarray,
    dt: float,
    norm_input: bool = False,
) -> np.ndarray:
    eye = np.eye(a.shape[0])
    out = []
    for f in freqs:
        z = np.exp(1j * 2.0 * np.pi * f * dt)
        h = c @ np.linalg.solve(z * eye - a, b)
        if norm_input:
            out.append(float(np.linalg.norm(h.reshape(-1), ord=2)))
        else:
            out.append(float(abs(h.reshape(-1)[0])))
    return np.asarray(out)


def run_numerical_studies(sweep_dir: Path, out_dir: Path, args: argparse.Namespace) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    plants = fit_identified_plants(sweep_dir, fit_start=3.0, fit_end=7.8, ridge=1.0e-10)
    t = np.arange(0.0, args.duration, args.dt)
    dist = {
        HIP: disturbance_profile(t, args, 6.0),
        KNEE: disturbance_profile(t, args, -4.0),
    }
    cases = {
        "O4+U1": {HIP: 1.00, KNEE: 1.00},
        "O5+U1": {HIP: 1.25, KNEE: 1.25},
    }
    k = np.array([[KP, KD]], dtype=float)
    ku = np.array([[U1_KU_Q, U1_KU_DQ]], dtype=float)

    ablation_rows: list[dict[str, Any]] = []
    ablation_series: dict[tuple[str, str], dict[str, np.ndarray]] = {}
    for case_label, scales in cases.items():
        for spec in ABLATIONS:
            series = {}
            for joint_id in JOINTS:
                plant = plants[joint_id]
                ko = np.diag([
                    NOMINAL_OBSERVER_Q * scales[joint_id],
                    NOMINAL_OBSERVER_DQ * scales[joint_id],
                ])
                series[JOINT_LABELS[joint_id]] = simulate_linear_joint(
                    plant.a, plant.b, k, ku, ko, dist[joint_id], ALPHA, 1.0, spec
                )
            row = {
                "case": case_label,
                "ablation": spec.key,
                "ablation_label": spec.label,
                **numerical_metrics(t, series["Hip"], series["Knee"], args),
            }
            ablation_rows.append(row)
            ablation_series[(case_label, spec.label)] = {
                "hip_error": series["Hip"]["q_error"],
                "knee_error": series["Knee"]["q_error"],
                "coord_error": series["Hip"]["q_error"] - series["Knee"]["q_error"],
                "hip_u": series["Hip"]["u"],
                "knee_u": series["Knee"]["u"],
            }

    residual_rows: list[dict[str, Any]] = []
    pole_rows: list[dict[str, Any]] = []
    freqs = np.logspace(np.log10(0.1), np.log10(100.0), 400)
    full = next(s for s in ABLATIONS if s.key == "full_eid")
    for case_label, scales in cases.items():
        for lam in [0.0, 0.5, 1.0]:
            series = {}
            for joint_id in JOINTS:
                plant = plants[joint_id]
                ko = np.diag([
                    NOMINAL_OBSERVER_Q * scales[joint_id],
                    NOMINAL_OBSERVER_DQ * scales[joint_id],
                ])
                series[JOINT_LABELS[joint_id]] = simulate_linear_joint(
                    plant.a, plant.b, k, ku, ko, dist[joint_id], ALPHA, lam, full
                )
                acl, bd, bv, c_error, c_u = generalized_augmented_matrix(
                    plant.a, plant.b, k, ku, ko, ALPHA, lam, delay_steps=0, spec=full
                )
                eig = np.linalg.eigvals(acl)
                h_dist = freq_response_mag(acl, bd, c_error, freqs, args.dt)
                h_noise = freq_response_mag(acl, bv, c_u, freqs, args.dt, norm_input=True)
                pole_rows.append({
                    "case": case_label,
                    "lambda": lam,
                    "joint": JOINT_LABELS[joint_id],
                    "max_pole_abs": float(np.max(np.abs(eig))),
                    "num_poles_outside_unit_circle": int(np.count_nonzero(np.abs(eig) >= 1.0)),
                    "disturbance_to_error_peak_mag": float(np.max(h_dist[(freqs >= 0.5) & (freqs <= 80.0)])),
                    "noise_to_control_peak_mag": float(np.max(h_noise[(freqs >= 0.5) & (freqs <= 80.0)])),
                })
            residual_rows.append({
                "case": case_label,
                "lambda": lam,
                **numerical_metrics(t, series["Hip"], series["Knee"], args),
            })

    ablation_df = pd.DataFrame(ablation_rows)
    residual_df = pd.DataFrame(residual_rows)
    pole_df = pd.DataFrame(pole_rows)
    out_dir.mkdir(parents=True, exist_ok=True)
    ablation_df.to_csv(out_dir / "numerical_ablation_metrics.csv", index=False)
    residual_df.to_csv(out_dir / "numerical_residual_lambda_metrics.csv", index=False)
    pole_df.to_csv(out_dir / "numerical_residual_lambda_poles_frequency.csv", index=False)
    plot_ablation_results(ablation_df, out_dir / "figures")
    plot_residual_lambda_results(residual_df, pole_df, out_dir / "figures")
    return ablation_df, residual_df, pole_df


def plot_mujoco_o4_o5(metrics: pd.DataFrame, freq_df: pd.DataFrame, cases: dict[str, dict[str, np.ndarray]], fig_dir: Path, args: argparse.Namespace) -> None:
    fig_dir.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 3, figsize=(6.85, 2.35))
    specs = [
        ("coord_rmse", "knee_delta_tau_total_rms", "Coord. RMSE [rad]", r"Knee $\Delta\tau$ RMS"),
        ("hip_rmse", "hip_delta_tau_total_rms", "Hip RMSE [rad]", r"Hip $\Delta\tau$ RMS"),
        ("knee_rmse", "knee_delta_tau_total_rms", "Knee RMSE [rad]", r"Knee $\Delta\tau$ RMS"),
    ]
    for ax, (xcol, ycol, xlabel, ylabel) in zip(axes, specs):
        for _, row in metrics.iterrows():
            ax.scatter(row[xcol], row[ycol], s=40, color=COLORS[row["label"]], edgecolor="black", linewidth=0.35)
            ax.annotate(row["label"], (row[xcol], row[ycol]), xytext=(4, 4), textcoords="offset points")
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
    for idx, ax in enumerate(axes):
        ax.text(-0.16, 1.06, f"({chr(ord('a') + idx)})", transform=ax.transAxes, fontweight="bold")
    fig.subplots_adjust(left=0.08, right=0.99, bottom=0.26, top=0.92, wspace=0.50)
    fig.savefig(fig_dir / "mujoco_o4u1_o5u1_pareto.png")
    fig.savefig(fig_dir / "mujoco_o4u1_o5u1_pareto.pdf")
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(6.85, 4.45), sharex=True)
    plot_specs = [
        ("knee_delta_tau_total", "Knee delta tau PSD"),
        ("knee_eta_u", r"Knee $\eta_u$ PSD"),
        ("hip_delta_tau_total", "Hip delta tau PSD"),
        ("coord_error", "Coord. error PSD"),
    ]
    for ax, (signal_name, ylabel) in zip(axes.ravel(), plot_specs):
        for label, arr in cases.items():
            t = arr["t"]
            fs = 1.0 / float(np.median(np.diff(t)))
            dw = window_mask(t, args.disturbance_start, args.disturbance_end)
            key_map = {
                "knee_delta_tau_total": "tau_total_k",
                "hip_delta_tau_total": "tau_total_h",
                "knee_eta_u": "eta_u_k",
                "coord_error": "e_coord",
            }
            x = arr[key_map[signal_name]][dw]
            if "delta" in signal_name:
                x = np.r_[0.0, np.diff(x)]
            f, p = welch_psd(x, fs)
            mask = (f >= 0.5) & (f <= 100.0)
            ax.semilogy(f[mask], p[mask] + 1.0e-18, lw=1.0, color=COLORS[label], label=label)
        ax.set_ylabel(ylabel)
    for ax in axes[-1, :]:
        ax.set_xlabel("frequency [Hz]")
    axes[0, 0].legend(frameon=False)
    for idx, ax in enumerate(axes.ravel()):
        ax.text(-0.16, 1.06, f"({chr(ord('a') + idx)})", transform=ax.transAxes, fontweight="bold")
    fig.subplots_adjust(left=0.10, right=0.99, bottom=0.12, top=0.94, hspace=0.38, wspace=0.35)
    fig.savefig(fig_dir / "mujoco_o4u1_o5u1_spectrum.png")
    fig.savefig(fig_dir / "mujoco_o4u1_o5u1_spectrum.pdf")
    plt.close(fig)


def plot_ablation_results(df: pd.DataFrame, fig_dir: Path) -> None:
    fig_dir.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 3, figsize=(7.25, 2.55))
    metrics = [
        ("coord_rmse", "Coord. RMSE [rad]"),
        ("knee_delta_u_rms", r"Knee $\Delta u$ RMS"),
        ("knee_eta_u_rms", r"Knee $\eta_u$ RMS"),
    ]
    labels = [s.label for s in ABLATIONS]
    x = np.arange(len(labels))
    width = 0.36
    for ax, (metric, ylabel) in zip(axes, metrics):
        for offset, case in [(-width / 2, "O4+U1"), (width / 2, "O5+U1")]:
            sub = df[df["case"] == case].set_index("ablation_label").loc[labels]
            ax.bar(x + offset, sub[metric].to_numpy(dtype=float), width=width, label=case, color=COLORS[case], alpha=0.82)
        ax.set_xticks(x)
        ax.set_xticklabels(["PD", "center", "input", "full"], rotation=20, ha="right")
        ax.set_ylabel(ylabel)
    axes[0].legend(frameon=False)
    for idx, ax in enumerate(axes):
        ax.text(-0.16, 1.06, f"({chr(ord('a') + idx)})", transform=ax.transAxes, fontweight="bold")
    fig.subplots_adjust(left=0.08, right=0.99, bottom=0.30, top=0.92, wspace=0.36)
    fig.savefig(fig_dir / "numerical_ablation_summary.png")
    fig.savefig(fig_dir / "numerical_ablation_summary.pdf")
    plt.close(fig)


def plot_residual_lambda_results(residual_df: pd.DataFrame, pole_df: pd.DataFrame, fig_dir: Path) -> None:
    fig_dir.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 3, figsize=(7.25, 2.45))
    for case in ["O4+U1", "O5+U1"]:
        sub = residual_df[residual_df["case"] == case].sort_values("lambda")
        axes[0].plot(sub["lambda"], sub["coord_rmse"], marker="o", color=COLORS[case], label=case)
        axes[1].plot(sub["lambda"], sub["knee_delta_u_rms"], marker="o", color=COLORS[case], label=case)
        psub = pole_df[(pole_df["case"] == case) & (pole_df["joint"] == "Knee")].sort_values("lambda")
        axes[2].plot(psub["lambda"], psub["noise_to_control_peak_mag"], marker="o", color=COLORS[case], label=case)
    axes[0].set_ylabel("Coord. RMSE [rad]")
    axes[1].set_ylabel(r"Knee $\Delta u$ RMS")
    axes[2].set_ylabel(r"Knee peak $v \rightarrow u$")
    for ax in axes:
        ax.set_xlabel(r"residual $\lambda$")
    axes[0].legend(frameon=False)
    for idx, ax in enumerate(axes):
        ax.text(-0.16, 1.06, f"({chr(ord('a') + idx)})", transform=ax.transAxes, fontweight="bold")
    fig.subplots_adjust(left=0.08, right=0.99, bottom=0.22, top=0.90, wspace=0.42)
    fig.savefig(fig_dir / "numerical_residual_lambda_tradeoff.png")
    fig.savefig(fig_dir / "numerical_residual_lambda_tradeoff.pdf")
    plt.close(fig)


def write_markdown_summary(
    out_dir: Path,
    mujoco_metrics_df: pd.DataFrame,
    mujoco_freq_df: pd.DataFrame,
    ablation_df: pd.DataFrame,
    residual_df: pd.DataFrame,
    pole_df: pd.DataFrame,
) -> Path:
    report_dir = analysis_report_dir(out_dir)
    report_dir.mkdir(parents=True, exist_ok=True)
    summary_path = report_dir / "bychen_future_work_completion_summary.md"

    def md_table(df: pd.DataFrame, cols: list[str], floatfmt: str = ".6g") -> str:
        lines = ["| " + " | ".join(cols) + " |", "| " + " | ".join(["---"] * len(cols)) + " |"]
        for _, row in df[cols].iterrows():
            cells = []
            for col in cols:
                val = row[col]
                if isinstance(val, (float, np.floating)):
                    cells.append(format(float(val), floatfmt))
                else:
                    cells.append(str(val))
            lines.append("| " + " | ".join(cells) + " |")
        return "\n".join(lines)

    mu_cols = [
        "label", "hip_rmse", "knee_rmse", "coord_rmse",
        "hip_delta_tau_total_rms", "knee_delta_tau_total_rms",
        "hip_eta_u_rms", "knee_eta_u_rms", "combined_flags",
    ]
    ab_cols = [
        "case", "ablation_label", "coord_rmse", "knee_delta_u_rms",
        "knee_eta_u_rms", "coord_recovery_rmse",
    ]
    res_cols = ["case", "lambda", "coord_rmse", "knee_delta_u_rms", "knee_eta_u_rms"]
    pole_cols = ["case", "lambda", "joint", "max_pole_abs", "noise_to_control_peak_mag"]

    lines = [
        "# Future-work completion analysis",
        "",
        "Scope: MuJoCo candidate simulation plus local identified-model numerical studies. "
        "These results are mechanism-screening evidence and do not replace hardware validation.",
        "",
        "## MuJoCo O4+U1 versus O5+U1",
        "",
        md_table(mujoco_metrics_df, mu_cols),
        "",
        "## Numerical ablation",
        "",
        md_table(ablation_df, ab_cols),
        "",
        "## Residual lambda numerical study",
        "",
        md_table(residual_df, res_cols),
        "",
        "## Residual lambda pole/noise summary",
        "",
        md_table(pole_df[pole_df["joint"] == "Knee"], pole_cols),
        "",
        "Generated files:",
        "",
        f"- `{repo_relpath(out_dir / 'mujoco_o4u1_o5u1_metrics.csv')}`",
        f"- `{repo_relpath(out_dir / 'mujoco_o4u1_o5u1_frequency_summary.csv')}`",
        f"- `{repo_relpath(out_dir / 'numerical_ablation_metrics.csv')}`",
        f"- `{repo_relpath(out_dir / 'numerical_residual_lambda_metrics.csv')}`",
        f"- `{repo_relpath(out_dir / 'numerical_residual_lambda_poles_frequency.csv')}`",
        f"- `{repo_relpath(out_dir / 'figures' / 'mujoco_o4u1_o5u1_pareto.png')}`",
        f"- `{repo_relpath(out_dir / 'figures' / 'mujoco_o4u1_o5u1_spectrum.png')}`",
        f"- `{repo_relpath(out_dir / 'figures' / 'numerical_ablation_summary.png')}`",
        f"- `{repo_relpath(out_dir / 'figures' / 'numerical_residual_lambda_tradeoff.png')}`",
        "",
    ]
    summary_path.write_text("\n".join(lines), encoding="utf-8")
    return summary_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-config", type=Path, default=Path("config/h1_real_p4_ku_u1_hip_knee_eid.yaml"))
    parser.add_argument("--scene", type=Path, default=Path("h1_official_mujoco/scene.xml"))
    parser.add_argument("--stepper", type=Path, default=Path("build/Debug/h1_controller_stepper.exe"))
    parser.add_argument("--sweep-dir", type=Path, default=Path("analysis_artifacts/bychen_mujoco_sweep"))
    parser.add_argument("--out-dir", type=Path, default=Path("analysis_artifacts/bychen_future_work_completion"))
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--disturbance-joints", default="1,2")
    parser.add_argument("--disturbance-torques", default="6,-4")
    parser.add_argument("--disturbance-start", type=float, default=4.0)
    parser.add_argument("--disturbance-plateau-start", type=float, default=4.2)
    parser.add_argument("--disturbance-plateau-end", type=float, default=5.2)
    parser.add_argument("--disturbance-end", type=float, default=5.4)
    parser.add_argument("--skip-mujoco", action="store_true")
    parser.add_argument("--skip-existing", action="store_true")
    args = parser.parse_args()

    configure_matplotlib()
    out_dir = args.out_dir if args.out_dir.is_absolute() else ROOT / args.out_dir
    sweep_dir = args.sweep_dir if args.sweep_dir.is_absolute() else ROOT / args.sweep_dir
    config_dir = out_dir / "configs"
    run_dir = out_dir / "runs"
    fig_dir = out_dir / "figures"
    out_dir.mkdir(parents=True, exist_ok=True)

    mujoco_rows: list[dict[str, Any]] = []
    mujoco_cases: dict[str, dict[str, np.ndarray]] = {}
    for candidate in MUJOCO_CANDIDATES:
        cfg_path = config_dir / f"{candidate.case}.yaml"
        case_run_dir = run_dir / candidate.case
        build_mujoco_config(args.base_config, cfg_path, candidate)
        ok = True
        if not args.skip_mujoco:
            ok = run_mujoco_case(cfg_path, case_run_dir, args)
        log_path = case_run_dir / "mujoco_closed_loop_log.csv"
        row: dict[str, Any] = {
            "case": candidate.case,
            "label": candidate.label,
            "hip_so": candidate.hip_so,
            "knee_so": candidate.knee_so,
            "run_success": bool(ok and log_path.exists()),
        }
        if row["run_success"]:
            arr = load_mujoco_arrays(log_path)
            row.update(mujoco_metrics(arr, candidate, args))
            mujoco_cases[candidate.label] = arr
        mujoco_rows.append(row)
    mujoco_metrics_df = pd.DataFrame(mujoco_rows)
    mujoco_metrics_df.to_csv(out_dir / "mujoco_o4u1_o5u1_metrics.csv", index=False)
    mujoco_freq_df = analyze_mujoco_frequency(mujoco_cases, args) if mujoco_cases else pd.DataFrame()
    mujoco_freq_df.to_csv(out_dir / "mujoco_o4u1_o5u1_frequency_summary.csv", index=False)
    if mujoco_cases:
        plot_mujoco_o4_o5(mujoco_metrics_df[mujoco_metrics_df["run_success"] == True], mujoco_freq_df, mujoco_cases, fig_dir, args)  # noqa: E712

    ablation_df, residual_df, pole_df = run_numerical_studies(sweep_dir, out_dir, args)
    summary_path = write_markdown_summary(out_dir, mujoco_metrics_df, mujoco_freq_df, ablation_df, residual_df, pole_df)

    print(f"summary={summary_path}")
    print(f"metrics={out_dir / 'mujoco_o4u1_o5u1_metrics.csv'}")
    print(f"figures={fig_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
