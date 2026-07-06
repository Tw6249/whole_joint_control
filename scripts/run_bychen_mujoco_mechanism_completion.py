#!/usr/bin/env python3
"""Complete the MuJoCo mechanism studies for the Chen-question report.

This script uses the real C++ controller stepper for all cases.  It covers
three layers that were previously mixed with local numerical substitutes:

1. candidate observer-gain mechanisms,
2. EID structural ablations through `eid_mode`, and
3. residual-definition scans through `residual_eta_lambda`.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import pandas as pd
import yaml
from scipy import signal

from report_paths import analysis_report_dir, repo_relpath


ROOT = Path(__file__).resolve().parents[1]
HIP = 1
KNEE = 2
JOINTS = [HIP, KNEE]
NOMINAL_OBSERVER_Q = 0.8
NOMINAL_OBSERVER_DQ = 0.2
U1_KU_Q = 6.0
U1_KU_DQ = 0.5
SATURATION_FLAG = 1 << 2
CASE_COLORS = {
    "A_O4U1": "#0072B2",
    "B_HipUp": "#009E73",
    "D_HipUpKneeDown": "#D55E00",
    "O5U1": "#CC79A7",
    "KneeVelDown": "#E69F00",
}
CASE_SHORT = {
    "A_O4U1": "A",
    "B_HipUp": "B",
    "D_HipUpKneeDown": "D",
    "O5U1": "O5",
    "KneeVelDown": "KVD",
}
CASE_LEGEND = {
    "A_O4U1": "A: O4+U1",
    "B_HipUp": "B: hip up",
    "D_HipUpKneeDown": "D: hip up, knee down",
    "O5U1": "O5+U1",
    "KneeVelDown": "KneeVelDown",
}


@dataclass(frozen=True)
class MujocoMechanismCase:
    layer: str
    case: str
    label: str
    purpose: str
    hip_q_scale: float = 1.0
    hip_dq_scale: float = 1.0
    knee_q_scale: float = 1.0
    knee_dq_scale: float = 1.0
    eid_mode: str = "full_eid"
    residual_eta_lambda: float = 1.0


def build_cases() -> list[MujocoMechanismCase]:
    candidates = [
        MujocoMechanismCase(
            "candidate", "A_O4U1", "A: O4+U1",
            "baseline: nominal hip/knee observer gains with U1 input compensation",
        ),
        MujocoMechanismCase(
            "candidate", "B_HipUp", "B: hip observer up",
            "increase hip observer gains only",
            hip_q_scale=1.25, hip_dq_scale=1.25,
        ),
        MujocoMechanismCase(
            "candidate", "D_HipUpKneeDown", "D: hip up, knee down",
            "test Chen's asymmetric direction",
            hip_q_scale=1.25, hip_dq_scale=1.25,
            knee_q_scale=0.75, knee_dq_scale=0.75,
        ),
        MujocoMechanismCase(
            "candidate", "O5U1", "O5+U1",
            "increase both hip and knee observer gains",
            hip_q_scale=1.25, hip_dq_scale=1.25,
            knee_q_scale=1.25, knee_dq_scale=1.25,
        ),
        MujocoMechanismCase(
            "candidate", "KneeVelDown", "knee velocity observer down",
            "keep knee position observer gain fixed and reduce only knee velocity observer gain",
            knee_dq_scale=0.75,
        ),
    ]

    structure: list[MujocoMechanismCase] = []
    for base_case, base_label, hip_scale, knee_scale in [
        ("O4U1", "O4+U1", 1.0, 1.0),
        ("O5U1", "O5+U1", 1.25, 1.25),
    ]:
        for mode, mode_label in [
            ("pd_inverse_only", "PD inverse only"),
            ("center_feedback_only", "center feedback only"),
            ("input_compensation_only", "input compensation only"),
            ("full_eid", "full EID"),
        ]:
            structure.append(
                MujocoMechanismCase(
                    "structure",
                    f"{base_case}_{mode}",
                    f"{base_label}: {mode_label}",
                    f"structural ablation at {base_label}",
                    hip_q_scale=hip_scale,
                    hip_dq_scale=hip_scale,
                    knee_q_scale=knee_scale,
                    knee_dq_scale=knee_scale,
                    eid_mode=mode,
                )
            )

    residual: list[MujocoMechanismCase] = []
    for base_case, base_label, hip_scale, knee_scale in [
        ("O4U1", "O4+U1", 1.0, 1.0),
        ("O5U1", "O5+U1", 1.25, 1.25),
    ]:
        for lam in [0.0, 0.5, 1.0]:
            residual.append(
                MujocoMechanismCase(
                    "residual",
                    f"{base_case}_lambda_{lam:g}".replace(".", "p"),
                    f"{base_label}: lambda={lam:g}",
                    f"residual-definition scan at {base_label}",
                    hip_q_scale=hip_scale,
                    hip_dq_scale=hip_scale,
                    knee_q_scale=knee_scale,
                    knee_dq_scale=knee_scale,
                    residual_eta_lambda=lam,
                )
            )
    return candidates + structure + residual


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


def panel_label(ax: plt.Axes, label: str) -> None:
    ax.text(
        -0.13,
        1.04,
        label,
        transform=ax.transAxes,
        fontweight="bold",
        va="bottom",
        ha="left",
    )


def pad_axis(ax: plt.Axes, xpad: float = 0.10, ypad: float = 0.14) -> None:
    ax.margins(x=xpad, y=ypad)


def yaml_joint(cfg: dict[str, Any], joint_id: int) -> dict[str, Any]:
    joints = cfg["controller"]["joints"]
    if joint_id in joints:
        return joints[joint_id]
    return joints[str(joint_id)]


def observer_gain_q(scale: float) -> float:
    return float(NOMINAL_OBSERVER_Q * scale)


def observer_gain_dq(scale: float) -> float:
    return float(NOMINAL_OBSERVER_DQ * scale)


def build_config(base_config: Path, out_path: Path, case: MujocoMechanismCase) -> None:
    cfg = yaml.safe_load(base_config.read_text(encoding="utf-8"))
    cfg.setdefault("experiment", {})
    cfg["experiment"]["id"] = "ByChen_MuJoCo_Mechanism"
    cfg["experiment"]["condition"] = case.case
    cfg["experiment"]["disturbance_method"] = "mujoco_input_torque_half_cosine"
    cfg["experiment"]["disturbance_target"] = "RightHipPitch,RightKnee"

    defaults = cfg["controller"]["defaults"]
    defaults["observer_gain_q"] = float(NOMINAL_OBSERVER_Q)
    defaults["observer_gain_dq"] = float(NOMINAL_OBSERVER_DQ)
    defaults["ku_q"] = float(U1_KU_Q)
    defaults["ku_dq"] = float(U1_KU_DQ)
    defaults["eid_mode"] = case.eid_mode
    defaults["residual_eta_lambda"] = float(case.residual_eta_lambda)

    joint_specs = {
        HIP: (case.hip_q_scale, case.hip_dq_scale),
        KNEE: (case.knee_q_scale, case.knee_dq_scale),
    }
    for joint_id, (q_scale, dq_scale) in joint_specs.items():
        jc = yaml_joint(cfg, joint_id)
        jc["enabled"] = True
        jc["observer_gain_q"] = observer_gain_q(q_scale)
        jc["observer_gain_dq"] = observer_gain_dq(dq_scale)
        jc["ku_q"] = float(U1_KU_Q)
        jc["ku_dq"] = float(U1_KU_DQ)
        jc["eid_mode"] = case.eid_mode
        jc["residual_eta_lambda"] = float(case.residual_eta_lambda)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(
        yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def run_mujoco(config: Path, out_dir: Path, args: argparse.Namespace) -> bool:
    log_path = out_dir / "mujoco_closed_loop_log.csv"
    if args.skip_existing and log_path.exists():
        return True
    cmd = [
        sys.executable,
        "scripts/run_mujoco.py",
        "--scene", str(args.scene),
        "--config", str(config),
        "--stepper", str(args.stepper),
        "--out-dir", str(out_dir),
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
    print(f"[MuJoCo] running {config.stem}", flush=True)
    try:
        subprocess.run(cmd, cwd=ROOT, check=True)
        return True
    except subprocess.CalledProcessError as exc:
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "run_error.txt").write_text(str(exc), encoding="utf-8")
        return False


def rms(x: np.ndarray) -> float:
    y = np.asarray(x, dtype=float)
    if y.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(np.square(y))))


def window_mask(t: np.ndarray, start: float, stop: float) -> np.ndarray:
    return (t >= start) & (t < stop)


def load_arrays(log_path: Path) -> dict[str, np.ndarray]:
    df = pd.read_csv(log_path)
    df = df[df["joint_id"].isin(JOINTS)].copy()
    for col in ["tau_applied", "tau_dist", "tau_total", "eta_u", "flags", "joint_flags"]:
        if col not in df.columns:
            df[col] = 0.0
    pivot = df.pivot_table(
        index="t",
        columns="joint_id",
        values=[
            "q_ref_shaped", "q_actual", "dq_actual", "tau_applied", "tau_dist",
            "tau_total", "eta_u", "flags", "joint_flags",
        ],
        aggfunc="first",
    ).dropna()
    t = pivot.index.to_numpy(dtype=float)
    out: dict[str, np.ndarray] = {"t": t}
    for joint_id, suffix in [(HIP, "h"), (KNEE, "k")]:
        for col in [
            "q_ref_shaped", "q_actual", "dq_actual", "tau_applied",
            "tau_dist", "tau_total", "eta_u",
        ]:
            out[f"{col}_{suffix}"] = pivot[(col, joint_id)].to_numpy(dtype=float)
        out[f"flags_{suffix}"] = pivot[("flags", joint_id)].to_numpy(dtype=float)
        out[f"joint_flags_{suffix}"] = pivot[("joint_flags", joint_id)].to_numpy(dtype=float)
    out["e_h"] = out["q_ref_shaped_h"] - out["q_actual_h"]
    out["e_k"] = out["q_ref_shaped_k"] - out["q_actual_k"]
    out["e_coord"] = out["e_h"] - out["e_k"]
    return out


def metric_row(
    case: MujocoMechanismCase,
    success: bool,
    run_dir: Path,
    args: argparse.Namespace,
) -> dict[str, Any]:
    row: dict[str, Any] = {
        "layer": case.layer,
        "case": case.case,
        "label": case.label,
        "purpose": case.purpose,
        "hip_observer_gain_q": observer_gain_q(case.hip_q_scale),
        "hip_observer_gain_dq": observer_gain_dq(case.hip_dq_scale),
        "knee_observer_gain_q": observer_gain_q(case.knee_q_scale),
        "knee_observer_gain_dq": observer_gain_dq(case.knee_dq_scale),
        "ku_q": U1_KU_Q,
        "ku_dq": U1_KU_DQ,
        "eid_mode": case.eid_mode,
        "residual_eta_lambda": case.residual_eta_lambda,
        "run_success": bool(success),
    }
    log_path = run_dir / "mujoco_closed_loop_log.csv"
    if not success or not log_path.exists():
        row["run_success"] = False
        return row

    arr = load_arrays(log_path)
    t = arr["t"]
    dt = float(np.median(np.diff(t))) if len(t) > 1 else float(args.dt)
    dw = window_mask(t, args.disturbance_start, args.disturbance_end)
    rw = window_mask(t, args.disturbance_end, min(args.duration, args.disturbance_end + 2.0))
    flags_h = arr["flags_h"].astype(np.int64)
    flags_k = arr["flags_k"].astype(np.int64)
    joint_flags_h = arr["joint_flags_h"].astype(np.int64)
    joint_flags_k = arr["joint_flags_k"].astype(np.int64)
    all_flags = np.concatenate([flags_h, flags_k, joint_flags_h, joint_flags_k])
    flag_rows = np.concatenate([
        flags_h | joint_flags_h,
        flags_k | joint_flags_k,
    ])

    def delta_rms(key: str, mask: np.ndarray) -> float:
        y = arr[key][mask]
        return rms(np.diff(y)) if len(y) > 1 else float("nan")

    def rate_rms(key: str, mask: np.ndarray) -> float:
        y = arr[key][mask]
        return rms(np.diff(y) / dt) if len(y) > 1 else float("nan")

    row.update({
        "samples": int(len(t)),
        "dt_log_s": dt,
        "disturbance_samples": int(np.count_nonzero(dw)),
        "recovery_samples": int(np.count_nonzero(rw)),
        "hip_rmse": rms(arr["e_h"][dw]),
        "knee_rmse": rms(arr["e_k"][dw]),
        "coord_rmse": rms(arr["e_coord"][dw]),
        "hip_recovery_rmse": rms(arr["e_h"][rw]),
        "knee_recovery_rmse": rms(arr["e_k"][rw]),
        "coord_recovery_rmse": rms(arr["e_coord"][rw]),
        "hip_peak_abs_error": float(np.max(np.abs(arr["e_h"][dw]))) if np.any(dw) else float("nan"),
        "knee_peak_abs_error": float(np.max(np.abs(arr["e_k"][dw]))) if np.any(dw) else float("nan"),
        "coord_peak_abs_error": float(np.max(np.abs(arr["e_coord"][dw]))) if np.any(dw) else float("nan"),
        "hip_tau_total_rms": rms(arr["tau_total_h"][dw]),
        "knee_tau_total_rms": rms(arr["tau_total_k"][dw]),
        "hip_delta_tau_total_rms": delta_rms("tau_total_h", dw),
        "knee_delta_tau_total_rms": delta_rms("tau_total_k", dw),
        "hip_tau_rate_total_rms": rate_rms("tau_total_h", dw),
        "knee_tau_rate_total_rms": rate_rms("tau_total_k", dw),
        "hip_eta_u_rms": rms(arr["eta_u_h"][dw]),
        "knee_eta_u_rms": rms(arr["eta_u_k"][dw]),
        "combined_tau_total_rms": rms(np.sqrt(arr["tau_total_h"][dw] ** 2 + arr["tau_total_k"][dw] ** 2)),
        "combined_delta_tau_total_rms": rms(np.sqrt(
            np.diff(arr["tau_total_h"][dw]) ** 2 + np.diff(arr["tau_total_k"][dw]) ** 2
        )) if np.count_nonzero(dw) > 1 else float("nan"),
        "combined_flags": int(np.bitwise_or.reduce(all_flags)) if len(all_flags) else 0,
        "saturation_joint_rows": int(np.count_nonzero((flag_rows & SATURATION_FLAG) != 0)),
        "saturation_joint_row_fraction": (
            float(np.count_nonzero((flag_rows & SATURATION_FLAG) != 0) / len(flag_rows))
            if len(flag_rows) else float("nan")
        ),
    })
    return row


def welch_psd(x: np.ndarray, fs: float) -> tuple[np.ndarray, np.ndarray]:
    y = np.asarray(x, dtype=float)
    y = y - np.nanmean(y)
    y = np.nan_to_num(y)
    if y.size < 4:
        return np.asarray([0.0]), np.asarray([0.0])
    nperseg = min(512, y.size)
    noverlap = nperseg // 2
    return signal.welch(
        y,
        fs=fs,
        window="hann",
        nperseg=nperseg,
        noverlap=noverlap,
        scaling="density",
    )


def band_power(freq: np.ndarray, psd: np.ndarray, lo: float, hi: float) -> float:
    mask = (freq >= lo) & (freq < hi)
    if np.count_nonzero(mask) < 2:
        return float("nan")
    return float(np.trapezoid(psd[mask], freq[mask]))


def frequency_rows(
    case: MujocoMechanismCase,
    success: bool,
    run_dir: Path,
    args: argparse.Namespace,
) -> list[dict[str, Any]]:
    log_path = run_dir / "mujoco_closed_loop_log.csv"
    if not success or not log_path.exists():
        return []
    arr = load_arrays(log_path)
    t = arr["t"]
    if len(t) < 4:
        return []
    fs = 1.0 / float(np.median(np.diff(t)))
    dw = window_mask(t, args.disturbance_start, args.disturbance_end)
    signal_specs = [
        ("coord_error", "e_coord", False),
        ("hip_delta_tau_total", "tau_total_h", True),
        ("knee_delta_tau_total", "tau_total_k", True),
        ("hip_eta_u", "eta_u_h", False),
        ("knee_eta_u", "eta_u_k", False),
    ]
    rows: list[dict[str, Any]] = []
    for signal_name, key, use_delta in signal_specs:
        x = arr[key][dw]
        if use_delta:
            x = np.diff(x, prepend=x[0]) if len(x) else x
        freq, psd = welch_psd(x, fs)
        usable = (freq >= 0.5) & (freq <= 100.0)
        if np.any(usable):
            usable_indices = np.where(usable)[0]
            idx = usable_indices[int(np.argmax(psd[usable]))]
        else:
            idx = int(np.argmax(psd))
        rows.append({
            "layer": case.layer,
            "case": case.case,
            "label": case.label,
            "signal": signal_name,
            "fs_hz": fs,
            "peak_freq_hz_0p5_100": float(freq[idx]),
            "peak_psd": float(psd[idx]),
            "bandpower_0p5_3hz": band_power(freq, psd, 0.5, 3.0),
            "bandpower_3_15hz": band_power(freq, psd, 3.0, 15.0),
            "bandpower_15_80hz": band_power(freq, psd, 15.0, 80.0),
        })
    return rows


def save_all(fig: plt.Figure, stem: Path) -> None:
    stem.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(stem.with_suffix(".png"))
    fig.savefig(stem.with_suffix(".pdf"))
    plt.close(fig)


def plot_candidate(metrics: pd.DataFrame, fig_dir: Path) -> None:
    sub = metrics[(metrics["layer"] == "candidate") & (metrics["run_success"] == True)].copy()  # noqa: E712
    if sub.empty:
        return
    fig, axes = plt.subplots(1, 3, figsize=(8.6, 3.0))
    specs = [
        ("coord_rmse", "combined_delta_tau_total_rms", "Coord. RMSE [rad]", r"Combined $\Delta\tau$ RMS"),
        ("hip_rmse", "hip_delta_tau_total_rms", "Hip RMSE [rad]", r"Hip $\Delta\tau$ RMS"),
        ("knee_rmse", "knee_delta_tau_total_rms", "Knee RMSE [rad]", r"Knee $\Delta\tau$ RMS"),
    ]
    label_offsets = {
        "A_O4U1": (7, 6),
        "B_HipUp": (7, -11),
        "D_HipUpKneeDown": (9, 10),
        "O5U1": (7, 6),
        "KneeVelDown": (8, -15),
    }
    for ax, (xcol, ycol, xlabel, ylabel) in zip(axes, specs):
        for _, row in sub.iterrows():
            case_key = str(row["case"])
            ax.scatter(
                float(row[xcol]),
                float(row[ycol]),
                s=46,
                color=CASE_COLORS.get(case_key, "#555555"),
                edgecolor="black",
                linewidth=0.35,
                zorder=3,
            )
            ax.annotate(
                CASE_SHORT.get(case_key, case_key),
                (float(row[xcol]), float(row[ycol])),
                xytext=label_offsets.get(case_key, (6, 5)),
                textcoords="offset points",
                fontsize=8,
                arrowprops={"arrowstyle": "-", "color": "#9CA3AF", "lw": 0.45, "shrinkA": 0, "shrinkB": 4},
            )
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        pad_axis(ax, 0.12, 0.18)
    legend_handles = [
        Line2D([0], [0], marker="o", color="none", markerfacecolor=CASE_COLORS[key], markeredgecolor="black", markersize=5.5, label=CASE_LEGEND[key])
        for key in ["A_O4U1", "B_HipUp", "D_HipUpKneeDown", "O5U1", "KneeVelDown"]
    ]
    fig.legend(handles=legend_handles, loc="upper center", bbox_to_anchor=(0.5, 1.02), ncol=5, frameon=False)
    for idx, ax in enumerate(axes):
        panel_label(ax, f"({chr(ord('a') + idx)})")
    fig.subplots_adjust(left=0.08, right=0.99, bottom=0.21, top=0.78, wspace=0.42)
    save_all(fig, fig_dir / "mujoco_mechanism_candidate_pareto")


def plot_structure(metrics: pd.DataFrame, fig_dir: Path) -> None:
    sub = metrics[(metrics["layer"] == "structure") & (metrics["run_success"] == True)].copy()  # noqa: E712
    if sub.empty:
        return
    sub["base"] = np.where(sub["case"].str.startswith("O5U1"), "O5+U1", "O4+U1")
    order = [
        "pd_inverse_only",
        "center_feedback_only",
        "input_compensation_only",
        "full_eid",
    ]
    tick_labels = ["PD", "center", "input", "full"]
    colors = {"O4+U1": "#0072B2", "O5+U1": "#D55E00"}
    fig, axes = plt.subplots(1, 3, figsize=(8.4, 3.0))
    specs = [
        ("coord_rmse", "Coord. RMSE [rad]"),
        ("knee_delta_tau_total_rms", r"Knee $\Delta\tau$ RMS"),
        ("knee_eta_u_rms", r"Knee $\eta_u$ RMS"),
    ]
    x = np.arange(len(order))
    width = 0.36
    for ax, (metric, ylabel) in zip(axes, specs):
        for offset, base in [(-width / 2, "O4+U1"), (width / 2, "O5+U1")]:
            values = []
            for mode in order:
                rows = sub[(sub["base"] == base) & (sub["eid_mode"] == mode)]
                values.append(float(rows.iloc[0][metric]) if not rows.empty else np.nan)
            ax.bar(x + offset, values, width=width, label=base, color=colors[base], alpha=0.84)
        ax.set_xticks(x)
        ax.set_xticklabels(tick_labels, rotation=0)
        ax.set_ylabel(ylabel)
        ax.margins(y=0.12)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2, frameon=False, bbox_to_anchor=(0.52, 1.02))
    for idx, ax in enumerate(axes):
        panel_label(ax, f"({chr(ord('a') + idx)})")
    fig.subplots_adjust(left=0.08, right=0.99, bottom=0.16, top=0.80, wspace=0.35)
    save_all(fig, fig_dir / "mujoco_mechanism_structure_ablation")


def plot_residual(metrics: pd.DataFrame, fig_dir: Path) -> None:
    sub = metrics[(metrics["layer"] == "residual") & (metrics["run_success"] == True)].copy()  # noqa: E712
    if sub.empty:
        return
    sub["base"] = np.where(sub["case"].str.startswith("O5U1"), "O5+U1", "O4+U1")
    colors = {"O4+U1": "#0072B2", "O5+U1": "#D55E00"}
    fig, axes = plt.subplots(1, 3, figsize=(8.4, 2.95), sharex=True)
    specs = [
        ("coord_rmse", "Coord. RMSE [rad]"),
        ("knee_delta_tau_total_rms", r"Knee $\Delta\tau$ RMS"),
        ("knee_eta_u_rms", r"Knee $\eta_u$ RMS"),
    ]
    for ax, (metric, ylabel) in zip(axes, specs):
        for base in ["O4+U1", "O5+U1"]:
            rows = sub[sub["base"] == base].sort_values("residual_eta_lambda")
            ax.plot(
                rows["residual_eta_lambda"].to_numpy(dtype=float),
                rows[metric].to_numpy(dtype=float),
                marker="o",
                color=colors[base],
                label=base,
                lw=1.5,
            )
        ax.set_xlabel(r"residual $\lambda$")
        ax.set_ylabel(ylabel)
        ax.set_xticks([0.0, 0.5, 1.0])
        ax.margins(y=0.14)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.52, 1.02), ncol=2, frameon=False)
    for idx, ax in enumerate(axes):
        panel_label(ax, f"({chr(ord('a') + idx)})")
    fig.subplots_adjust(left=0.08, right=0.99, bottom=0.18, top=0.80, wspace=0.35)
    save_all(fig, fig_dir / "mujoco_mechanism_residual_lambda")


def plot_spectrum_summary(freq: pd.DataFrame, fig_dir: Path) -> None:
    if freq.empty:
        return
    cand = freq[(freq["layer"] == "candidate") & (freq["signal"].isin([
        "coord_error",
        "knee_delta_tau_total",
        "knee_eta_u",
    ]))].copy()
    if cand.empty:
        return
    order = ["A_O4U1", "B_HipUp", "D_HipUpKneeDown", "O5U1", "KneeVelDown"]
    specs = [
        ("coord_error", "bandpower_0p5_3hz", "Coord. error\n0.5-3 Hz"),
        ("knee_delta_tau_total", "bandpower_15_80hz", "Knee delta tau\n15-80 Hz"),
        ("knee_eta_u", "bandpower_15_80hz", "Knee eta_u\n15-80 Hz"),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(8.6, 3.1))
    for ax, (signal_name, metric, ylabel) in zip(axes, specs):
        rows = cand[cand["signal"] == signal_name].set_index("case")
        values = [float(rows.loc[c][metric]) if c in rows.index else np.nan for c in order]
        ax.bar(np.arange(len(order)), values, color=[CASE_COLORS.get(c, "#4C78A8") for c in order], alpha=0.88)
        ax.set_xticks(np.arange(len(order)))
        ax.set_xticklabels([CASE_SHORT.get(c, c) for c in order], rotation=0)
        ax.set_ylabel(ylabel)
        ax.margins(y=0.16)
    legend_handles = [
        Line2D([0], [0], marker="s", color="none", markerfacecolor=CASE_COLORS[key], markeredgecolor="none", markersize=6, label=CASE_LEGEND[key])
        for key in order
    ]
    fig.legend(handles=legend_handles, loc="upper center", bbox_to_anchor=(0.5, 1.02), ncol=5, frameon=False)
    for idx, ax in enumerate(axes):
        panel_label(ax, f"({chr(ord('a') + idx)})")
    fig.subplots_adjust(left=0.08, right=0.99, bottom=0.16, top=0.78, wspace=0.36)
    save_all(fig, fig_dir / "mujoco_mechanism_spectrum_summary")


def add_relative_columns(metrics: pd.DataFrame) -> pd.DataFrame:
    out = metrics.copy()
    base = out[(out["layer"] == "candidate") & (out["case"] == "A_O4U1")]
    if base.empty:
        return out
    base_row = base.iloc[0]
    for col in [
        "hip_rmse",
        "knee_rmse",
        "coord_rmse",
        "hip_delta_tau_total_rms",
        "knee_delta_tau_total_rms",
        "combined_delta_tau_total_rms",
        "hip_eta_u_rms",
        "knee_eta_u_rms",
    ]:
        if col in out.columns and pd.notna(base_row.get(col)) and float(base_row[col]) != 0.0:
            out[f"{col}_vs_A_pct"] = 100.0 * (out[col].astype(float) / float(base_row[col]) - 1.0)
    return out


def markdown_table(df: pd.DataFrame, columns: list[str], floatfmt: str = ".6g") -> str:
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join(["---"] * len(columns)) + " |",
    ]
    for _, row in df[columns].iterrows():
        cells = []
        for col in columns:
            val = row[col]
            if isinstance(val, (float, np.floating)):
                cells.append(format(float(val), floatfmt))
            else:
                cells.append(str(val))
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def write_summary(out_dir: Path, metrics: pd.DataFrame, freq: pd.DataFrame, args: argparse.Namespace) -> Path:
    report_dir = analysis_report_dir(out_dir)
    report_dir.mkdir(parents=True, exist_ok=True)
    summary_path = report_dir / "mujoco_mechanism_completion_summary.md"
    metrics_rel = add_relative_columns(metrics)
    cand_cols = [
        "case", "hip_rmse", "knee_rmse", "coord_rmse",
        "knee_delta_tau_total_rms", "knee_eta_u_rms",
        "combined_flags", "saturation_joint_rows",
    ]
    structure_cols = [
        "case", "eid_mode", "coord_rmse", "knee_delta_tau_total_rms",
        "knee_eta_u_rms", "coord_recovery_rmse", "combined_flags",
    ]
    residual_cols = [
        "case", "residual_eta_lambda", "coord_rmse",
        "knee_delta_tau_total_rms", "knee_eta_u_rms", "combined_flags",
    ]
    freq_cols = [
        "case", "signal", "peak_freq_hz_0p5_100",
        "bandpower_0p5_3hz", "bandpower_3_15hz", "bandpower_15_80hz",
    ]

    def safe_table(df: pd.DataFrame, columns: list[str], floatfmt: str = ".6g") -> str:
        missing = [col for col in columns if col not in df.columns]
        if df.empty or missing:
            return "_No successful rows available for this table._"
        return markdown_table(df, columns, floatfmt)

    candidate_df = metrics_rel[metrics_rel["layer"] == "candidate"] if "layer" in metrics_rel else pd.DataFrame()
    structure_df = metrics[metrics["layer"] == "structure"] if "layer" in metrics else pd.DataFrame()
    residual_df = metrics[metrics["layer"] == "residual"] if "layer" in metrics else pd.DataFrame()
    if "layer" in freq and "signal" in freq:
        freq_df = freq[
            (freq["layer"] == "candidate")
            & (freq["signal"].isin(["coord_error", "knee_delta_tau_total", "knee_eta_u"]))
        ]
    else:
        freq_df = pd.DataFrame()

    lines = [
        "# MuJoCo mechanism completion",
        "",
        "Scope: all rows in this folder were generated with `scripts/run_mujoco.py` and "
        "the C++ `h1_controller_stepper` executable.  The results are simulation-based "
        "mechanism screening evidence, not hardware validation.",
        "",
        f"Protocol: duration `{args.duration:g}` s, dt `{args.dt:g}` s, disturbance joints "
        f"`{args.disturbance_joints}`, disturbance torques `{args.disturbance_torques}` N m, "
        f"half-cosine timing `{args.disturbance_start:g}/"
        f"{args.disturbance_plateau_start:g}/{args.disturbance_plateau_end:g}/"
        f"{args.disturbance_end:g}` s.",
        "",
        "## Candidate layer",
        "",
        safe_table(candidate_df, cand_cols, ".6f"),
        "",
        "Relative changes against `A_O4U1` are available in "
        f"`{repo_relpath(out_dir / 'mujoco_mechanism_metrics_with_relative.csv')}`.",
        "",
        "## Structural ablation layer",
        "",
        safe_table(structure_df, structure_cols, ".6f"),
        "",
        "## Residual lambda layer",
        "",
        safe_table(residual_df, residual_cols, ".6f"),
        "",
        "## Frequency summary",
        "",
        safe_table(freq_df, freq_cols, ".6g"),
        "",
        "Generated artifacts:",
        "",
        f"- `{repo_relpath(out_dir / 'mujoco_mechanism_metrics.csv')}`",
        f"- `{repo_relpath(out_dir / 'mujoco_mechanism_metrics_with_relative.csv')}`",
        f"- `{repo_relpath(out_dir / 'mujoco_mechanism_frequency_summary.csv')}`",
        f"- `{repo_relpath(out_dir / 'figures' / 'mujoco_mechanism_candidate_pareto.png')}` and `{repo_relpath(out_dir / 'figures' / 'mujoco_mechanism_candidate_pareto.pdf')}`",
        f"- `{repo_relpath(out_dir / 'figures' / 'mujoco_mechanism_structure_ablation.png')}` and `{repo_relpath(out_dir / 'figures' / 'mujoco_mechanism_structure_ablation.pdf')}`",
        f"- `{repo_relpath(out_dir / 'figures' / 'mujoco_mechanism_residual_lambda.png')}` and `{repo_relpath(out_dir / 'figures' / 'mujoco_mechanism_residual_lambda.pdf')}`",
        f"- `{repo_relpath(out_dir / 'figures' / 'mujoco_mechanism_spectrum_summary.png')}` and `{repo_relpath(out_dir / 'figures' / 'mujoco_mechanism_spectrum_summary.pdf')}`",
        "",
    ]
    summary_path.write_text(
        "\n".join(lines),
        encoding="utf-8",
    )
    return summary_path


def parse_layers(value: str) -> set[str]:
    layers = {item.strip() for item in value.split(",") if item.strip()}
    valid = {"candidate", "structure", "residual"}
    unknown = layers - valid
    if unknown:
        raise argparse.ArgumentTypeError(f"unknown layer(s): {', '.join(sorted(unknown))}")
    return layers or valid


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-config", type=Path, default=Path("config/h1_real_p4_ku_u1_hip_knee_eid.yaml"))
    parser.add_argument("--scene", type=Path, default=Path("h1_official_mujoco/scene.xml"))
    parser.add_argument("--stepper", type=Path, default=Path("build/Debug/h1_controller_stepper.exe"))
    parser.add_argument("--out-dir", type=Path, default=Path("analysis_artifacts/bychen_mujoco_mechanism_completion"))
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--disturbance-joints", default="1,2")
    parser.add_argument("--disturbance-torques", default="6,-4")
    parser.add_argument("--disturbance-start", type=float, default=4.0)
    parser.add_argument("--disturbance-plateau-start", type=float, default=4.2)
    parser.add_argument("--disturbance-plateau-end", type=float, default=5.2)
    parser.add_argument("--disturbance-end", type=float, default=5.4)
    parser.add_argument("--layers", type=parse_layers, default=parse_layers("candidate,structure,residual"))
    parser.add_argument("--skip-runs", action="store_true")
    parser.add_argument("--skip-existing", action="store_true")
    args = parser.parse_args()

    configure_matplotlib()
    out_dir = args.out_dir if args.out_dir.is_absolute() else ROOT / args.out_dir
    config_dir = out_dir / "configs"
    run_dir = out_dir / "runs"
    fig_dir = out_dir / "figures"
    out_dir.mkdir(parents=True, exist_ok=True)

    cases = [case for case in build_cases() if case.layer in args.layers]
    metric_rows: list[dict[str, Any]] = []
    freq_rows_all: list[dict[str, Any]] = []

    for case in cases:
        config_path = config_dir / f"{case.case}.yaml"
        case_run_dir = run_dir / case.case
        build_config(args.base_config, config_path, case)
        success = True
        if not args.skip_runs:
            success = run_mujoco(config_path, case_run_dir, args)
        metric_rows.append(metric_row(case, success, case_run_dir, args))
        freq_rows_all.extend(frequency_rows(case, success, case_run_dir, args))

    metrics = pd.DataFrame(metric_rows)
    freq = pd.DataFrame(freq_rows_all)
    metrics_path = out_dir / "mujoco_mechanism_metrics.csv"
    metrics_rel_path = out_dir / "mujoco_mechanism_metrics_with_relative.csv"
    freq_path = out_dir / "mujoco_mechanism_frequency_summary.csv"
    metrics.to_csv(metrics_path, index=False)
    add_relative_columns(metrics).to_csv(metrics_rel_path, index=False)
    freq.to_csv(freq_path, index=False)

    plot_candidate(metrics, fig_dir)
    plot_structure(metrics, fig_dir)
    plot_residual(metrics, fig_dir)
    plot_spectrum_summary(freq, fig_dir)
    summary_path = write_summary(out_dir, metrics, freq, args)

    print(f"metrics={metrics_path}")
    print(f"metrics_with_relative={metrics_rel_path}")
    print(f"frequency={freq_path}")
    print(f"summary={summary_path}")
    print(f"figures={fig_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
