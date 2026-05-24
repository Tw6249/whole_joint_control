#!/usr/bin/env python3
"""Generate diagnostics for the EID right-leg coupling test."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ANALYSIS_ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = ANALYSIS_ROOT / "data/eid_right_leg_tests"
ANALYSIS_DIR = DATA_ROOT / "analysis"
ANALYSIS_START_S = 1.0
RIGHT_HIP_PITCH = 1
RIGHT_KNEE = 2
JOINT_LABELS = {
    RIGHT_HIP_PITCH: "RightHipPitch",
    RIGHT_KNEE: "RightKnee",
}
TAU_LIMITS = {
    RIGHT_HIP_PITCH: 200.0,
    RIGHT_KNEE: 300.0,
}
PHYSICAL_Q_LIMITS = {
    RIGHT_HIP_PITCH: (-3.14, 2.53),
    RIGHT_KNEE: (-0.26, 2.05),
}
FIELDS = (
    "t",
    "q_ref_shaped",
    "dq_ref_shaped",
    "q_actual",
    "dq_actual",
    "u_star",
    "u_t",
    "u_raw",
    "motor_tau",
    "eta_q",
    "eta_dq",
    "x_bar_q",
    "x_bar_dq",
    "r_d_q",
    "r_d_dq",
    "e_q",
    "e_dq",
    "observer_qacc",
    "q_error_shaped",
    "dq_error_shaped",
)


@dataclass(frozen=True)
class CaseSpec:
    key: str
    label: str
    csv_path: Path
    joints: tuple[int, ...]


def read_case(path: Path) -> dict[int, dict[str, np.ndarray]]:
    out: dict[int, dict[str, list[float]]] = {}
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            joint_id = int(row["joint_id"])
            joint_data = out.setdefault(joint_id, {field: [] for field in FIELDS})
            for field in FIELDS:
                joint_data[field].append(float(row[field]))
    return {
        joint_id: {field: np.asarray(values, dtype=float) for field, values in joint_data.items()}
        for joint_id, joint_data in out.items()
    }


def rmse(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(x * x))) if x.size else 0.0


def abs_mean(x: np.ndarray) -> float:
    return float(np.mean(np.abs(x))) if x.size else 0.0


def finite_diff(q: np.ndarray, t: np.ndarray) -> np.ndarray:
    if q.size < 2:
        return np.asarray([], dtype=float)
    dt = np.diff(t)
    dq = np.diff(q) / dt
    return dq[np.isfinite(dq)]


def saturation_fraction(tau: np.ndarray, limit: float, threshold: float) -> float:
    return float(np.mean(np.abs(tau) >= threshold * limit)) if tau.size else 0.0


def count_sign_flips(x: np.ndarray, threshold: float) -> int:
    if x.size < 2:
        return 0
    y = np.where(np.abs(x) >= threshold, np.sign(x), 0.0)
    nz = y[y != 0.0]
    if nz.size < 2:
        return 0
    return int(np.sum(nz[1:] * nz[:-1] < 0.0))


def collect_metrics(cases: tuple[CaseSpec, ...], data_by_case: dict[str, dict[int, dict[str, np.ndarray]]]) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for case in cases:
        data = data_by_case[case.key]
        for joint_id in case.joints:
            raw = data[joint_id]
            mask = raw["t"] >= ANALYSIS_START_S
            s = {field: values[mask] for field, values in raw.items()}
            limit = TAU_LIMITS[joint_id]
            fd = finite_diff(s["q_actual"], s["t"])
            rows.append({
                "case": case.key,
                "case_label": case.label,
                "joint_id": str(joint_id),
                "joint": JOINT_LABELS[joint_id],
                "analysis_window_s": f"{ANALYSIS_START_S:.1f}-15.0",
                "samples": str(s["t"].size),
                "q_ref_min": f"{np.min(s['q_ref_shaped']):.6g}",
                "q_ref_max": f"{np.max(s['q_ref_shaped']):.6g}",
                "dq_ref_abs_max": f"{np.max(np.abs(s['dq_ref_shaped'])):.6g}",
                "dq_actual_abs_max": f"{np.max(np.abs(s['dq_actual'])):.6g}",
                "dq_actual_std": f"{np.std(s['dq_actual']):.6g}",
                "fd_dq_std_50hz": f"{np.std(fd):.6g}" if fd.size else "nan",
                "q_rmse": f"{rmse(s['q_ref_shaped'] - s['q_actual']):.6g}",
                "dq_rmse": f"{rmse(s['dq_ref_shaped'] - s['dq_actual']):.6g}",
                "u_t_abs_max": f"{np.max(np.abs(s['u_t'])):.6g}",
                "u_t_abs_mean": f"{abs_mean(s['u_t']):.6g}",
                "u_star_abs_mean": f"{abs_mean(s['u_star']):.6g}",
                "u_t_sat_90pct_frac": f"{saturation_fraction(s['u_t'], limit, 0.90):.6g}",
                "u_t_sat_99pct_frac": f"{saturation_fraction(s['u_t'], limit, 0.99):.6g}",
                "u_t_gt_50pct_frac": f"{saturation_fraction(s['u_t'], limit, 0.50):.6g}",
                "tau_sign_flips_over_50pct": str(count_sign_flips(s["u_t"], 0.50 * limit)),
                "eta_dq_abs_mean": f"{abs_mean(s['eta_dq']):.6g}",
                "x_bar_dq_abs_mean": f"{abs_mean(s['x_bar_dq']):.6g}",
                "r_d_q_abs_max": f"{np.max(np.abs(s['r_d_q'])):.6g}",
                "e_q_abs_mean": f"{abs_mean(s['e_q']):.6g}",
                "e_dq_abs_mean": f"{abs_mean(s['e_dq']):.6g}",
                "observer_qacc_abs_mean": f"{abs_mean(s['observer_qacc']):.6g}",
            })
    return rows


def write_metrics_csv(rows: list[dict[str, str]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def setup_style() -> None:
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "DejaVu Sans"],
        "font.size": 10,
        "axes.labelsize": 11,
        "axes.titlesize": 12,
        "legend.fontsize": 8.5,
        "figure.dpi": 150,
    })


def savefig(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(path, dpi=220, bbox_inches="tight", facecolor="white")
    plt.close()


def plot_single_knee_stability(data_by_case: dict[str, dict[int, dict[str, np.ndarray]]], path: Path) -> None:
    s = data_by_case["single_knee"][RIGHT_KNEE]
    steady = s["t"] >= ANALYSIS_START_S
    q_error = s["q_ref_shaped"] - s["q_actual"]
    dq_error = s["dq_ref_shaped"] - s["dq_actual"]
    q_rmse = rmse(q_error[steady])
    dq_std = float(np.std(s["dq_actual"][steady]))
    tau_mean = abs_mean(s["u_t"][steady])
    tau_sat = saturation_fraction(s["u_t"][steady], TAU_LIMITS[RIGHT_KNEE], 0.90)
    eta_dq_mean = abs_mean(s["eta_dq"][steady])

    fig, axes = plt.subplots(4, 1, figsize=(12, 10.5), sharex=True)
    fig.suptitle("Single-joint EID stability evidence — RightKnee only", fontweight="bold")

    axes[0].plot(s["t"], s["q_ref_shaped"], "k--", linewidth=1.1, label="q reference")
    axes[0].plot(s["t"], s["q_actual"], color="#2563eb", linewidth=1.2, label="q actual")
    axes[0].set_ylabel("q [rad]")
    axes[0].legend(loc="upper right", ncol=2)
    axes[0].text(
        0.02,
        0.92,
        f"steady q RMSE={q_rmse:.5f} rad",
        transform=axes[0].transAxes,
        va="top",
        bbox=dict(boxstyle="round,pad=0.25", facecolor="white", alpha=0.8),
    )

    axes[1].plot(s["t"], s["dq_ref_shaped"], "k--", linewidth=1.0, label="dq reference")
    axes[1].plot(s["t"], s["dq_actual"], color="#16a34a", linewidth=1.1, label="dq actual")
    axes[1].set_ylabel("dq [rad/s]")
    axes[1].legend(loc="upper right", ncol=2)
    axes[1].text(
        0.02,
        0.92,
        f"steady std(dq)={dq_std:.5f} rad/s",
        transform=axes[1].transAxes,
        va="top",
        bbox=dict(boxstyle="round,pad=0.25", facecolor="white", alpha=0.8),
    )

    axes[2].plot(s["t"], s["u_star"], "k--", linewidth=0.9, alpha=0.75, label="u* inverse reference")
    axes[2].plot(s["t"], s["u_t"], color="#ea580c", linewidth=1.1, label="EID torque")
    axes[2].axhline(TAU_LIMITS[RIGHT_KNEE], color="0.5", linestyle=":", linewidth=0.8)
    axes[2].axhline(-TAU_LIMITS[RIGHT_KNEE], color="0.5", linestyle=":", linewidth=0.8)
    axes[2].set_ylabel("tau [N·m]")
    axes[2].legend(loc="upper right", ncol=2)
    axes[2].text(
        0.02,
        0.92,
        f"steady mean |tau|={tau_mean:.5f} N·m, >90% limit={100.0 * tau_sat:.2f}%",
        transform=axes[2].transAxes,
        va="top",
        bbox=dict(boxstyle="round,pad=0.25", facecolor="white", alpha=0.8),
    )

    axes[3].plot(s["t"], s["eta_dq"], color="#0891b2", linewidth=1.0, label="eta_dq")
    axes[3].plot(s["t"], s["r_d_q"], color="#7c3aed", linewidth=0.9, alpha=0.75, label="r_d_q")
    axes[3].set_ylabel("observer / virtual target")
    axes[3].set_xlabel("Time [s]")
    axes[3].legend(loc="upper right", ncol=2)
    axes[3].text(
        0.02,
        0.92,
        f"steady mean |eta_dq|={eta_dq_mean:.5f}",
        transform=axes[3].transAxes,
        va="top",
        bbox=dict(boxstyle="round,pad=0.25", facecolor="white", alpha=0.8),
    )

    for ax in axes:
        ax.axvspan(0.0, ANALYSIS_START_S, color="0.85", alpha=0.35, label=None)
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0.0, 15.0)
    savefig(path)


def plot_knee_single_vs_dual(data_by_case: dict[str, dict[int, dict[str, np.ndarray]]], path: Path) -> None:
    single = data_by_case["single_knee"][RIGHT_KNEE]
    dual = data_by_case["dual_hip_knee"][RIGHT_KNEE]
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("Same RightKnee reference: single-joint vs hip+knee control", fontweight="bold")

    axes[0].plot(single["t"], single["q_ref_shaped"], "k--", linewidth=1.1, label="q ref")
    axes[0].plot(single["t"], single["q_actual"], color="#2563eb", linewidth=1.2, label="single q")
    axes[0].plot(dual["t"], dual["q_actual"], color="#dc2626", linewidth=1.0, label="dual q")
    axes[0].set_ylabel("q [rad]")
    axes[0].legend(loc="upper right", ncol=3)

    axes[1].plot(single["t"], single["dq_ref_shaped"], "k--", linewidth=1.0, label="dq ref")
    axes[1].plot(single["t"], single["dq_actual"], color="#2563eb", linewidth=1.0, label="single dq")
    axes[1].plot(dual["t"], dual["dq_actual"], color="#dc2626", linewidth=0.9, label="dual dq")
    axes[1].set_ylabel("dq [rad/s]")
    axes[1].legend(loc="upper right", ncol=3)

    axes[2].plot(single["t"], single["u_t"], color="#2563eb", linewidth=1.0, label="single EID torque")
    axes[2].plot(dual["t"], dual["u_t"], color="#dc2626", linewidth=0.9, label="dual EID torque")
    axes[2].axhline(TAU_LIMITS[RIGHT_KNEE], color="0.4", linestyle="--", linewidth=0.8)
    axes[2].axhline(-TAU_LIMITS[RIGHT_KNEE], color="0.4", linestyle="--", linewidth=0.8)
    axes[2].set_ylabel("tau [N·m]")
    axes[2].legend(loc="upper right", ncol=2)

    axes[3].plot(single["t"], single["eta_dq"], color="#2563eb", linewidth=1.0, label="single eta_dq")
    axes[3].plot(dual["t"], dual["eta_dq"], color="#dc2626", linewidth=0.9, label="dual eta_dq")
    axes[3].set_ylabel("eta_dq")
    axes[3].set_xlabel("Time [s]")
    axes[3].legend(loc="upper right", ncol=2)

    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0.0, 15.0)
    savefig(path)


def plot_dual_diagnostics(data_by_case: dict[str, dict[int, dict[str, np.ndarray]]], path: Path) -> None:
    data = data_by_case["dual_hip_knee"]
    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
    fig.suptitle("Dual-joint EID diagnostics: saturation and observer amplification", fontweight="bold")
    colors = {RIGHT_HIP_PITCH: "#2563eb", RIGHT_KNEE: "#dc2626"}

    for joint_id in (RIGHT_HIP_PITCH, RIGHT_KNEE):
        s = data[joint_id]
        label = JOINT_LABELS[joint_id]
        color = colors[joint_id]
        axes[0].plot(s["t"], s["u_star"], linestyle="--", color=color, linewidth=0.85, alpha=0.75, label=f"{label} u*")
        axes[0].plot(s["t"], s["u_t"], color=color, linewidth=1.0, label=f"{label} u_t")
        axes[1].plot(s["t"], s["r_d_q"], color=color, linewidth=0.9, label=f"{label} r_d_q")
        axes[1].plot(s["t"], s["q_ref_shaped"], linestyle="--", color=color, linewidth=0.85, alpha=0.7, label=f"{label} q_ref")
        axes[2].plot(s["t"], s["x_bar_dq"], color=color, linewidth=0.9, label=f"{label} x_bar_dq")
        axes[2].plot(s["t"], s["eta_dq"], linestyle="--", color=color, linewidth=0.8, alpha=0.75, label=f"{label} eta_dq")
        axes[3].plot(s["t"], s["e_q"], color=color, linewidth=0.9, label=f"{label} e_q")
        axes[3].plot(s["t"], s["e_dq"], linestyle="--", color=color, linewidth=0.8, alpha=0.75, label=f"{label} e_dq")

    axes[0].axhline(TAU_LIMITS[RIGHT_HIP_PITCH], color="#2563eb", linestyle=":", linewidth=0.8)
    axes[0].axhline(-TAU_LIMITS[RIGHT_HIP_PITCH], color="#2563eb", linestyle=":", linewidth=0.8)
    axes[0].axhline(TAU_LIMITS[RIGHT_KNEE], color="#dc2626", linestyle=":", linewidth=0.8)
    axes[0].axhline(-TAU_LIMITS[RIGHT_KNEE], color="#dc2626", linestyle=":", linewidth=0.8)
    axes[0].set_ylabel("torque [N·m]")
    axes[1].set_ylabel("virtual q target [rad]")
    axes[2].set_ylabel("observer velocity")
    axes[3].set_ylabel("internal error")
    axes[3].set_xlabel("Time [s]")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", ncol=2)
        ax.set_xlim(0.0, 15.0)
    savefig(path)


def plot_metric_bars(rows: list[dict[str, str]], path: Path) -> None:
    labels = []
    dq_std = []
    tau_mean_pct = []
    sat_90 = []
    eta_mean = []
    for row in rows:
        labels.append(f"{row['case']}\n{row['joint']}")
        joint_id = int(row["joint_id"])
        limit = TAU_LIMITS[joint_id]
        dq_std.append(float(row["dq_actual_std"]))
        tau_mean_pct.append(100.0 * float(row["u_t_abs_mean"]) / limit)
        sat_90.append(100.0 * float(row["u_t_sat_90pct_frac"]))
        eta_mean.append(float(row["eta_dq_abs_mean"]))

    x = np.arange(len(labels))
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    fig.suptitle(f"Quantitative evidence after {ANALYSIS_START_S:.0f}s startup transient", fontweight="bold")
    specs = (
        (dq_std, "std(dq_actual) [rad/s]", "#4f46e5"),
        (tau_mean_pct, "mean |u_t| / tau_limit [%]", "#ea580c"),
        (sat_90, "|u_t| > 90% limit [% samples]", "#dc2626"),
        (eta_mean, "mean |eta_dq|", "#0891b2"),
    )
    for ax, (values, ylabel, color) in zip(axes.ravel(), specs):
        ax.bar(x, values, color=color, alpha=0.82)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=20, ha="right")
        ax.set_ylabel(ylabel)
        ax.grid(True, axis="y", alpha=0.3)
        ymax = max(values) if values else 1.0
        ax.set_ylim(0.0, ymax * 1.18 if ymax > 0.0 else 1.0)
        for xi, value in zip(x, values):
            ax.text(xi, value, f"{value:.2g}", ha="center", va="bottom", fontsize=8)
    savefig(path)


def plot_zoom_chattering(data_by_case: dict[str, dict[int, dict[str, np.ndarray]]], path: Path) -> None:
    data = data_by_case["dual_hip_knee"]
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle("Zoomed dual-joint chattering window", fontweight="bold")
    colors = {RIGHT_HIP_PITCH: "#2563eb", RIGHT_KNEE: "#dc2626"}
    window = (8.0, 10.0)
    for joint_id in (RIGHT_HIP_PITCH, RIGHT_KNEE):
        s = data[joint_id]
        mask = (s["t"] >= window[0]) & (s["t"] <= window[1])
        label = JOINT_LABELS[joint_id]
        color = colors[joint_id]
        axes[0].plot(s["t"][mask], s["q_ref_shaped"][mask], linestyle="--", color=color, linewidth=0.9, alpha=0.75, label=f"{label} q_ref")
        axes[0].plot(s["t"][mask], s["q_actual"][mask], color=color, linewidth=1.0, label=f"{label} q")
        axes[1].plot(s["t"][mask], s["dq_actual"][mask], color=color, linewidth=0.9, label=f"{label} dq")
        axes[2].plot(s["t"][mask], s["u_t"][mask], color=color, linewidth=0.9, label=f"{label} tau")
    axes[0].set_ylabel("q [rad]")
    axes[1].set_ylabel("dq [rad/s]")
    axes[2].set_ylabel("tau [N·m]")
    axes[2].set_xlabel("Time [s]")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", ncol=2)
        ax.set_xlim(*window)
    savefig(path)


def plot_theory_chain(path: Path) -> None:
    fig, ax = plt.subplots(figsize=(13, 4.2))
    ax.axis("off")
    boxes = [
        ("Two joints move\nhip-knee coupling", 0.02),
        ("Single-joint plant\nignores off-diagonal dynamics", 0.22),
        ("Observer residual grows\neta_dq, x_bar_dq", 0.42),
        ("Inverse model scales error\nby 1/dt and 1/dt^2", 0.62),
        ("u* and u_t saturate\nno slew-rate limit", 0.80),
    ]
    for text, x0 in boxes:
        rect = plt.Rectangle((x0, 0.38), 0.17, 0.34, transform=ax.transAxes, facecolor="#eff6ff", edgecolor="#2563eb", linewidth=1.2)
        ax.add_patch(rect)
        ax.text(x0 + 0.085, 0.55, text, transform=ax.transAxes, ha="center", va="center", fontsize=10)
    for _, x0 in boxes[:-1]:
        ax.annotate("", xy=(x0 + 0.195, 0.55), xytext=(x0 + 0.17, 0.55), xycoords=ax.transAxes, arrowprops=dict(arrowstyle="->", lw=1.5, color="#334155"))
    ax.text(0.5, 0.17, "Closed-loop result: alternating large torque commands drive high apparent velocity oscillations while position still tracks approximately.", transform=ax.transAxes, ha="center", fontsize=11, fontweight="bold")
    savefig(path)


def write_markdown_summary(rows: list[dict[str, str]], path: Path) -> None:
    selected_cols = [
        "case",
        "joint",
        "analysis_window_s",
        "dq_ref_abs_max",
        "dq_actual_abs_max",
        "dq_actual_std",
        "q_rmse",
        "u_t_abs_mean",
        "u_t_sat_90pct_frac",
        "eta_dq_abs_mean",
        "r_d_q_abs_max",
    ]
    lines = []
    lines.append("| " + " | ".join(selected_cols) + " |")
    lines.append("| " + " | ".join(["---"] * len(selected_cols)) + " |")
    for row in rows:
        values = []
        for col in selected_cols:
            value = row[col]
            if col.endswith("frac"):
                value = f"{100.0 * float(value):.2f}%"
            values.append(value)
        lines.append("| " + " | ".join(values) + " |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, default=DATA_ROOT)
    parser.add_argument("--out-dir", type=Path, default=ANALYSIS_DIR)
    args = parser.parse_args()

    setup_style()
    cases = (
        CaseSpec(
            "single_knee",
            "Single RightKnee",
            args.data_root / "right_knee_only/mujoco_closed_loop_log.csv",
            (RIGHT_KNEE,),
        ),
        CaseSpec(
            "dual_hip_knee",
            "RightHipPitch + RightKnee",
            args.data_root / "right_hip_pitch_and_knee/mujoco_closed_loop_log.csv",
            (RIGHT_HIP_PITCH, RIGHT_KNEE),
        ),
    )
    data_by_case = {case.key: read_case(case.csv_path) for case in cases}
    rows = collect_metrics(cases, data_by_case)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    write_metrics_csv(rows, args.out_dir / "eid_coupling_metrics.csv")
    write_markdown_summary(rows, args.out_dir / "eid_coupling_metrics_table.md")
    plot_single_knee_stability(data_by_case, args.out_dir / "fig0_single_knee_stability_evidence.png")
    plot_knee_single_vs_dual(data_by_case, args.out_dir / "fig1_same_knee_single_vs_dual.png")
    plot_dual_diagnostics(data_by_case, args.out_dir / "fig2_dual_internal_diagnostics.png")
    plot_metric_bars(rows, args.out_dir / "fig3_quantitative_bars.png")
    plot_zoom_chattering(data_by_case, args.out_dir / "fig4_zoomed_chattering.png")
    plot_theory_chain(args.out_dir / "fig5_theory_chain.png")
    print(f"Wrote analysis outputs to: {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
