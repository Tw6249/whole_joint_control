#!/usr/bin/env python3
"""Create compact paper-style summary figures for the hip-knee report."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path("analysis_artifacts")
DOMAIN_METRICS = ROOT / "hip_knee_domain_experiment" / "hip_knee_domain_metrics.csv"
PREVIEW_METRICS = ROOT / "preview_mpc_interpolation" / "preview_mpc_metrics.csv"
OUT_DIR = ROOT / "hip_knee_domain_experiment" / "figures"

METHOD_LABELS = {
    "G0_no_eid": "G0",
    "G1_input_domain": "G1",
}

METHOD_COLORS = {
    "G0_no_eid": "#8A8F98",
    "G1_input_domain": "#009E73",
}

PREVIEW_LABELS = {
    "quintic_stop": "Quintic-stop",
    "mpc_1": "MPC-1",
    "mpc_2": "MPC-2",
    "mpc_3": "MPC-3",
}

PREVIEW_COLORS = {
    "quintic_stop": "#8A8F98",
    "mpc_1": "#E69F00",
    "mpc_2": "#0072B2",
    "mpc_3": "#009E73",
}


def set_style() -> None:
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


def save(fig: plt.Figure, stem: str) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_DIR / f"{stem}.png")
    fig.savefig(OUT_DIR / f"{stem}.pdf")
    plt.close(fig)


def plot_domain_summary() -> None:
    df = pd.read_csv(DOMAIN_METRICS)
    experiments = [
        "E1_same_phase_no_disturbance",
        "E2_anti_phase_hip_disturbance",
        "E3_gait_like_load_disturbance",
    ]
    exp_labels = ["E1", "E2", "E3"]
    methods = ["G0_no_eid", "G1_input_domain"]
    specs = [
        ("rmse_coord", "coord. RMSE [rad]", "linear"),
        ("u_rms", r"$u_\mathrm{rms}$ [Nm]", "log"),
        ("du_rms", r"$\Delta u_\mathrm{rms}$ [Nm/sample]", "log"),
    ]

    x = np.arange(len(experiments))
    width = 0.28
    fig, axes = plt.subplots(1, 3, figsize=(6.85, 2.35))

    for ax, (metric, ylabel, scale) in zip(axes, specs):
        for idx, method in enumerate(methods):
            values = [
                float(df[(df["experiment"] == exp) & (df["method"] == method)][metric].iloc[0])
                for exp in experiments
            ]
            ax.bar(
                x + (idx - 0.5) * width,
                values,
                width=width,
                color=METHOD_COLORS[method],
                label=METHOD_LABELS[method],
            )
        ax.set_xticks(x)
        ax.set_xticklabels(exp_labels)
        ax.set_ylabel(ylabel)
        ax.set_yscale(scale)
        ax.grid(True, axis="y")
        ax.grid(False, axis="x")

    axes[0].legend(loc="upper left", frameon=False, ncol=3, bbox_to_anchor=(0.0, 1.20))
    fig.tight_layout(w_pad=1.1)
    save(fig, "paper_domain_summary")


def plot_preview_summary() -> None:
    df = pd.read_csv(PREVIEW_METRICS)
    scenarios = ["smooth_sine", "multi_reversal", "step_hold"]
    scenario_labels = ["Sine", "Reversal", "Step"]
    methods = ["quintic_stop", "mpc_1", "mpc_2", "mpc_3"]
    specs = [
        ("ddq_rms", r"$\ddot q_\mathrm{rms}$ [rad/s$^2$]"),
        ("jerk_rms", r"$j_\mathrm{rms}$ [rad/s$^3$]"),
    ]

    x = np.arange(len(scenarios))
    width = 0.19
    fig, axes = plt.subplots(1, 2, figsize=(6.85, 2.35))

    for ax, (metric, ylabel) in zip(axes, specs):
        for idx, method in enumerate(methods):
            values = [
                float(df[(df["scenario"] == scene) & (df["method"] == method)][metric].iloc[0])
                for scene in scenarios
            ]
            ax.bar(
                x + (idx - 1.5) * width,
                values,
                width=width,
                color=PREVIEW_COLORS[method],
                label=PREVIEW_LABELS[method],
            )
        ax.set_xticks(x)
        ax.set_xticklabels(scenario_labels)
        ax.set_ylabel(ylabel)
        ax.set_yscale("log")
        ax.grid(True, axis="y")
        ax.grid(False, axis="x")

    axes[0].legend(loc="upper left", frameon=False, ncol=4, bbox_to_anchor=(0.0, 1.20))
    fig.tight_layout(w_pad=1.2)
    save(fig, "paper_preview_summary")


def main() -> int:
    set_style()
    plot_domain_summary()
    plot_preview_summary()
    print(f"wrote={OUT_DIR / 'paper_domain_summary.png'}")
    print(f"wrote={OUT_DIR / 'paper_preview_summary.png'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
