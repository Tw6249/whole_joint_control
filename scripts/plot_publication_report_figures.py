#!/usr/bin/env python3
"""Generate publication-style figures for the hip-knee EID report.

The script only reads existing experiment outputs. It does not rerun MuJoCo or
change metric definitions.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import test_preview_mpc_interpolation as preview_mpc


HIP = 1
KNEE = 2

METHODS = ["G0_no_eid", "G1_input_domain"]
METHOD_LABELS = {
    "G0_no_eid": "PD (G0)",
    "G1_input_domain": "Input-domain EID (G1)",
}
METHOD_COLORS = {
    "G0_no_eid": "#7F7F7F",
    "G1_input_domain": "#0072B2",
}
METHOD_LINESTYLES = {
    "G0_no_eid": (0, (4, 2)),
    "G1_input_domain": "solid",
}

EXPERIMENTS = [
    "E1_same_phase_no_disturbance",
    "E2_anti_phase_hip_disturbance",
    "E3_gait_like_load_disturbance",
]
EXPERIMENT_LABELS = {
    "E1_same_phase_no_disturbance": "E1: same phase\nno disturbance",
    "E2_anti_phase_hip_disturbance": "E2: anti phase\nhip disturbance",
    "E3_gait_like_load_disturbance": "E3: gait-like\nload disturbance",
}
DISTURBANCE_WINDOWS = {
    "E1_same_phase_no_disturbance": None,
    "E2_anti_phase_hip_disturbance": (2.0, 5.0),
    "E3_gait_like_load_disturbance": (2.0, 5.0),
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


def applied_tau(df: pd.DataFrame) -> np.ndarray:
    return (
        df["motor_kp"].to_numpy() * (df["motor_q"].to_numpy() - df["q_actual"].to_numpy())
        + df["motor_kd"].to_numpy() * (df["motor_dq"].to_numpy() - df["dq_actual"].to_numpy())
        + df["motor_tau"].to_numpy()
    )


def load_pair_arrays(log_path: Path) -> dict[str, np.ndarray]:
    df = pd.read_csv(log_path)
    df = df[df["joint_id"].isin([HIP, KNEE])].copy()
    df["tau"] = applied_tau(df)
    pivot = df.pivot_table(
        index="t",
        columns="joint_id",
        values=["q_ref_shaped", "dq_ref_shaped", "q_actual", "dq_actual", "tau"],
        aggfunc="first",
    ).dropna()
    return {
        "t": pivot.index.to_numpy(dtype=float),
        "q_ref_h": pivot[("q_ref_shaped", HIP)].to_numpy(dtype=float),
        "q_ref_k": pivot[("q_ref_shaped", KNEE)].to_numpy(dtype=float),
        "dq_ref_h": pivot[("dq_ref_shaped", HIP)].to_numpy(dtype=float),
        "dq_ref_k": pivot[("dq_ref_shaped", KNEE)].to_numpy(dtype=float),
        "q_h": pivot[("q_actual", HIP)].to_numpy(dtype=float),
        "q_k": pivot[("q_actual", KNEE)].to_numpy(dtype=float),
        "dq_h": pivot[("dq_actual", HIP)].to_numpy(dtype=float),
        "dq_k": pivot[("dq_actual", KNEE)].to_numpy(dtype=float),
        "tau_h": pivot[("tau", HIP)].to_numpy(dtype=float),
        "tau_k": pivot[("tau", KNEE)].to_numpy(dtype=float),
    }


def save_all(fig: plt.Figure, stem: Path) -> None:
    stem.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(stem.with_suffix(".pdf"))
    fig.savefig(stem.with_suffix(".png"))
    plt.close(fig)


def plot_main_metrics(metrics_path: Path, out_dir: Path) -> None:
    metrics = pd.read_csv(metrics_path)
    metrics["experiment"] = pd.Categorical(metrics["experiment"], EXPERIMENTS, ordered=True)
    metrics["method"] = pd.Categorical(metrics["method"], METHODS, ordered=True)
    metrics = metrics.sort_values(["experiment", "method"])

    specs = [
        ("rmse_coord", r"Coordination RMSE [rad]", "linear"),
        ("u_rms", r"Torque RMS [Nm]", "log"),
        ("du_rms", r"$\Delta$ torque RMS [Nm/sample]", "log"),
    ]
    x = np.arange(len(EXPERIMENTS))
    width = 0.28

    fig, axes = plt.subplots(1, 3, figsize=(6.85, 2.35))
    for ax, (metric, ylabel, scale) in zip(axes, specs):
        for idx, method in enumerate(METHODS):
            subset = metrics[metrics["method"] == method].set_index("experiment")
            values = [subset.loc[exp, metric] for exp in EXPERIMENTS]
            ax.bar(
            x + (idx - 0.5) * width,
                values,
                width=width,
                color=METHOD_COLORS[method],
                label=METHOD_LABELS[method],
                edgecolor="black",
                linewidth=0.35,
            )
        ax.set_xticks(x)
        ax.set_xticklabels(["E1", "E2", "E3"])
        ax.set_ylabel(ylabel)
        ax.set_yscale(scale)
        ax.tick_params(axis="x", length=0)
        if scale == "log":
            ax.set_ylim(bottom=max(1e-2, metrics[metric].min() * 0.55))
    axes[0].text(-0.18, 1.05, "(a)", transform=axes[0].transAxes, fontweight="bold")
    axes[1].text(-0.18, 1.05, "(b)", transform=axes[1].transAxes, fontweight="bold")
    axes[2].text(-0.18, 1.05, "(c)", transform=axes[2].transAxes, fontweight="bold")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.5, 1.08))
    fig.subplots_adjust(top=0.78, bottom=0.28, left=0.08, right=0.99, wspace=0.43)
    save_all(fig, out_dir / "paper_main_metrics")


def plot_closed_loop_evidence(base_dir: Path, out_dir: Path) -> None:
    fig, axes = plt.subplots(3, 3, figsize=(6.85, 5.55))
    floor = 1.0e-4
    for row, experiment in enumerate(EXPERIMENTS):
        ax_phase, ax_coord, ax_tau = axes[row]
        first = True
        for method in METHODS:
            log_path = base_dir / "runs" / f"{experiment}__{method}" / "mujoco_closed_loop_log.csv"
            arr = load_pair_arrays(log_path)
            color = METHOD_COLORS[method]
            linestyle = METHOD_LINESTYLES[method]
            if first:
                ax_phase.plot(
                    arr["q_ref_h"],
                    arr["q_ref_k"],
                    color="black",
                    linewidth=1.2,
                    label="reference",
                )
                first = False
            ax_phase.plot(
                arr["q_h"],
                arr["q_k"],
                color=color,
                linestyle=linestyle,
                linewidth=1.0,
                label=METHOD_LABELS[method],
            )
            e_coord = (arr["q_ref_h"] - arr["q_h"]) - (arr["q_ref_k"] - arr["q_k"])
            tau_norm = np.sqrt(arr["tau_h"] ** 2 + arr["tau_k"] ** 2)
            ax_coord.plot(
                arr["t"],
                np.maximum(np.abs(e_coord), floor),
                color=color,
                linestyle=linestyle,
                linewidth=1.0,
            )
            ax_tau.plot(
                arr["t"],
                np.maximum(tau_norm, floor),
                color=color,
                linestyle=linestyle,
                linewidth=1.0,
            )
        for ax in (ax_coord, ax_tau):
            window = DISTURBANCE_WINDOWS[experiment]
            if window is not None:
                ax.axvspan(window[0], window[1], color="#D9D9D9", alpha=0.35, lw=0)
        ax_phase.set_ylabel(f"{EXPERIMENT_LABELS[experiment]}\n$q_k$ [rad]")
        ax_coord.set_yscale("log")
        ax_tau.set_yscale("log")
        ax_coord.set_ylim(bottom=1e-4)
        ax_tau.set_ylim(bottom=1e-2)
        if row == 0:
            ax_phase.set_title("Phase portrait")
            ax_coord.set_title(r"$|e_h-e_k|$")
            ax_tau.set_title(r"$\|u\|_2$")
        if row == 2:
            ax_phase.set_xlabel(r"$q_h$ [rad]")
            ax_coord.set_xlabel("time [s]")
            ax_tau.set_xlabel("time [s]")
        ax_coord.set_ylabel("[rad]")
        ax_tau.set_ylabel("[Nm]")
    axes[0, 0].text(-0.22, 1.10, "(a)", transform=axes[0, 0].transAxes, fontweight="bold")
    axes[0, 1].text(-0.22, 1.10, "(b)", transform=axes[0, 1].transAxes, fontweight="bold")
    axes[0, 2].text(-0.22, 1.10, "(c)", transform=axes[0, 2].transAxes, fontweight="bold")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, frameon=False, bbox_to_anchor=(0.54, 1.02))
    fig.subplots_adjust(top=0.90, bottom=0.08, left=0.10, right=0.98, hspace=0.52, wspace=0.46)
    save_all(fig, out_dir / "paper_closed_loop_evidence")


def plot_state_tracking_timeseries(base_dir: Path, out_dir: Path) -> None:
    fig, axes = plt.subplots(3, 4, figsize=(6.85, 5.8), sharex=True)
    specs = [
        ("q_ref_h", "q_h", r"$q_h$ [rad]"),
        ("q_ref_k", "q_k", r"$q_k$ [rad]"),
        ("dq_ref_h", "dq_h", r"$\dot q_h$ [rad/s]"),
        ("dq_ref_k", "dq_k", r"$\dot q_k$ [rad/s]"),
    ]
    for row, experiment in enumerate(EXPERIMENTS):
        first_method = METHODS[0]
        ref_arr = load_pair_arrays(
            base_dir / "runs" / f"{experiment}__{first_method}" / "mujoco_closed_loop_log.csv"
        )
        for col, (ref_key, actual_key, ylabel) in enumerate(specs):
            ax = axes[row, col]
            ax.plot(
                ref_arr["t"],
                ref_arr[ref_key],
                color="black",
                linewidth=1.1,
                label="reference" if row == 0 and col == 0 else None,
            )
            for method in METHODS:
                arr = load_pair_arrays(
                    base_dir / "runs" / f"{experiment}__{method}" / "mujoco_closed_loop_log.csv"
                )
                ax.plot(
                    arr["t"],
                    arr[actual_key],
                    color=METHOD_COLORS[method],
                    linestyle=METHOD_LINESTYLES[method],
                    linewidth=0.9,
                    label=METHOD_LABELS[method] if row == 0 and col == 0 else None,
                )
            window = DISTURBANCE_WINDOWS[experiment]
            if window is not None:
                ax.axvspan(window[0], window[1], color="#D9D9D9", alpha=0.35, lw=0)
            if row == 0:
                title = [r"Hip position", r"Knee position", r"Hip velocity", r"Knee velocity"][col]
                ax.set_title(title)
            if col == 0:
                ax.set_ylabel(f"{EXPERIMENT_LABELS[experiment]}\n{ylabel}")
            else:
                ax.set_ylabel(ylabel)
            if row == 2:
                ax.set_xlabel("time [s]")
    axes[0, 0].text(-0.28, 1.11, "(a)", transform=axes[0, 0].transAxes, fontweight="bold")
    axes[0, 1].text(-0.28, 1.11, "(b)", transform=axes[0, 1].transAxes, fontweight="bold")
    axes[0, 2].text(-0.28, 1.11, "(c)", transform=axes[0, 2].transAxes, fontweight="bold")
    axes[0, 3].text(-0.28, 1.11, "(d)", transform=axes[0, 3].transAxes, fontweight="bold")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, frameon=False, bbox_to_anchor=(0.54, 1.02))
    fig.subplots_adjust(top=0.89, bottom=0.08, left=0.10, right=0.99, hspace=0.52, wspace=0.45)
    save_all(fig, out_dir / "paper_state_tracking_timeseries")


def plot_state_tracking_g0_g1(base_dir: Path, out_dir: Path) -> None:
    methods = ["G0_no_eid", "G1_input_domain"]
    fig, axes = plt.subplots(3, 4, figsize=(6.85, 5.8), sharex=True)
    specs = [
        ("q_ref_h", "q_h", r"$q_h$ [rad]"),
        ("q_ref_k", "q_k", r"$q_k$ [rad]"),
        ("dq_ref_h", "dq_h", r"$\dot q_h$ [rad/s]"),
        ("dq_ref_k", "dq_k", r"$\dot q_k$ [rad/s]"),
    ]
    for row, experiment in enumerate(EXPERIMENTS):
        ref_arr = load_pair_arrays(
            base_dir / "runs" / f"{experiment}__G0_no_eid" / "mujoco_closed_loop_log.csv"
        )
        for col, (ref_key, actual_key, ylabel) in enumerate(specs):
            ax = axes[row, col]
            ax.plot(
                ref_arr["t"],
                ref_arr[ref_key],
                color="black",
                linewidth=1.15,
                label="reference" if row == 0 and col == 0 else None,
            )
            for method in methods:
                arr = load_pair_arrays(
                    base_dir / "runs" / f"{experiment}__{method}" / "mujoco_closed_loop_log.csv"
                )
                ax.plot(
                    arr["t"],
                    arr[actual_key],
                    color=METHOD_COLORS[method],
                    linestyle=METHOD_LINESTYLES[method],
                    linewidth=1.0,
                    label=METHOD_LABELS[method] if row == 0 and col == 0 else None,
                )
            window = DISTURBANCE_WINDOWS[experiment]
            if window is not None:
                ax.axvspan(window[0], window[1], color="#D9D9D9", alpha=0.35, lw=0)
            if row == 0:
                ax.set_title(["Hip position", "Knee position", "Hip velocity", "Knee velocity"][col])
            if col == 0:
                ax.set_ylabel(f"{EXPERIMENT_LABELS[experiment]}\n{ylabel}")
            else:
                ax.set_ylabel(ylabel)
            if row == 2:
                ax.set_xlabel("time [s]")
    axes[0, 0].text(-0.28, 1.11, "(a)", transform=axes[0, 0].transAxes, fontweight="bold")
    axes[0, 1].text(-0.28, 1.11, "(b)", transform=axes[0, 1].transAxes, fontweight="bold")
    axes[0, 2].text(-0.28, 1.11, "(c)", transform=axes[0, 2].transAxes, fontweight="bold")
    axes[0, 3].text(-0.28, 1.11, "(d)", transform=axes[0, 3].transAxes, fontweight="bold")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.54, 1.02))
    fig.subplots_adjust(top=0.89, bottom=0.08, left=0.10, right=0.99, hspace=0.52, wspace=0.45)
    save_all(fig, out_dir / "paper_state_tracking_g0_g1")


def plot_preview_mpc(preview_metrics_path: Path, out_dir: Path) -> None:
    metrics = pd.read_csv(preview_metrics_path)
    scenarios = ["smooth_sine", "multi_reversal", "step_hold"]
    scenario_labels = ["smooth\nsine", "multi\nreversal", "step\nhold"]
    methods = ["quintic_stop", "mpc_1", "mpc_2", "mpc_3"]
    labels = ["quintic\nstop", "MPC\n1 point", "MPC\n2 points", "MPC\n3 points"]
    colors = ["#7F7F7F", "#E69F00", "#0072B2", "#009E73"]
    specs = [
        ("ddq_rms", r"Normalized acceleration RMS"),
        ("jerk_rms", r"Normalized jerk RMS"),
    ]
    x = np.arange(len(scenarios))
    width = 0.18
    fig, axes = plt.subplots(1, 2, figsize=(6.85, 2.35), sharey=True)
    for ax, (metric, ylabel) in zip(axes, specs):
        for idx, method in enumerate(methods):
            values = []
            for scenario in scenarios:
                sub = metrics[metrics["scenario"] == scenario].set_index("method")
                values.append(sub.loc[method, metric] / sub.loc["quintic_stop", metric])
            ax.bar(
                x + (idx - 1.5) * width,
                values,
                width=width,
                color=colors[idx],
                label=labels[idx],
                edgecolor="black",
                linewidth=0.35,
            )
        ax.axhline(1.0, color="black", linewidth=0.7, linestyle=(0, (3, 2)))
        ax.set_xticks(x)
        ax.set_xticklabels(scenario_labels)
        ax.set_yscale("log")
        ax.set_ylim(0.02, 1.35)
        ax.set_ylabel(ylabel)
        ax.tick_params(axis="x", length=0)
    axes[0].text(-0.15, 1.05, "(a)", transform=axes[0].transAxes, fontweight="bold")
    axes[1].text(-0.15, 1.05, "(b)", transform=axes[1].transAxes, fontweight="bold")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, frameon=False, bbox_to_anchor=(0.5, 1.08))
    fig.subplots_adjust(top=0.76, bottom=0.22, left=0.09, right=0.98, wspace=0.30)
    save_all(fig, out_dir / "paper_preview_mpc_summary")


def plot_preview_mpc_timeseries(out_dir: Path) -> None:
    cfg = preview_mpc.ExperimentConfig()
    results = preview_mpc.run_all(cfg)
    colors = {
        "quintic_stop": "#7F7F7F",
        "mpc_1": "#E69F00",
        "mpc_2": "#0072B2",
        "mpc_3": "#009E73",
    }
    labels = {
        "quintic_stop": "quintic stop",
        "mpc_1": "MPC 1 point",
        "mpc_2": "MPC 2 points",
        "mpc_3": "MPC 3 points",
    }
    scenario_labels = {
        "smooth_sine": "smooth sine",
        "multi_reversal": "multi reversal",
        "step_hold": "step hold",
    }
    fig, axes = plt.subplots(3, 3, figsize=(6.85, 5.35), sharex=True)
    fields = [("q", r"$q$ [rad]"), ("dq", r"$\dot q$ [rad/s]"), ("ddq", r"$\ddot q$ [rad/s$^2$]")]
    for row, scenario in enumerate(["smooth_sine", "multi_reversal", "step_hold"]):
        target = preview_mpc.SCENARIOS[scenario]
        methods = results[scenario]
        t_ref = next(iter(methods.values())).t
        policy_t = np.arange(0.0, cfg.duration + cfg.policy_dt * 0.5, cfg.policy_dt)
        policy_q = np.asarray(target(policy_t), dtype=float)
        for col, (field, ylabel) in enumerate(fields):
            ax = axes[row, col]
            if field == "q":
                ax.plot(t_ref, np.asarray(target(t_ref), dtype=float), color="black", linewidth=1.0)
                ax.scatter(policy_t, policy_q, s=6, color="black", zorder=3, linewidths=0.0)
            for method, traj in methods.items():
                ax.plot(
                    traj.t,
                    getattr(traj, field),
                    color=colors[method],
                    linestyle=(0, (4, 2)) if method == "quintic_stop" else "solid",
                    linewidth=0.9,
                    label=labels[method] if row == 0 and col == 0 else None,
                )
            if row == 0:
                ax.set_title(["Position", "Velocity", "Acceleration"][col])
            if col == 0:
                ax.set_ylabel(f"{scenario_labels[scenario]}\n{ylabel}")
            else:
                ax.set_ylabel(ylabel)
            if row == 2:
                ax.set_xlabel("time [s]")
    axes[0, 0].text(-0.25, 1.11, "(a)", transform=axes[0, 0].transAxes, fontweight="bold")
    axes[0, 1].text(-0.25, 1.11, "(b)", transform=axes[0, 1].transAxes, fontweight="bold")
    axes[0, 2].text(-0.25, 1.11, "(c)", transform=axes[0, 2].transAxes, fontweight="bold")
    handles, labels_out = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels_out, loc="upper center", ncol=4, frameon=False, bbox_to_anchor=(0.55, 1.02))
    fig.subplots_adjust(top=0.89, bottom=0.08, left=0.10, right=0.99, hspace=0.52, wspace=0.42)
    save_all(fig, out_dir / "paper_preview_mpc_timeseries")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hip-knee-dir", type=Path, default=Path("analysis_artifacts/hip_knee_domain_experiment"))
    parser.add_argument("--preview-dir", type=Path, default=Path("analysis_artifacts/preview_mpc_interpolation"))
    parser.add_argument("--out-dir", type=Path, default=Path("analysis_artifacts/hip_knee_domain_experiment/figures_publication"))
    args = parser.parse_args()

    configure_matplotlib()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    plot_main_metrics(args.hip_knee_dir / "hip_knee_domain_metrics.csv", args.out_dir)
    plot_state_tracking_g0_g1(args.hip_knee_dir, args.out_dir)
    plot_state_tracking_timeseries(args.hip_knee_dir, args.out_dir)
    plot_closed_loop_evidence(args.hip_knee_dir, args.out_dir)
    plot_preview_mpc(args.preview_dir / "preview_mpc_metrics.csv", args.out_dir)
    plot_preview_mpc_timeseries(args.out_dir)
    print(f"figures={args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
