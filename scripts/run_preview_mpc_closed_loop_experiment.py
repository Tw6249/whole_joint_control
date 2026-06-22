#!/usr/bin/env python3
"""Run closed-loop MuJoCo experiments for quintic-stop vs 2-point Preview-MPC references."""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml


HIP = 1
KNEE = 2
JOINTS = [HIP, KNEE]


@dataclass(frozen=True)
class JointPolicy:
    center: float
    amplitude: float
    frequency_hz: float
    phase_rad: float


@dataclass(frozen=True)
class ExperimentSpec:
    exp_id: str
    label: str
    hip: JointPolicy
    knee: JointPolicy
    disturbance_joints: tuple[int, ...] = ()
    disturbance_torques: tuple[float, ...] = ()
    disturbance_start: float = 2.0
    disturbance_end: float = 5.0


@dataclass(frozen=True)
class ControllerSpec:
    controller_id: str
    label: str
    config_kind: str


@dataclass(frozen=True)
class ReferenceSpec:
    reference_id: str
    label: str
    interpolation: str
    reference_points: int


EXPERIMENTS = [
    ExperimentSpec(
        exp_id="E1_same_phase_no_disturbance",
        label="E1 same-phase no disturbance",
        hip=JointPolicy(center=-0.30, amplitude=0.25, frequency_hz=0.25, phase_rad=-math.pi / 2.0),
        knee=JointPolicy(center=0.75, amplitude=0.20, frequency_hz=0.25, phase_rad=-math.pi / 2.0),
    ),
    ExperimentSpec(
        exp_id="E2_anti_phase_hip_disturbance",
        label="E2 anti-phase hip disturbance",
        hip=JointPolicy(center=-0.30, amplitude=0.25, frequency_hz=0.25, phase_rad=-math.pi / 2.0),
        knee=JointPolicy(center=0.75, amplitude=0.20, frequency_hz=0.25, phase_rad=math.pi / 2.0),
        disturbance_joints=(HIP,),
        disturbance_torques=(18.0,),
    ),
    ExperimentSpec(
        exp_id="E3_gait_like_load_disturbance",
        label="E3 gait-like load disturbance",
        hip=JointPolicy(center=-0.25, amplitude=0.45, frequency_hz=0.35, phase_rad=-math.pi / 2.0),
        knee=JointPolicy(center=0.75, amplitude=0.35, frequency_hz=0.35, phase_rad=0.15),
        disturbance_joints=(HIP, KNEE),
        disturbance_torques=(12.0, -10.0),
    ),
]

CONTROLLERS = [
    ControllerSpec("pd", "PD", "position_pd"),
    ControllerSpec("input_eid", "Input-domain EID", "eid"),
]

REFERENCES = [
    ReferenceSpec("quintic_stop", "Quintic stop", "open_loop", 1),
    ReferenceSpec("preview_mpc_2", "Preview-MPC 2", "preview_mpc", 2),
]

COMBO_ORDER = [
    ("pd", "quintic_stop"),
    ("pd", "preview_mpc_2"),
    ("input_eid", "quintic_stop"),
    ("input_eid", "preview_mpc_2"),
]

COMBO_LABELS = {
    ("pd", "quintic_stop"): "PD + quintic",
    ("pd", "preview_mpc_2"): "PD + Preview-MPC",
    ("input_eid", "quintic_stop"): "Input EID + quintic",
    ("input_eid", "preview_mpc_2"): "Input EID + Preview-MPC",
}

COMBO_COLORS = {
    ("pd", "quintic_stop"): "#7F7F7F",
    ("pd", "preview_mpc_2"): "#56B4E9",
    ("input_eid", "quintic_stop"): "#E69F00",
    ("input_eid", "preview_mpc_2"): "#0072B2",
}

COMBO_LINESTYLES = {
    ("pd", "quintic_stop"): (0, (4, 2)),
    ("pd", "preview_mpc_2"): "solid",
    ("input_eid", "quintic_stop"): (0, (1, 1)),
    ("input_eid", "preview_mpc_2"): "solid",
}

EXPERIMENT_LABELS = {
    "E1_same_phase_no_disturbance": "E1",
    "E2_anti_phase_hip_disturbance": "E2",
    "E3_gait_like_load_disturbance": "E3",
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


def configure_joint(joint_cfg: dict, policy: JointPolicy, enabled: bool, reference: ReferenceSpec) -> None:
    joint_cfg["enabled"] = bool(enabled)
    joint_cfg["policy_interpolation"] = reference.interpolation
    joint_cfg["policy_reference_points"] = reference.reference_points
    joint_cfg["policy_source"] = "sine"
    joint_cfg["policy_center"] = float(policy.center)
    joint_cfg["policy_amplitude"] = float(policy.amplitude)
    joint_cfg["policy_frequency_hz"] = float(policy.frequency_hz)
    joint_cfg["policy_phase_rad"] = float(policy.phase_rad)


def build_config(base_config: Path,
                 out_path: Path,
                 experiment: ExperimentSpec,
                 controller: ControllerSpec,
                 reference: ReferenceSpec) -> None:
    cfg = yaml.safe_load(base_config.read_text(encoding="utf-8"))
    cfg["controller"]["kind"] = controller.config_kind
    for raw_joint_id, joint_cfg in cfg["controller"]["joints"].items():
        joint_id = int(raw_joint_id)
        if joint_id == HIP:
            configure_joint(joint_cfg, experiment.hip, True, reference)
        elif joint_id == KNEE:
            configure_joint(joint_cfg, experiment.knee, True, reference)
        else:
            joint_cfg["enabled"] = False

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True), encoding="utf-8")


def run_mujoco(config: Path,
               out_dir: Path,
               experiment: ExperimentSpec,
               duration: float,
               dt: float,
               stepper: Path) -> None:
    cmd = [
        sys.executable,
        "scripts/run_mujoco.py",
        "--config", str(config),
        "--stepper", str(stepper),
        "--out-dir", str(out_dir),
        "--duration", str(duration),
        "--dt", str(dt),
        "--export-summary",
    ]
    if experiment.disturbance_joints:
        cmd.extend([
            "--disturbance-joints", ",".join(str(j) for j in experiment.disturbance_joints),
            "--disturbance-torques", ",".join(str(v) for v in experiment.disturbance_torques),
            "--disturbance-start", str(experiment.disturbance_start),
            "--disturbance-end", str(experiment.disturbance_end),
        ])
    subprocess.run(cmd, check=True)


def applied_tau(df: pd.DataFrame) -> np.ndarray:
    return (
        df["motor_kp"].to_numpy() * (df["motor_q"].to_numpy() - df["q_actual"].to_numpy())
        + df["motor_kd"].to_numpy() * (df["motor_dq"].to_numpy() - df["dq_actual"].to_numpy())
        + df["motor_tau"].to_numpy()
    )


def load_pair_arrays(log_path: Path) -> dict[str, np.ndarray]:
    df = pd.read_csv(log_path)
    df = df[df["joint_id"].isin(JOINTS)].copy()
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


def metrics_from_arrays(a: dict[str, np.ndarray]) -> dict[str, float]:
    t = a["t"]
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 1.0
    e_h = a["q_ref_h"] - a["q_h"]
    e_k = a["q_ref_k"] - a["q_k"]
    tau_h = a["tau_h"]
    tau_k = a["tau_k"]
    dq_ref_rate_h = np.diff(a["dq_ref_h"]) / dt
    dq_ref_rate_k = np.diff(a["dq_ref_k"]) / dt
    du_h = np.diff(tau_h)
    du_k = np.diff(tau_k)
    tau_rate_h = du_h / dt
    tau_rate_k = du_k / dt
    return {
        "rmse_h": float(np.sqrt(np.mean(e_h * e_h))),
        "rmse_k": float(np.sqrt(np.mean(e_k * e_k))),
        "rmse_coord": float(np.sqrt(np.mean((e_h - e_k) ** 2))),
        "dq_ref_rate_rms": float(np.sqrt(np.mean(dq_ref_rate_h * dq_ref_rate_h + dq_ref_rate_k * dq_ref_rate_k))),
        "u_rms": float(np.sqrt(np.mean(tau_h * tau_h + tau_k * tau_k))),
        "du_rms": float(np.sqrt(np.mean(du_h * du_h + du_k * du_k))) if len(du_h) else 0.0,
        "tau_rate_rms": float(np.sqrt(np.mean(tau_rate_h * tau_rate_h + tau_rate_k * tau_rate_k))) if len(tau_rate_h) else 0.0,
        "tau_h_abs_max": float(np.max(np.abs(tau_h))),
        "tau_k_abs_max": float(np.max(np.abs(tau_k))),
        "samples": int(len(t)),
    }


def write_metrics(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = [
        "experiment", "controller", "controller_label", "reference", "reference_label",
        "rmse_h", "rmse_k", "rmse_coord", "dq_ref_rate_rms",
        "u_rms", "du_rms", "tau_rate_rms", "tau_h_abs_max", "tau_k_abs_max", "samples",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def save_all(fig: plt.Figure, stem: Path) -> None:
    stem.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(stem.with_suffix(".pdf"))
    fig.savefig(stem.with_suffix(".png"))
    plt.close(fig)


def plot_metrics(metrics: pd.DataFrame, out_dir: Path) -> None:
    specs = [
        ("rmse_coord", "Coordination RMSE [rad]", "linear"),
        ("dq_ref_rate_rms", r"Reference $\ddot q$ RMS [rad/s$^2$]", "log"),
        ("u_rms", "Torque RMS [Nm]", "linear"),
        ("du_rms", r"$\Delta$ torque RMS [Nm/sample]", "linear"),
    ]
    x = np.arange(len(EXPERIMENTS))
    width = 0.18
    fig, axes = plt.subplots(1, 4, figsize=(6.85, 2.45))
    for ax, (metric, ylabel, scale) in zip(axes, specs):
        for idx, combo in enumerate(COMBO_ORDER):
            controller, reference = combo
            subset = metrics[
                (metrics["controller"] == controller) & (metrics["reference"] == reference)
            ].set_index("experiment")
            values = [subset.loc[exp.exp_id, metric] for exp in EXPERIMENTS]
            ax.bar(
                x + (idx - 1.5) * width,
                values,
                width=width,
                color=COMBO_COLORS[combo],
                label=COMBO_LABELS[combo],
                edgecolor="black",
                linewidth=0.30,
            )
        ax.set_xticks(x)
        ax.set_xticklabels([EXPERIMENT_LABELS[exp.exp_id] for exp in EXPERIMENTS])
        ax.set_ylabel(ylabel)
        ax.set_yscale(scale)
        ax.tick_params(axis="x", length=0)
        if scale == "log":
            ax.set_ylim(bottom=max(1.0e-2, metrics[metric].min() * 0.55))
    for idx, ax in enumerate(axes):
        ax.text(-0.20, 1.06, f"({chr(ord('a') + idx)})", transform=ax.transAxes, fontweight="bold")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, frameon=False, bbox_to_anchor=(0.5, 1.02))
    fig.subplots_adjust(top=0.84, bottom=0.25, left=0.07, right=0.99, wspace=0.52)
    save_all(fig, out_dir / "paper_preview_closed_loop_metrics")


def plot_state_timeseries(logs: dict[tuple[str, str, str], dict[str, np.ndarray]], out_dir: Path) -> None:
    fig, axes = plt.subplots(3, 4, figsize=(6.85, 5.80), sharex=True)
    specs = [
        ("q_ref_h", "q_h", r"Hip position", r"$q_h$ [rad]"),
        ("q_ref_k", "q_k", r"Knee position", r"$q_k$ [rad]"),
        ("dq_ref_h", "dq_h", r"Hip velocity", r"$\dot q_h$ [rad/s]"),
        ("dq_ref_k", "dq_k", r"Knee velocity", r"$\dot q_k$ [rad/s]"),
    ]
    for row, experiment in enumerate(EXPERIMENTS):
        exp_id = experiment.exp_id
        q_ref = logs[(exp_id, "pd", "quintic_stop")]
        mpc_ref = logs[(exp_id, "pd", "preview_mpc_2")]
        for col, (ref_key, actual_key, title, ylabel) in enumerate(specs):
            ax = axes[row, col]
            ax.plot(
                q_ref["t"],
                q_ref[ref_key],
                color="black",
                linestyle=(0, (4, 2)),
                linewidth=1.0,
                label="quintic ref" if row == 0 and col == 0 else None,
            )
            ax.plot(
                mpc_ref["t"],
                mpc_ref[ref_key],
                color="black",
                linestyle="solid",
                linewidth=1.0,
                label="Preview ref" if row == 0 and col == 0 else None,
            )
            for combo in COMBO_ORDER:
                arr = logs[(exp_id, combo[0], combo[1])]
                ax.plot(
                    arr["t"],
                    arr[actual_key],
                    color=COMBO_COLORS[combo],
                    linestyle=COMBO_LINESTYLES[combo],
                    linewidth=0.85,
                    label=COMBO_LABELS[combo] if row == 0 and col == 0 else None,
                )
            if experiment.disturbance_joints:
                ax.axvspan(
                    experiment.disturbance_start,
                    experiment.disturbance_end,
                    color="#D9D9D9",
                    alpha=0.35,
                    lw=0,
                )
            if row == 0:
                ax.set_title(title)
            if col == 0:
                ax.set_ylabel(f"{EXPERIMENT_LABELS[exp_id]}\n{ylabel}")
            else:
                ax.set_ylabel(ylabel)
            if row == 2:
                ax.set_xlabel("time [s]")
    for idx, ax in enumerate(axes[0]):
        ax.text(-0.28, 1.10, f"({chr(ord('a') + idx)})", transform=ax.transAxes, fontweight="bold")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.52, 1.03))
    fig.subplots_adjust(top=0.84, bottom=0.08, left=0.10, right=0.99, hspace=0.52, wspace=0.45)
    save_all(fig, out_dir / "paper_preview_closed_loop_state_timeseries")


def plot_e2_evidence(logs: dict[tuple[str, str, str], dict[str, np.ndarray]], out_dir: Path) -> None:
    experiment = "E2_anti_phase_hip_disturbance"
    fig, axes = plt.subplots(1, 3, figsize=(6.85, 2.35))
    floor = 1.0e-6
    for combo in COMBO_ORDER:
        arr = logs[(experiment, combo[0], combo[1])]
        color = COMBO_COLORS[combo]
        linestyle = COMBO_LINESTYLES[combo]
        label = COMBO_LABELS[combo]
        ref_acc_norm = np.sqrt(
            np.gradient(arr["dq_ref_h"], arr["t"]) ** 2 +
            np.gradient(arr["dq_ref_k"], arr["t"]) ** 2
        )
        e_coord = (arr["q_ref_h"] - arr["q_h"]) - (arr["q_ref_k"] - arr["q_k"])
        tau_norm = np.sqrt(arr["tau_h"] ** 2 + arr["tau_k"] ** 2)
        axes[0].plot(arr["t"], np.maximum(ref_acc_norm, floor), color=color, linestyle=linestyle, linewidth=1.0, label=label)
        axes[1].plot(arr["t"], np.maximum(np.abs(e_coord), floor), color=color, linestyle=linestyle, linewidth=1.0)
        axes[2].plot(arr["t"], np.maximum(tau_norm, floor), color=color, linestyle=linestyle, linewidth=1.0)
    for ax in axes:
        ax.axvspan(2.0, 5.0, color="#D9D9D9", alpha=0.35, lw=0)
        ax.set_xlabel("time [s]")
        ax.set_yscale("log")
    axes[0].set_ylabel(r"$\|\ddot q_{ref}\|_2$ [rad/s$^2$]")
    axes[1].set_ylabel(r"$|e_h-e_k|$ [rad]")
    axes[2].set_ylabel(r"$\|u\|_2$ [Nm]")
    axes[0].set_title("Reference acceleration")
    axes[1].set_title("Coordination error")
    axes[2].set_title("Torque norm")
    for idx, ax in enumerate(axes):
        ax.text(-0.16, 1.06, f"({chr(ord('a') + idx)})", transform=ax.transAxes, fontweight="bold")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, frameon=False, bbox_to_anchor=(0.5, 1.02))
    fig.subplots_adjust(top=0.82, bottom=0.25, left=0.08, right=0.99, wspace=0.42)
    save_all(fig, out_dir / "paper_preview_closed_loop_e2")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-config", type=Path, default=Path("config/h1_hip_knee_dual_tuned.yaml"))
    parser.add_argument("--out-dir", type=Path, default=Path("analysis_artifacts/preview_mpc_closed_loop"))
    parser.add_argument("--stepper", type=Path, default=Path("build/Debug/h1_controller_stepper.exe"))
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--skip-runs", action="store_true")
    args = parser.parse_args()

    configure_matplotlib()
    config_dir = args.out_dir / "configs"
    runs_dir = args.out_dir / "runs"
    fig_dir = args.out_dir / "figures_publication"
    rows: list[dict[str, object]] = []
    logs: dict[tuple[str, str, str], dict[str, np.ndarray]] = {}

    for experiment in EXPERIMENTS:
        for controller in CONTROLLERS:
            for reference in REFERENCES:
                run_name = f"{experiment.exp_id}__{controller.controller_id}__{reference.reference_id}"
                config_path = config_dir / f"{run_name}.yaml"
                out_dir = runs_dir / run_name
                build_config(args.base_config, config_path, experiment, controller, reference)
                if not args.skip_runs:
                    run_mujoco(config_path, out_dir, experiment, args.duration, args.dt, args.stepper)
                arr = load_pair_arrays(out_dir / "mujoco_closed_loop_log.csv")
                logs[(experiment.exp_id, controller.controller_id, reference.reference_id)] = arr
                row: dict[str, object] = {
                    "experiment": experiment.exp_id,
                    "controller": controller.controller_id,
                    "controller_label": controller.label,
                    "reference": reference.reference_id,
                    "reference_label": reference.label,
                }
                row.update(metrics_from_arrays(arr))
                rows.append(row)

    metrics_path = args.out_dir / "preview_mpc_closed_loop_metrics.csv"
    write_metrics(metrics_path, rows)
    metrics = pd.DataFrame(rows)
    plot_metrics(metrics, fig_dir)
    plot_state_timeseries(logs, fig_dir)
    plot_e2_evidence(logs, fig_dir)

    print(f"metrics={metrics_path}")
    print(f"figures={fig_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
