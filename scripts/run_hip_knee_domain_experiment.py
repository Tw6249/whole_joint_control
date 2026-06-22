#!/usr/bin/env python3
"""Run hip-knee MuJoCo experiments comparing no EID and input-domain EID."""

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
JOINT_LABELS = {HIP: "RightHipPitch", KNEE: "RightKnee"}


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
    disturbance_ramp: float = 0.1


@dataclass(frozen=True)
class ControllerSpec:
    method: str
    label: str
    config_kind: str
    stepper: str


EXPERIMENTS = [
    ExperimentSpec(
        exp_id="E1_same_phase_no_disturbance",
        label="E1 same-phase hip-knee motion, no disturbance",
        hip=JointPolicy(center=-0.30, amplitude=0.25, frequency_hz=0.25, phase_rad=-math.pi / 2.0),
        knee=JointPolicy(center=0.75, amplitude=0.20, frequency_hz=0.25, phase_rad=-math.pi / 2.0),
    ),
    ExperimentSpec(
        exp_id="E2_anti_phase_hip_disturbance",
        label="E2 anti-phase hip-knee motion, hip disturbance",
        hip=JointPolicy(center=-0.30, amplitude=0.25, frequency_hz=0.25, phase_rad=-math.pi / 2.0),
        knee=JointPolicy(center=0.75, amplitude=0.20, frequency_hz=0.25, phase_rad=math.pi / 2.0),
        disturbance_joints=(HIP,),
        disturbance_torques=(18.0,),
    ),
    ExperimentSpec(
        exp_id="E3_gait_like_load_disturbance",
        label="E3 gait-like hip-knee motion, load disturbance",
        hip=JointPolicy(center=-0.25, amplitude=0.45, frequency_hz=0.35, phase_rad=-math.pi / 2.0),
        knee=JointPolicy(center=0.75, amplitude=0.35, frequency_hz=0.35, phase_rad=0.15),
        disturbance_joints=(HIP, KNEE),
        disturbance_torques=(12.0, -10.0),
    ),
]


CONTROLLERS = [
    ControllerSpec(
        method="G0_no_eid",
        label="G0 no EID",
        config_kind="position_pd",
        stepper="build/Debug/h1_controller_stepper.exe",
    ),
    ControllerSpec(
        method="G1_input_domain",
        label="G1 input-domain EID",
        config_kind="eid",
        stepper="build/Debug/h1_controller_stepper.exe",
    ),
]


def configure_joint(joint_cfg: dict, policy: JointPolicy, enabled: bool) -> None:
    joint_cfg["enabled"] = bool(enabled)
    joint_cfg["policy_source"] = "sine"
    joint_cfg["policy_center"] = float(policy.center)
    joint_cfg["policy_amplitude"] = float(policy.amplitude)
    joint_cfg["policy_frequency_hz"] = float(policy.frequency_hz)
    joint_cfg["policy_phase_rad"] = float(policy.phase_rad)


def build_config(base_config: Path, out_path: Path, experiment: ExperimentSpec, controller: ControllerSpec) -> None:
    cfg = yaml.safe_load(base_config.read_text(encoding="utf-8"))
    cfg["controller"]["kind"] = controller.config_kind
    for raw_joint_id, joint_cfg in cfg["controller"]["joints"].items():
        joint_id = int(raw_joint_id)
        if joint_id == HIP:
            configure_joint(joint_cfg, experiment.hip, True)
        elif joint_id == KNEE:
            configure_joint(joint_cfg, experiment.knee, True)
        else:
            joint_cfg["enabled"] = False

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(
        yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def run_mujoco(config: Path, out_dir: Path, controller: ControllerSpec,
               experiment: ExperimentSpec, duration: float, dt: float) -> None:
    cmd = [
        sys.executable,
        "scripts/run_mujoco.py",
        "--config", str(config),
        "--stepper", controller.stepper,
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
            "--disturbance-ramp", str(experiment.disturbance_ramp),
            "--disturbance-waveform", "smooth_rect",
        ])
    subprocess.run(cmd, check=True)


def applied_tau(df: pd.DataFrame) -> np.ndarray:
    if "tau_sent" in df.columns:
        return df["tau_sent"].to_numpy()
    return (
        df["motor_kp"].to_numpy() * (df["motor_q"].to_numpy() - df["q_actual"].to_numpy())
        + df["motor_kd"].to_numpy() * (df["motor_dq"].to_numpy() - df["dq_actual"].to_numpy())
        + df["motor_tau"].to_numpy()
    )


def load_pair_log(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    df = df[df["joint_id"].isin(JOINTS)].copy()
    df["tau_applied"] = applied_tau(df)
    return df


def pair_arrays(df: pd.DataFrame) -> dict[str, np.ndarray]:
    pivot = df.pivot_table(
        index="t",
        columns="joint_id",
        values=["q_ref_shaped", "q_actual", "tau_applied"],
        aggfunc="first",
    ).dropna()
    t = pivot.index.to_numpy(dtype=float)
    return {
        "t": t,
        "q_ref_h": pivot[("q_ref_shaped", HIP)].to_numpy(dtype=float),
        "q_ref_k": pivot[("q_ref_shaped", KNEE)].to_numpy(dtype=float),
        "q_h": pivot[("q_actual", HIP)].to_numpy(dtype=float),
        "q_k": pivot[("q_actual", KNEE)].to_numpy(dtype=float),
        "tau_h": pivot[("tau_applied", HIP)].to_numpy(dtype=float),
        "tau_k": pivot[("tau_applied", KNEE)].to_numpy(dtype=float),
    }


def metrics_from_arrays(a: dict[str, np.ndarray]) -> dict[str, float]:
    e_h = a["q_ref_h"] - a["q_h"]
    e_k = a["q_ref_k"] - a["q_k"]
    tau_h = a["tau_h"]
    tau_k = a["tau_k"]
    du_h = np.diff(tau_h)
    du_k = np.diff(tau_k)
    return {
        "rmse_h": float(np.sqrt(np.mean(e_h * e_h))),
        "rmse_k": float(np.sqrt(np.mean(e_k * e_k))),
        "rmse_hk": float(np.sqrt(np.mean(e_h * e_h + e_k * e_k))),
        "rmse_coord": float(np.sqrt(np.mean((e_h - e_k) ** 2))),
        "u_rms": float(np.sqrt(np.mean(tau_h * tau_h + tau_k * tau_k))),
        "du_rms": float(np.sqrt(np.mean(du_h * du_h + du_k * du_k))) if len(du_h) else 0.0,
        "tau_h_abs_max": float(np.max(np.abs(tau_h))),
        "tau_k_abs_max": float(np.max(np.abs(tau_k))),
        "samples": int(len(a["t"])),
    }


def write_metrics(out_path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = [
        "experiment", "method", "label", "controller",
        "rmse_h", "rmse_k", "rmse_hk", "rmse_coord",
        "u_rms", "du_rms", "tau_h_abs_max", "tau_k_abs_max", "samples",
    ]
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def plot_experiment(experiment: ExperimentSpec, logs: dict[str, dict[str, np.ndarray]], out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    colors = {
        "G0_no_eid": "#555555",
        "G1_input_domain": "#1b9e77",
    }

    fig, axes = plt.subplots(5, 1, figsize=(12, 14))
    first = next(iter(logs.values()))
    t = first["t"]
    axes[0].plot(t, first["q_ref_h"], color="black", linewidth=1.2, label="hip ref")
    axes[1].plot(t, first["q_ref_k"], color="black", linewidth=1.2, label="knee ref")
    axes[2].plot(first["q_ref_h"], first["q_ref_k"], color="black", linewidth=1.2, label="ref phase")

    for method, a in logs.items():
        color = colors[method]
        axes[0].plot(a["t"], a["q_h"], color=color, linewidth=1.0, label=method)
        axes[1].plot(a["t"], a["q_k"], color=color, linewidth=1.0, label=method)
        axes[2].plot(a["q_h"], a["q_k"], color=color, linewidth=1.0, label=method)
        axes[3].plot(a["t"], a["tau_h"], color=color, linewidth=1.0, label=f"{method} hip")
        axes[3].plot(a["t"], a["tau_k"], color=color, linewidth=1.0, linestyle="--", label=f"{method} knee")
        axes[4].plot(a["t"][1:], np.diff(a["tau_h"]), color=color, linewidth=1.0, label=f"{method} hip")
        axes[4].plot(a["t"][1:], np.diff(a["tau_k"]), color=color, linewidth=1.0, linestyle="--", label=f"{method} knee")

    axes[0].set_ylabel("hip q [rad]")
    axes[1].set_ylabel("knee q [rad]")
    axes[2].set_xlabel("hip q [rad]")
    axes[2].set_ylabel("knee q [rad]")
    axes[3].set_ylabel("tau [Nm]")
    axes[4].set_ylabel("delta tau [Nm/sample]")
    axes[4].set_xlabel("time [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", frameon=False, fontsize=8, ncol=2)
    fig.suptitle(experiment.label)
    fig.tight_layout()
    fig.savefig(out_dir / f"{experiment.exp_id}_comparison.png", dpi=180)
    plt.close(fig)


def plot_metric_bars(metrics: pd.DataFrame, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    specs = [
        ("rmse_h", "hip RMSE [rad]"),
        ("rmse_k", "knee RMSE [rad]"),
        ("rmse_coord", "coord RMSE [rad]"),
        ("u_rms", "torque RMS [Nm]"),
        ("du_rms", "delta torque RMS [Nm/sample]"),
    ]
    colors = {"G0_no_eid": "#555555", "G1_input_domain": "#1b9e77"}
    for exp_id, exp_df in metrics.groupby("experiment", sort=False):
        fig, axes = plt.subplots(1, len(specs), figsize=(15, 4))
        for ax, (col, title) in zip(axes, specs):
            vals = exp_df.set_index("method")[col]
            methods = [c.method for c in CONTROLLERS]
            ax.bar(methods, [vals[m] for m in methods], color=[colors[m] for m in methods], width=0.62)
            ax.set_title(title)
            ax.tick_params(axis="x", labelrotation=35)
            ax.grid(True, axis="y", alpha=0.25)
        fig.suptitle(exp_id)
        fig.tight_layout()
        fig.savefig(out_dir / f"{exp_id}_metrics.png", dpi=180)
        plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-config", type=Path, default=Path("config/h1_hip_knee_dual_tuned.yaml"))
    parser.add_argument("--out-dir", type=Path, default=Path("analysis_artifacts/hip_knee_domain_experiment"))
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--skip-runs", action="store_true")
    args = parser.parse_args()

    config_dir = args.out_dir / "configs"
    runs_dir = args.out_dir / "runs"
    fig_dir = args.out_dir / "figures"
    rows: list[dict[str, object]] = []

    for experiment in EXPERIMENTS:
        logs: dict[str, dict[str, np.ndarray]] = {}
        for controller in CONTROLLERS:
            run_name = f"{experiment.exp_id}__{controller.method}"
            config_path = config_dir / f"{run_name}.yaml"
            out_dir = runs_dir / run_name
            build_config(args.base_config, config_path, experiment, controller)
            if not args.skip_runs:
                run_mujoco(config_path, out_dir, controller, experiment, args.duration, args.dt)
            arrays = pair_arrays(load_pair_log(out_dir / "mujoco_closed_loop_log.csv"))
            logs[controller.method] = arrays
            row: dict[str, object] = {
                "experiment": experiment.exp_id,
                "method": controller.method,
                "label": experiment.label,
                "controller": controller.label,
            }
            row.update(metrics_from_arrays(arrays))
            rows.append(row)
        plot_experiment(experiment, logs, fig_dir)

    metrics_path = args.out_dir / "hip_knee_domain_metrics.csv"
    write_metrics(metrics_path, rows)
    plot_metric_bars(pd.DataFrame(rows), fig_dir)

    print(f"metrics={metrics_path}")
    print(f"figures={fig_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
