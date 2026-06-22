#!/usr/bin/env python3
"""Run a small MuJoCo shadow version of the first hardware experiment plan.

This is intentionally a smoke/checkout script, not the full P1/P2/P4/P3 matrix.
It runs P1 plus one or two P2 disturbance cases with the same C++ controller
stepper that the MuJoCo backend and hardware entry use.
"""

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
JOINT_LABEL = {HIP: "hip", KNEE: "knee"}


@dataclass(frozen=True)
class JointPolicy:
    center: float
    amplitude: float
    frequency_hz: float
    phase_rad: float


@dataclass(frozen=True)
class TrialSpec:
    trial_id: str
    plan_id: str
    label: str
    hip: JointPolicy
    knee: JointPolicy
    disturbed_joint: int | None = None
    disturbance_torque: float = 0.0
    disturbance_start: float = 2.5
    disturbance_end: float = 3.0
    disturbance_ramp: float = 0.1


@dataclass(frozen=True)
class ControllerSpec:
    controller_id: str
    label: str
    kind: str
    color: str


CONTROLLERS = [
    ControllerSpec("pd", "PD", "position_pd", "#5F6368"),
    ControllerSpec("input_eid", "Input-domain EID", "eid", "#1B9E77"),
]


def trial_specs(include_p2k: bool) -> list[TrialSpec]:
    same_phase = -math.pi / 2.0
    anti_phase_knee = math.pi / 2.0
    p1 = TrialSpec(
        trial_id="P1_same_phase_no_disturbance",
        plan_id="P1",
        label="P1 same-phase no disturbance",
        hip=JointPolicy(center=-0.30, amplitude=0.10, frequency_hz=0.25, phase_rad=same_phase),
        knee=JointPolicy(center=0.75, amplitude=0.08, frequency_hz=0.25, phase_rad=same_phase),
    )
    p2h = TrialSpec(
        trial_id="P2H_anti_phase_hip_disturbance",
        plan_id="P2-H",
        label="P2-H anti-phase hip software torque",
        hip=JointPolicy(center=-0.30, amplitude=0.10, frequency_hz=0.25, phase_rad=same_phase),
        knee=JointPolicy(center=0.75, amplitude=0.08, frequency_hz=0.25, phase_rad=anti_phase_knee),
        disturbed_joint=HIP,
        disturbance_torque=12.0,
    )
    trials = [p1, p2h]
    if include_p2k:
        trials.append(
            TrialSpec(
                trial_id="P2K_anti_phase_knee_disturbance",
                plan_id="P2-K",
                label="P2-K anti-phase knee software torque",
                hip=p2h.hip,
                knee=p2h.knee,
                disturbed_joint=KNEE,
                disturbance_torque=-10.0,
            )
        )
    return trials


def configure_joint(joint_cfg: dict, policy: JointPolicy, enabled: bool) -> None:
    joint_cfg["enabled"] = bool(enabled)
    joint_cfg["policy_source"] = "sine"
    joint_cfg["policy_interpolation"] = "open_loop"
    joint_cfg["policy_center"] = float(policy.center)
    joint_cfg["policy_amplitude"] = float(policy.amplitude)
    joint_cfg["policy_frequency_hz"] = float(policy.frequency_hz)
    joint_cfg["policy_phase_rad"] = float(policy.phase_rad)


def build_config(base_config: Path,
                 out_path: Path,
                 trial: TrialSpec,
                 controller: ControllerSpec) -> None:
    cfg = yaml.safe_load(base_config.read_text(encoding="utf-8"))
    cfg["controller"]["kind"] = controller.kind
    cfg["mock_duration"] = 0.0
    for raw_joint_id, joint_cfg in cfg["controller"]["joints"].items():
        joint_id = int(raw_joint_id)
        if joint_id == HIP:
            configure_joint(joint_cfg, trial.hip, True)
        elif joint_id == KNEE:
            configure_joint(joint_cfg, trial.knee, True)
        else:
            joint_cfg["enabled"] = False

    cfg.pop("software_disturbance", None)
    software_disturbance_lines: list[str] = []
    if trial.disturbed_joint is not None:
        software_disturbance_lines = [
            "",
            "software_disturbance:",
            "  enabled: true",
            f"  joints: [{trial.disturbed_joint}]",
            f"  torques: [{float(trial.disturbance_torque)}]",
            f"  start_s: {float(trial.disturbance_start)}",
            f"  end_s: {float(trial.disturbance_end)}",
            f"  ramp_s: {float(trial.disturbance_ramp)}",
            "  waveform: smooth_rect",
        ]

    out_path.parent.mkdir(parents=True, exist_ok=True)
    text = yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True)
    if software_disturbance_lines:
        text = text.rstrip() + "\n" + "\n".join(software_disturbance_lines) + "\n"
    out_path.write_text(text, encoding="utf-8")


def run_mujoco(config: Path,
               out_dir: Path,
               duration: float,
               dt: float,
               log_hz: float,
               stepper: Path) -> None:
    cmd = [
        sys.executable,
        "scripts/run_mujoco.py",
        "--config", str(config),
        "--stepper", str(stepper),
        "--out-dir", str(out_dir),
        "--duration", str(duration),
        "--dt", str(dt),
        "--log-hz", str(log_hz),
        "--export-summary",
    ]
    subprocess.run(cmd, check=True)


def load_pair_log(log_path: Path) -> dict[str, np.ndarray]:
    df = pd.read_csv(log_path)
    df = df[df["joint_id"].isin(JOINTS)].copy()
    for col in ["tau_controller", "tau_disturbance", "tau_sent", "saturation_flag"]:
        if col not in df.columns:
            df[col] = 0.0
    pivot = df.pivot_table(
        index="t",
        columns="joint_id",
        values=[
            "q_ref_shaped", "dq_ref_shaped", "q_actual", "dq_actual",
            "tau_controller", "tau_disturbance", "tau_sent", "saturation_flag",
        ],
        aggfunc="first",
    ).dropna()
    t = pivot.index.to_numpy(dtype=float)
    return {
        "t": t,
        "q_ref_h": pivot[("q_ref_shaped", HIP)].to_numpy(dtype=float),
        "q_ref_k": pivot[("q_ref_shaped", KNEE)].to_numpy(dtype=float),
        "dq_ref_h": pivot[("dq_ref_shaped", HIP)].to_numpy(dtype=float),
        "dq_ref_k": pivot[("dq_ref_shaped", KNEE)].to_numpy(dtype=float),
        "q_h": pivot[("q_actual", HIP)].to_numpy(dtype=float),
        "q_k": pivot[("q_actual", KNEE)].to_numpy(dtype=float),
        "dq_h": pivot[("dq_actual", HIP)].to_numpy(dtype=float),
        "dq_k": pivot[("dq_actual", KNEE)].to_numpy(dtype=float),
        "tau_controller_h": pivot[("tau_controller", HIP)].to_numpy(dtype=float),
        "tau_controller_k": pivot[("tau_controller", KNEE)].to_numpy(dtype=float),
        "tau_disturbance_h": pivot[("tau_disturbance", HIP)].to_numpy(dtype=float),
        "tau_disturbance_k": pivot[("tau_disturbance", KNEE)].to_numpy(dtype=float),
        "tau_h": pivot[("tau_sent", HIP)].to_numpy(dtype=float),
        "tau_k": pivot[("tau_sent", KNEE)].to_numpy(dtype=float),
        "saturation_h": pivot[("saturation_flag", HIP)].to_numpy(dtype=float),
        "saturation_k": pivot[("saturation_flag", KNEE)].to_numpy(dtype=float),
    }


def rms(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(x * x))) if x.size else 0.0


def window_mask(t: np.ndarray, start: float, end: float, closed_end: bool = True) -> np.ndarray:
    if closed_end:
        return (t >= start) & (t <= end)
    return (t >= start) & (t < end)


def metrics_from_arrays(a: dict[str, np.ndarray], trial: TrialSpec) -> dict[str, float]:
    t = a["t"]
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 1.0
    e_h = a["q_ref_h"] - a["q_h"]
    e_k = a["q_ref_k"] - a["q_k"]
    e_coord = e_h - e_k
    tau_h = a["tau_h"]
    tau_k = a["tau_k"]
    tau_rate_h = np.diff(tau_h) / dt if len(tau_h) > 1 else np.array([])
    tau_rate_k = np.diff(tau_k) / dt if len(tau_k) > 1 else np.array([])

    row = {
        "rmse_h": rms(e_h),
        "rmse_k": rms(e_k),
        "rmse_coord": rms(e_coord),
        "coord_peak": float(np.max(np.abs(e_coord))) if e_coord.size else 0.0,
        "tau_rms": float(np.sqrt(np.mean(tau_h * tau_h + tau_k * tau_k))) if tau_h.size else 0.0,
        "tau_rate_rms": float(np.sqrt(np.mean(tau_rate_h * tau_rate_h + tau_rate_k * tau_rate_k)))
                        if tau_rate_h.size else 0.0,
        "tau_h_abs_max": float(np.max(np.abs(tau_h))) if tau_h.size else 0.0,
        "tau_k_abs_max": float(np.max(np.abs(tau_k))) if tau_k.size else 0.0,
        "saturation_count": int(np.sum(a["saturation_h"]) + np.sum(a["saturation_k"])),
        "samples": int(len(t)),
    }

    if trial.disturbed_joint is not None:
        pre = window_mask(t, trial.disturbance_start - 0.5, trial.disturbance_start, closed_end=False)
        during = window_mask(t, trial.disturbance_start, trial.disturbance_end)
        post = window_mask(t, trial.disturbance_end, trial.disturbance_end + 1.0)
        for suffix, mask in [("pre", pre), ("during", during), ("post", post)]:
            row[f"rmse_h_{suffix}"] = rms(e_h[mask])
            row[f"rmse_k_{suffix}"] = rms(e_k[mask])
            row[f"rmse_coord_{suffix}"] = rms(e_coord[mask])
        row["delta_rmse_h_during"] = row["rmse_h_during"] - row["rmse_h_pre"]
        row["delta_rmse_k_during"] = row["rmse_k_during"] - row["rmse_k_pre"]
        row["delta_rmse_coord_during"] = row["rmse_coord_during"] - row["rmse_coord_pre"]
        row["coord_peak_during"] = float(np.max(np.abs(e_coord[during]))) if np.any(during) else 0.0
        if trial.disturbed_joint == HIP:
            denom = max(row["rmse_h_during"], 1.0e-9)
            row["cross_ratio"] = row["rmse_k_during"] / denom
        else:
            denom = max(row["rmse_k_during"], 1.0e-9)
            row["cross_ratio"] = row["rmse_h_during"] / denom
    return row


def add_suppression_ratios(rows: list[dict[str, object]]) -> None:
    by_trial: dict[str, dict[str, dict[str, object]]] = {}
    for row in rows:
        by_trial.setdefault(str(row["trial_id"]), {})[str(row["controller_id"])] = row
    for trial_rows in by_trial.values():
        pd_row = trial_rows.get("pd")
        eid_row = trial_rows.get("input_eid")
        if not pd_row or not eid_row:
            continue
        for row in [pd_row, eid_row]:
            row["S_coord"] = ""
            row["S_cross"] = ""
        if "rmse_coord_during" in pd_row and float(pd_row["rmse_coord_during"]) > 0.0:
            eid_row["S_coord"] = float(eid_row["rmse_coord_during"]) / float(pd_row["rmse_coord_during"])
        if "cross_ratio" in pd_row and float(pd_row["cross_ratio"]) > 0.0:
            eid_row["S_cross"] = float(eid_row["cross_ratio"]) / float(pd_row["cross_ratio"])


def write_metrics(path: Path, rows: list[dict[str, object]]) -> None:
    all_fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in all_fields:
                all_fields.append(key)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=all_fields)
        writer.writeheader()
        writer.writerows(rows)


def shade_disturbance(ax: plt.Axes, trial: TrialSpec) -> None:
    if trial.disturbed_joint is not None:
        ax.axvspan(trial.disturbance_start, trial.disturbance_end, color="#D9D9D9", alpha=0.45, lw=0)


def plot_trial_comparison(trial: TrialSpec,
                          logs: dict[str, dict[str, np.ndarray]],
                          controllers: list[ControllerSpec],
                          out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(5, 1, figsize=(11.0, 12.0), sharex=False)
    first = logs[controllers[0].controller_id]
    axes[0].plot(first["t"], first["q_ref_h"], color="black", lw=1.2, label="hip ref")
    axes[1].plot(first["t"], first["q_ref_k"], color="black", lw=1.2, label="knee ref")
    axes[4].plot(first["q_ref_h"], first["q_ref_k"], color="black", lw=1.2, label="ref")

    for controller in controllers:
        a = logs[controller.controller_id]
        e_h = a["q_ref_h"] - a["q_h"]
        e_k = a["q_ref_k"] - a["q_k"]
        e_coord = e_h - e_k
        color = controller.color
        axes[0].plot(a["t"], a["q_h"], color=color, lw=1.0, label=controller.label)
        axes[1].plot(a["t"], a["q_k"], color=color, lw=1.0, label=controller.label)
        axes[2].plot(a["t"], e_h, color=color, lw=0.9, label=f"{controller.label} hip")
        axes[2].plot(a["t"], e_k, color=color, lw=0.9, ls="--", label=f"{controller.label} knee")
        axes[2].plot(a["t"], e_coord, color=color, lw=1.2, alpha=0.75, ls=":", label=f"{controller.label} coord")
        axes[3].plot(a["t"], a["tau_h"], color=color, lw=0.9, label=f"{controller.label} hip sent")
        axes[3].plot(a["t"], a["tau_k"], color=color, lw=0.9, ls="--", label=f"{controller.label} knee sent")
        if np.max(np.abs(a["tau_disturbance_h"])) > 0:
            axes[3].plot(a["t"], a["tau_disturbance_h"], color="#C43C39", lw=1.1, alpha=0.85, label="hip disturbance")
        if np.max(np.abs(a["tau_disturbance_k"])) > 0:
            axes[3].plot(a["t"], a["tau_disturbance_k"], color="#C43C39", lw=1.1, alpha=0.85, label="knee disturbance")
        axes[4].plot(a["q_h"], a["q_k"], color=color, lw=1.0, label=controller.label)

    axes[0].set_ylabel("hip q [rad]")
    axes[1].set_ylabel("knee q [rad]")
    axes[2].set_ylabel("error [rad]")
    axes[3].set_ylabel("torque [Nm]")
    axes[3].set_xlabel("time [s]")
    axes[4].set_xlabel("hip q [rad]")
    axes[4].set_ylabel("knee q [rad]")
    for ax in axes[:4]:
        shade_disturbance(ax, trial)
    for ax in axes:
        ax.grid(True, alpha=0.24)
        ax.legend(loc="best", frameon=False, fontsize=8, ncol=2)
    fig.suptitle(trial.label)
    fig.tight_layout()
    fig.savefig(out_dir / f"{trial.trial_id}_comparison.png", dpi=180)
    plt.close(fig)


def plot_window_zoom(trial: TrialSpec,
                     logs: dict[str, dict[str, np.ndarray]],
                     controllers: list[ControllerSpec],
                     out_dir: Path) -> None:
    if trial.disturbed_joint is None:
        return
    out_dir.mkdir(parents=True, exist_ok=True)
    t0 = trial.disturbance_start - 0.5
    t1 = trial.disturbance_end + 1.0
    fig, axes = plt.subplots(3, 1, figsize=(11.0, 7.2), sharex=True)
    for controller in controllers:
        a = logs[controller.controller_id]
        mask = (a["t"] >= t0) & (a["t"] <= t1)
        e_h = a["q_ref_h"] - a["q_h"]
        e_k = a["q_ref_k"] - a["q_k"]
        e_coord = e_h - e_k
        color = controller.color
        axes[0].plot(a["t"][mask], e_h[mask], color=color, lw=1.0, label=f"{controller.label} hip")
        axes[0].plot(a["t"][mask], e_k[mask], color=color, lw=1.0, ls="--", label=f"{controller.label} knee")
        axes[1].plot(a["t"][mask], e_coord[mask], color=color, lw=1.2, label=controller.label)
        axes[2].plot(a["t"][mask], a["tau_h"][mask], color=color, lw=1.0, label=f"{controller.label} hip")
        axes[2].plot(a["t"][mask], a["tau_k"][mask], color=color, lw=1.0, ls="--", label=f"{controller.label} knee")
        tau_dist = a["tau_disturbance_h"] if trial.disturbed_joint == HIP else a["tau_disturbance_k"]
        if np.max(np.abs(tau_dist)) > 0:
            axes[2].plot(a["t"][mask], tau_dist[mask], color="#C43C39", lw=1.2, alpha=0.85, label="disturbance")

    axes[0].set_ylabel("joint error [rad]")
    axes[1].set_ylabel("coord error [rad]")
    axes[2].set_ylabel("torque [Nm]")
    axes[2].set_xlabel("time [s]")
    for ax in axes:
        shade_disturbance(ax, trial)
        ax.grid(True, alpha=0.24)
        ax.legend(loc="best", frameon=False, fontsize=8, ncol=2)
    fig.suptitle(f"{trial.label} disturbance-window zoom")
    fig.tight_layout()
    fig.savefig(out_dir / f"{trial.trial_id}_window_zoom.png", dpi=180)
    plt.close(fig)


def plot_metric_bars(metrics: pd.DataFrame, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    specs = [
        ("rmse_h", "hip RMSE [rad]"),
        ("rmse_k", "knee RMSE [rad]"),
        ("rmse_coord", "coord RMSE [rad]"),
        ("tau_rms", "torque RMS [Nm]"),
        ("tau_rate_rms", "torque rate RMS [Nm/s]"),
    ]
    fig, axes = plt.subplots(len(specs), 1, figsize=(10.5, 12.0), sharex=True)
    x_labels = []
    x = np.arange(len(metrics))
    colors = [next(c.color for c in CONTROLLERS if c.controller_id == cid) for cid in metrics["controller_id"]]
    for ax, (metric, ylabel) in zip(axes, specs):
        ax.bar(x, metrics[metric], color=colors, edgecolor="black", linewidth=0.35)
        ax.set_ylabel(ylabel)
        ax.grid(True, axis="y", alpha=0.24)
    for _, row in metrics.iterrows():
        x_labels.append(f"{row['plan_id']}\n{row['controller_id']}")
    axes[-1].set_xticks(x)
    axes[-1].set_xticklabels(x_labels, rotation=0)
    fig.suptitle("MuJoCo hardware-plan smoke metrics")
    fig.tight_layout()
    fig.savefig(out_dir / "smoke_metric_bars.png", dpi=180)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-config", type=Path, default=Path("config/h1_hip_knee_dual_tuned.yaml"))
    parser.add_argument("--out-dir", type=Path, default=Path("analysis_artifacts/mujoco_hardware_plan_smoke"))
    parser.add_argument("--stepper", type=Path, default=Path("build/Debug/h1_controller_stepper.exe"))
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--log-hz", type=float, default=500.0)
    parser.add_argument("--include-p2k", action="store_true")
    parser.add_argument("--skip-runs", action="store_true")
    args = parser.parse_args()

    config_dir = args.out_dir / "configs"
    runs_dir = args.out_dir / "runs"
    fig_dir = args.out_dir / "figures"
    rows: list[dict[str, object]] = []

    trials = trial_specs(args.include_p2k)
    for trial in trials:
        logs: dict[str, dict[str, np.ndarray]] = {}
        for controller in CONTROLLERS:
            run_name = f"{trial.trial_id}__{controller.controller_id}"
            config_path = config_dir / f"{run_name}.yaml"
            run_dir = runs_dir / run_name
            build_config(args.base_config, config_path, trial, controller)
            if not args.skip_runs:
                run_mujoco(config_path, run_dir, args.duration, args.dt, args.log_hz, args.stepper)
            arr = load_pair_log(run_dir / "mujoco_closed_loop_log.csv")
            logs[controller.controller_id] = arr
            row: dict[str, object] = {
                "trial_id": trial.trial_id,
                "plan_id": trial.plan_id,
                "trial_label": trial.label,
                "controller_id": controller.controller_id,
                "controller": controller.label,
                "disturbed_joint": "" if trial.disturbed_joint is None else JOINT_LABEL[trial.disturbed_joint],
                "disturbance_torque": "" if trial.disturbed_joint is None else trial.disturbance_torque,
                "disturbance_start": "" if trial.disturbed_joint is None else trial.disturbance_start,
                "disturbance_end": "" if trial.disturbed_joint is None else trial.disturbance_end,
            }
            row.update(metrics_from_arrays(arr, trial))
            rows.append(row)
        plot_trial_comparison(trial, logs, CONTROLLERS, fig_dir)
        plot_window_zoom(trial, logs, CONTROLLERS, fig_dir)

    add_suppression_ratios(rows)
    metrics_path = args.out_dir / "mujoco_hardware_plan_smoke_metrics.csv"
    write_metrics(metrics_path, rows)
    plot_metric_bars(pd.DataFrame(rows), fig_dir)

    print(f"metrics={metrics_path}")
    print(f"figures={fig_dir}")
    print(f"runs={runs_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
