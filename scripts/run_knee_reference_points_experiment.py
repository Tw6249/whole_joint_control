#!/usr/bin/env python3
"""Compare 1-, 2-, and 4-point policy velocity estimates on the right knee."""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


JOINT_ID = 2


def build_config(base_text: str, reference_points: int) -> str:
    lines = base_text.splitlines()
    out: list[str] = []
    in_controller = False
    in_defaults = False
    in_joints = False
    current_joint: int | None = None
    inserted_reference_points = False

    for raw in lines:
        stripped = raw.strip()
        indent = len(raw) - len(raw.lstrip(" "))

        if indent == 0:
            in_controller = stripped == "controller:"
            in_defaults = False
            in_joints = False
            current_joint = None

        if in_controller and indent == 2 and stripped.startswith("kind:"):
            out.append("  kind: position_pd")
            continue

        if in_controller and indent == 2 and stripped == "defaults:":
            in_defaults = True
            in_joints = False
            current_joint = None
            inserted_reference_points = False
            out.append(raw)
            continue

        if in_controller and indent == 2 and stripped == "joints:":
            if in_defaults and not inserted_reference_points:
                out.append(f"    policy_reference_points: {reference_points}")
                inserted_reference_points = True
            in_defaults = False
            in_joints = True
            current_joint = None
            out.append(raw)
            continue

        if in_defaults and indent == 4 and stripped.startswith("policy_reference_points:"):
            out.append(f"    policy_reference_points: {reference_points}")
            inserted_reference_points = True
            continue

        if in_defaults and indent == 4 and stripped.startswith("policy_source:") and not inserted_reference_points:
            out.append(f"    policy_reference_points: {reference_points}")
            inserted_reference_points = True
            out.append(raw)
            continue

        if in_controller and in_joints and indent == 4 and stripped.endswith(":"):
            try:
                current_joint = int(stripped[:-1])
            except ValueError:
                current_joint = None
            out.append(raw)
            continue

        if in_controller and in_joints and indent == 6 and stripped.startswith("enabled:"):
            out.append(f"      enabled: {'true' if current_joint == JOINT_ID else 'false'}")
            continue

        out.append(raw)

    return "\n".join(out) + "\n"


def run_mujoco(config: Path, out_dir: Path, duration: float, dt: float) -> None:
    cmd = [
        sys.executable,
        "scripts/run_mujoco.py",
        "--config",
        str(config),
        "--out-dir",
        str(out_dir),
        "--duration",
        str(duration),
        "--dt",
        str(dt),
        "--export-summary",
    ]
    subprocess.run(cmd, check=True)


def load_joint_log(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    return df[df["joint_id"] == JOINT_ID].copy()


def applied_tau(df: pd.DataFrame) -> np.ndarray:
    if "tau_sent" in df.columns:
        return df["tau_sent"].to_numpy()
    return (
        df["motor_kp"].to_numpy() * (df["motor_q"].to_numpy() - df["q_actual"].to_numpy())
        + df["motor_kd"].to_numpy() * (df["motor_dq"].to_numpy() - df["dq_actual"].to_numpy())
        + df["motor_tau"].to_numpy()
    )


def metrics(df: pd.DataFrame) -> dict[str, float]:
    dt = float(np.median(np.diff(df["t"])))
    q_err = df["q_ref_shaped"].to_numpy() - df["q_actual"].to_numpy()
    dq_err = df["dq_ref_shaped"].to_numpy() - df["dq_actual"].to_numpy()
    dq_ref = df["dq_ref_shaped"].to_numpy()
    tau = applied_tau(df)
    tau_rate = np.diff(tau) / dt
    dq_ref_rate = np.diff(dq_ref) / dt
    return {
        "q_rmse": float(np.sqrt(np.mean(q_err * q_err))),
        "dq_rmse": float(np.sqrt(np.mean(dq_err * dq_err))),
        "dq_ref_abs_max": float(np.max(np.abs(dq_ref))),
        "dq_ref_rate_rms": float(np.sqrt(np.mean(dq_ref_rate * dq_ref_rate))),
        "tau_abs_max": float(np.max(np.abs(tau))),
        "tau_rms": float(np.sqrt(np.mean(tau * tau))),
        "tau_rate_rms": float(np.sqrt(np.mean(tau_rate * tau_rate))),
    }


def write_metrics(path: Path, logs: dict[str, pd.DataFrame]) -> None:
    rows = []
    for name, df in logs.items():
        row = {"method": name}
        row.update(metrics(df))
        rows.append(row)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_results(logs: dict[str, pd.DataFrame], metrics_path: Path, fig_dir: Path) -> None:
    fig_dir.mkdir(parents=True, exist_ok=True)
    colors = {"points_1": "#c4512b", "points_2": "#5b6fb9", "points_4": "#0f7f7b"}
    labels = {"points_1": "1 point", "points_2": "2 points", "points_4": "4 points"}

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    first = next(iter(logs.values()))
    axes[0].plot(first["t"], first["q_ref_shaped"], color="black", linewidth=1.0, label="q ref")
    for name, df in logs.items():
        axes[0].plot(df["t"], df["q_actual"], color=colors[name], linewidth=1.0, label=f"{labels[name]} actual")
    axes[0].set_ylabel("q [rad]")
    axes[0].legend(loc="upper right", frameon=False, ncol=4)

    for name, df in logs.items():
        axes[1].plot(df["t"], df["dq_ref_shaped"], color=colors[name], linewidth=1.0, label=f"{labels[name]} dq ref")
    axes[1].plot(first["t"], first["dq_actual"], color="#555555", linewidth=0.8, alpha=0.6, label="actual dq")
    axes[1].set_ylabel("dq [rad/s]")
    axes[1].legend(loc="upper right", frameon=False, ncol=4)

    for name, df in logs.items():
        axes[2].plot(df["t"], applied_tau(df), color=colors[name], linewidth=1.0, label=f"{labels[name]} tau")
    axes[2].set_ylabel("tau [Nm]")
    axes[2].set_xlabel("time [s]")
    axes[2].legend(loc="upper right", frameon=False, ncol=3)

    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle("Right knee MuJoCo: 1-, 2-, and 4-point policy references")
    fig.tight_layout()
    fig.savefig(fig_dir / "right_knee_timeseries.png", dpi=180)
    plt.close(fig)

    metric_df = pd.read_csv(metrics_path).set_index("method")
    fig, axes = plt.subplots(1, 4, figsize=(13, 4))
    specs = [
        ("q_rmse", "q RMSE [rad]", "linear"),
        ("dq_ref_abs_max", "max |dq ref| [rad/s]", "linear"),
        ("tau_rms", "tau RMS [Nm]", "linear"),
        ("tau_rate_rms", "tau rate RMS [Nm/s]", "linear"),
    ]
    rows = ["points_1", "points_2", "points_4"]
    bar_labels = ["1 point", "2 points", "4 points"]
    for ax, (col, title, scale) in zip(axes, specs):
        vals = [metric_df.loc[row, col] for row in rows]
        ax.bar(bar_labels, vals, color=[colors[row] for row in rows], width=0.62)
        ax.set_title(title)
        ax.set_yscale(scale)
        ax.grid(True, axis="y", alpha=0.25)
        for i, val in enumerate(vals):
            ax.text(i, val, f"{val:.3g}", ha="center", va="bottom", fontsize=9)
    fig.suptitle("Right knee reference point comparison metrics")
    fig.tight_layout()
    fig.savefig(fig_dir / "right_knee_metrics.png", dpi=180)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-config", type=Path, default=Path("config/h1_full_body_mujoco_fit.yaml"))
    parser.add_argument("--out-dir", type=Path, default=Path("analysis_artifacts/right_knee_reference_points"))
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--skip-runs", action="store_true")
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    base_text = args.base_config.read_text(encoding="utf-8")
    configs = {}
    for points in [1, 2, 4]:
        config = args.out_dir / f"right_knee_points_{points}.yaml"
        config.write_text(build_config(base_text, points), encoding="utf-8")
        configs[f"points_{points}"] = config

    if not args.skip_runs:
        for name, config in configs.items():
            run_mujoco(config, args.out_dir / name, args.duration, args.dt)

    logs = {
        name: load_joint_log(args.out_dir / name / "mujoco_closed_loop_log.csv")
        for name in configs
    }
    metrics_path = args.out_dir / "right_knee_reference_points_metrics.csv"
    write_metrics(metrics_path, logs)
    plot_results(logs, metrics_path, args.out_dir / "figures")

    print(f"metrics={metrics_path}")
    print(f"figure={args.out_dir / 'figures' / 'right_knee_timeseries.png'}")
    print(f"figure={args.out_dir / 'figures' / 'right_knee_metrics.png'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
