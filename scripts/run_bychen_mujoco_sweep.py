#!/usr/bin/env python3
"""Run the MuJoCo 2x2 asymmetric observer-gain sweep for the Chen reply."""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml

from report_paths import analysis_report_dir, repo_relpath


REPO_ROOT = Path(__file__).resolve().parents[1]
HIP = 1
KNEE = 2
JOINTS = [HIP, KNEE]
JOINT_NAMES = {HIP: "Hip", KNEE: "Knee"}
NOMINAL_OBSERVER_Q = 0.8
NOMINAL_OBSERVER_DQ = 0.2
U1_KU_Q = 6.0
U1_KU_DQ = 0.5


@dataclass(frozen=True)
class Candidate:
    case_id: str
    label: str
    hip_so: float
    knee_so: float
    purpose: str


CANDIDATES = [
    Candidate("A", "A baseline", 1.00, 1.00, "O4+U1 baseline"),
    Candidate("B", "B hip-high", 1.25, 1.00, "increase hip observer only"),
    Candidate("C", "C knee-low", 1.00, 0.75, "decrease knee observer only"),
    Candidate("D", "D asymmetric", 1.25, 0.75, "Chen proposed direction"),
]


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


def set_joint_gain(joint_cfg: dict[str, Any], so: float) -> None:
    joint_cfg["observer_gain_q"] = float(NOMINAL_OBSERVER_Q * so)
    joint_cfg["observer_gain_dq"] = float(NOMINAL_OBSERVER_DQ * so)
    joint_cfg["ku_q"] = float(U1_KU_Q)
    joint_cfg["ku_dq"] = float(U1_KU_DQ)


def build_config(base_config: Path, out_path: Path, candidate: Candidate) -> None:
    cfg = yaml.safe_load(base_config.read_text(encoding="utf-8"))
    cfg.setdefault("experiment", {})
    cfg["experiment"]["id"] = "P2D_MuJoCo"
    cfg["experiment"]["condition"] = (
        f"bychen_mujoco_{candidate.case_id}_"
        f"so_h{candidate.hip_so:g}_so_k{candidate.knee_so:g}_u1"
    )
    cfg["experiment"]["disturbance_method"] = "mujoco_input_torque_half_cosine"
    cfg["experiment"]["disturbance_target"] = "RightHipPitch,RightKnee"

    defaults = cfg["controller"]["defaults"]
    defaults["observer_gain_q"] = float(NOMINAL_OBSERVER_Q)
    defaults["observer_gain_dq"] = float(NOMINAL_OBSERVER_DQ)
    defaults["ku_q"] = float(U1_KU_Q)
    defaults["ku_dq"] = float(U1_KU_DQ)

    hip_cfg = yaml_joint(cfg, HIP)
    knee_cfg = yaml_joint(cfg, KNEE)
    hip_cfg["enabled"] = True
    knee_cfg["enabled"] = True
    set_joint_gain(hip_cfg, candidate.hip_so)
    set_joint_gain(knee_cfg, candidate.knee_so)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(
        yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def run_mujoco(config: Path, out_dir: Path, args: argparse.Namespace) -> bool:
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
    try:
        subprocess.run(cmd, check=True, cwd=REPO_ROOT)
        return True
    except subprocess.CalledProcessError as exc:
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "run_error.txt").write_text(str(exc), encoding="utf-8")
        return False


def applied_tau(df: pd.DataFrame) -> np.ndarray:
    if "tau_applied" in df.columns:
        return df["tau_applied"].to_numpy(dtype=float)
    return (
        df["motor_kp"].to_numpy(dtype=float) * (
            df["motor_q"].to_numpy(dtype=float) - df["q_actual"].to_numpy(dtype=float)
        )
        + df["motor_kd"].to_numpy(dtype=float) * (
            df["motor_dq"].to_numpy(dtype=float) - df["dq_actual"].to_numpy(dtype=float)
        )
        + df["motor_tau"].to_numpy(dtype=float)
    )


def rms(x: np.ndarray) -> float:
    if len(x) == 0:
        return float("nan")
    return float(np.sqrt(np.mean(np.square(x))))


def load_arrays(log_path: Path) -> dict[str, np.ndarray]:
    df = pd.read_csv(log_path)
    df = df[df["joint_id"].isin(JOINTS)].copy()
    df["tau_cmd"] = applied_tau(df)
    if "tau_dist" not in df.columns:
        df["tau_dist"] = 0.0
    if "disturbance_scale" not in df.columns:
        df["disturbance_scale"] = 0.0
    if "tau_total" not in df.columns:
        df["tau_total"] = df["tau_cmd"] + df.get("tau_dist", 0.0)
    if "eta_u" not in df.columns:
        df["eta_u"] = 0.0

    pivot = df.pivot_table(
        index="t",
        columns="joint_id",
        values=[
            "q_ref_shaped", "q_actual", "tau_cmd", "tau_total",
            "tau_dist", "eta_u", "disturbance_scale", "flags", "joint_flags",
        ],
        aggfunc="first",
    ).dropna()
    t = pivot.index.to_numpy(dtype=float)
    return {
        "t": t,
        "q_ref_h": pivot[("q_ref_shaped", HIP)].to_numpy(dtype=float),
        "q_ref_k": pivot[("q_ref_shaped", KNEE)].to_numpy(dtype=float),
        "q_h": pivot[("q_actual", HIP)].to_numpy(dtype=float),
        "q_k": pivot[("q_actual", KNEE)].to_numpy(dtype=float),
        "tau_cmd_h": pivot[("tau_cmd", HIP)].to_numpy(dtype=float),
        "tau_cmd_k": pivot[("tau_cmd", KNEE)].to_numpy(dtype=float),
        "tau_total_h": pivot[("tau_total", HIP)].to_numpy(dtype=float),
        "tau_total_k": pivot[("tau_total", KNEE)].to_numpy(dtype=float),
        "tau_dist_h": pivot[("tau_dist", HIP)].to_numpy(dtype=float),
        "tau_dist_k": pivot[("tau_dist", KNEE)].to_numpy(dtype=float),
        "eta_u_h": pivot[("eta_u", HIP)].to_numpy(dtype=float),
        "eta_u_k": pivot[("eta_u", KNEE)].to_numpy(dtype=float),
        "disturbance_scale": pivot[("disturbance_scale", HIP)].to_numpy(dtype=float),
        "flags_h": pivot[("flags", HIP)].to_numpy(dtype=float),
        "flags_k": pivot[("flags", KNEE)].to_numpy(dtype=float),
        "joint_flags_h": pivot[("joint_flags", HIP)].to_numpy(dtype=float),
        "joint_flags_k": pivot[("joint_flags", KNEE)].to_numpy(dtype=float),
    }


def metrics_from_arrays(a: dict[str, np.ndarray], args: argparse.Namespace) -> dict[str, float | int]:
    t = a["t"]
    mask = (t >= args.disturbance_start) & (t < args.disturbance_end)
    if not np.any(mask):
        mask = np.ones_like(t, dtype=bool)

    e_h = a["q_ref_h"] - a["q_h"]
    e_k = a["q_ref_k"] - a["q_k"]
    e_coord = e_h - e_k
    dt = float(np.median(np.diff(t))) if len(t) > 1 else float(args.dt)

    def w(key: str) -> np.ndarray:
        return a[key][mask]

    def delta_rms(key: str) -> float:
        values = w(key)
        return rms(np.diff(values)) if len(values) > 1 else float("nan")

    def rate_rms(key: str) -> float:
        values = w(key)
        return rms(np.diff(values) / dt) if len(values) > 1 else float("nan")

    flags = np.concatenate([
        a["flags_h"], a["flags_k"], a["joint_flags_h"], a["joint_flags_k"],
    ])

    return {
        "samples": int(len(t)),
        "window_samples": int(np.count_nonzero(mask)),
        "dt_log_s": dt,
        "hip_rmse": rms(e_h[mask]),
        "knee_rmse": rms(e_k[mask]),
        "coord_rmse": rms(e_coord[mask]),
        "hip_peak_abs_error": float(np.max(np.abs(e_h[mask]))),
        "knee_peak_abs_error": float(np.max(np.abs(e_k[mask]))),
        "coord_peak_abs_error": float(np.max(np.abs(e_coord[mask]))),
        "hip_tau_cmd_rms": rms(w("tau_cmd_h")),
        "knee_tau_cmd_rms": rms(w("tau_cmd_k")),
        "hip_delta_tau_cmd_rms": delta_rms("tau_cmd_h"),
        "knee_delta_tau_cmd_rms": delta_rms("tau_cmd_k"),
        "hip_tau_total_rms": rms(w("tau_total_h")),
        "knee_tau_total_rms": rms(w("tau_total_k")),
        "hip_delta_tau_total_rms": delta_rms("tau_total_h"),
        "knee_delta_tau_total_rms": delta_rms("tau_total_k"),
        "combined_delta_tau_total_rms": rms(np.sqrt(
            np.diff(w("tau_total_h")) ** 2 + np.diff(w("tau_total_k")) ** 2
        )) if np.count_nonzero(mask) > 1 else float("nan"),
        "combined_tau_total_rms": rms(np.sqrt(w("tau_total_h") ** 2 + w("tau_total_k") ** 2)),
        "hip_tau_rate_total_rms": rate_rms("tau_total_h"),
        "knee_tau_rate_total_rms": rate_rms("tau_total_k"),
        "hip_eta_u_rms": rms(w("eta_u_h")),
        "knee_eta_u_rms": rms(w("eta_u_k")),
        "max_abs_tau_dist_h": float(np.max(np.abs(w("tau_dist_h")))),
        "max_abs_tau_dist_k": float(np.max(np.abs(w("tau_dist_k")))),
        "combined_flags": int(np.bitwise_or.reduce(flags.astype(np.int64))) if len(flags) else 0,
    }


def metric_row(candidate: Candidate, success: bool, run_dir: Path, args: argparse.Namespace) -> dict[str, Any]:
    row: dict[str, Any] = {
        "case": candidate.case_id,
        "label": candidate.label,
        "purpose": candidate.purpose,
        "hip_so": candidate.hip_so,
        "knee_so": candidate.knee_so,
        "hip_observer_gain_q": NOMINAL_OBSERVER_Q * candidate.hip_so,
        "hip_observer_gain_dq": NOMINAL_OBSERVER_DQ * candidate.hip_so,
        "knee_observer_gain_q": NOMINAL_OBSERVER_Q * candidate.knee_so,
        "knee_observer_gain_dq": NOMINAL_OBSERVER_DQ * candidate.knee_so,
        "ku_q": U1_KU_Q,
        "ku_dq": U1_KU_DQ,
        "run_success": bool(success),
    }
    log_path = run_dir / "mujoco_closed_loop_log.csv"
    if success and log_path.exists():
        row.update(metrics_from_arrays(load_arrays(log_path), args))
    return row


def write_metrics(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def add_relative_columns(metrics: pd.DataFrame) -> pd.DataFrame:
    out = metrics.copy()
    base = out[out["case"] == "A"]
    if base.empty:
        return out
    base_row = base.iloc[0]
    for col in [
        "hip_rmse", "knee_rmse", "coord_rmse",
        "hip_delta_tau_total_rms", "knee_delta_tau_total_rms",
        "combined_delta_tau_total_rms",
        "hip_tau_total_rms", "knee_tau_total_rms",
    ]:
        if col in out.columns and pd.notna(base_row.get(col)) and float(base_row[col]) != 0.0:
            out[f"{col}_vs_A_pct"] = 100.0 * (out[col].astype(float) / float(base_row[col]) - 1.0)
    return out


def save_all(fig: plt.Figure, stem: Path) -> None:
    stem.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(stem.with_suffix(".pdf"))
    fig.savefig(stem.with_suffix(".png"))
    plt.close(fig)


def plot_pareto(metrics: pd.DataFrame, out_dir: Path) -> None:
    ok = metrics[metrics["run_success"] == True].copy()  # noqa: E712
    colors = {"A": "#0072B2", "B": "#009E73", "C": "#E69F00", "D": "#D55E00"}
    specs = [
        ("hip_rmse", "hip_delta_tau_total_rms", "Hip RMSE [rad]", r"Hip $\Delta\tau$ RMS [Nm/sample]"),
        ("knee_rmse", "knee_delta_tau_total_rms", "Knee RMSE [rad]", r"Knee $\Delta\tau$ RMS [Nm/sample]"),
        ("coord_rmse", "combined_delta_tau_total_rms", "Coord. RMSE [rad]", r"Combined $\Delta\tau$ RMS [Nm/sample]"),
    ]
    label_offsets = {
        "hip_rmse": {"A": (4, 3), "B": (4, -12), "C": (4, 3), "D": (4, 3)},
        "knee_rmse": {"A": (4, 9), "B": (4, -12), "C": (4, -12), "D": (4, 9)},
        "coord_rmse": {"A": (4, 9), "B": (4, -12), "C": (4, -12), "D": (4, 9)},
    }
    fig, axes = plt.subplots(1, 3, figsize=(6.85, 2.35))
    for ax, (xcol, ycol, xlabel, ylabel) in zip(axes, specs):
        for _, row in ok.iterrows():
            case = str(row["case"])
            ax.scatter(
                float(row[xcol]), float(row[ycol]),
                s=34, color=colors.get(case, "black"),
                edgecolor="black", linewidth=0.35, zorder=3,
            )
            ax.annotate(
                case,
                (float(row[xcol]), float(row[ycol])),
                xytext=label_offsets.get(xcol, {}).get(case, (4, 4)),
                textcoords="offset points",
            )
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        xvals = ok[xcol].to_numpy(dtype=float)
        yvals = ok[ycol].to_numpy(dtype=float)
        xpad = max((float(np.max(xvals)) - float(np.min(xvals))) * 0.20, 1.0e-6)
        ypad = max((float(np.max(yvals)) - float(np.min(yvals))) * 0.24, 1.0e-6)
        ax.set_xlim(float(np.min(xvals)) - xpad, float(np.max(xvals)) + xpad)
        ax.set_ylim(float(np.min(yvals)) - ypad, float(np.max(yvals)) + ypad)
    for idx, ax in enumerate(axes):
        ax.text(-0.18, 1.06, f"({chr(ord('a') + idx)})", transform=ax.transAxes, fontweight="bold")
    fig.subplots_adjust(left=0.08, right=0.99, bottom=0.26, top=0.92, wspace=0.48)
    save_all(fig, out_dir / "bychen_mujoco_pareto")


def plot_heatmap(metrics: pd.DataFrame, out_dir: Path) -> None:
    ok = metrics[metrics["run_success"] == True].copy()  # noqa: E712
    if ok.empty:
        return
    hip_values = sorted(ok["hip_so"].unique())
    knee_values = sorted(ok["knee_so"].unique(), reverse=True)
    specs = [
        ("coord_rmse", "Coord. RMSE [rad]"),
        ("knee_delta_tau_total_rms", r"Knee $\Delta\tau$ RMS"),
    ]
    fig, axes = plt.subplots(1, 2, figsize=(4.9, 2.35))
    for ax, (metric, label) in zip(axes, specs):
        grid = np.full((len(knee_values), len(hip_values)), np.nan)
        for i, knee_so in enumerate(knee_values):
            for j, hip_so in enumerate(hip_values):
                sub = ok[(ok["hip_so"] == hip_so) & (ok["knee_so"] == knee_so)]
                if not sub.empty:
                    grid[i, j] = float(sub.iloc[0][metric])
        im = ax.imshow(grid, cmap="viridis_r", aspect="auto")
        ax.set_xticks(range(len(hip_values)))
        ax.set_xticklabels([f"{v:g}" for v in hip_values])
        ax.set_yticks(range(len(knee_values)))
        ax.set_yticklabels([f"{v:g}" for v in knee_values])
        ax.set_xlabel(r"$s_{o,h}$")
        ax.set_ylabel(r"$s_{o,k}$")
        ax.set_title(label)
        for i in range(len(knee_values)):
            for j in range(len(hip_values)):
                if np.isfinite(grid[i, j]):
                    ax.text(j, i, f"{grid[i, j]:.4f}", ha="center", va="center", color="white", fontsize=7)
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.03)
    fig.subplots_adjust(left=0.11, right=0.96, bottom=0.24, top=0.84, wspace=0.55)
    save_all(fig, out_dir / "bychen_mujoco_2x2_heatmap")


def write_markdown_summary(path: Path, metrics: pd.DataFrame, args: argparse.Namespace, artifact_dir: Path) -> None:
    ok = add_relative_columns(metrics)
    columns = [
        "case", "hip_so", "knee_so",
        "hip_rmse", "knee_rmse", "coord_rmse",
        "hip_delta_tau_total_rms", "knee_delta_tau_total_rms",
        "combined_delta_tau_total_rms",
        "hip_eta_u_rms", "knee_eta_u_rms",
        "combined_flags",
    ]

    def markdown_table(df: pd.DataFrame, table_columns: list[str], floatfmt: str) -> str:
        view = df[table_columns].copy()
        header = "| " + " | ".join(table_columns) + " |"
        divider = "| " + " | ".join(["---"] * len(table_columns)) + " |"
        rows = []
        for _, row in view.iterrows():
            cells = []
            for col in table_columns:
                value = row[col]
                if isinstance(value, (float, np.floating)):
                    cells.append(format(float(value), floatfmt))
                else:
                    cells.append(str(value))
            rows.append("| " + " | ".join(cells) + " |")
        return "\n".join([header, divider, *rows])

    lines = [
        "# Bychen MuJoCo 2x2 sweep",
        "",
        "Scope: Unitree H1 MuJoCo simulation with the existing C++ EID stepper. "
        "These numbers are simulation evidence for mechanism screening, not hardware validation.",
        "",
        f"Disturbance: joints `{args.disturbance_joints}`, torques `{args.disturbance_torques}` N m, "
        f"half-cosine window {args.disturbance_start:g}/{args.disturbance_plateau_start:g}/"
        f"{args.disturbance_plateau_end:g}/{args.disturbance_end:g} s.",
        "",
        markdown_table(ok, columns, ".6f"),
        "",
        "Relative changes are computed against case A. Negative values are lower than A.",
        "",
    ]
    relative_cols = ["case"] + [c for c in ok.columns if c.endswith("_vs_A_pct")]
    if len(relative_cols) > 1:
        lines.extend([
            markdown_table(ok, relative_cols, ".2f"),
            "",
        ])
    lines.extend([
        "Generated artifacts:",
        "",
        f"- `{repo_relpath(artifact_dir / 'bychen_mujoco_metrics.csv')}`",
        f"- `{repo_relpath(artifact_dir / 'bychen_mujoco_metrics_with_relative.csv')}`",
        f"- `{repo_relpath(artifact_dir / 'figures' / 'bychen_mujoco_pareto.png')}` and `{repo_relpath(artifact_dir / 'figures' / 'bychen_mujoco_pareto.pdf')}`",
        f"- `{repo_relpath(artifact_dir / 'figures' / 'bychen_mujoco_2x2_heatmap.png')}` and `{repo_relpath(artifact_dir / 'figures' / 'bychen_mujoco_2x2_heatmap.pdf')}`",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-config", type=Path, default=Path("config/h1_real_p4_ku_u1_hip_knee_eid.yaml"))
    parser.add_argument("--scene", type=Path, default=Path("h1_official_mujoco/scene.xml"))
    parser.add_argument("--stepper", type=Path, default=Path("build/Debug/h1_controller_stepper.exe"))
    parser.add_argument("--out-dir", type=Path, default=Path("analysis_artifacts/bychen_mujoco_sweep"))
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--disturbance-joints", default="1,2")
    parser.add_argument("--disturbance-torques", default="6,-4")
    parser.add_argument("--disturbance-start", type=float, default=4.0)
    parser.add_argument("--disturbance-plateau-start", type=float, default=4.2)
    parser.add_argument("--disturbance-plateau-end", type=float, default=5.2)
    parser.add_argument("--disturbance-end", type=float, default=5.4)
    parser.add_argument("--skip-runs", action="store_true")
    args = parser.parse_args()

    configure_matplotlib()
    out_dir = args.out_dir
    config_dir = out_dir / "configs"
    runs_dir = out_dir / "runs"
    fig_dir = out_dir / "figures"

    rows: list[dict[str, Any]] = []
    for candidate in CANDIDATES:
        config_path = config_dir / f"bychen_mujoco_{candidate.case_id}.yaml"
        run_dir = runs_dir / candidate.case_id
        build_config(args.base_config, config_path, candidate)
        success = True
        if not args.skip_runs:
            success = run_mujoco(config_path, run_dir, args)
        rows.append(metric_row(candidate, success, run_dir, args))

    metrics_path = out_dir / "bychen_mujoco_metrics.csv"
    write_metrics(metrics_path, rows)
    metrics = pd.DataFrame(rows)
    metrics_rel = add_relative_columns(metrics)
    metrics_rel_path = out_dir / "bychen_mujoco_metrics_with_relative.csv"
    metrics_rel.to_csv(metrics_rel_path, index=False)
    plot_pareto(metrics, fig_dir)
    plot_heatmap(metrics, fig_dir)
    report_dir = analysis_report_dir(out_dir)
    report_dir.mkdir(parents=True, exist_ok=True)
    summary_path = report_dir / "bychen_mujoco_summary.md"
    write_markdown_summary(summary_path, metrics, args, out_dir)

    print(f"metrics={metrics_path}")
    print(f"metrics_with_relative={metrics_rel_path}")
    print(f"summary={summary_path}")
    print(f"figures={fig_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
