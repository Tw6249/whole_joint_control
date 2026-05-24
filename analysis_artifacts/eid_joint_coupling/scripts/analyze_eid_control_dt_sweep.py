#!/usr/bin/env python3
"""Analyze how larger EID control dt changes right-leg oscillations."""

from __future__ import annotations

import argparse
import csv
import re
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ANALYSIS_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = ANALYSIS_ROOT / "data/eid_right_leg_tests_dt_sweep"
ANALYSIS_DIR = DEFAULT_ROOT / "analysis"
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
DT_RE = re.compile(r"_dt_(\d+p\d+)$")


@dataclass(frozen=True)
class RunSpec:
    case: str
    joint_id: int
    dt: float
    csv_path: Path


def parse_dt_tag(text: str) -> float:
    return float(text.replace("p", "."))


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


def mean_abs(x: np.ndarray) -> float:
    return float(np.mean(np.abs(x))) if x.size else 0.0


def sat_fraction(x: np.ndarray, limit: float, threshold: float = 0.9) -> float:
    return float(np.mean(np.abs(x) >= threshold * limit)) if x.size else 0.0


def clip_window(s: dict[str, np.ndarray], start_s: float) -> dict[str, np.ndarray]:
    mask = s["t"] >= start_s
    return {k: v[mask] for k, v in s.items()}


def collect_runs(root: Path) -> list[RunSpec]:
    runs: list[RunSpec] = []
    for csv_path in sorted(root.glob("**/mujoco_closed_loop_log.csv")):
        parent = csv_path.parent.name
        m = DT_RE.search(parent)
        if not m:
            continue
        dt = parse_dt_tag(m.group(1))
        case = parent.split("_dt_")[0]
        for joint_id in (RIGHT_KNEE, RIGHT_HIP_PITCH):
            # Only keep the joint actually present in this case folder.
            if case.startswith("right_knee_only") and joint_id != RIGHT_KNEE:
                continue
            if case.startswith("right_hip_pitch_and_knee"):
                runs.append(RunSpec(case=case, joint_id=joint_id, dt=dt, csv_path=csv_path))
            else:
                runs.append(RunSpec(case=case, joint_id=RIGHT_KNEE, dt=dt, csv_path=csv_path))
                break
    return runs


def collect_metrics(root: Path) -> list[dict[str, str]]:
    runs = collect_runs(root)
    rows: list[dict[str, str]] = []
    for run in runs:
        data = clip_window(read_case(run.csv_path)[run.joint_id], ANALYSIS_START_S)
        limit = TAU_LIMITS[run.joint_id]
        rows.append({
            "case": run.case,
            "joint_id": str(run.joint_id),
            "joint": JOINT_LABELS[run.joint_id],
            "dt_s": f"{run.dt:.3f}",
            "samples": str(data["t"].size),
            "q_rmse": f"{rmse(data['q_ref_shaped'] - data['q_actual']):.6g}",
            "dq_rmse": f"{rmse(data['dq_ref_shaped'] - data['dq_actual']):.6g}",
            "dq_actual_std": f"{np.std(data['dq_actual']):.6g}",
            "u_t_abs_mean": f"{mean_abs(data['u_t']):.6g}",
            "u_t_sat_90pct_frac": f"{sat_fraction(data['u_t'], limit):.6g}",
            "eta_dq_abs_mean": f"{mean_abs(data['eta_dq']):.6g}",
            "r_d_q_abs_max": f"{np.max(np.abs(data['r_d_q'])):.6g}",
        })
    rows.sort(key=lambda r: (r["case"], int(r["joint_id"]), float(r["dt_s"])))
    return rows


def write_csv(rows: list[dict[str, str]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def write_markdown(rows: list[dict[str, str]], path: Path) -> None:
    cols = ["case", "joint", "dt_s", "q_rmse", "dq_actual_std", "u_t_abs_mean", "u_t_sat_90pct_frac", "eta_dq_abs_mean", "r_d_q_abs_max"]
    lines = ["| " + " | ".join(cols) + " |", "| " + " | ".join(["---"] * len(cols)) + " |"]
    for row in rows:
        vals = []
        for c in cols:
            v = row[c]
            if c.endswith("frac"):
                v = f"{100.0 * float(v):.2f}%"
            vals.append(v)
        lines.append("| " + " | ".join(vals) + " |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def setup_style() -> None:
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "DejaVu Sans"],
        "font.size": 10,
        "axes.labelsize": 11,
        "axes.titlesize": 12,
        "legend.fontsize": 8,
        "figure.dpi": 150,
    })


def savefig(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(path, dpi=220, bbox_inches="tight", facecolor="white")
    plt.close()


def plot_metrics(rows: list[dict[str, str]], path: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    fig.suptitle(f"Control-dt sweep after {ANALYSIS_START_S:.0f}s transient", fontweight="bold")
    metrics = (
        ("q_rmse", "q RMSE [rad]", "#4f46e5"),
        ("dq_actual_std", "std(dq_actual) [rad/s]", "#dc2626"),
        ("u_t_abs_mean", "mean |u_t| [N·m]", "#ea580c"),
        ("eta_dq_abs_mean", "mean |eta_dq|", "#0891b2"),
    )
    x = sorted({float(r["dt_s"]) for r in rows})
    for ax, (metric, ylabel, color) in zip(axes.ravel(), metrics):
        for case, linestyle in (("right_knee_only", "--"), ("right_hip_pitch_and_knee", "-")):
            for joint_id, marker in ((RIGHT_KNEE, "o"), (RIGHT_HIP_PITCH, "s")):
                series = [r for r in rows if r["case"] == case and int(r["joint_id"]) == joint_id]
                if not series:
                    continue
                series.sort(key=lambda r: float(r["dt_s"]))
                ax.plot(
                    [float(r["dt_s"]) for r in series],
                    [float(r[metric]) for r in series],
                    marker=marker,
                    linestyle=linestyle,
                    color=color if joint_id == RIGHT_KNEE else "#2563eb",
                    linewidth=1.2,
                    label=f"{case} {JOINT_LABELS[joint_id]}",
                )
        ax.set_xlabel("control dt [s]")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.set_xticks(x)
        ax.legend(loc="upper left", framealpha=0.9, ncol=2)
    savefig(path)


def plot_joint_timeseries(root: Path, case: str, joint_id: int, path: Path) -> None:
    runs = []
    for csv_path in sorted((root / case).parent.glob(f"{case}_dt_*/mujoco_closed_loop_log.csv")):
        dt_tag = csv_path.parent.name.split("_dt_")[-1]
        runs.append((parse_dt_tag(dt_tag), csv_path))
    runs.sort(key=lambda item: item[0])

    fields = [
        ("q_actual", "q [rad]"),
        ("dq_actual", "dq [rad/s]"),
        ("u_t", "tau [N·m]"),
    ]
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(f"{JOINT_LABELS[joint_id]} across control dt values", fontweight="bold")

    for dt, csv_path in runs:
        data = clip_window(read_case(csv_path)[joint_id], ANALYSIS_START_S)
        label = f"dt={dt:.3f}s"
        for ax, (field, ylabel) in zip(axes, fields):
            ax.plot(data["t"], data[field], linewidth=1.0, label=label)
            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.3)
    for ax in axes:
        ax.legend(loc="upper right", framealpha=0.9)
        ax.set_xlim(ANALYSIS_START_S, 15.0)
    axes[-1].set_xlabel("Time [s]")
    savefig(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--out-dir", type=Path, default=ANALYSIS_DIR)
    args = parser.parse_args()

    setup_style()
    rows = collect_metrics(args.root)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(rows, args.out_dir / "eid_dt_sweep_metrics.csv")
    write_markdown(rows, args.out_dir / "eid_dt_sweep_metrics_table.md")
    plot_metrics(rows, args.out_dir / "fig1_dt_sweep_metrics.png")
    plot_joint_timeseries(args.root, "right_hip_pitch_and_knee", RIGHT_HIP_PITCH, args.out_dir / "fig2_dt_sweep_right_hip_pitch.png")
    plot_joint_timeseries(args.root, "right_hip_pitch_and_knee", RIGHT_KNEE, args.out_dir / "fig3_dt_sweep_right_knee.png")
    print(f"Wrote dt sweep analysis outputs to: {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
