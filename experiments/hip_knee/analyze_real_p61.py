#!/usr/bin/env python3
"""Analyze P6.1 same-batch real H1 retest logs."""

from __future__ import annotations

# Support direct execution as well as python -m from the project root.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))


import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MANIFEST_DIR = REPO_ROOT / "analysis_artifacts" / "real_p61_batch"
DEFAULT_OUT_DIR = REPO_ROOT / "analysis_artifacts" / "real_p61_summary"
DEFAULT_FIG_DIR = REPO_ROOT / "docs" / "figures"

JOINTS = {1: "hip", 2: "knee"}
CANDIDATE_ORDER = [
    "A_O4_U1",
    "B_hipO5_kneeO4_U1",
    "E_O5_U1",
    "C_hipO4_kneeO3_U1",
    "D_hipO5_kneeO3_U1",
    "kneeDqDown_U1",
]
COLORS = {
    "A_O4_U1": "#4C78A8",
    "B_hipO5_kneeO4_U1": "#F58518",
    "E_O5_U1": "#54A24B",
    "C_hipO4_kneeO3_U1": "#B279A2",
    "D_hipO5_kneeO3_U1": "#E45756",
    "kneeDqDown_U1": "#72B7B2",
}


@dataclass
class LogSelection:
    manifest: Path
    candidate: str
    role: str
    stage: str
    repeat: str
    condition: str
    config: str
    log_path: Path
    complete: bool
    duration_s: float
    returncode: int
    status: str


def configure_matplotlib() -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 130,
            "savefig.dpi": 180,
            "font.size": 9,
            "axes.grid": True,
            "grid.alpha": 0.25,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "legend.frameon": False,
        }
    )


def rmse(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float)
    if x.size == 0:
        return math.nan
    return float(np.sqrt(np.mean(np.square(x))))


def rms(x: np.ndarray) -> float:
    return rmse(x)


def duration_of_log(path: Path) -> float:
    if not path.exists():
        return 0.0
    try:
        df = pd.read_csv(path, usecols=["t"])
    except Exception:
        return 0.0
    if df.empty:
        return 0.0
    return float(df["t"].max() - df["t"].min())


def read_manifests(manifest_dir: Path) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for manifest in sorted(manifest_dir.glob("p61_*_batch_manifest_*.csv")):
        with manifest.open(newline="", encoding="utf-8") as fh:
            for row in csv.DictReader(fh):
                row["_manifest"] = str(manifest)
                rows.append(row)
    return rows


def select_latest_complete_logs(manifest_dir: Path) -> list[LogSelection]:
    rows = read_manifests(manifest_dir)
    selections: dict[tuple[str, str, str], LogSelection] = {}

    for row in rows:
        log_value = row.get("log_path", "")
        if not log_value:
            # In early batch manifests, short but returncode=0 logs may not be
            # backfilled. They should not beat later complete补跑 records.
            continue
        log_path = REPO_ROOT / log_value
        duration = duration_of_log(log_path)
        expected = float(row.get("duration_s") or 10.0)
        complete = duration >= 0.90 * expected
        selection = LogSelection(
            manifest=Path(row["_manifest"]),
            candidate=row["candidate"],
            role=row.get("role", ""),
            stage=row["stage"],
            repeat=row["repeat"],
            condition=row["condition"],
            config=row.get("config", ""),
            log_path=log_path,
            complete=complete,
            duration_s=duration,
            returncode=int(row.get("returncode") or 0),
            status=row.get("status", ""),
        )
        key = (selection.stage, selection.candidate, selection.repeat)
        prev = selections.get(key)
        if prev is None:
            selections[key] = selection
            continue
        if selection.complete and not prev.complete:
            selections[key] = selection
        elif selection.complete == prev.complete and selection.manifest.name > prev.manifest.name:
            selections[key] = selection

    return sorted(
        selections.values(),
        key=lambda s: (
            0 if s.stage == "gate" else 1,
            int(s.repeat[1:] if s.repeat.startswith("r") else s.repeat),
            CANDIDATE_ORDER.index(s.candidate) if s.candidate in CANDIDATE_ORDER else 999,
        ),
    )


def load_log(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    df = df[df["joint_id"].isin(JOINTS)].copy()
    df["t_rel"] = df["t"] - df["t"].min()
    df["q_ref"] = df["debug_0"]
    df["dq_ref"] = df["debug_1"]
    df["q_err"] = df["q_ref"] - df["q"]
    df["dq_err"] = df["dq_ref"] - df["dq"]
    df["eta_q"] = df["debug_9"]
    df["eta_dq"] = df["debug_10"]
    df["eta_u"] = df["debug_32"]
    return df


def pivot_joint_signal(df: pd.DataFrame, column: str) -> pd.DataFrame:
    piv = df.pivot_table(index="cycle", columns="joint_id", values=column, aggfunc="first")
    return piv.rename(columns={1: f"hip_{column}", 2: f"knee_{column}"})


def coord_error_series(df: pd.DataFrame) -> pd.DataFrame:
    t = df.groupby("cycle")["t_rel"].first()
    err = pivot_joint_signal(df, "q_err")
    out = pd.concat([t, err], axis=1).dropna()
    out["coord_q_err"] = out["hip_q_err"] - out["knee_q_err"]
    return out


def band_power(x: np.ndarray, dt: float, f_low: float, f_high: float | None) -> float:
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if x.size < 8 or not np.isfinite(dt) or dt <= 0.0:
        return math.nan
    x = x - np.mean(x)
    freqs = np.fft.rfftfreq(x.size, d=dt)
    spec = np.abs(np.fft.rfft(x)) ** 2 / x.size
    if f_high is None:
        mask = freqs >= f_low
    else:
        mask = (freqs >= f_low) & (freqs < f_high)
    if not np.any(mask):
        return 0.0
    return float(np.sum(spec[mask]))


def run_metrics(selection: LogSelection) -> dict[str, object]:
    df = load_log(selection.log_path)
    coord = coord_error_series(df)
    windows = {
        "full": (0.0, selection.duration_s),
        "disturbance": (4.0, 5.4),
        "plateau": (4.2, 5.2),
        "recovery": (5.4, 7.4),
    }

    row: dict[str, object] = {
        "stage": selection.stage,
        "candidate": selection.candidate,
        "role": selection.role,
        "repeat": selection.repeat,
        "condition": selection.condition,
        "config": selection.config,
        "log_path": str(selection.log_path.relative_to(REPO_ROOT)),
        "duration_s": selection.duration_s,
        "complete": int(selection.complete),
        "returncode": selection.returncode,
        "status": selection.status,
    }

    for prefix, (t0, t1) in windows.items():
        csub = coord[(coord["t_rel"] >= t0) & (coord["t_rel"] < t1)]
        row[f"{prefix}_coord_rmse_rad"] = rmse(csub["coord_q_err"].to_numpy())
        row[f"{prefix}_coord_peak_abs_rad"] = float(csub["coord_q_err"].abs().max()) if not csub.empty else math.nan
        for joint_id, name in JOINTS.items():
            sub = df[(df["joint_id"] == joint_id) & (df["t_rel"] >= t0) & (df["t_rel"] < t1)].copy()
            row[f"{prefix}_{name}_q_rmse_rad"] = rmse(sub["q_err"].to_numpy())
            row[f"{prefix}_{name}_q_peak_abs_rad"] = float(sub["q_err"].abs().max()) if not sub.empty else math.nan
            row[f"{prefix}_{name}_dq_rmse_rad_s"] = rmse(sub["dq_err"].to_numpy())
            row[f"{prefix}_{name}_tau_est_rms_nm"] = rms(sub["tau_est"].to_numpy())
            row[f"{prefix}_{name}_tau_cmd_rms_nm"] = rms(sub["tau_cmd"].to_numpy())
            row[f"{prefix}_{name}_eta_u_rms_nm"] = rms(sub["eta_u"].to_numpy())
            row[f"{prefix}_{name}_eta_q_rms"] = rms(sub["eta_q"].to_numpy())
            row[f"{prefix}_{name}_eta_dq_rms"] = rms(sub["eta_dq"].to_numpy())
            tau = sub.sort_values("t_rel")["tau_est"].to_numpy()
            tau_cmd = sub.sort_values("t_rel")["tau_cmd"].to_numpy()
            row[f"{prefix}_{name}_delta_tau_est_rms_nm"] = rms(np.diff(tau)) if tau.size > 1 else math.nan
            row[f"{prefix}_{name}_delta_tau_cmd_rms_nm"] = rms(np.diff(tau_cmd)) if tau_cmd.size > 1 else math.nan
            row[f"{prefix}_{name}_tau_limit_like_count"] = int((sub["tau_cmd"].abs() >= 99.0).sum())
            row[f"{prefix}_{name}_flags_nonzero_count"] = int((sub["flags"] != 0).sum())

    for joint_id, name in JOINTS.items():
        sub = df[(df["joint_id"] == joint_id) & (df["t_rel"] >= 4.0) & (df["t_rel"] < 7.4)].sort_values("t_rel")
        dt = float(sub["dt"].median()) if not sub.empty else math.nan
        for signal in ["tau_est", "eta_u", "dq_err"]:
            x = sub[signal].to_numpy()
            row[f"freq_{name}_{signal}_power_0p5_2hz"] = band_power(x, dt, 0.5, 2.0)
            row[f"freq_{name}_{signal}_power_2_8hz"] = band_power(x, dt, 2.0, 8.0)
            row[f"freq_{name}_{signal}_power_8hz_plus"] = band_power(x, dt, 8.0, None)

    # Recovery time: first time after 5.4s that |coord error| returns below
    # the pre-disturbance 95th percentile and stays below for 0.2 s.
    pre = coord[(coord["t_rel"] >= 3.0) & (coord["t_rel"] < 4.0)]
    rec = coord[(coord["t_rel"] >= 5.4) & (coord["t_rel"] < 7.4)].copy()
    if pre.empty or rec.empty:
        row["coord_recovery_time_s"] = math.nan
    else:
        threshold = float(np.percentile(np.abs(pre["coord_q_err"]), 95))
        below = np.abs(rec["coord_q_err"].to_numpy()) <= threshold
        ts = rec["t_rel"].to_numpy()
        recovery = math.nan
        for i in range(len(ts)):
            j = np.searchsorted(ts, ts[i] + 0.2)
            if j > i and np.all(below[i:j]):
                recovery = float(ts[i] - 5.4)
                break
        row["coord_recovery_time_s"] = recovery

    return row


def summarize(run_df: pd.DataFrame) -> pd.DataFrame:
    disturbed = run_df[(run_df["stage"] == "disturbance") & (run_df["complete"] == 1)].copy()
    metric_cols = [
        c
        for c in disturbed.columns
        if c.endswith(("_rad", "_rad_s", "_nm", "_count", "_s", "_8hz_plus", "_2_8hz", "_0p5_2hz"))
        or "_power_" in c
    ]
    rows = []
    for candidate in CANDIDATE_ORDER:
        sub = disturbed[disturbed["candidate"] == candidate]
        if sub.empty:
            continue
        row: dict[str, object] = {
            "candidate": candidate,
            "role": sub["role"].iloc[0],
            "n": len(sub),
        }
        for col in metric_cols:
            row[f"{col}_mean"] = float(sub[col].mean())
            row[f"{col}_std"] = float(sub[col].std(ddof=1)) if len(sub) > 1 else 0.0
        rows.append(row)

    summary = pd.DataFrame(rows)
    if not summary.empty and "A_O4_U1" in set(summary["candidate"]):
        base = summary[summary["candidate"] == "A_O4_U1"].iloc[0]
        for col in list(summary.columns):
            if not col.endswith("_mean") or not np.isfinite(base[col]) or abs(base[col]) < 1e-12:
                continue
            summary[col.replace("_mean", "_rel_to_A_pct")] = 100.0 * (summary[col] / base[col] - 1.0)
    return summary


def plot_window_metrics(summary: pd.DataFrame, fig_dir: Path) -> None:
    metrics = [
        ("disturbance_hip_q_rmse_rad_mean", "Hip window RMSE [rad]"),
        ("disturbance_knee_q_rmse_rad_mean", "Knee window RMSE [rad]"),
        ("disturbance_coord_rmse_rad_mean", "Coord window RMSE [rad]"),
        ("recovery_coord_rmse_rad_mean", "Coord recovery RMSE [rad]"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(13, 7), sharex=True)
    x = np.arange(len(CANDIDATE_ORDER))
    labels = [c.replace("_", "\n") for c in CANDIDATE_ORDER]
    for ax, (col, title) in zip(axes.flat, metrics):
        values = [summary.loc[summary["candidate"] == c, col].iloc[0] for c in CANDIDATE_ORDER]
        err_col = col.replace("_mean", "_std")
        errs = [summary.loc[summary["candidate"] == c, err_col].iloc[0] for c in CANDIDATE_ORDER]
        ax.bar(x, values, yerr=errs, color=[COLORS[c] for c in CANDIDATE_ORDER], capsize=3)
        ax.set_title(title)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=0, fontsize=8)
    fig.suptitle("P6.1 Disturbance and Recovery Window Metrics")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(fig_dir / "real_p61_candidate_window_metrics.png")
    plt.close(fig)


def plot_torque_frequency(summary: pd.DataFrame, fig_dir: Path) -> None:
    metrics = [
        ("disturbance_hip_delta_tau_est_rms_nm_mean", "Hip delta tau_est RMS [N m]"),
        ("disturbance_knee_delta_tau_est_rms_nm_mean", "Knee delta tau_est RMS [N m]"),
        ("freq_hip_tau_est_power_8hz_plus_mean", "Hip tau_est power >8 Hz"),
        ("freq_knee_tau_est_power_8hz_plus_mean", "Knee tau_est power >8 Hz"),
        ("freq_hip_eta_u_power_8hz_plus_mean", "Hip eta_u power >8 Hz"),
        ("freq_knee_eta_u_power_8hz_plus_mean", "Knee eta_u power >8 Hz"),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(15, 7), sharex=True)
    x = np.arange(len(CANDIDATE_ORDER))
    labels = [c.replace("_", "\n") for c in CANDIDATE_ORDER]
    for ax, (col, title) in zip(axes.flat, metrics):
        values = [summary.loc[summary["candidate"] == c, col].iloc[0] for c in CANDIDATE_ORDER]
        ax.bar(x, values, color=[COLORS[c] for c in CANDIDATE_ORDER])
        ax.set_title(title)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=0, fontsize=8)
    fig.suptitle("P6.1 Torque Roughness and High-Frequency Power")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(fig_dir / "real_p61_torque_frequency_summary.png")
    plt.close(fig)


def plot_recovery_segments(selections: list[LogSelection], fig_dir: Path) -> None:
    disturbed = [s for s in selections if s.stage == "disturbance" and s.complete]
    bins = np.linspace(3.8, 7.4, 361)
    centers = 0.5 * (bins[:-1] + bins[1:])
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for candidate in CANDIDATE_ORDER:
        series = []
        for s in disturbed:
            if s.candidate != candidate:
                continue
            coord = coord_error_series(load_log(s.log_path))
            for_signal = {
                "hip": coord[["t_rel", "hip_q_err"]].rename(columns={"hip_q_err": "value"}),
                "knee": coord[["t_rel", "knee_q_err"]].rename(columns={"knee_q_err": "value"}),
                "coord": coord[["t_rel", "coord_q_err"]].rename(columns={"coord_q_err": "value"}),
            }
            series.append(for_signal)
        if not series:
            continue
        for ax, key, title in zip(axes, ["hip", "knee", "coord"], ["Hip error", "Knee error", "Coord error"]):
            traces = []
            for item in series:
                df = item[key]
                values = np.interp(centers, df["t_rel"], df["value"], left=np.nan, right=np.nan)
                traces.append(values)
            arr = np.vstack(traces)
            mean = np.nanmean(arr, axis=0)
            ax.plot(centers, mean, label=candidate, color=COLORS[candidate], lw=1.3)
            ax.set_ylabel("rad")
            ax.set_title(title)
            ax.axvspan(4.0, 5.4, color="#DDDDDD", alpha=0.45)
            ax.axvspan(4.2, 5.2, color="#BBBBBB", alpha=0.25)
    axes[-1].set_xlabel("relative time [s]")
    axes[0].legend(ncol=3, fontsize=8)
    fig.suptitle("P6.1 Mean Disturbance and Recovery Segments")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(fig_dir / "real_p61_recovery_segments.png")
    plt.close(fig)


def write_markdown_tables(summary: pd.DataFrame, out_dir: Path) -> None:
    cols = [
        "candidate",
        "n",
        "disturbance_hip_q_rmse_rad_mean",
        "disturbance_knee_q_rmse_rad_mean",
        "disturbance_coord_rmse_rad_mean",
        "recovery_coord_rmse_rad_mean",
        "disturbance_hip_delta_tau_est_rms_nm_mean",
        "disturbance_knee_delta_tau_est_rms_nm_mean",
        "freq_knee_tau_est_power_8hz_plus_mean",
    ]
    table = summary[cols].copy()
    table.columns = [
        "candidate",
        "n",
        "hip_win_rmse",
        "knee_win_rmse",
        "coord_win_rmse",
        "coord_recovery_rmse",
        "hip_delta_tau",
        "knee_delta_tau",
        "knee_tau_power_gt8",
    ]
    out = ["| " + " | ".join(table.columns) + " |", "| " + " | ".join(["---"] * len(table.columns)) + " |"]
    for _, row in table.iterrows():
        vals = []
        for col in table.columns:
            value = row[col]
            if isinstance(value, (int, np.integer)):
                vals.append(str(value))
            elif isinstance(value, (float, np.floating)):
                vals.append(f"{value:.6g}")
            else:
                vals.append(str(value))
        out.append("| " + " | ".join(vals) + " |")
    (out_dir / "p61_candidate_summary_table.md").write_text("\n".join(out) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest-dir", type=Path, default=DEFAULT_MANIFEST_DIR)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--fig-dir", type=Path, default=DEFAULT_FIG_DIR)
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    args.fig_dir.mkdir(parents=True, exist_ok=True)
    configure_matplotlib()

    selections = select_latest_complete_logs(args.manifest_dir)
    selected_rows = [
        {
            "stage": s.stage,
            "candidate": s.candidate,
            "repeat": s.repeat,
            "condition": s.condition,
            "manifest": str(s.manifest.relative_to(REPO_ROOT)),
            "log_path": str(s.log_path.relative_to(REPO_ROOT)),
            "duration_s": s.duration_s,
            "complete": int(s.complete),
            "returncode": s.returncode,
            "status": s.status,
        }
        for s in selections
    ]
    pd.DataFrame(selected_rows).to_csv(args.out_dir / "p61_selected_logs.csv", index=False)

    run_rows = [run_metrics(s) for s in selections]
    run_df = pd.DataFrame(run_rows)
    run_df.to_csv(args.out_dir / "p61_run_metrics.csv", index=False)

    summary = summarize(run_df)
    summary.to_csv(args.out_dir / "p61_summary_by_candidate.csv", index=False)
    write_markdown_tables(summary, args.out_dir)

    plot_window_metrics(summary, args.fig_dir)
    plot_torque_frequency(summary, args.fig_dir)
    plot_recovery_segments(selections, args.fig_dir)

    print(args.out_dir)
    print(args.fig_dir / "real_p61_candidate_window_metrics.png")
    print(args.fig_dir / "real_p61_torque_frequency_summary.png")
    print(args.fig_dir / "real_p61_recovery_segments.png")


if __name__ == "__main__":
    main()
