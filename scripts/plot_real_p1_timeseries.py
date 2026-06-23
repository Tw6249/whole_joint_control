#!/usr/bin/env python3
"""Plot real P1 PD/EID hip-knee logs with a robust relative time axis."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


JOINTS = {1: "RightHipPitch", 2: "RightKnee"}
COLORS = {1: "#0072B2", 2: "#D55E00"}


def load_log(path: Path, method: str) -> pd.DataFrame:
    df = pd.read_csv(path).copy()

    # Older logs wrote large absolute t values with default stream precision,
    # which can collapse many control cycles onto the same plotted timestamp.
    # Rebuild relative time from cycle/dt whenever t does not resolve cycles.
    cycles = df.drop_duplicates("cycle")[["cycle", "dt"]].sort_values("cycle").copy()
    t_unique_ratio = df["t"].nunique() / max(1, df["cycle"].nunique())
    if t_unique_ratio < 0.5:
        cycles["t_rel"] = cycles["dt"].shift(fill_value=0.0).cumsum()
        df = df.merge(cycles[["cycle", "t_rel"]], on="cycle", how="left")
    else:
        df["t_rel"] = df["t"] - df["t"].min()

    df["q_ref"] = df["debug_0"]
    df["dq_ref"] = df["debug_1"]
    if method.upper() == "EID":
        df["q_raw_ref"] = df["debug_26"]
        df["dq_raw_ref"] = df["debug_27"]
    else:
        df["q_raw_ref"] = df["debug_9"] if "debug_9" in df else df["debug_0"]
        df["dq_raw_ref"] = df["debug_10"] if "debug_10" in df else df["debug_1"]
    df["q_err"] = df["q_ref"] - df["q"]
    df["dq_err"] = df["dq_ref"] - df["dq"]
    return df


def configure_matplotlib() -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 130,
            "savefig.dpi": 180,
            "font.size": 9,
            "axes.grid": True,
            "grid.alpha": 0.22,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "lines.solid_capstyle": "round",
        }
    )


def plot_position(data: dict[str, pd.DataFrame], out_dir: Path) -> None:
    fig, axes = plt.subplots(4, 2, figsize=(13, 10), sharex="col")
    for col, (method, df) in enumerate(data.items()):
        for joint_id, joint_name in JOINTS.items():
            sub = df[df["joint_id"] == joint_id]
            ax = axes[0 if joint_id == 1 else 1, col]
            ax.plot(sub["t_rel"], sub["q_ref"], color="black", lw=1.1, label="q_ref shaped")
            ax.plot(sub["t_rel"], sub["q"], color=COLORS[joint_id], lw=0.85, alpha=0.9, label="q_actual")
            ax.set_title(f"{method} {joint_name} position")
            ax.set_ylabel("rad")
            ax.legend(loc="upper right", fontsize=8)

            ax = axes[2 if joint_id == 1 else 3, col]
            ax.plot(sub["t_rel"], sub["q_err"], color=COLORS[joint_id], lw=0.85)
            ax.axhline(0, color="black", lw=0.7, alpha=0.5)
            rmse = float(np.sqrt(np.mean(np.square(sub["q_err"]))))
            ax.set_title(f"{method} {joint_name} q error, RMSE={rmse:.4f} rad")
            ax.set_ylabel("rad")
            ax.set_xlabel("time [s]")
    fig.suptitle("P1 Real Experiment: Position Tracking and Error", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out_dir / "p1_pd_eid_position_error_timeseries_rebuilt_time.png")
    plt.close(fig)


def plot_reference_zoom(data: dict[str, pd.DataFrame], out_dir: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(13, 6), sharex=True)
    for col, (method, df) in enumerate(data.items()):
        for row, (joint_id, joint_name) in enumerate(JOINTS.items()):
            sub = df[(df["joint_id"] == joint_id) & (df["t_rel"] <= 2.0)]
            ax = axes[row, col]
            ax.plot(sub["t_rel"], sub["q_raw_ref"], color="#888888", lw=0.9, label="raw policy ref")
            ax.plot(sub["t_rel"], sub["q_ref"], color="black", lw=1.1, label="startup-shaped ref")
            ax.plot(sub["t_rel"], sub["q"], color=COLORS[joint_id], lw=0.85, alpha=0.9, label="actual")
            ax.set_title(f"{method} {joint_name}, first 2s")
            ax.set_ylabel("rad")
            ax.legend(loc="best", fontsize=8)
            if row == 1:
                ax.set_xlabel("time [s]")
    fig.suptitle("Reference Diagnostic: raw policy, startup-shaped reference, actual", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(out_dir / "p1_pd_eid_reference_diagnostic_zoom_rebuilt_time.png")
    plt.close(fig)


def write_metrics(data: dict[str, pd.DataFrame], out_dir: Path) -> None:
    rows = []
    for method, df in data.items():
        for joint_id, joint_name in JOINTS.items():
            sub = df[df["joint_id"] == joint_id]
            rows.append(
                {
                    "method": method,
                    "joint_id": joint_id,
                    "joint": joint_name,
                    "duration_s": sub["t_rel"].max() - sub["t_rel"].min(),
                    "q_rmse_rad": float(np.sqrt(np.mean(np.square(sub["q_err"])))),
                    "q_peak_abs_err_rad": float(sub["q_err"].abs().max()),
                    "dq_rmse_rad_s": float(np.sqrt(np.mean(np.square(sub["dq_err"])))),
                    "tau_est_rms_nm": float(np.sqrt(np.mean(np.square(sub["tau_est"])))),
                    "tau_cmd_rms_nm": float(np.sqrt(np.mean(np.square(sub["tau_cmd"])))),
                    "flags_unique": ";".join(str(x) for x in sorted(sub["flags"].unique())),
                }
            )
    pd.DataFrame(rows).to_csv(out_dir / "p1_pd_eid_metrics.csv", index=False)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pd-log", type=Path, required=True)
    parser.add_argument("--eid-log", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("analysis_artifacts/real_p1_timeseries"))
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    configure_matplotlib()
    data = {"PD": load_log(args.pd_log, "PD"), "EID": load_log(args.eid_log, "EID")}
    plot_position(data, args.out_dir)
    plot_reference_zoom(data, args.out_dir)
    write_metrics(data, args.out_dir)
    print(args.out_dir)


if __name__ == "__main__":
    main()
