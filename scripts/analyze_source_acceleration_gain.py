#!/usr/bin/env python3
"""Sweep source-joint acceleration gain in the row-2/row-3 decomposition."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "analysis_artifacts" / "hip_knee_mutual_coupling"
INPUT = OUT_DIR / "hip_knee_mutual_coupling_timeseries.csv"
FIG_DIR = OUT_DIR / "figures"


def rms(values: np.ndarray) -> float:
    x = np.asarray(values, dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(x * x)))


def mean_curve(detail: pd.DataFrame, method: str, column: str, grid: np.ndarray) -> np.ndarray:
    curves = []
    for _, run in detail[detail["method"] == method].groupby(["repeat", "log_path"], sort=True):
        x = run["t_rel"].to_numpy(dtype=float)
        y = run[column].to_numpy(dtype=float)
        mask = np.isfinite(x) & np.isfinite(y)
        if mask.sum() < 2:
            continue
        lo = max(float(np.min(x[mask])), float(grid[0]))
        hi = min(float(np.max(x[mask])), float(grid[-1]))
        valid = (grid >= lo) & (grid <= hi)
        yi = np.full_like(grid, np.nan, dtype=float)
        yi[valid] = np.interp(grid[valid], x[mask], y[mask])
        curves.append(yi)
    if not curves:
        return np.full_like(grid, np.nan, dtype=float)
    return np.nanmean(np.vstack(curves), axis=0)


def summarize_direction(row2: np.ndarray, row3: np.ndarray, gain: float) -> dict[str, float]:
    row2_gain = gain * row2
    remaining = row3 - row2
    row3_gain = remaining + row2_gain
    row2_gain_rms = rms(row2_gain)
    remaining_rms = rms(remaining)
    share = 100.0 * row2_gain_rms / (row2_gain_rms + remaining_rms) if row2_gain_rms + remaining_rms > 0 else float("nan")
    return {
        "acc_gain": gain,
        "row2_acc_torque_rms": row2_gain_rms,
        "row3_total_residual_rms": rms(row3_gain),
        "remaining_after_removing_row2_rms": remaining_rms,
        "row2_rms_share_percent": share,
    }


def unconstrained_best_gain(row2: np.ndarray, row3: np.ndarray) -> float:
    remaining = row3 - row2
    den = float(np.dot(row2, row2))
    return float(-np.dot(row2, remaining) / den) if den > 0 else float("nan")


def write_figures(summary: pd.DataFrame) -> None:
    import matplotlib.pyplot as plt

    FIG_DIR.mkdir(parents=True, exist_ok=True)
    colors = {"PD": "#7f7f7f", "EID": "#0072B2"}
    directions = [
        ("hip_to_knee", "Increase hip acceleration only"),
        ("knee_to_hip", "Increase knee acceleration only"),
    ]

    fig, axes = plt.subplots(1, 2, figsize=(9.8, 3.6), sharey=False)
    for ax, (direction, title) in zip(axes, directions):
        for method in ["PD", "EID"]:
            g = summary[(summary["direction"] == direction) & (summary["method"] == method)]
            ax.plot(g["acc_gain"], g["row3_total_residual_rms"], color=colors[method], lw=1.8, label=f"{method} row-3 total")
            ax.plot(g["acc_gain"], g["row2_acc_torque_rms"], color=colors[method], lw=1.5, ls="--", label=f"{method} row-2 acc")
            ax.plot(
                g["acc_gain"],
                g["remaining_after_removing_row2_rms"],
                color=colors[method],
                lw=1.2,
                ls=":",
                label=f"{method} residual after removing row-2",
            )
        ax.axvline(1.0, color="black", lw=0.7, ls="--", alpha=0.55)
        ax.set_title(title)
        ax.set_xlabel("source acceleration gain")
        ax.set_ylabel("RMS [N m]")
        ax.grid(True, alpha=0.25)
        ax.legend(frameon=False, fontsize=7)
    fig.tight_layout()
    fig.savefig(FIG_DIR / "source_acceleration_gain_rms.png", dpi=220, bbox_inches="tight")
    fig.savefig(FIG_DIR / "source_acceleration_gain_rms.pdf", bbox_inches="tight")
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(9.8, 3.4), sharey=True)
    for ax, (direction, title) in zip(axes, directions):
        for method in ["PD", "EID"]:
            g = summary[(summary["direction"] == direction) & (summary["method"] == method)]
            ax.plot(g["acc_gain"], g["row2_rms_share_percent"], color=colors[method], lw=1.8, label=method)
        ax.axvline(1.0, color="black", lw=0.7, ls="--", alpha=0.55)
        ax.set_title(title)
        ax.set_xlabel("source acceleration gain")
        ax.set_ylabel("row-2 share of row-2 + remaining RMS [%]")
        ax.set_ylim(0, 100)
        ax.grid(True, alpha=0.25)
        ax.legend(frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(FIG_DIR / "source_acceleration_gain_share.png", dpi=220, bbox_inches="tight")
    fig.savefig(FIG_DIR / "source_acceleration_gain_share.pdf", bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    detail = pd.read_csv(INPUT)
    grid = np.arange(3.0, 7.4 + 0.005, 0.01)
    gains = np.round(np.arange(0.0, 3.0 + 1.0e-9, 0.1), 3)
    specs = [
        ("hip_to_knee", "d_k_from_hip_acc", "d_k_total"),
        ("knee_to_hip", "d_h_from_knee_acc", "d_h_total"),
    ]
    rows = []
    best_rows = []
    for direction, row2_col, row3_col in specs:
        for method_raw, method in [("pd", "PD"), ("eid", "EID")]:
            row2 = mean_curve(detail, method_raw, row2_col, grid)
            row3 = mean_curve(detail, method_raw, row3_col, grid)
            for gain in gains:
                rows.append({"method": method, "direction": direction, **summarize_direction(row2, row3, float(gain))})
            best = unconstrained_best_gain(row2, row3)
            best_rows.append(
                {
                    "method": method,
                    "direction": direction,
                    "unconstrained_best_acc_gain_for_min_row3_rms": best,
                    "best_gain_if_only_increase_from_original": max(1.0, best),
                }
            )
    summary = pd.DataFrame(rows)
    best_summary = pd.DataFrame(best_rows)
    summary.to_csv(OUT_DIR / "source_acceleration_gain_summary.csv", index=False)
    best_summary.to_csv(OUT_DIR / "source_acceleration_gain_best.csv", index=False)
    write_figures(summary)
    print(best_summary.to_string(index=False, float_format=lambda v: f"{v:.4f}"))
    print(summary[summary["acc_gain"].isin([1.0, 2.0, 3.0])].to_string(index=False, float_format=lambda v: f"{v:.4f}"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
