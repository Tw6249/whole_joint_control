#!/usr/bin/env python3
"""Data-level projection of row-2 acceleration torque onto row-3 residual."""

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


def fit_scale(row2: np.ndarray, row3: np.ndarray) -> dict[str, float]:
    x = np.asarray(row2, dtype=float)
    y = np.asarray(row3, dtype=float)
    mask = np.isfinite(x) & np.isfinite(y)
    x = x[mask]
    y = y[mask]
    den = float(np.dot(x, x))
    scale = float(np.dot(x, y) / den) if den > 0.0 else float("nan")
    remaining = y - scale * x
    before = rms(y)
    after = rms(remaining)
    at_raw = rms(y - x)
    corr = float(np.corrcoef(x, y)[0, 1]) if x.size > 1 else float("nan")
    return {
        "scale": scale,
        "scale_percent_of_original_row2": 100.0 * scale,
        "extra_gain_from_original_percent": 100.0 * (scale - 1.0),
        "row2_rms": rms(x),
        "row3_rms_before": before,
        "remaining_rms_after": after,
        "remaining_rms_at_raw_row2": at_raw,
        "rms_reduction_percent": 100.0 * (1.0 - after / before) if before > 0 else float("nan"),
        "energy_explained_percent": 100.0 * (1.0 - (after * after) / (before * before)) if before > 0 else float("nan"),
        "corr": corr,
    }


def write_figures(curves: pd.DataFrame, summary: pd.DataFrame) -> None:
    import matplotlib.pyplot as plt

    FIG_DIR.mkdir(parents=True, exist_ok=True)
    colors = {"PD": "#7f7f7f", "EID": "#0072B2"}
    directions = [
        ("hip_to_knee", "Hip acc term -> knee residual"),
        ("knee_to_hip", "Knee acc term -> hip residual"),
    ]

    gain_grid = np.linspace(0.0, 1.5, 301)
    fig, axes = plt.subplots(1, 2, figsize=(9.4, 3.45), sharey=False)
    for ax, (direction, title) in zip(axes, directions):
        for method in ["PD", "EID"]:
            row = summary[(summary["direction"] == direction) & (summary["method"] == method)].iloc[0]
            c = curves[(curves["direction"] == direction) & (curves["method"] == method)]
            x = c["row2"].to_numpy(dtype=float)
            y = c["row3"].to_numpy(dtype=float)
            rms_curve = np.array([rms(y - gain * x) for gain in gain_grid])
            ax.plot(gain_grid, rms_curve, color=colors[method], lw=1.8, label=f"{method}, best s={row['scale']:.2f}")
            ax.scatter([row["scale"]], [row["remaining_rms_after"]], color=colors[method], s=24, zorder=3)
        ax.axvline(1.0, color="black", lw=0.7, ls="--", alpha=0.6)
        ax.set_title(title)
        ax.set_xlabel("scale applied to row-2 curve")
        ax.set_ylabel("RMS of row-3 minus scaled row-2 [N m]")
        ax.grid(True, alpha=0.25)
        ax.legend(frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(FIG_DIR / "row2_data_projection_gain_scan.png", dpi=220, bbox_inches="tight")
    fig.savefig(FIG_DIR / "row2_data_projection_gain_scan.pdf", bbox_inches="tight")
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(9.4, 5.2), sharex=True)
    for col, (direction, title) in enumerate(directions):
        for row_idx, method in enumerate(["PD", "EID"]):
            ax = axes[row_idx, col]
            fit = summary[(summary["direction"] == direction) & (summary["method"] == method)].iloc[0]
            c = curves[(curves["direction"] == direction) & (curves["method"] == method)]
            ax.plot(c["t_rel"], c["row3"], color="black", lw=1.2, label="row-3 residual")
            ax.plot(
                c["t_rel"],
                fit["scale"] * c["row2"],
                color=colors[method],
                lw=1.2,
                label=f"best scaled row-2, s={fit['scale']:.2f}",
            )
            ax.axhline(0.0, color="black", lw=0.5, alpha=0.5)
            ax.set_title(f"{method}: {title}")
            ax.set_ylabel("[N m]")
            ax.grid(True, alpha=0.22)
            ax.legend(frameon=False, fontsize=7)
    axes[-1, 0].set_xlabel("time from log start [s]")
    axes[-1, 1].set_xlabel("time from log start [s]")
    fig.tight_layout()
    fig.savefig(FIG_DIR / "row2_data_projection_overlay.png", dpi=220, bbox_inches="tight")
    fig.savefig(FIG_DIR / "row2_data_projection_overlay.pdf", bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    detail = pd.read_csv(INPUT)
    grid = np.arange(3.0, 7.4 + 0.005, 0.01)
    specs = [
        ("hip_to_knee", "d_k_from_hip_acc", "d_k_total"),
        ("knee_to_hip", "d_h_from_knee_acc", "d_h_total"),
    ]
    summary_rows = []
    curve_rows = []
    for direction, row2_col, row3_col in specs:
        for method_raw, method in [("pd", "PD"), ("eid", "EID")]:
            row2 = mean_curve(detail, method_raw, row2_col, grid)
            row3 = mean_curve(detail, method_raw, row3_col, grid)
            fit = fit_scale(row2, row3)
            summary_rows.append({"method": method, "direction": direction, **fit})
            for t, x, y in zip(grid, row2, row3):
                curve_rows.append({"method": method, "direction": direction, "t_rel": t, "row2": x, "row3": y})

    summary = pd.DataFrame(summary_rows)
    curves = pd.DataFrame(curve_rows)
    summary.to_csv(OUT_DIR / "row2_data_projection_summary.csv", index=False)
    curves.to_csv(OUT_DIR / "row2_data_projection_curves.csv", index=False)
    write_figures(curves, summary)
    print(summary.to_string(index=False, float_format=lambda v: f"{v:.4f}"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
