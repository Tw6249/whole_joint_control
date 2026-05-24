#!/usr/bin/env python3
"""Plot EID position tracking time series for single_knee_open_loop_db."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ANALYSIS_ROOT = Path(__file__).resolve().parents[1]
PARQUET_PATH = ANALYSIS_ROOT / "data/mujoco_fit/single_knee_open_loop_db/eid_single_knee_open_loop/timeseries.parquet"
OUTPUT_PATH = ANALYSIS_ROOT / "data/mujoco_fit/single_knee_open_loop_db/eid_position_tracking.png"

df = pd.read_parquet(PARQUET_PATH)

t = df["t"].to_numpy(dtype=float)
q_ref = df["q_ref_shaped"].to_numpy(dtype=float)
q_actual = df["q_actual"].to_numpy(dtype=float)
q_error = df["q_error_shaped"].to_numpy(dtype=float)
x_hat_q = df["x_hat_q"].to_numpy(dtype=float)
x_bar_q = df["x_bar_q"].to_numpy(dtype=float)
eta_q = df["eta_q"].to_numpy(dtype=float)
u_t = df["u_t"].to_numpy(dtype=float)

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["Arial", "DejaVu Sans"],
    "font.size": 11,
    "axes.labelsize": 12,
    "axes.titlesize": 13,
    "legend.fontsize": 9,
    "figure.dpi": 150,
})

fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
fig.suptitle("EID Position Tracking — single_knee_open_loop_db (RightKnee)", fontweight="bold", fontsize=14)

# ---- Panel 1: Position tracking ----
ax = axes[0]
ax.plot(t, q_ref, "k--", linewidth=1.2, alpha=0.8, label=r"$q^*$ (reference)")
ax.plot(t, q_actual, color="#3b82f6", linewidth=1.5, label=r"$q$ (actual)")
ax.plot(t, x_bar_q, color="#ef4444", linewidth=1.0, alpha=0.7, label=r"$\bar{x}_q$ (compensated prediction)")
ax.set_ylabel("Position [rad]")
ax.legend(loc="upper right", ncol=3, framealpha=0.9)
ax.grid(True, alpha=0.3)
ax.set_ylim(0.55, 0.72)

# ---- Panel 2: Tracking error ----
ax = axes[1]
ax.plot(t, q_error, color="#1d4ed8", linewidth=1.2, label=r"$e_q = q^* - q$")
ax.axhline(0, color="gray", linewidth=0.5, linestyle="-", alpha=0.4)
ax.fill_between(t, 0, q_error, alpha=0.12, color="#3b82f6")
ax.set_ylabel("Position Error [rad]")
ax.legend(loc="upper right", framealpha=0.9)
ax.grid(True, alpha=0.3)

rmse = float(np.sqrt(np.mean(q_error ** 2)))
max_err = float(np.max(np.abs(q_error)))
ax.text(0.02, 0.95, f"RMSE = {rmse:.5f} rad  |  Max |e| = {max_err:.5f} rad",
        transform=ax.transAxes, fontsize=9, va="top",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="lightyellow", alpha=0.8))

# ---- Panel 3: EID disturbance estimate + torque ----
ax = axes[2]
ax2 = ax.twinx()
line_eta = ax.plot(t, eta_q, color="#8b5cf6", linewidth=1.2, label=r"$\eta_q$ (disturbance estimate)")
line_u = ax2.plot(t, u_t, color="#f59e0b", linewidth=1.0, alpha=0.8, label=r"$u$ (torque output)")
ax.set_ylabel(r"$\eta_q$ [rad]", color="#8b5cf6")
ax2.set_ylabel("Torque [N·m]", color="#f59e0b")
ax.tick_params(axis="y", labelcolor="#8b5cf6")
ax2.tick_params(axis="y", labelcolor="#f59e0b")

lines = line_eta + line_u
labels = [ln.get_label() for ln in lines]
ax.legend(lines, labels, loc="upper right", framealpha=0.9)
ax.grid(True, alpha=0.3)

ax.set_xlabel("Time [s]")
ax.set_xlim(t[0], t[-1])

fig.tight_layout()
fig.savefig(OUTPUT_PATH, dpi=200, bbox_inches="tight", facecolor="white", edgecolor="none")
plt.close(fig)

print(f"Saved: {OUTPUT_PATH}")
print(f"Stats: RMSE={rmse:.5f} rad, max|error|={max_err:.5f} rad, samples={len(t)}")
