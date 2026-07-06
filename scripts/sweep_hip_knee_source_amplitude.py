#!/usr/bin/env python3
"""Sweep source-joint motion amplitude and recompute hip-knee residuals.

This is different from post-scaling an already computed curve. For each gain,
the source joint trajectory is changed first, then MuJoCo inverse dynamics is
called again.
"""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pandas as pd
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from analyze_hip_knee_mutual_coupling import (  # noqa: E402
    HIP,
    KNEE,
    HipKneeInverseDynamics,
    load_plant,
    local_tau,
    yaml_joint,
)


STEADY_START = 3.0
STEADY_STOP = 7.4
GRID_DT = 0.01


def read_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def joint_center_and_limits(config_path: Path, joint_id: int) -> tuple[float, float, float]:
    joint = yaml_joint(read_yaml(config_path), joint_id)
    plant = joint["plant"]
    return float(joint["policy_center"]), float(plant["q_min"]), float(plant["q_max"])


def interp_run(run: pd.DataFrame, grid: np.ndarray, columns: list[str]) -> dict[str, np.ndarray]:
    x = run["t_rel"].to_numpy(dtype=float)
    out: dict[str, np.ndarray] = {}
    mask_x = np.isfinite(x)
    for col in columns:
        y = run[col].to_numpy(dtype=float)
        mask = mask_x & np.isfinite(y)
        yi = np.full_like(grid, np.nan, dtype=float)
        if mask.sum() >= 2:
            lo = max(float(np.min(x[mask])), float(grid[0]))
            hi = min(float(np.max(x[mask])), float(grid[-1]))
            valid = (grid >= lo) & (grid <= hi)
            yi[valid] = np.interp(grid[valid], x[mask], y[mask])
        out[col] = yi
    return out


def rms(values: np.ndarray) -> float:
    x = np.asarray(values, dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(x * x)))


def run_source_gain(
    inv: HipKneeInverseDynamics,
    base: dict[str, np.ndarray],
    config_path: Path,
    source: str,
    gain: float,
) -> tuple[np.ndarray, np.ndarray, float]:
    qh = base["q_h"].copy()
    qk = base["q_k"].copy()
    dqh = base["dq_h"].copy()
    dqk = base["dq_k"].copy()
    ddqh = base["qdd_h"].copy()
    ddqk = base["qdd_k"].copy()

    if source == "hip":
        center, q_min, q_max = joint_center_and_limits(config_path, HIP)
        qh = center + gain * (qh - center)
        dqh = gain * dqh
        ddqh = gain * ddqh
        target_plant = load_plant(config_path, KNEE)
        tau_local_target = local_tau(qk, dqk, ddqk, target_plant)
    elif source == "knee":
        center, q_min, q_max = joint_center_and_limits(config_path, KNEE)
        qk = center + gain * (qk - center)
        dqk = gain * dqk
        ddqk = gain * ddqk
        target_plant = load_plant(config_path, HIP)
        tau_local_target = local_tau(qh, dqh, ddqh, target_plant)
    else:
        raise ValueError(f"unknown source: {source}")

    valid = (
        np.isfinite(qh)
        & np.isfinite(qk)
        & np.isfinite(dqh)
        & np.isfinite(dqk)
        & np.isfinite(ddqh)
        & np.isfinite(ddqk)
        & (center + gain * ((base["q_h"] if source == "hip" else base["q_k"]) - center) >= q_min)
        & (center + gain * ((base["q_h"] if source == "hip" else base["q_k"]) - center) <= q_max)
    )
    valid_fraction = float(np.mean(valid)) if valid.size else float("nan")

    row2 = np.full_like(qh, np.nan, dtype=float)
    row3 = np.full_like(qh, np.nan, dtype=float)
    for idx in np.where(valid)[0]:
        tau_full_h, tau_full_k = inv.tau_pair(qh[idx], qk[idx], dqh[idx], dqk[idx], ddqh[idx], ddqk[idx])
        tau_g_h, tau_g_k = inv.tau_pair(qh[idx], qk[idx], 0.0, 0.0, 0.0, 0.0)
        if source == "hip":
            _, tau_hacc_k = inv.tau_pair(qh[idx], qk[idx], 0.0, 0.0, ddqh[idx], 0.0)
            row2[idx] = tau_hacc_k - tau_g_k
            row3[idx] = tau_full_k - tau_local_target[idx]
        else:
            tau_kacc_h, _ = inv.tau_pair(qh[idx], qk[idx], 0.0, 0.0, 0.0, ddqk[idx])
            row2[idx] = tau_kacc_h - tau_g_h
            row3[idx] = tau_full_h - tau_local_target[idx]
    return row2, row3, valid_fraction


def main() -> int:
    out_dir = ROOT / "analysis_artifacts" / "hip_knee_mutual_coupling"
    detail = pd.read_csv(out_dir / "hip_knee_mutual_coupling_timeseries.csv")
    xml_path = ROOT / "h1_official_mujoco" / "h1.xml"
    inv = HipKneeInverseDynamics(xml_path)

    grid = np.arange(STEADY_START, STEADY_STOP + GRID_DT / 2.0, GRID_DT)
    gains = np.round(np.arange(0.5, 2.0 + 1.0e-9, 0.1), 3)
    columns = ["q_h", "q_k", "dq_h", "dq_k", "qdd_h", "qdd_k"]
    rows: list[dict[str, object]] = []
    curve_rows: list[dict[str, object]] = []

    run_groups = list(detail.groupby(["method", "repeat", "log_path", "config_path"], sort=True))
    for gain in gains:
        for source, direction in [("hip", "hip_to_knee"), ("knee", "knee_to_hip")]:
            for method in ["pd", "eid"]:
                row2_curves = []
                row3_curves = []
                valid_fractions = []
                for (run_method, repeat, log_path, config_path_str), run in run_groups:
                    if run_method != method:
                        continue
                    config_path = ROOT / str(config_path_str)
                    base = interp_run(run, grid, columns)
                    row2, row3, valid_fraction = run_source_gain(inv, base, config_path, source, float(gain))
                    row2_curves.append(row2)
                    row3_curves.append(row3)
                    valid_fractions.append(valid_fraction)
                    for t, v2, v3 in zip(grid, row2, row3):
                        curve_rows.append(
                            {
                                "method": method,
                                "repeat": repeat,
                                "direction": direction,
                                "source": source,
                                "gain": gain,
                                "t_rel": t,
                                "row2_acc_torque": v2,
                                "row3_total_residual": v3,
                                "log_path": log_path,
                            }
                        )
                row2_mean = np.nanmean(np.vstack(row2_curves), axis=0)
                row3_mean = np.nanmean(np.vstack(row3_curves), axis=0)
                rows.append(
                    {
                        "method": method,
                        "direction": direction,
                        "source": source,
                        "gain": gain,
                        "row2_acc_torque_rms": rms(row2_mean),
                        "row3_total_residual_rms": rms(row3_mean),
                        "row3_minus_row2_rms": rms(row3_mean - row2_mean),
                        "valid_fraction_mean": float(np.nanmean(valid_fractions)),
                    }
                )

    summary = pd.DataFrame(rows)
    curves = pd.DataFrame(curve_rows)
    summary_path = out_dir / "source_amplitude_sweep_summary.csv"
    curves_path = out_dir / "source_amplitude_sweep_curves.csv"
    summary.to_csv(summary_path, index=False)
    curves.to_csv(curves_path, index=False)

    write_plot(summary, out_dir)
    print(summary_path)
    print(curves_path)
    print(summary.to_string(index=False, float_format=lambda v: f"{v:.4f}"))
    return 0


def write_plot(summary: pd.DataFrame, out_dir: Path) -> None:
    import matplotlib.pyplot as plt

    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    colors = {"pd": "#7f7f7f", "eid": "#0072B2"}
    labels = {"pd": "PD", "eid": "EID"}
    titles = {
        "hip_to_knee": "Increase hip trajectory amplitude",
        "knee_to_hip": "Increase knee trajectory amplitude",
    }
    fig, axes = plt.subplots(1, 2, figsize=(9.6, 3.6), sharey=False)
    for ax, direction in zip(axes, ["hip_to_knee", "knee_to_hip"]):
        for method in ["pd", "eid"]:
            g = summary[(summary["direction"] == direction) & (summary["method"] == method)]
            ax.plot(
                g["gain"],
                g["row3_total_residual_rms"],
                color=colors[method],
                lw=1.8,
                label=f"{labels[method]} row-3 residual",
            )
            ax.plot(
                g["gain"],
                g["row2_acc_torque_rms"],
                color=colors[method],
                lw=1.5,
                ls="--",
                label=f"{labels[method]} row-2 acc term",
            )
        ax.axvline(1.0, color="black", lw=0.7, ls="--", alpha=0.55)
        ax.set_title(titles[direction])
        ax.set_xlabel("source motion amplitude gain")
        ax.set_ylabel("RMS [N m]")
        ax.grid(True, alpha=0.25)
        ax.legend(frameon=False, fontsize=7)
    fig.tight_layout()
    fig.savefig(fig_dir / "source_amplitude_sweep_residual.png", dpi=220, bbox_inches="tight")
    fig.savefig(fig_dir / "source_amplitude_sweep_residual.pdf", bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
