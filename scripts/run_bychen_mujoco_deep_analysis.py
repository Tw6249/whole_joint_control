#!/usr/bin/env python3
"""Frequency and identified augmented closed-loop analysis for the MuJoCo sweep."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml
from scipy import signal

from report_paths import analysis_report_dir, repo_relpath


ROOT = Path(__file__).resolve().parents[1]
HIP = 1
KNEE = 2
JOINTS = [HIP, KNEE]
JOINT_LABELS = {HIP: "Hip", KNEE: "Knee"}
CASES = ["A", "B", "C", "D"]
CASE_COLORS = {
    "A": "#0072B2",
    "B": "#009E73",
    "C": "#E69F00",
    "D": "#D55E00",
}
SIGNAL_COLUMNS = [
    "q_ref_shaped", "q_actual", "dq_actual", "tau_total", "tau_dist",
    "eta_q", "eta_dq", "eta_u",
]
BANDS = {
    "0p5_3hz": (0.5, 3.0),
    "3_15hz": (3.0, 15.0),
    "15_80hz": (15.0, 80.0),
}
DELAY_STEPS = [0, 1, 2, 5]


@dataclass(frozen=True)
class IdentifiedPlant:
    joint_id: int
    label: str
    a: np.ndarray
    b: np.ndarray
    x0: np.ndarray
    u0: float
    r2_q: float
    r2_dq: float
    rmse_q: float
    rmse_dq: float
    samples: int


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


def load_config(config_path: Path) -> dict[str, Any]:
    return yaml.safe_load(config_path.read_text(encoding="utf-8"))


def yaml_joint(cfg: dict[str, Any], joint_id: int) -> dict[str, Any]:
    joints = cfg["controller"]["joints"]
    if joint_id in joints:
        return joints[joint_id]
    return joints[str(joint_id)]


def load_case_log(sweep_dir: Path, case: str) -> pd.DataFrame:
    path = sweep_dir / "runs" / case / "mujoco_closed_loop_log.csv"
    df = pd.read_csv(path)
    df = df[df["joint_id"].isin(JOINTS)].copy()
    df["case"] = case
    return df


def pivot_case(df: pd.DataFrame) -> dict[str, np.ndarray]:
    values = [c for c in SIGNAL_COLUMNS if c in df.columns]
    pivot = df.pivot_table(index="t", columns="joint_id", values=values, aggfunc="first").dropna()
    t = pivot.index.to_numpy(dtype=float)
    out: dict[str, np.ndarray] = {"t": t}
    for joint_id, suffix in [(HIP, "h"), (KNEE, "k")]:
        for col in values:
            out[f"{col}_{suffix}"] = pivot[(col, joint_id)].to_numpy(dtype=float)
    out["e_h"] = out["q_ref_shaped_h"] - out["q_actual_h"]
    out["e_k"] = out["q_ref_shaped_k"] - out["q_actual_k"]
    out["e_coord"] = out["e_h"] - out["e_k"]
    out["tau_dist_coord"] = out["tau_dist_h"] - out["tau_dist_k"]
    out["delta_tau_total_h"] = np.r_[0.0, np.diff(out["tau_total_h"])]
    out["delta_tau_total_k"] = np.r_[0.0, np.diff(out["tau_total_k"])]
    return out


def window_mask(t: np.ndarray, window: str) -> np.ndarray:
    windows = {
        "full": (float(t[0]), float(t[-1]) + 1.0e-12),
        "post_startup": (3.0, float(t[-1]) + 1.0e-12),
        "disturbance": (4.0, 5.4),
        "plateau": (4.2, 5.2),
    }
    start, stop = windows[window]
    return (t >= start) & (t < stop)


def sampling_rate(t: np.ndarray) -> float:
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.002
    return 1.0 / dt


def finite_centered(x: np.ndarray) -> np.ndarray:
    y = np.asarray(x, dtype=float)
    y = np.where(np.isfinite(y), y, np.nan)
    if np.isnan(y).all():
        return np.zeros_like(y)
    y = y - np.nanmean(y)
    return np.nan_to_num(y, nan=0.0)


def welch_psd(x: np.ndarray, fs: float) -> tuple[np.ndarray, np.ndarray]:
    y = finite_centered(x)
    if len(y) < 16:
        return np.array([]), np.array([])
    nperseg = min(512, len(y))
    if len(y) <= 256:
        nperseg = len(y)
    noverlap = max(0, nperseg // 2)
    freq, psd = signal.welch(
        y, fs=fs, window="hann", nperseg=nperseg, noverlap=noverlap,
        detrend="constant", scaling="density",
    )
    return freq, psd


def spectral_params(n: int, max_nperseg: int = 512) -> tuple[int, int, int]:
    nperseg = min(max_nperseg, n)
    if n < max_nperseg:
        nperseg = max(16, n // 2)
    noverlap = max(0, nperseg // 2)
    step = max(1, nperseg - noverlap)
    n_segments = 1 + max(0, (n - nperseg) // step)
    return nperseg, noverlap, n_segments


def csd_coherence(x: np.ndarray, y: np.ndarray, fs: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    x0 = finite_centered(x)
    y0 = finite_centered(y)
    n = min(len(x0), len(y0))
    x0 = x0[:n]
    y0 = y0[:n]
    if n < 16:
        return np.array([]), np.array([]), np.array([])
    nperseg, noverlap, _ = spectral_params(n, max_nperseg=256)
    freq, pxy = signal.csd(
        x0, y0, fs=fs, window="hann", nperseg=nperseg, noverlap=noverlap,
        detrend="constant", scaling="density",
    )
    freq_c, coh = signal.coherence(
        x0, y0, fs=fs, window="hann", nperseg=nperseg, noverlap=noverlap,
        detrend="constant",
    )
    if not np.array_equal(freq, freq_c):
        coh = np.interp(freq, freq_c, coh)
    return freq, pxy, coh


def transfer_function_estimate(
    x: np.ndarray,
    y: np.ndarray,
    fs: float,
    max_nperseg: int = 512,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, int]:
    x0 = finite_centered(x)
    y0 = finite_centered(y)
    n = min(len(x0), len(y0))
    x0 = x0[:n]
    y0 = y0[:n]
    if n < 16:
        return np.array([]), np.array([]), np.array([]), float("nan"), 0
    nperseg, noverlap, n_segments = spectral_params(n, max_nperseg=max_nperseg)
    freq, pxx = signal.welch(
        x0, fs=fs, window="hann", nperseg=nperseg, noverlap=noverlap,
        detrend="constant", scaling="density",
    )
    freq_csd, pxy = signal.csd(
        x0, y0, fs=fs, window="hann", nperseg=nperseg, noverlap=noverlap,
        detrend="constant", scaling="density",
    )
    if not np.array_equal(freq, freq_csd):
        pxy = np.interp(freq, freq_csd, pxy.real) + 1j * np.interp(freq, freq_csd, pxy.imag)
    freq_coh, coh = signal.coherence(
        x0, y0, fs=fs, window="hann", nperseg=nperseg, noverlap=noverlap,
        detrend="constant",
    )
    if not np.array_equal(freq, freq_coh):
        coh = np.interp(freq, freq_coh, coh)
    h = pxy / np.maximum(pxx, 1.0e-24)
    threshold = 1.0 - 0.05 ** (1.0 / max(n_segments - 1, 1))
    return freq, h, coh, threshold, n_segments


def band_power(freq: np.ndarray, psd: np.ndarray, lo: float, hi: float) -> float:
    mask = (freq >= lo) & (freq < hi)
    if np.count_nonzero(mask) < 2:
        return float("nan")
    return float(np.trapezoid(psd[mask], freq[mask]))


def nearest_value(freq: np.ndarray, values: np.ndarray, target: float) -> float:
    if len(freq) == 0:
        return float("nan")
    idx = int(np.argmin(np.abs(freq - target)))
    return float(values[idx])


def nearest_complex(freq: np.ndarray, values: np.ndarray, target: float) -> complex:
    if len(freq) == 0:
        return complex(float("nan"), float("nan"))
    idx = int(np.argmin(np.abs(freq - target)))
    return complex(values[idx])


def mag_db(values: np.ndarray | float) -> np.ndarray | float:
    arr = np.maximum(np.abs(values), 1.0e-18)
    out = 20.0 * np.log10(arr)
    if np.isscalar(values):
        return float(out)
    return out


def analyze_frequency(sweep_dir: Path, out_dir: Path) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    freq_dir = out_dir / "frequency"
    fig_dir = out_dir / "figures"
    freq_dir.mkdir(parents=True, exist_ok=True)
    fig_dir.mkdir(parents=True, exist_ok=True)
    psd_rows: list[dict[str, Any]] = []
    coh_rows: list[dict[str, Any]] = []
    frf_rows: list[dict[str, Any]] = []
    psd_curves: dict[tuple[str, str, str], tuple[np.ndarray, np.ndarray]] = {}
    coh_curves: dict[tuple[str, str], tuple[np.ndarray, np.ndarray]] = {}
    frf_curves: dict[tuple[str, str], tuple[np.ndarray, np.ndarray, np.ndarray, float]] = {}

    signal_specs = [
        ("e_hip", "e_h", "rad"),
        ("e_knee", "e_k", "rad"),
        ("e_coord", "e_coord", "rad"),
        ("tau_total_hip", "tau_total_h", "N m"),
        ("tau_total_knee", "tau_total_k", "N m"),
        ("delta_tau_total_hip", "delta_tau_total_h", "N m/sample"),
        ("delta_tau_total_knee", "delta_tau_total_k", "N m/sample"),
        ("eta_q_hip", "eta_q_h", "rad"),
        ("eta_q_knee", "eta_q_k", "rad"),
        ("eta_dq_hip", "eta_dq_h", "rad/s"),
        ("eta_dq_knee", "eta_dq_k", "rad/s"),
        ("eta_u_hip", "eta_u_h", "N m"),
        ("eta_u_knee", "eta_u_k", "N m"),
    ]
    pair_specs = [
        ("tau_dist_hip_to_e_hip", "tau_dist_h", "e_h"),
        ("tau_dist_knee_to_e_knee", "tau_dist_k", "e_k"),
        ("tau_dist_coord_to_e_coord", "tau_dist_coord", "e_coord"),
        ("eta_u_hip_to_tau_total_hip", "eta_u_h", "tau_total_h"),
        ("eta_u_knee_to_tau_total_knee", "eta_u_k", "tau_total_k"),
    ]

    for case in CASES:
        arrays = pivot_case(load_case_log(sweep_dir, case))
        t = arrays["t"]
        fs = sampling_rate(t)
        for window in ["full", "post_startup", "disturbance"]:
            mask = window_mask(t, window)
            if np.count_nonzero(mask) < 16:
                continue
            for signal_name, key, unit in signal_specs:
                freq, psd = welch_psd(arrays[key][mask], fs)
                if len(freq) == 0:
                    continue
                usable = (freq >= 0.5) & (freq <= 100.0)
                peak_freq = float("nan")
                peak_psd = float("nan")
                if np.any(usable):
                    idx = np.where(usable)[0][int(np.argmax(psd[usable]))]
                    peak_freq = float(freq[idx])
                    peak_psd = float(psd[idx])
                row = {
                    "case": case,
                    "window": window,
                    "signal": signal_name,
                    "unit": unit,
                    "samples": int(np.count_nonzero(mask)),
                    "fs_hz": fs,
                    "peak_freq_hz_0p5_100": peak_freq,
                    "peak_psd": peak_psd,
                    "psd_at_0p8_hz": nearest_value(freq, psd, 0.8),
                    "psd_at_5_hz": nearest_value(freq, psd, 5.0),
                    "psd_at_20_hz": nearest_value(freq, psd, 20.0),
                }
                for band, (lo, hi) in BANDS.items():
                    row[f"bandpower_{band}"] = band_power(freq, psd, lo, hi)
                psd_rows.append(row)
                if window in {"post_startup", "disturbance"} and signal_name in {
                    "e_coord", "tau_total_knee", "delta_tau_total_knee", "eta_u_knee",
                }:
                    psd_curves[(case, window, signal_name)] = (freq, psd)

            for pair_name, in_key, out_key in pair_specs:
                freq, pxy, coh = csd_coherence(arrays[in_key][mask], arrays[out_key][mask], fs)
                if len(freq) == 0:
                    continue
                usable = (freq >= 0.5) & (freq <= 80.0)
                peak_freq = float("nan")
                peak_coh = float("nan")
                phase_at_peak = float("nan")
                cross_mag = float("nan")
                if np.any(usable):
                    idx = np.where(usable)[0][int(np.argmax(coh[usable]))]
                    peak_freq = float(freq[idx])
                    peak_coh = float(coh[idx])
                    phase_at_peak = float(np.angle(pxy[idx]))
                    cross_mag = float(np.abs(pxy[idx]))
                coh_rows.append({
                    "case": case,
                    "window": window,
                    "pair": pair_name,
                    "input_signal": in_key,
                    "output_signal": out_key,
                    "samples": int(np.count_nonzero(mask)),
                    "fs_hz": fs,
                    "peak_coherence_freq_hz_0p5_80": peak_freq,
                    "peak_coherence": peak_coh,
                    "cross_spectrum_mag_at_peak": cross_mag,
                    "cross_spectrum_phase_rad_at_peak": phase_at_peak,
                    "coherence_at_0p8_hz": nearest_value(freq, coh, 0.8),
                    "coherence_at_5_hz": nearest_value(freq, coh, 5.0),
                    "coherence_at_20_hz": nearest_value(freq, coh, 20.0),
                })
                if window == "disturbance" and pair_name in {
                    "tau_dist_hip_to_e_hip", "tau_dist_knee_to_e_knee", "tau_dist_coord_to_e_coord",
                }:
                    coh_curves[(case, pair_name)] = (freq, coh)
                if window == "post_startup" and pair_name in {
                    "tau_dist_hip_to_e_hip", "tau_dist_knee_to_e_knee", "tau_dist_coord_to_e_coord",
                }:
                    f_h, h_est, h_coh, h_threshold, n_segments = transfer_function_estimate(
                        arrays[in_key][mask], arrays[out_key][mask], fs
                    )
                    if len(f_h) == 0:
                        continue
                    usable_h = (f_h >= 0.5) & (f_h <= 80.0)
                    if np.any(usable_h):
                        peak_idx = np.where(usable_h)[0][int(np.argmax(np.abs(h_est[usable_h])))]
                        h_08 = nearest_complex(f_h, h_est, 0.8)
                        h_5 = nearest_complex(f_h, h_est, 5.0)
                        h_20 = nearest_complex(f_h, h_est, 20.0)
                        frf_rows.append({
                            "case": case,
                            "window": window,
                            "pair": pair_name,
                            "input_signal": in_key,
                            "output_signal": out_key,
                            "samples": int(np.count_nonzero(mask)),
                            "fs_hz": fs,
                            "n_segments": n_segments,
                            "coherence_95_threshold": h_threshold,
                            "peak_mag_freq_hz_0p5_80": float(f_h[peak_idx]),
                            "peak_mag_db": float(mag_db(h_est[peak_idx])),
                            "phase_deg_at_peak": float(np.rad2deg(np.angle(h_est[peak_idx]))),
                            "coherence_at_peak_mag": float(h_coh[peak_idx]),
                            "mag_db_at_0p8_hz": float(mag_db(h_08)),
                            "phase_deg_at_0p8_hz": float(np.rad2deg(np.angle(h_08))),
                            "coherence_at_0p8_hz": nearest_value(f_h, h_coh, 0.8),
                            "mag_db_at_5_hz": float(mag_db(h_5)),
                            "phase_deg_at_5_hz": float(np.rad2deg(np.angle(h_5))),
                            "coherence_at_5_hz": nearest_value(f_h, h_coh, 5.0),
                            "mag_db_at_20_hz": float(mag_db(h_20)),
                            "phase_deg_at_20_hz": float(np.rad2deg(np.angle(h_20))),
                            "coherence_at_20_hz": nearest_value(f_h, h_coh, 20.0),
                        })
                        frf_curves[(case, pair_name)] = (f_h, h_est, h_coh, h_threshold)

    psd_df = pd.DataFrame(psd_rows)
    coh_df = pd.DataFrame(coh_rows)
    frf_df = pd.DataFrame(frf_rows)
    psd_df.to_csv(freq_dir / "bychen_mujoco_psd_peaks.csv", index=False)
    coh_df.to_csv(freq_dir / "bychen_mujoco_coherence_summary.csv", index=False)
    frf_df.to_csv(freq_dir / "bychen_mujoco_empirical_frf_summary.csv", index=False)
    plot_psd_curves(psd_curves, fig_dir)
    plot_coherence_curves(coh_curves, fig_dir)
    plot_empirical_frf_bode(frf_curves, fig_dir)
    return psd_df, coh_df, frf_df


def plot_psd_curves(
    curves: dict[tuple[str, str, str], tuple[np.ndarray, np.ndarray]],
    fig_dir: Path,
) -> None:
    specs = [
        ("disturbance", "e_coord", "Coord. error PSD [rad^2/Hz]"),
        ("disturbance", "tau_total_knee", "Knee tau PSD [(N m)^2/Hz]"),
        ("disturbance", "delta_tau_total_knee", r"Knee $\Delta\tau$ PSD"),
        ("disturbance", "eta_u_knee", r"Knee $\eta_u$ PSD"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(8.2, 5.2), sharex=True)
    for ax, (window, sig, ylabel) in zip(axes.ravel(), specs):
        for case in CASES:
            item = curves.get((case, window, sig))
            if item is None:
                continue
            freq, psd = item
            mask = (freq >= 0.5) & (freq <= 100.0)
            ax.semilogy(freq[mask], psd[mask] + 1.0e-18, color=CASE_COLORS[case], label=case, lw=1.15)
        ax.set_ylabel(ylabel)
        ax.set_xlim(0.5, 100.0)
    for ax in axes[-1, :]:
        ax.set_xlabel("frequency [Hz]")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, ncol=4, frameon=False, loc="upper center", bbox_to_anchor=(0.5, 1.00))
    for idx, ax in enumerate(axes.ravel()):
        ax.text(-0.16, 1.06, f"({chr(ord('a') + idx)})", transform=ax.transAxes, fontweight="bold")
    fig.subplots_adjust(left=0.10, right=0.99, bottom=0.11, top=0.88, hspace=0.40, wspace=0.32)
    fig.savefig(fig_dir / "bychen_mujoco_log_psd.png")
    fig.savefig(fig_dir / "bychen_mujoco_log_psd.pdf")
    plt.close(fig)


def plot_coherence_curves(
    curves: dict[tuple[str, str], tuple[np.ndarray, np.ndarray]],
    fig_dir: Path,
) -> None:
    specs = [
        ("tau_dist_hip_to_e_hip", r"Hip: $\tau_d \rightarrow e_h$"),
        ("tau_dist_knee_to_e_knee", r"Knee: $\tau_d \rightarrow e_k$"),
        ("tau_dist_coord_to_e_coord", r"Coord.: $\tau_{d,h}-\tau_{d,k} \rightarrow e_c$"),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(8.4, 2.9), sharey=True)
    for ax, (pair, ylabel) in zip(axes, specs):
        for case in CASES:
            item = curves.get((case, pair))
            if item is None:
                continue
            freq, coh = item
            mask = (freq >= 0.5) & (freq <= 80.0)
            ax.semilogx(freq[mask], coh[mask], color=CASE_COLORS[case], label=case, lw=1.15)
        ax.set_title(ylabel)
        ax.set_xlabel("frequency [Hz]")
        ax.set_ylim(-0.02, 1.02)
        ax.set_xlim(0.5, 80.0)
    axes[0].set_ylabel("coherence")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, ncol=4, frameon=False, loc="upper center", bbox_to_anchor=(0.5, 1.02))
    for idx, ax in enumerate(axes):
        ax.text(-0.18, 1.06, f"({chr(ord('a') + idx)})", transform=ax.transAxes, fontweight="bold")
    fig.subplots_adjust(left=0.08, right=0.99, bottom=0.22, top=0.76, wspace=0.26)
    fig.savefig(fig_dir / "bychen_mujoco_disturbance_coherence.png")
    fig.savefig(fig_dir / "bychen_mujoco_disturbance_coherence.pdf")
    plt.close(fig)


def plot_empirical_frf_bode(
    curves: dict[tuple[str, str], tuple[np.ndarray, np.ndarray, np.ndarray, float]],
    fig_dir: Path,
) -> None:
    specs = [
        ("tau_dist_hip_to_e_hip", r"Hip: $\tau_d \rightarrow e_h$"),
        ("tau_dist_knee_to_e_knee", r"Knee: $\tau_d \rightarrow e_k$"),
        ("tau_dist_coord_to_e_coord", r"Coord.: $\tau_{d,h}-\tau_{d,k} \rightarrow e_c$"),
    ]
    fig, axes = plt.subplots(3, 3, figsize=(8.8, 6.6), sharex=True)
    for c, (pair, label) in enumerate(specs):
        threshold_drawn = False
        for case in CASES:
            item = curves.get((case, pair))
            if item is None:
                continue
            freq, h_est, coh, threshold = item
            mask = (freq >= 0.5) & (freq <= 80.0)
            if not np.any(mask):
                continue
            f = freq[mask]
            h = h_est[mask]
            gamma2 = coh[mask]
            axes[0, c].semilogx(f, mag_db(h), color=CASE_COLORS[case], label=case, lw=1.05)
            axes[1, c].semilogx(
                f,
                np.rad2deg(np.angle(h)),
                color=CASE_COLORS[case],
                label=case,
                lw=1.05,
            )
            axes[2, c].semilogx(f, gamma2, color=CASE_COLORS[case], label=case, lw=1.05)
            if not threshold_drawn and np.isfinite(threshold):
                axes[2, c].axhline(threshold, color="#666666", ls="--", lw=0.8)
                threshold_drawn = True
        axes[0, c].set_title(label)
        axes[1, c].set_ylim(-190.0, 190.0)
        axes[2, c].set_ylim(-0.02, 1.02)
        for r in range(3):
            axes[r, c].set_xlim(0.5, 80.0)
    axes[0, 0].set_ylabel(r"$20\log_{10}|H_1|$")
    axes[1, 0].set_ylabel("phase [deg]")
    axes[2, 0].set_ylabel(r"coherence $\gamma^2$")
    for ax in axes[-1, :]:
        ax.set_xlabel("frequency [Hz]")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, ncol=4, frameon=False, loc="upper center", bbox_to_anchor=(0.5, 0.995))
    for idx, ax in enumerate(axes.ravel()):
        ax.text(-0.18, 1.06, f"({chr(ord('a') + idx)})", transform=ax.transAxes, fontweight="bold")
    fig.subplots_adjust(left=0.08, right=0.99, bottom=0.08, top=0.88, hspace=0.34, wspace=0.30)
    fig.savefig(fig_dir / "bychen_mujoco_empirical_frf_bode.png")
    fig.savefig(fig_dir / "bychen_mujoco_empirical_frf_bode.pdf")
    plt.close(fig)


def fit_identified_plants(sweep_dir: Path, fit_start: float, fit_end: float, ridge: float) -> dict[int, IdentifiedPlant]:
    rows_by_joint: dict[int, list[pd.DataFrame]] = {HIP: [], KNEE: []}
    for case in CASES:
        df = load_case_log(sweep_dir, case)
        for joint_id in JOINTS:
            sub = df[(df["joint_id"] == joint_id) & (df["t"] >= fit_start) & (df["t"] < fit_end)].copy()
            sub = sub.sort_values("t")
            rows_by_joint[joint_id].append(sub)

    plants: dict[int, IdentifiedPlant] = {}
    for joint_id, chunks in rows_by_joint.items():
        phi_blocks = []
        y_blocks = []
        q_all = []
        dq_all = []
        u_all = []
        for sub in chunks:
            q = sub["q_actual"].to_numpy(dtype=float)
            dq = sub["dq_actual"].to_numpy(dtype=float)
            u = sub["tau_total"].to_numpy(dtype=float)
            if len(q) < 3:
                continue
            q_all.append(q[:-1])
            dq_all.append(dq[:-1])
            u_all.append(u[:-1])
        q0 = float(np.mean(np.concatenate(q_all)))
        dq0 = float(np.mean(np.concatenate(dq_all)))
        u0 = float(np.mean(np.concatenate(u_all)))
        x0 = np.array([q0, dq0], dtype=float)

        for sub in chunks:
            q = sub["q_actual"].to_numpy(dtype=float)
            dq = sub["dq_actual"].to_numpy(dtype=float)
            u = sub["tau_total"].to_numpy(dtype=float)
            if len(q) < 3:
                continue
            phi = np.column_stack([q[:-1] - q0, dq[:-1] - dq0, u[:-1] - u0])
            y = np.column_stack([q[1:] - q0, dq[1:] - dq0])
            phi_blocks.append(phi)
            y_blocks.append(y)
        phi = np.vstack(phi_blocks)
        y = np.vstack(y_blocks)
        lhs = phi.T @ phi + ridge * np.eye(phi.shape[1])
        theta = np.linalg.solve(lhs, phi.T @ y)
        y_hat = phi @ theta
        residual = y - y_hat
        sst = np.sum((y - y.mean(axis=0)) ** 2, axis=0)
        sse = np.sum(residual ** 2, axis=0)
        r2 = 1.0 - sse / np.maximum(sst, 1.0e-18)
        rmse = np.sqrt(np.mean(residual ** 2, axis=0))
        a = theta[:2, :].T
        b = theta[2:3, :].T
        plants[joint_id] = IdentifiedPlant(
            joint_id=joint_id,
            label=JOINT_LABELS[joint_id],
            a=a,
            b=b,
            x0=x0,
            u0=u0,
            r2_q=float(r2[0]),
            r2_dq=float(r2[1]),
            rmse_q=float(rmse[0]),
            rmse_dq=float(rmse[1]),
            samples=int(phi.shape[0]),
        )
    return plants


def augmented_mats_identified(
    a: np.ndarray,
    b: np.ndarray,
    kp: float,
    kd: float,
    ko_q: float,
    ko_dq: float,
    ku_q: float,
    ku_dq: float,
    alpha: float,
    delay_steps: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    k = np.array([[kp, kd]], dtype=float)
    ko = np.diag([ko_q, ko_dq])
    ku = np.array([[ku_q, ku_dq]], dtype=float)
    nx = 6 + 2 * delay_steps
    acl = np.zeros((nx, nx), dtype=float)
    bd = np.zeros((nx, 1), dtype=float)
    bv = np.zeros((nx, 2), dtype=float)

    # State order: x, xhat, eta, z1, ..., zm where zi stores x_{k-i}.
    acl[0:2, 0:2] = a
    acl[0:2, 2:4] = -b @ k
    acl[0:2, 4:6] = -b @ (k + ku)
    bd[0:2, 0:1] = b

    acl[2:4, 2:4] = a - b @ k
    acl[2:4, 4:6] = a - b @ (k + ku)

    if delay_steps == 0:
        acl[4:6, 0:2] = alpha * ko
    else:
        z_m_start = 6 + 2 * (delay_steps - 1)
        acl[4:6, z_m_start:z_m_start + 2] = alpha * ko
    acl[4:6, 2:4] = -alpha * ko
    acl[4:6, 4:6] = (1.0 - alpha) * np.eye(2) - alpha * ko
    bv[4:6, :] = alpha * ko

    if delay_steps > 0:
        acl[6:8, 0:2] = np.eye(2)
        for i in range(1, delay_steps):
            dst = 6 + 2 * i
            src = 6 + 2 * (i - 1)
            acl[dst:dst + 2, src:src + 2] = np.eye(2)

    c_q_error = np.zeros((1, nx), dtype=float)
    c_q_error[0, 0] = -1.0
    c_u = np.zeros((1, nx), dtype=float)
    c_u[0, 2:4] = -k
    c_u[0, 4:6] = -(k + ku)
    return acl, bd, bv, c_q_error, c_u


def freq_response_complex(
    a: np.ndarray,
    b: np.ndarray,
    c: np.ndarray,
    freqs: np.ndarray,
    dt: float,
) -> np.ndarray:
    eye = np.eye(a.shape[0])
    out = []
    for f in freqs:
        z = np.exp(1j * 2.0 * np.pi * f * dt)
        out.append(c @ np.linalg.solve(z * eye - a, b))
    return np.stack(out, axis=0)


def freq_response_mag(
    a: np.ndarray,
    b: np.ndarray,
    c: np.ndarray,
    freqs: np.ndarray,
    dt: float,
    input_norm: bool = False,
) -> np.ndarray:
    h = freq_response_complex(a, b, c, freqs, dt)
    if input_norm:
        return np.linalg.norm(h.reshape(h.shape[0], -1), axis=1)
    return np.abs(h.reshape(h.shape[0], -1)[:, 0])


def analyze_linearized(
    sweep_dir: Path,
    out_dir: Path,
    fit_start: float,
    fit_end: float,
    ridge: float,
    dt: float,
) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    lin_dir = out_dir / "linearized"
    fig_dir = out_dir / "figures"
    lin_dir.mkdir(parents=True, exist_ok=True)
    fig_dir.mkdir(parents=True, exist_ok=True)
    plants = fit_identified_plants(sweep_dir, fit_start, fit_end, ridge)
    plant_rows = []
    for plant in plants.values():
        plant_rows.append({
            "joint": plant.label,
            "joint_id": plant.joint_id,
            "a11": plant.a[0, 0],
            "a12": plant.a[0, 1],
            "a21": plant.a[1, 0],
            "a22": plant.a[1, 1],
            "b1": plant.b[0, 0],
            "b2": plant.b[1, 0],
            "q0": plant.x0[0],
            "dq0": plant.x0[1],
            "u0": plant.u0,
            "r2_q": plant.r2_q,
            "r2_dq": plant.r2_dq,
            "rmse_q": plant.rmse_q,
            "rmse_dq": plant.rmse_dq,
            "samples": plant.samples,
        })
    plant_df = pd.DataFrame(plant_rows)
    plant_df.to_csv(lin_dir / "bychen_mujoco_identified_plants.csv", index=False)

    pole_rows = []
    eig_rows = []
    freq_rows = []
    freqs = np.logspace(np.log10(0.1), np.log10(100.0), 400)
    for case in CASES:
        cfg = load_config(sweep_dir / "configs" / f"bychen_mujoco_{case}.yaml")
        defaults = cfg["controller"]["defaults"]
        kp = float(defaults["kp"])
        kd = float(defaults["kd"])
        alpha = float(defaults["filter_alpha"])
        for joint_id in JOINTS:
            jc = yaml_joint(cfg, joint_id)
            ko_q = float(jc.get("observer_gain_q", defaults["observer_gain_q"]))
            ko_dq = float(jc.get("observer_gain_dq", defaults["observer_gain_dq"]))
            ku_q = float(jc.get("ku_q", defaults["ku_q"]))
            ku_dq = float(jc.get("ku_dq", defaults["ku_dq"]))
            plant = plants[joint_id]
            for delay in DELAY_STEPS:
                acl, bd, bv, cq, cu = augmented_mats_identified(
                    plant.a, plant.b, kp, kd, ko_q, ko_dq, ku_q, ku_dq, alpha, delay
                )
                eig = np.linalg.eigvals(acl)
                idx = int(np.argmax(np.abs(eig)))
                for eig_idx, pole in enumerate(eig):
                    eig_rows.append({
                        "case": case,
                        "joint": plant.label,
                        "joint_id": joint_id,
                        "delay_steps": delay,
                        "delay_s": delay * dt,
                        "eig_index": eig_idx,
                        "eig_real": float(pole.real),
                        "eig_imag": float(pole.imag),
                        "eig_abs": float(np.abs(pole)),
                        "eig_angle_rad": float(np.angle(pole)),
                    })
                pole_rows.append({
                    "case": case,
                    "joint": plant.label,
                    "joint_id": joint_id,
                    "delay_steps": delay,
                    "delay_s": delay * dt,
                    "max_pole_magnitude": float(np.max(np.abs(eig))),
                    "dominant_pole_real": float(eig[idx].real),
                    "dominant_pole_imag": float(eig[idx].imag),
                    "num_poles_outside_unit_circle": int(np.count_nonzero(np.abs(eig) >= 1.0)),
                    "ko_q": ko_q,
                    "ko_dq": ko_dq,
                    "ku_q": ku_q,
                    "ku_dq": ku_dq,
                })
                h_dist_complex = freq_response_complex(acl, bd, cq, freqs, dt).reshape(len(freqs), -1)[:, 0]
                h_noise_complex = freq_response_complex(acl, bv, cu, freqs, dt)
                h_dist = np.abs(h_dist_complex)
                h_dist_phase = np.rad2deg(np.unwrap(np.angle(h_dist_complex)))
                h_noise_u = np.linalg.norm(h_noise_complex.reshape(len(freqs), -1), axis=1)
                for response_name, mag, phase, magnitude_type in [
                    ("disturbance_to_q_error", h_dist, h_dist_phase, "siso_abs"),
                    ("measurement_noise_to_u", h_noise_u, None, "largest_singular_value"),
                ]:
                    usable = (freqs >= 0.5) & (freqs <= 80.0)
                    peak_idx = np.where(usable)[0][int(np.argmax(mag[usable]))]
                    phase_at_peak = float(phase[peak_idx]) if phase is not None else float("nan")
                    freq_rows.append({
                        "case": case,
                        "joint": plant.label,
                        "joint_id": joint_id,
                        "delay_steps": delay,
                        "response": response_name,
                        "magnitude_type": magnitude_type,
                        "peak_freq_hz_0p5_80": float(freqs[peak_idx]),
                        "peak_mag": float(mag[peak_idx]),
                        "peak_mag_db": float(mag_db(mag[peak_idx])),
                        "phase_deg_at_peak": phase_at_peak,
                        "mag_at_0p8_hz": nearest_value(freqs, mag, 0.8),
                        "mag_db_at_0p8_hz": nearest_value(freqs, mag_db(mag), 0.8),
                        "phase_deg_at_0p8_hz": nearest_value(freqs, phase, 0.8) if phase is not None else float("nan"),
                        "mag_at_5_hz": nearest_value(freqs, mag, 5.0),
                        "mag_db_at_5_hz": nearest_value(freqs, mag_db(mag), 5.0),
                        "phase_deg_at_5_hz": nearest_value(freqs, phase, 5.0) if phase is not None else float("nan"),
                        "mag_at_20_hz": nearest_value(freqs, mag, 20.0),
                        "mag_db_at_20_hz": nearest_value(freqs, mag_db(mag), 20.0),
                        "phase_deg_at_20_hz": nearest_value(freqs, phase, 20.0) if phase is not None else float("nan"),
                    })
    pole_df = pd.DataFrame(pole_rows)
    eig_df = pd.DataFrame(eig_rows)
    freq_df = pd.DataFrame(freq_rows)
    pole_df.to_csv(lin_dir / "bychen_mujoco_augmented_poles.csv", index=False)
    eig_df.to_csv(lin_dir / "bychen_mujoco_augmented_eigenvalues.csv", index=False)
    freq_df.to_csv(lin_dir / "bychen_mujoco_augmented_frequency_response.csv", index=False)
    plot_pole_delay_heatmap(pole_df, fig_dir)
    plot_z_plane_poles(eig_df, fig_dir)
    plot_identified_bode_disturbance(plants, sweep_dir, fig_dir, dt)
    plot_identified_noise_singular_value(plants, sweep_dir, fig_dir, dt)
    plot_augmented_frequency(plants, sweep_dir, fig_dir, dt)
    return plant_df, pole_df, eig_df, freq_df


def plot_pole_delay_heatmap(pole_df: pd.DataFrame, fig_dir: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(7.25, 2.55), sharey=True)
    for ax, joint in zip(axes, ["Hip", "Knee"]):
        sub = pole_df[pole_df["joint"] == joint]
        grid = np.full((len(DELAY_STEPS), len(CASES)), np.nan)
        for i, delay in enumerate(DELAY_STEPS):
            for j, case in enumerate(CASES):
                item = sub[(sub["delay_steps"] == delay) & (sub["case"] == case)]
                if not item.empty:
                    grid[i, j] = float(item.iloc[0]["max_pole_magnitude"])
        im = ax.imshow(grid, aspect="auto", cmap="magma")
        ax.set_title(f"{joint}: augmented poles")
        ax.set_xticks(range(len(CASES)))
        ax.set_xticklabels(CASES)
        ax.set_yticks(range(len(DELAY_STEPS)))
        ax.set_yticklabels([str(d) for d in DELAY_STEPS])
        ax.set_xlabel("case")
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                if np.isfinite(grid[i, j]):
                    ax.text(j, i, f"{grid[i, j]:.4f}", ha="center", va="center", color="white", fontsize=7)
    axes[0].set_ylabel("measurement delay [steps]")
    fig.subplots_adjust(left=0.08, right=0.86, bottom=0.20, top=0.84, wspace=0.28)
    cbar_ax = fig.add_axes([0.89, 0.20, 0.018, 0.64])
    cbar = fig.colorbar(im, cax=cbar_ax)
    cbar.set_label("max |z|")
    fig.savefig(fig_dir / "bychen_mujoco_augmented_delay_poles.png")
    fig.savefig(fig_dir / "bychen_mujoco_augmented_delay_poles.pdf")
    plt.close(fig)


def plot_z_plane_poles(eig_df: pd.DataFrame, fig_dir: Path) -> None:
    delay_panels = [0, 2]
    fig, axes = plt.subplots(2, 2, figsize=(5.95, 5.45), sharex=True, sharey=True)
    theta = np.linspace(0.0, 2.0 * np.pi, 500)
    for r, joint in enumerate(["Hip", "Knee"]):
        for c, delay in enumerate(delay_panels):
            ax = axes[r, c]
            ax.plot(np.cos(theta), np.sin(theta), color="#444444", lw=0.9, ls="--")
            ax.axhline(0.0, color="#999999", lw=0.5)
            ax.axvline(0.0, color="#999999", lw=0.5)
            for case in CASES:
                sub = eig_df[
                    (eig_df["joint"] == joint)
                    & (eig_df["delay_steps"] == delay)
                    & (eig_df["case"] == case)
                ]
                if sub.empty:
                    continue
                ax.scatter(
                    sub["eig_real"],
                    sub["eig_imag"],
                    s=12,
                    color=CASE_COLORS[case],
                    label=case,
                    alpha=0.82,
                    edgecolors="none",
                )
            ax.set_title(f"{joint}, m={delay}")
            ax.set_aspect("equal", adjustable="box")
            ax.set_xlim(-1.05, 1.05)
            ax.set_ylim(-1.05, 1.05)
    for ax in axes[-1, :]:
        ax.set_xlabel(r"Re$(z)$")
    for ax in axes[:, 0]:
        ax.set_ylabel(r"Im$(z)$")
    axes[0, 0].legend(ncol=4, frameon=False, loc="lower center", bbox_to_anchor=(1.1, 1.16))
    for idx, ax in enumerate(axes.ravel()):
        ax.text(-0.18, 1.06, f"({chr(ord('a') + idx)})", transform=ax.transAxes, fontweight="bold")
    fig.subplots_adjust(left=0.12, right=0.98, bottom=0.10, top=0.88, hspace=0.32, wspace=0.20)
    fig.savefig(fig_dir / "bychen_mujoco_augmented_zplane_poles.png")
    fig.savefig(fig_dir / "bychen_mujoco_augmented_zplane_poles.pdf")
    plt.close(fig)


def plot_identified_bode_disturbance(
    plants: dict[int, IdentifiedPlant],
    sweep_dir: Path,
    fig_dir: Path,
    dt: float,
    delay_steps: int = 0,
) -> None:
    freqs = np.logspace(np.log10(0.1), np.log10(100.0), 500)
    fig, axes = plt.subplots(2, 2, figsize=(8.2, 4.9), sharex=True)
    for c, joint_id in enumerate(JOINTS):
        plant = plants[joint_id]
        for case in CASES:
            cfg = load_config(sweep_dir / "configs" / f"bychen_mujoco_{case}.yaml")
            defaults = cfg["controller"]["defaults"]
            jc = yaml_joint(cfg, joint_id)
            acl, bd, _, cq, _ = augmented_mats_identified(
                plant.a,
                plant.b,
                float(defaults["kp"]),
                float(defaults["kd"]),
                float(jc.get("observer_gain_q", defaults["observer_gain_q"])),
                float(jc.get("observer_gain_dq", defaults["observer_gain_dq"])),
                float(jc.get("ku_q", defaults["ku_q"])),
                float(jc.get("ku_dq", defaults["ku_dq"])),
                float(defaults["filter_alpha"]),
                delay_steps=delay_steps,
            )
            h = freq_response_complex(acl, bd, cq, freqs, dt).reshape(len(freqs), -1)[:, 0]
            axes[0, c].semilogx(freqs, mag_db(h), color=CASE_COLORS[case], label=case, lw=1.15)
            axes[1, c].semilogx(
                freqs,
                np.rad2deg(np.unwrap(np.angle(h))),
                color=CASE_COLORS[case],
                label=case,
                lw=1.15,
            )
        axes[0, c].set_title(f"{plant.label}: disturbance to q error")
        axes[0, c].set_xlim(0.1, 100.0)
    axes[0, 0].set_ylabel(r"$20\log_{10}|G_{de}|$")
    axes[1, 0].set_ylabel("phase [deg]")
    for ax in axes[-1, :]:
        ax.set_xlabel("frequency [Hz]")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, ncol=4, frameon=False, loc="upper center", bbox_to_anchor=(0.5, 1.00))
    for idx, ax in enumerate(axes.ravel()):
        ax.text(-0.16, 1.06, f"({chr(ord('a') + idx)})", transform=ax.transAxes, fontweight="bold")
    fig.subplots_adjust(left=0.10, right=0.99, bottom=0.12, top=0.84, hspace=0.35, wspace=0.25)
    fig.savefig(fig_dir / "bychen_mujoco_identified_bode_disturbance.png")
    fig.savefig(fig_dir / "bychen_mujoco_identified_bode_disturbance.pdf")
    plt.close(fig)


def plot_identified_noise_singular_value(
    plants: dict[int, IdentifiedPlant],
    sweep_dir: Path,
    fig_dir: Path,
    dt: float,
    delay_steps: int = 0,
) -> None:
    freqs = np.logspace(np.log10(0.1), np.log10(100.0), 500)
    fig, axes = plt.subplots(1, 2, figsize=(8.0, 2.9), sharey=True)
    for ax, joint_id in zip(axes, JOINTS):
        plant = plants[joint_id]
        for case in CASES:
            cfg = load_config(sweep_dir / "configs" / f"bychen_mujoco_{case}.yaml")
            defaults = cfg["controller"]["defaults"]
            jc = yaml_joint(cfg, joint_id)
            acl, _, bv, _, cu = augmented_mats_identified(
                plant.a,
                plant.b,
                float(defaults["kp"]),
                float(defaults["kd"]),
                float(jc.get("observer_gain_q", defaults["observer_gain_q"])),
                float(jc.get("observer_gain_dq", defaults["observer_gain_dq"])),
                float(jc.get("ku_q", defaults["ku_q"])),
                float(jc.get("ku_dq", defaults["ku_dq"])),
                float(defaults["filter_alpha"]),
                delay_steps=delay_steps,
            )
            h = freq_response_complex(acl, bv, cu, freqs, dt)
            sigma = np.linalg.norm(h.reshape(len(freqs), -1), axis=1)
            ax.semilogx(freqs, mag_db(sigma), color=CASE_COLORS[case], label=case, lw=1.15)
        ax.set_title(f"{plant.label}: measurement noise to control")
        ax.set_xlabel("frequency [Hz]")
        ax.set_xlim(0.1, 100.0)
    axes[0].set_ylabel(r"$20\log_{10}\bar{\sigma}(G_{vu})$")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, ncol=4, frameon=False, loc="upper center", bbox_to_anchor=(0.5, 1.02))
    for idx, ax in enumerate(axes):
        ax.text(-0.16, 1.06, f"({chr(ord('a') + idx)})", transform=ax.transAxes, fontweight="bold")
    fig.subplots_adjust(left=0.10, right=0.99, bottom=0.20, top=0.74, wspace=0.18)
    fig.savefig(fig_dir / "bychen_mujoco_identified_noise_singular_value.png")
    fig.savefig(fig_dir / "bychen_mujoco_identified_noise_singular_value.pdf")
    plt.close(fig)


def plot_augmented_frequency(
    plants: dict[int, IdentifiedPlant],
    sweep_dir: Path,
    fig_dir: Path,
    dt: float,
) -> None:
    freqs = np.linspace(0.1, 100.0, 300)
    fig, axes = plt.subplots(2, 2, figsize=(8.2, 5.1), sharex=True)
    for r, joint_id in enumerate(JOINTS):
        plant = plants[joint_id]
        for case in CASES:
            cfg = load_config(sweep_dir / "configs" / f"bychen_mujoco_{case}.yaml")
            defaults = cfg["controller"]["defaults"]
            jc = yaml_joint(cfg, joint_id)
            acl, bd, bv, cq, cu = augmented_mats_identified(
                plant.a,
                plant.b,
                float(defaults["kp"]),
                float(defaults["kd"]),
                float(jc.get("observer_gain_q", defaults["observer_gain_q"])),
                float(jc.get("observer_gain_dq", defaults["observer_gain_dq"])),
                float(jc.get("ku_q", defaults["ku_q"])),
                float(jc.get("ku_dq", defaults["ku_dq"])),
                float(defaults["filter_alpha"]),
                delay_steps=0,
            )
            h_dist = freq_response_mag(acl, bd, cq, freqs, dt)
            h_noise_u = freq_response_mag(acl, bv, cu, freqs, dt, input_norm=True)
            axes[r, 0].semilogy(freqs, h_dist + 1.0e-18, color=CASE_COLORS[case], label=case, lw=1.15)
            axes[r, 1].semilogy(freqs, h_noise_u + 1.0e-18, color=CASE_COLORS[case], label=case, lw=1.15)
        axes[r, 0].set_ylabel(f"{plant.label}\n|d -> e_q|")
        axes[r, 1].set_ylabel(f"{plant.label}\n|v -> u|")
        axes[r, 0].set_xlim(0.1, 100.0)
    axes[0, 0].set_title("Input disturbance to q error")
    axes[0, 1].set_title("Measurement noise to control")
    for ax in axes[-1, :]:
        ax.set_xlabel("frequency [Hz]")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, ncol=4, frameon=False, loc="upper center", bbox_to_anchor=(0.5, 1.00))
    for idx, ax in enumerate(axes.ravel()):
        ax.text(-0.18, 1.06, f"({chr(ord('a') + idx)})", transform=ax.transAxes, fontweight="bold")
    fig.subplots_adjust(left=0.11, right=0.99, bottom=0.11, top=0.84, hspace=0.42, wspace=0.32)
    fig.savefig(fig_dir / "bychen_mujoco_identified_augmented_frequency_response.png")
    fig.savefig(fig_dir / "bychen_mujoco_identified_augmented_frequency_response.pdf")
    plt.close(fig)


def markdown_table(df: pd.DataFrame, columns: list[str], floatfmt: str = ".6f") -> str:
    header = "| " + " | ".join(columns) + " |"
    divider = "| " + " | ".join(["---"] * len(columns)) + " |"
    lines = [header, divider]
    for _, row in df[columns].iterrows():
        cells = []
        for col in columns:
            val = row[col]
            if isinstance(val, (float, np.floating)):
                cells.append(format(float(val), floatfmt))
            else:
                cells.append(str(val))
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def write_summary(
    out_dir: Path,
    psd_df: pd.DataFrame,
    coh_df: pd.DataFrame,
    frf_df: pd.DataFrame,
    plant_df: pd.DataFrame,
    pole_df: pd.DataFrame,
    eig_df: pd.DataFrame,
    freq_df: pd.DataFrame,
) -> Path:
    report_dir = analysis_report_dir(out_dir)
    report_dir.mkdir(parents=True, exist_ok=True)
    summary = report_dir / "bychen_mujoco_deep_summary.md"
    psd_sel = psd_df[
        (psd_df["window"] == "disturbance")
        & (psd_df["signal"].isin(["e_coord", "delta_tau_total_knee", "eta_u_knee"]))
    ][["case", "signal", "peak_freq_hz_0p5_100", "peak_psd", "bandpower_0p5_3hz", "bandpower_15_80hz"]]
    coh_sel = coh_df[
        (coh_df["window"] == "disturbance")
        & (coh_df["pair"] == "tau_dist_coord_to_e_coord")
    ][["case", "peak_coherence_freq_hz_0p5_80", "peak_coherence", "cross_spectrum_phase_rad_at_peak"]]
    frf_sel = frf_df[
        frf_df["pair"] == "tau_dist_coord_to_e_coord"
    ][["case", "peak_mag_freq_hz_0p5_80", "peak_mag_db", "phase_deg_at_peak", "coherence_at_peak_mag", "coherence_95_threshold"]]
    poles_sel = pole_df[
        (pole_df["delay_steps"].isin([0, 2]))
    ][["case", "joint", "delay_steps", "max_pole_magnitude", "num_poles_outside_unit_circle"]]
    dist_freq_sel = freq_df[
        (freq_df["delay_steps"] == 0)
        & (freq_df["response"] == "disturbance_to_q_error")
    ][["case", "joint", "peak_freq_hz_0p5_80", "peak_mag_db", "phase_deg_at_peak", "mag_db_at_20_hz"]]
    freq_sel = freq_df[
        (freq_df["delay_steps"] == 0)
        & (freq_df["response"] == "measurement_noise_to_u")
    ][["case", "joint", "peak_freq_hz_0p5_80", "peak_mag_db", "mag_db_at_20_hz"]]

    lines = [
        "# Bychen MuJoCo deep analysis",
        "",
        "Scope: post-processing of the already generated A-D MuJoCo logs. "
        "The frequency layer reports Welch PSD/CSD/coherence and a closed-loop empirical H1 FRF estimate. "
        "The augmented layer identifies a local discrete plant from MuJoCo logs and then constructs the EID augmented matrix with the controller's prior/update timing. "
        f"The eigenvalue table contains {len(eig_df)} poles across cases, joints, and delay settings.",
        "",
        "## Frequency layer: selected disturbance-window PSD peaks",
        "",
        markdown_table(psd_sel, psd_sel.columns.tolist(), ".6g"),
        "",
        "## Frequency layer: disturbance-to-coordination coherence",
        "",
        markdown_table(coh_sel, coh_sel.columns.tolist(), ".6g"),
        "",
        "## Frequency layer: closed-loop empirical FRF H1 estimate",
        "",
        markdown_table(frf_sel, frf_sel.columns.tolist(), ".6g"),
        "",
        "## Identified MuJoCo local plants",
        "",
        markdown_table(
            plant_df,
            ["joint", "a11", "a12", "a21", "a22", "b1", "b2", "r2_q", "r2_dq", "rmse_q", "rmse_dq", "samples"],
            ".6g",
        ),
        "",
        "## Identified augmented closed-loop poles",
        "",
        markdown_table(poles_sel, poles_sel.columns.tolist(), ".6g"),
        "",
        "## Identified augmented Bode response: input disturbance to q error",
        "",
        markdown_table(dist_freq_sel, dist_freq_sel.columns.tolist(), ".6g"),
        "",
        "## Identified augmented singular-value response: measurement noise to control",
        "",
        markdown_table(freq_sel, freq_sel.columns.tolist(), ".6g"),
        "",
        "Generated files:",
        "",
        f"- `{repo_relpath(out_dir / 'frequency' / 'bychen_mujoco_psd_peaks.csv')}`",
        f"- `{repo_relpath(out_dir / 'frequency' / 'bychen_mujoco_coherence_summary.csv')}`",
        f"- `{repo_relpath(out_dir / 'frequency' / 'bychen_mujoco_empirical_frf_summary.csv')}`",
        f"- `{repo_relpath(out_dir / 'linearized' / 'bychen_mujoco_identified_plants.csv')}`",
        f"- `{repo_relpath(out_dir / 'linearized' / 'bychen_mujoco_augmented_poles.csv')}`",
        f"- `{repo_relpath(out_dir / 'linearized' / 'bychen_mujoco_augmented_eigenvalues.csv')}`",
        f"- `{repo_relpath(out_dir / 'linearized' / 'bychen_mujoco_augmented_frequency_response.csv')}`",
        f"- `{repo_relpath(out_dir / 'figures' / 'bychen_mujoco_log_psd.png')}`",
        f"- `{repo_relpath(out_dir / 'figures' / 'bychen_mujoco_disturbance_coherence.png')}`",
        f"- `{repo_relpath(out_dir / 'figures' / 'bychen_mujoco_empirical_frf_bode.png')}`",
        f"- `{repo_relpath(out_dir / 'figures' / 'bychen_mujoco_augmented_delay_poles.png')}`",
        f"- `{repo_relpath(out_dir / 'figures' / 'bychen_mujoco_augmented_zplane_poles.png')}`",
        f"- `{repo_relpath(out_dir / 'figures' / 'bychen_mujoco_identified_bode_disturbance.png')}`",
        f"- `{repo_relpath(out_dir / 'figures' / 'bychen_mujoco_identified_noise_singular_value.png')}`",
        f"- `{repo_relpath(out_dir / 'figures' / 'bychen_mujoco_identified_augmented_frequency_response.png')}`",
        "",
    ]
    summary.write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sweep-dir", type=Path, default=Path("analysis_artifacts/bychen_mujoco_sweep"))
    parser.add_argument("--out-dir", type=Path, default=Path("analysis_artifacts/bychen_mujoco_sweep/mujoco_deep_analysis"))
    parser.add_argument("--fit-start", type=float, default=3.0)
    parser.add_argument("--fit-end", type=float, default=7.8)
    parser.add_argument("--ridge", type=float, default=1.0e-10)
    parser.add_argument("--dt", type=float, default=0.002)
    args = parser.parse_args()

    sweep_dir = args.sweep_dir
    if not sweep_dir.is_absolute():
        sweep_dir = ROOT / sweep_dir
    out_dir = args.out_dir
    if not out_dir.is_absolute():
        out_dir = ROOT / out_dir

    configure_matplotlib()
    out_dir.mkdir(parents=True, exist_ok=True)
    psd_df, coh_df, frf_df = analyze_frequency(sweep_dir, out_dir)
    plant_df, pole_df, eig_df, freq_df = analyze_linearized(
        sweep_dir, out_dir, args.fit_start, args.fit_end, args.ridge, args.dt
    )
    summary_path = write_summary(out_dir, psd_df, coh_df, frf_df, plant_df, pole_df, eig_df, freq_df)
    print(f"summary={summary_path}")
    print(f"frequency={out_dir / 'frequency'}")
    print(f"linearized={out_dir / 'linearized'}")
    print(f"figures={out_dir / 'figures'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
