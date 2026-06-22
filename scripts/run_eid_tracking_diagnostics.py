#!/usr/bin/env python3
"""Comprehensive MuJoCo diagnostics for EID tracking error sources.

The script is intentionally self-contained: it writes derived configs under the
chosen output directory and calls scripts/run_mujoco.py. It does not modify the
controller implementation or baseline configs.
"""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml


HIP = 1
KNEE = 2
JOINTS = [HIP, KNEE]
JOINT_LABEL = {HIP: "hip", KNEE: "knee"}


@dataclass(frozen=True)
class JointPolicy:
    center: float
    amplitude: float
    frequency_hz: float
    phase_rad: float


@dataclass(frozen=True)
class Scenario:
    scenario_id: str
    label: str
    hip: JointPolicy
    knee: JointPolicy
    disturbed_joint: int | None = None
    disturbance_torque: float = 0.0
    disturbance_start: float = 1.5
    disturbance_end: float = 2.0
    disturbance_ramp: float = 0.08


@dataclass(frozen=True)
class Method:
    method_id: str
    label: str
    kind: str = "eid"
    ko_scale: float = 1.0
    ku_scale: float = 1.0
    ku_sign: float = 1.0
    filter_alpha: float | None = None
    q_channel: bool = True
    dq_channel: bool = True


METHODS_NO_DIST = [
    Method("pd", "PD", kind="position_pd"),
    Method("eid_ku0", "EID observer, Ku=0", ku_scale=0.0),
    Method("eid_full", "EID full"),
    Method("eid_ku_half", "EID Ku 0.5x", ku_scale=0.5),
    Method("eid_ku_neg", "EID Ku -1x", ku_sign=-1.0),
    Method("eid_q_only", "EID q-only", dq_channel=False),
    Method("eid_dq_only", "EID dq-only", q_channel=False),
    Method("eid_alpha_low", "EID alpha low", filter_alpha=0.005),
    Method("eid_alpha_high", "EID alpha high", filter_alpha=0.08),
]

METHODS_DIST = [
    Method("pd", "PD", kind="position_pd"),
    Method("eid_full", "EID full"),
    Method("eid_ku_half", "EID Ku 0.5x", ku_scale=0.5),
    Method("eid_ku_neg", "EID Ku -1x", ku_sign=-1.0),
    Method("eid_q_only", "EID q-only", dq_channel=False),
    Method("eid_dq_only", "EID dq-only", q_channel=False),
]


def scenarios(include_negative: bool) -> list[Scenario]:
    same = -math.pi / 2.0
    anti_knee = math.pi / 2.0
    base_hip = JointPolicy(center=-0.30, amplitude=0.10, frequency_hz=0.25, phase_rad=same)
    same_knee = JointPolicy(center=0.75, amplitude=0.08, frequency_hz=0.25, phase_rad=same)
    anti_knee_policy = JointPolicy(center=0.75, amplitude=0.08, frequency_hz=0.25, phase_rad=anti_knee)
    out = [
        Scenario("S0_no_disturbance", "same-phase no disturbance", base_hip, same_knee),
        Scenario("S1_hip_pos", "anti-phase hip + torque", base_hip, anti_knee_policy, HIP, 12.0),
        Scenario("S3_knee_pos", "anti-phase knee + torque", base_hip, anti_knee_policy, KNEE, 10.0),
    ]
    if include_negative:
        out.extend([
            Scenario("S2_hip_neg", "anti-phase hip - torque", base_hip, anti_knee_policy, HIP, -12.0),
            Scenario("S4_knee_neg", "anti-phase knee - torque", base_hip, anti_knee_policy, KNEE, -10.0),
        ])
    return out


def configure_joint(joint_cfg: dict, policy: JointPolicy, enabled: bool) -> None:
    joint_cfg["enabled"] = bool(enabled)
    joint_cfg["policy_source"] = "sine"
    joint_cfg["policy_interpolation"] = "open_loop"
    joint_cfg["policy_center"] = float(policy.center)
    joint_cfg["policy_amplitude"] = float(policy.amplitude)
    joint_cfg["policy_frequency_hz"] = float(policy.frequency_hz)
    joint_cfg["policy_phase_rad"] = float(policy.phase_rad)


def apply_method_to_joint(joint_cfg: dict, method: Method) -> None:
    if method.kind == "position_pd":
        return

    base_ko_q = float(joint_cfg.get("observer_gain_q", 0.0))
    base_ko_dq = float(joint_cfg.get("observer_gain_dq", 0.0))
    base_ku_q = float(joint_cfg.get("ku_q", 0.0))
    base_ku_dq = float(joint_cfg.get("ku_dq", 0.0))

    joint_cfg["observer_gain_q"] = base_ko_q * method.ko_scale if method.q_channel else 0.0
    joint_cfg["observer_gain_dq"] = base_ko_dq * method.ko_scale if method.dq_channel else 0.0
    joint_cfg["ku_q"] = base_ku_q * method.ku_scale * method.ku_sign if method.q_channel else 0.0
    joint_cfg["ku_dq"] = base_ku_dq * method.ku_scale * method.ku_sign if method.dq_channel else 0.0
    if method.filter_alpha is not None:
        joint_cfg["filter_alpha"] = float(method.filter_alpha)


def build_config(base_config: Path,
                 out_path: Path,
                 scenario: Scenario,
                 method: Method,
                 filter_alpha_override: float | None = None) -> None:
    cfg = yaml.safe_load(base_config.read_text(encoding="utf-8"))
    cfg["controller"]["kind"] = method.kind
    cfg.pop("software_disturbance", None)
    for raw_joint_id, joint_cfg in cfg["controller"]["joints"].items():
        joint_id = int(raw_joint_id)
        if joint_id == HIP:
            configure_joint(joint_cfg, scenario.hip, True)
            apply_method_to_joint(joint_cfg, method)
            if filter_alpha_override is not None:
                joint_cfg["filter_alpha"] = float(filter_alpha_override)
        elif joint_id == KNEE:
            configure_joint(joint_cfg, scenario.knee, True)
            apply_method_to_joint(joint_cfg, method)
            if filter_alpha_override is not None:
                joint_cfg["filter_alpha"] = float(filter_alpha_override)
        else:
            joint_cfg["enabled"] = False
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True), encoding="utf-8")


def run_mujoco(config: Path,
               out_dir: Path,
               scenario: Scenario,
               duration: float,
               dt: float,
               log_hz: float,
               stepper: Path) -> None:
    cmd = [
        sys.executable,
        "scripts/run_mujoco.py",
        "--config", str(config),
        "--stepper", str(stepper),
        "--out-dir", str(out_dir),
        "--duration", str(duration),
        "--dt", str(dt),
        "--log-hz", str(log_hz),
        "--export-summary",
    ]
    if scenario.disturbed_joint is not None:
        cmd.extend([
            "--disturbance-joints", str(scenario.disturbed_joint),
            "--disturbance-torques", str(scenario.disturbance_torque),
            "--disturbance-start", str(scenario.disturbance_start),
            "--disturbance-end", str(scenario.disturbance_end),
            "--disturbance-ramp", str(scenario.disturbance_ramp),
            "--disturbance-waveform", "smooth_rect",
        ])
    subprocess.run(cmd, check=True)


def load_pair_log(log_path: Path) -> dict[str, np.ndarray]:
    df = pd.read_csv(log_path)
    df = df[df["joint_id"].isin(JOINTS)].copy()
    for col in [
        "eta_u", "eta_q", "eta_dq", "x_bar_q", "x_bar_dq", "u_star",
        "u_feedback", "tau_controller", "tau_disturbance", "tau_before_limit",
        "tau_sent", "saturation_flag", "q_ref_shaped", "dq_ref_shaped",
        "q_actual", "dq_actual",
    ]:
        if col not in df.columns:
            df[col] = 0.0
    pivot = df.pivot_table(
        index="t",
        columns="joint_id",
        values=[
            "q_ref_shaped", "dq_ref_shaped", "q_actual", "dq_actual",
            "eta_u", "eta_q", "eta_dq", "x_bar_q", "x_bar_dq",
            "u_star", "u_feedback", "tau_controller", "tau_disturbance",
            "tau_before_limit", "tau_sent", "saturation_flag",
        ],
        aggfunc="first",
    ).dropna()

    def col(name: str, joint: int) -> np.ndarray:
        return pivot[(name, joint)].to_numpy(dtype=float)

    t = pivot.index.to_numpy(dtype=float)
    return {
        "t": t,
        "q_ref_h": col("q_ref_shaped", HIP), "q_ref_k": col("q_ref_shaped", KNEE),
        "dq_ref_h": col("dq_ref_shaped", HIP), "dq_ref_k": col("dq_ref_shaped", KNEE),
        "q_h": col("q_actual", HIP), "q_k": col("q_actual", KNEE),
        "dq_h": col("dq_actual", HIP), "dq_k": col("dq_actual", KNEE),
        "eta_u_h": col("eta_u", HIP), "eta_u_k": col("eta_u", KNEE),
        "eta_q_h": col("eta_q", HIP), "eta_q_k": col("eta_q", KNEE),
        "eta_dq_h": col("eta_dq", HIP), "eta_dq_k": col("eta_dq", KNEE),
        "x_bar_q_h": col("x_bar_q", HIP), "x_bar_q_k": col("x_bar_q", KNEE),
        "x_bar_dq_h": col("x_bar_dq", HIP), "x_bar_dq_k": col("x_bar_dq", KNEE),
        "u_star_h": col("u_star", HIP), "u_star_k": col("u_star", KNEE),
        "u_feedback_h": col("u_feedback", HIP), "u_feedback_k": col("u_feedback", KNEE),
        "tau_dist_h": col("tau_disturbance", HIP), "tau_dist_k": col("tau_disturbance", KNEE),
        "tau_before_h": col("tau_before_limit", HIP), "tau_before_k": col("tau_before_limit", KNEE),
        "tau_h": col("tau_sent", HIP), "tau_k": col("tau_sent", KNEE),
        "sat_h": col("saturation_flag", HIP), "sat_k": col("saturation_flag", KNEE),
    }


def rms(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(x * x))) if x.size else 0.0


def peak_time(t: np.ndarray, x: np.ndarray) -> float:
    if not x.size:
        return float("nan")
    return float(t[int(np.argmax(np.abs(x)))])


def window(t: np.ndarray, start: float, end: float) -> np.ndarray:
    return (t >= start) & (t <= end)


def dominant_frequency(t: np.ndarray, x: np.ndarray, low_cut_hz: float = 1.0) -> tuple[float, float]:
    if len(t) < 4:
        return 0.0, 0.0
    dt = float(np.median(np.diff(t)))
    y = x - np.mean(x)
    freq = np.fft.rfftfreq(len(y), d=dt)
    amp = np.abs(np.fft.rfft(y)) / max(len(y), 1)
    mask = freq >= low_cut_hz
    if not np.any(mask):
        return 0.0, 0.0
    idxs = np.nonzero(mask)[0]
    idx = idxs[int(np.argmax(amp[mask]))]
    return float(freq[idx]), float(amp[idx])


def metrics(arr: dict[str, np.ndarray], scenario: Scenario) -> dict[str, object]:
    t = arr["t"]
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 1.0
    e_h = arr["q_ref_h"] - arr["q_h"]
    e_k = arr["q_ref_k"] - arr["q_k"]
    e_coord = e_h - e_k
    e_bar_h = arr["q_ref_h"] - arr["x_bar_q_h"]
    e_bar_k = arr["q_ref_k"] - arr["x_bar_q_k"]
    tau_rate_h = np.diff(arr["tau_h"]) / dt if len(t) > 1 else np.array([])
    tau_rate_k = np.diff(arr["tau_k"]) / dt if len(t) > 1 else np.array([])
    eta_u_rate_h = np.diff(arr["eta_u_h"]) / dt if len(t) > 1 else np.array([])
    eta_u_rate_k = np.diff(arr["eta_u_k"]) / dt if len(t) > 1 else np.array([])

    row: dict[str, object] = {
        "rmse_h": rms(e_h),
        "rmse_k": rms(e_k),
        "rmse_coord": rms(e_coord),
        "rmse_bar_h": rms(e_bar_h),
        "rmse_bar_k": rms(e_bar_k),
        "eta_u_h_mean": float(np.mean(arr["eta_u_h"])) if len(t) else 0.0,
        "eta_u_k_mean": float(np.mean(arr["eta_u_k"])) if len(t) else 0.0,
        "eta_u_h_rms": rms(arr["eta_u_h"]),
        "eta_u_k_rms": rms(arr["eta_u_k"]),
        "eta_u_rate_rms": float(np.sqrt(np.mean(eta_u_rate_h * eta_u_rate_h + eta_u_rate_k * eta_u_rate_k))) if len(eta_u_rate_h) else 0.0,
        "tau_rms": float(np.sqrt(np.mean(arr["tau_h"] ** 2 + arr["tau_k"] ** 2))) if len(t) else 0.0,
        "tau_rate_rms": float(np.sqrt(np.mean(tau_rate_h * tau_rate_h + tau_rate_k * tau_rate_k))) if len(tau_rate_h) else 0.0,
        "sat_count": int(np.sum(arr["sat_h"]) + np.sum(arr["sat_k"])),
        "samples": int(len(t)),
    }
    row["eta_u_h_dom_freq"], row["eta_u_h_dom_amp"] = dominant_frequency(t, arr["eta_u_h"])
    row["eta_u_k_dom_freq"], row["eta_u_k_dom_amp"] = dominant_frequency(t, arr["eta_u_k"])
    row["tau_dom_freq"], row["tau_dom_amp"] = dominant_frequency(t, np.hypot(arr["tau_h"], arr["tau_k"]))

    if scenario.disturbed_joint is not None:
        pre = window(t, scenario.disturbance_start - 0.5, scenario.disturbance_start)
        during = window(t, scenario.disturbance_start, scenario.disturbance_end)
        post = window(t, scenario.disturbance_end, scenario.disturbance_end + 0.8)
        for name, mask in [("pre", pre), ("during", during), ("post", post)]:
            row[f"rmse_h_{name}"] = rms(e_h[mask])
            row[f"rmse_k_{name}"] = rms(e_k[mask])
            row[f"rmse_coord_{name}"] = rms(e_coord[mask])
            row[f"eta_u_h_mean_{name}"] = float(np.mean(arr["eta_u_h"][mask])) if np.any(mask) else 0.0
            row[f"eta_u_k_mean_{name}"] = float(np.mean(arr["eta_u_k"][mask])) if np.any(mask) else 0.0
        row["delta_rmse_h_during"] = float(row["rmse_h_during"]) - float(row["rmse_h_pre"])
        row["delta_rmse_k_during"] = float(row["rmse_k_during"]) - float(row["rmse_k_pre"])
        row["delta_rmse_coord_during"] = float(row["rmse_coord_during"]) - float(row["rmse_coord_pre"])
        if scenario.disturbed_joint == HIP:
            row["disturbed_rmse_during"] = row["rmse_h_during"]
            row["undisturbed_rmse_during"] = row["rmse_k_during"]
            row["cross_ratio"] = float(row["rmse_k_during"]) / max(float(row["rmse_h_during"]), 1.0e-9)
            err_sig = e_h[during]
            eta_sig = arr["eta_u_h"][during]
            tau_dist = arr["tau_dist_h"][during]
        else:
            row["disturbed_rmse_during"] = row["rmse_k_during"]
            row["undisturbed_rmse_during"] = row["rmse_h_during"]
            row["cross_ratio"] = float(row["rmse_h_during"]) / max(float(row["rmse_k_during"]), 1.0e-9)
            err_sig = e_k[during]
            eta_sig = arr["eta_u_k"][during]
            tau_dist = arr["tau_dist_k"][during]
        tw = t[during]
        row["t_error_peak"] = peak_time(tw, err_sig)
        row["t_eta_u_peak"] = peak_time(tw, eta_sig)
        row["eta_lag_s"] = float(row["t_eta_u_peak"]) - float(row["t_error_peak"])
        row["eta_disturbance_mean_ratio"] = (
            float(np.mean(eta_sig)) / float(np.mean(tau_dist))
            if np.any(during) and abs(float(np.mean(tau_dist))) > 1.0e-9 else 0.0
        )
    return row


def add_ratios(rows: list[dict[str, object]]) -> None:
    grouped: dict[str, dict[str, dict[str, object]]] = {}
    for row in rows:
        grouped.setdefault(str(row["scenario_id"]), {})[str(row["method_id"])] = row
    for methods in grouped.values():
        pd_row = methods.get("pd")
        if not pd_row:
            continue
        for row in methods.values():
            if "rmse_coord_during" in row and float(pd_row.get("rmse_coord_during", 0.0)) > 0.0:
                row["S_coord_vs_pd"] = float(row["rmse_coord_during"]) / float(pd_row["rmse_coord_during"])
            if "cross_ratio" in row and float(pd_row.get("cross_ratio", 0.0)) > 0.0:
                row["S_cross_vs_pd"] = float(row["cross_ratio"]) / float(pd_row["cross_ratio"])
            if float(pd_row.get("rmse_h", 0.0)) > 0.0:
                row["rmse_h_vs_pd"] = float(row["rmse_h"]) / float(pd_row["rmse_h"])
            if float(pd_row.get("rmse_k", 0.0)) > 0.0:
                row["rmse_k_vs_pd"] = float(row["rmse_k"]) / float(pd_row["rmse_k"])


def write_rows(path: Path, rows: list[dict[str, object]]) -> None:
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def method_color(method_id: str) -> str:
    colors = {
        "pd": "#5F6368",
        "eid_ku0": "#8E6C8A",
        "eid_full": "#1B9E77",
        "eid_ku_half": "#56B4E9",
        "eid_ku_neg": "#D55E00",
        "eid_q_only": "#CC79A7",
        "eid_dq_only": "#E69F00",
        "eid_alpha_low": "#009E73",
        "eid_alpha_high": "#0072B2",
    }
    return colors.get(method_id, "#333333")


def shade(ax: plt.Axes, scenario: Scenario) -> None:
    if scenario.disturbed_joint is not None:
        ax.axvspan(scenario.disturbance_start, scenario.disturbance_end, color="#D9D9D9", alpha=0.42, lw=0)


def plot_no_disturbance(logs: dict[str, dict[str, np.ndarray]], out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    focus = ["pd", "eid_ku0", "eid_full", "eid_ku_half", "eid_q_only", "eid_dq_only"]
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    first = next(iter(logs.values()))
    axes[0].plot(first["t"], first["q_ref_h"], color="black", lw=1.2, label="hip ref")
    axes[1].plot(first["t"], first["q_ref_k"], color="black", lw=1.2, label="knee ref")
    for mid in focus:
        if mid not in logs:
            continue
        a = logs[mid]
        color = method_color(mid)
        e_h = a["q_ref_h"] - a["q_h"]
        e_k = a["q_ref_k"] - a["q_k"]
        e_coord = e_h - e_k
        axes[0].plot(a["t"], a["q_h"], color=color, lw=0.9, label=mid)
        axes[1].plot(a["t"], a["q_k"], color=color, lw=0.9, label=mid)
        axes[2].plot(a["t"], e_coord, color=color, lw=1.0, label=mid)
        axes[3].plot(a["t"], a["eta_u_h"], color=color, lw=0.9, label=f"{mid} hip")
        axes[3].plot(a["t"], a["eta_u_k"], color=color, lw=0.9, ls="--", label=f"{mid} knee")
    axes[0].set_ylabel("hip q [rad]")
    axes[1].set_ylabel("knee q [rad]")
    axes[2].set_ylabel("coord error [rad]")
    axes[3].set_ylabel("eta_u [Nm]")
    axes[3].set_xlabel("time [s]")
    for ax in axes:
        ax.grid(True, alpha=0.24)
        ax.legend(frameon=False, fontsize=7, ncol=3)
    fig.suptitle("D1-D3/D7-D8: no-disturbance EID tracking diagnostics")
    fig.tight_layout()
    fig.savefig(out_dir / "D_no_disturbance_timeseries.png", dpi=180)
    plt.close(fig)


def plot_disturbance_zoom(scenario: Scenario,
                          logs: dict[str, dict[str, np.ndarray]],
                          out_dir: Path) -> None:
    if scenario.disturbed_joint is None:
        return
    out_dir.mkdir(parents=True, exist_ok=True)
    focus = ["pd", "eid_full", "eid_ku_half", "eid_ku_neg", "eid_q_only", "eid_dq_only"]
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    t0 = scenario.disturbance_start - 0.4
    t1 = scenario.disturbance_end + 0.8
    for mid in focus:
        if mid not in logs:
            continue
        a = logs[mid]
        mask = (a["t"] >= t0) & (a["t"] <= t1)
        color = method_color(mid)
        e_h = a["q_ref_h"] - a["q_h"]
        e_k = a["q_ref_k"] - a["q_k"]
        e_coord = e_h - e_k
        axes[0].plot(a["t"][mask], e_h[mask], color=color, lw=1.0, label=f"{mid} hip")
        axes[0].plot(a["t"][mask], e_k[mask], color=color, lw=1.0, ls="--", label=f"{mid} knee")
        axes[1].plot(a["t"][mask], e_coord[mask], color=color, lw=1.0, label=mid)
        eta = a["eta_u_h"] if scenario.disturbed_joint == HIP else a["eta_u_k"]
        axes[2].plot(a["t"][mask], eta[mask], color=color, lw=1.0, label=mid)
        axes[3].plot(a["t"][mask], a["tau_h"][mask], color=color, lw=0.9, label=f"{mid} hip")
        axes[3].plot(a["t"][mask], a["tau_k"][mask], color=color, lw=0.9, ls="--", label=f"{mid} knee")
    first = next(iter(logs.values()))
    mask = (first["t"] >= t0) & (first["t"] <= t1)
    tau_dist = first["tau_dist_h"] if scenario.disturbed_joint == HIP else first["tau_dist_k"]
    axes[3].plot(first["t"][mask], tau_dist[mask], color="#C43C39", lw=1.4, label="disturbance")
    axes[0].set_ylabel("joint error [rad]")
    axes[1].set_ylabel("coord error [rad]")
    axes[2].set_ylabel("eta_u [Nm]")
    axes[3].set_ylabel("tau_sent [Nm]")
    axes[3].set_xlabel("time [s]")
    for ax in axes:
        shade(ax, scenario)
        ax.grid(True, alpha=0.24)
        ax.legend(frameon=False, fontsize=7, ncol=3)
    fig.suptitle(f"{scenario.scenario_id}: disturbance-window diagnostics")
    fig.tight_layout()
    fig.savefig(out_dir / f"{scenario.scenario_id}_window_diagnostics.png", dpi=180)
    plt.close(fig)


def plot_metric_overview(metrics_df: pd.DataFrame, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    no = metrics_df[metrics_df["scenario_id"] == "S0_no_disturbance"].copy()
    no_order = [m.method_id for m in METHODS_NO_DIST if m.method_id in set(no["method_id"])]
    no["method_id"] = pd.Categorical(no["method_id"], no_order, ordered=True)
    no = no.sort_values("method_id")
    fig, axes = plt.subplots(5, 1, figsize=(11, 12), sharex=True)
    specs = [
        ("rmse_coord", "coord RMSE [rad]"),
        ("eta_u_h_rms", "hip eta_u RMS [Nm]"),
        ("eta_u_k_rms", "knee eta_u RMS [Nm]"),
        ("tau_rate_rms", "tau rate RMS [Nm/s]"),
        ("eta_u_rate_rms", "eta_u rate RMS [Nm/s]"),
    ]
    colors = [method_color(str(m)) for m in no["method_id"]]
    x = np.arange(len(no))
    for ax, (col, ylabel) in zip(axes, specs):
        ax.bar(x, no[col].astype(float), color=colors, edgecolor="black", linewidth=0.3)
        ax.set_ylabel(ylabel)
        ax.grid(True, axis="y", alpha=0.24)
    axes[-1].set_xticks(x)
    axes[-1].set_xticklabels(no["method_id"], rotation=25, ha="right")
    fig.suptitle("No-disturbance diagnostic metrics")
    fig.tight_layout()
    fig.savefig(out_dir / "D_no_disturbance_metric_overview.png", dpi=180)
    plt.close(fig)

    dist = metrics_df[metrics_df["scenario_id"] != "S0_no_disturbance"].copy()
    if not dist.empty:
        pivot = dist.pivot_table(
            index="scenario_id",
            columns="method_id",
            values="S_coord_vs_pd",
            aggfunc="first",
        )
        fig, ax = plt.subplots(figsize=(9, 3.6))
        im = ax.imshow(pivot.to_numpy(dtype=float), cmap="viridis", aspect="auto", vmin=0.0, vmax=2.0)
        ax.set_xticks(np.arange(len(pivot.columns)))
        ax.set_xticklabels(pivot.columns, rotation=25, ha="right")
        ax.set_yticks(np.arange(len(pivot.index)))
        ax.set_yticklabels(pivot.index)
        ax.set_title("S_coord vs PD, disturbance windows")
        fig.colorbar(im, ax=ax, label="S_coord")
        fig.tight_layout()
        fig.savefig(out_dir / "D_disturbance_S_coord_heatmap.png", dpi=180)
        plt.close(fig)


def plot_spectrum(logs: dict[str, dict[str, np.ndarray]], out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    focus = ["pd", "eid_full", "eid_q_only", "eid_dq_only", "eid_alpha_high"]
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for mid in focus:
        if mid not in logs:
            continue
        a = logs[mid]
        t = a["t"]
        if len(t) < 4:
            continue
        dt = float(np.median(np.diff(t)))
        freq = np.fft.rfftfreq(len(t), d=dt)
        eta = a["eta_u_h"] - np.mean(a["eta_u_h"])
        tau = np.hypot(a["tau_h"], a["tau_k"])
        tau = tau - np.mean(tau)
        axes[0].plot(freq, np.abs(np.fft.rfft(eta)) / len(t), color=method_color(mid), lw=0.9, label=mid)
        axes[1].plot(freq, np.abs(np.fft.rfft(tau)) / len(t), color=method_color(mid), lw=0.9, label=mid)
    axes[0].set_ylabel("hip eta_u amp")
    axes[1].set_ylabel("tau norm amp")
    axes[1].set_xlabel("frequency [Hz]")
    axes[0].set_xlim(0, 80)
    for ax in axes:
        ax.grid(True, alpha=0.24)
        ax.legend(frameon=False, fontsize=8, ncol=3)
    fig.suptitle("No-disturbance spectrum diagnostic")
    fig.tight_layout()
    fig.savefig(out_dir / "D_no_disturbance_spectrum.png", dpi=180)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-config", type=Path, default=Path("config/h1_hip_knee_dual_tuned.yaml"))
    parser.add_argument("--out-dir", type=Path, default=Path("analysis_artifacts/eid_tracking_diagnostics"))
    parser.add_argument("--stepper", type=Path, default=Path("build/Debug/h1_controller_stepper.exe"))
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--log-hz", type=float, default=500.0)
    parser.add_argument("--filter-alpha", type=float, default=None,
                        help="Override filter_alpha for the enabled hip/knee joints in every generated config.")
    parser.add_argument("--include-negative", action="store_true")
    parser.add_argument("--skip-runs", action="store_true")
    args = parser.parse_args()

    config_dir = args.out_dir / "configs"
    runs_dir = args.out_dir / "runs"
    fig_dir = args.out_dir / "figures"
    all_rows: list[dict[str, object]] = []
    no_dist_logs: dict[str, dict[str, np.ndarray]] = {}
    logs_by_scenario: dict[str, dict[str, dict[str, np.ndarray]]] = {}

    for scenario in scenarios(args.include_negative):
        methods = METHODS_NO_DIST if scenario.disturbed_joint is None else METHODS_DIST
        scenario_logs: dict[str, dict[str, np.ndarray]] = {}
        for method in methods:
            run_name = f"{scenario.scenario_id}__{method.method_id}"
            config_path = config_dir / f"{run_name}.yaml"
            run_dir = runs_dir / run_name
            build_config(args.base_config, config_path, scenario, method, args.filter_alpha)
            if not args.skip_runs:
                run_mujoco(config_path, run_dir, scenario, args.duration, args.dt, args.log_hz, args.stepper)
            arr = load_pair_log(run_dir / "mujoco_closed_loop_log.csv")
            scenario_logs[method.method_id] = arr
            row: dict[str, object] = {
                "scenario_id": scenario.scenario_id,
                "scenario": scenario.label,
                "method_id": method.method_id,
                "method": method.label,
                "disturbed_joint": "" if scenario.disturbed_joint is None else JOINT_LABEL[scenario.disturbed_joint],
                "disturbance_torque": "" if scenario.disturbed_joint is None else scenario.disturbance_torque,
            }
            row.update(metrics(arr, scenario))
            all_rows.append(row)
        logs_by_scenario[scenario.scenario_id] = scenario_logs
        if scenario.disturbed_joint is None:
            no_dist_logs = scenario_logs
        else:
            plot_disturbance_zoom(scenario, scenario_logs, fig_dir)

    add_ratios(all_rows)
    metrics_path = args.out_dir / "eid_tracking_diagnostics_metrics.csv"
    write_rows(metrics_path, all_rows)
    metrics_df = pd.DataFrame(all_rows)
    plot_no_disturbance(no_dist_logs, fig_dir)
    plot_metric_overview(metrics_df, fig_dir)
    plot_spectrum(no_dist_logs, fig_dir)

    print(f"metrics={metrics_path}")
    print(f"figures={fig_dir}")
    print(f"runs={runs_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
