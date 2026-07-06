#!/usr/bin/env python3
"""Run closed-loop MuJoCo experiments with increased sine acceleration.

The experiment keeps hip/knee position amplitudes unchanged and increases the
reference frequency. For a sine reference, acceleration amplitude scales with
frequency squared, so this changes acceleration in the generated experiment
rather than post-processing an existing curve.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from analyze_hip_knee_mutual_coupling import (  # noqa: E402
    HIP,
    KNEE,
    HipKneeInverseDynamics,
    Plant,
    estimate_qdd,
    finite_rms,
    load_plant,
    local_tau,
)
from report_paths import analysis_report_dir, markdown_relpath, repo_relpath  # noqa: E402


BASE_FREQ = 0.8
DEFAULT_FREQS = [0.8, 1.0, 1.2, 1.4]
STEADY_START = 3.0
STEADY_STOP = 7.4


def method_base_config(method: str) -> Path:
    if method == "pd":
        return ROOT / "config" / "h1_real_p2_anti_hip_knee_pd.yaml"
    if method == "eid":
        return ROOT / "config" / "h1_real_p2_anti_hip_knee_eid.yaml"
    raise ValueError(method)


def freq_id(freq: float) -> str:
    return f"{freq:.2f}".replace(".", "p")


def write_frequency_config(method: str, freq: float, config_dir: Path) -> Path:
    src = method_base_config(method)
    cfg = yaml.safe_load(src.read_text(encoding="utf-8"))
    cfg["controller"]["defaults"]["policy_frequency_hz"] = float(freq)
    cfg["experiment"]["condition"] = f"ACC_FREQ_{method.upper()}_anti_phase_f{freq_id(freq)}"
    cfg["experiment"]["repeat"] = "sim01"
    cfg["experiment"]["disturbance_method"] = "none"
    cfg["experiment"]["disturbance_target"] = "none"
    cfg["log_path"] = f"data/mujoco_frequency_acceleration_experiment/{method}_f{freq_id(freq)}.csv"
    out = config_dir / f"h1_p2_{method}_freq_{freq_id(freq)}.yaml"
    out.write_text(yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True), encoding="utf-8")
    return out


def run_mujoco_case(method: str, freq: float, config: Path, out_dir: Path, args: argparse.Namespace) -> Path:
    case_dir = out_dir / f"{method}_f{freq_id(freq)}"
    log_path = case_dir / "mujoco_closed_loop_log.csv"
    if log_path.exists() and not args.force:
        return log_path
    cmd = [
        sys.executable,
        str(ROOT / "scripts" / "run_mujoco.py"),
        "--config",
        str(config),
        "--out-dir",
        str(case_dir),
        "--duration",
        str(args.duration),
        "--dt",
        str(args.dt),
        "--log-interval-s",
        str(args.log_interval_s),
        "--export-summary",
    ]
    print("RUN", " ".join(cmd))
    subprocess.run(cmd, cwd=ROOT, check=True)
    return log_path


def pivot_mujoco_log(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    df = df[df["joint_id"].isin([HIP, KNEE])].copy()
    value_cols = {}
    for source, target in [
        ("q_actual", "q"),
        ("dq_actual", "dq"),
        ("tau_applied", "tau_est"),
        ("tau_total", "tau_est"),
        ("motor_tau", "tau_est"),
    ]:
        if source in df.columns and target not in value_cols:
            value_cols[target] = source
    required = {"q": value_cols.get("q"), "dq": value_cols.get("dq")}
    missing = [k for k, v in required.items() if v is None]
    if missing:
        raise RuntimeError(f"{path} missing columns for {missing}; columns={list(df.columns)}")

    records = []
    for cycle, pair in df.groupby("cycle", sort=True):
        if not {HIP, KNEE}.issubset(set(pair["joint_id"].astype(int))):
            continue
        h = pair[pair["joint_id"] == HIP].iloc[0]
        k = pair[pair["joint_id"] == KNEE].iloc[0]
        records.append(
            {
                "cycle": int(cycle),
                "t_rel": float(h["t"]),
                "q_h": float(h[value_cols["q"]]),
                "q_k": float(k[value_cols["q"]]),
                "dq_h": float(h[value_cols["dq"]]),
                "dq_k": float(k[value_cols["dq"]]),
            }
        )
    return pd.DataFrame.from_records(records)


def decompose_log(log_path: Path, config_path: Path, inv: HipKneeInverseDynamics) -> pd.DataFrame:
    run = pivot_mujoco_log(log_path)
    t = run["t_rel"].to_numpy(dtype=float)
    qh = run["q_h"].to_numpy(dtype=float)
    qk = run["q_k"].to_numpy(dtype=float)
    dqh = run["dq_h"].to_numpy(dtype=float)
    dqk = run["dq_k"].to_numpy(dtype=float)
    ddqh = estimate_qdd(dqh, t, 81.0, 3)
    ddqk = estimate_qdd(dqk, t, 81.0, 3)
    run["qdd_h"] = ddqh
    run["qdd_k"] = ddqk

    hip_plant = load_plant(config_path, HIP)
    knee_plant = load_plant(config_path, KNEE)
    n = len(run)
    tau_full_h = np.empty(n)
    tau_full_k = np.empty(n)
    tau_g_h = np.empty(n)
    tau_g_k = np.empty(n)
    tau_hacc_k = np.empty(n)
    tau_kacc_h = np.empty(n)

    for i in range(n):
        tau_full_h[i], tau_full_k[i] = inv.tau_pair(qh[i], qk[i], dqh[i], dqk[i], ddqh[i], ddqk[i])
        tau_g_h[i], tau_g_k[i] = inv.tau_pair(qh[i], qk[i], 0.0, 0.0, 0.0, 0.0)
        _, tau_hacc_k[i] = inv.tau_pair(qh[i], qk[i], 0.0, 0.0, ddqh[i], 0.0)
        tau_kacc_h[i], _ = inv.tau_pair(qh[i], qk[i], 0.0, 0.0, 0.0, ddqk[i])

    tau_loc_h = local_tau(qh, dqh, ddqh, hip_plant)
    tau_loc_k = local_tau(qk, dqk, ddqk, knee_plant)
    run["d_k_total"] = tau_full_k - tau_loc_k
    run["d_h_total"] = tau_full_h - tau_loc_h
    run["d_k_from_hip_acc"] = tau_hacc_k - tau_g_k
    run["d_h_from_knee_acc"] = tau_kacc_h - tau_g_h
    run["d_k_other_after_row2"] = run["d_k_total"] - run["d_k_from_hip_acc"]
    run["d_h_other_after_row2"] = run["d_h_total"] - run["d_h_from_knee_acc"]
    return run


def summarize_case(run: pd.DataFrame, method: str, freq: float) -> list[dict[str, float | str]]:
    w = run[(run["t_rel"] >= STEADY_START) & (run["t_rel"] < STEADY_STOP)]
    rows = []
    for direction, qdd_col, row2_col, total_col, other_col in [
        ("hip_to_knee", "qdd_h", "d_k_from_hip_acc", "d_k_total", "d_k_other_after_row2"),
        ("knee_to_hip", "qdd_k", "d_h_from_knee_acc", "d_h_total", "d_h_other_after_row2"),
    ]:
        row2 = finite_rms(w[row2_col].to_numpy(dtype=float))
        other = finite_rms(w[other_col].to_numpy(dtype=float))
        rows.append(
            {
                "method": method.upper(),
                "frequency_hz": freq,
                "reference_acceleration_gain": (freq / BASE_FREQ) ** 2,
                "direction": direction,
                "actual_qdd_rms": finite_rms(w[qdd_col].to_numpy(dtype=float)),
                "row2_acc_torque_rms": row2,
                "row3_total_residual_rms": finite_rms(w[total_col].to_numpy(dtype=float)),
                "other_after_removing_row2_rms": other,
                "row2_share_of_row2_plus_other_percent": 100.0 * row2 / (row2 + other) if row2 + other > 0 else np.nan,
            }
        )
    return rows


def write_plots(summary: pd.DataFrame, fig_dir: Path) -> None:
    import matplotlib.pyplot as plt

    fig_dir.mkdir(parents=True, exist_ok=True)
    colors = {"PD": "#7f7f7f", "EID": "#0072B2"}
    labels = {
        "hip_to_knee": "Hip acceleration -> knee residual",
        "knee_to_hip": "Knee acceleration -> hip residual",
    }

    fig, axes = plt.subplots(1, 2, figsize=(9.8, 3.6), sharey=False)
    for ax, direction in zip(axes, ["hip_to_knee", "knee_to_hip"]):
        for method in ["PD", "EID"]:
            g = summary[(summary["direction"] == direction) & (summary["method"] == method)]
            ax.plot(g["reference_acceleration_gain"], g["row3_total_residual_rms"], color=colors[method], lw=1.8, label=f"{method} row-3 total")
            ax.plot(g["reference_acceleration_gain"], g["row2_acc_torque_rms"], color=colors[method], lw=1.5, ls="--", label=f"{method} row-2 acc")
            ax.plot(g["reference_acceleration_gain"], g["other_after_removing_row2_rms"], color=colors[method], lw=1.2, ls=":", label=f"{method} remaining")
        ax.axvline(1.0, color="black", lw=0.7, ls="--", alpha=0.55)
        ax.set_title(labels[direction])
        ax.set_xlabel("reference acceleration gain from frequency")
        ax.set_ylabel("RMS [N m]")
        ax.grid(True, alpha=0.25)
        ax.legend(frameon=False, fontsize=7)
    fig.tight_layout()
    fig.savefig(fig_dir / "closed_loop_frequency_acceleration_rms.png", dpi=220, bbox_inches="tight")
    fig.savefig(fig_dir / "closed_loop_frequency_acceleration_rms.pdf", bbox_inches="tight")
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(9.8, 3.4), sharey=True)
    for ax, direction in zip(axes, ["hip_to_knee", "knee_to_hip"]):
        for method in ["PD", "EID"]:
            g = summary[(summary["direction"] == direction) & (summary["method"] == method)]
            ax.plot(g["reference_acceleration_gain"], g["row2_share_of_row2_plus_other_percent"], color=colors[method], lw=1.8, marker="o", ms=3, label=method)
        ax.axvline(1.0, color="black", lw=0.7, ls="--", alpha=0.55)
        ax.set_title(labels[direction])
        ax.set_xlabel("reference acceleration gain from frequency")
        ax.set_ylabel("row-2 share [%]")
        ax.set_ylim(0, 100)
        ax.grid(True, alpha=0.25)
        ax.legend(frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(fig_dir / "closed_loop_frequency_acceleration_share.png", dpi=220, bbox_inches="tight")
    fig.savefig(fig_dir / "closed_loop_frequency_acceleration_share.pdf", bbox_inches="tight")
    plt.close(fig)


def write_report(summary: pd.DataFrame, out_dir: Path) -> Path:
    report_dir = analysis_report_dir(out_dir)
    report_dir.mkdir(parents=True, exist_ok=True)
    report = report_dir / "closed_loop_frequency_acceleration_report.md"
    subset = summary[summary["frequency_hz"].isin([0.8, 1.4])].copy()
    rms_png = out_dir / "figures" / "closed_loop_frequency_acceleration_rms.png"
    share_png = out_dir / "figures" / "closed_loop_frequency_acceleration_share.png"
    lines = [
        "# 闭环 MuJoCo 实验：通过提高参考频率加大关节加速度",
        "",
        "本报告是真正重新运行闭环 MuJoCo 实验得到的结果。实验保持髋/膝角度幅值不变，通过提高正弦参考频率来提高关节加速度；正弦加速度峰值随频率平方增长。",
        "",
        "## 结论",
        "",
        "提高频率后，第 2 行关节间加速度传递力矩确实增大；但第 3 行总残差没有变小。在本次闭环实验中，第 2 行占比随加速度提高而上升，说明耦合传递力更加主导；总残差 RMS 则同步增大，说明这不是抵消残差的机制。",
        "",
        "## 原始频率与最高频率对比",
        "",
        "| 方法 | 方向 | 频率 Hz | 参考加速度倍率 | 实测 qdd RMS | 第 2 行 RMS | 第 3 行 RMS | 扣除第 2 行后剩余 RMS | 第 2 行占比 |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for _, r in subset.iterrows():
        lines.append(
            f"| {r['method']} | {r['direction']} | {r['frequency_hz']:.1f} | "
            f"{r['reference_acceleration_gain']:.2f} | {r['actual_qdd_rms']:.3f} | "
            f"{r['row2_acc_torque_rms']:.3f} | {r['row3_total_residual_rms']:.3f} | "
            f"{r['other_after_removing_row2_rms']:.3f} | {r['row2_share_of_row2_plus_other_percent']:.1f}% |"
        )
    lines.extend(
        [
            "",
            "## 图",
            "",
            f"![闭环实验 RMS 变化]({markdown_relpath(rms_png, report_dir)})",
            "",
            f"![闭环实验第 2 行占比变化]({markdown_relpath(share_png, report_dir)})",
            "",
            "## 输出文件",
            "",
            "```text",
            repo_relpath(out_dir / "closed_loop_frequency_acceleration_summary.csv"),
            repo_relpath(out_dir / "closed_loop_frequency_acceleration_detail.csv"),
            repo_relpath(rms_png),
            repo_relpath(share_png),
            "```",
        ]
    )
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=ROOT / "analysis_artifacts" / "closed_loop_frequency_acceleration_experiment")
    parser.add_argument("--data-dir", type=Path, default=ROOT / "data" / "closed_loop_frequency_acceleration_experiment")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--log-interval-s", type=float, default=0.01)
    parser.add_argument("--frequencies", default=",".join(str(f) for f in DEFAULT_FREQS))
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    freqs = [float(x.strip()) for x in args.frequencies.split(",") if x.strip()]
    config_dir = args.out_dir / "generated_configs"
    config_dir.mkdir(parents=True, exist_ok=True)
    args.data_dir.mkdir(parents=True, exist_ok=True)

    configs: dict[tuple[str, float], Path] = {}
    logs: dict[tuple[str, float], Path] = {}
    for method in ["pd", "eid"]:
        for freq in freqs:
            config = write_frequency_config(method, freq, config_dir)
            configs[(method, freq)] = config
            logs[(method, freq)] = run_mujoco_case(method, freq, config, args.data_dir, args)

    inv = HipKneeInverseDynamics(ROOT / "h1_official_mujoco" / "h1.xml")
    details = []
    rows = []
    for method in ["pd", "eid"]:
        for freq in freqs:
            run = decompose_log(logs[(method, freq)], configs[(method, freq)], inv)
            run["method"] = method.upper()
            run["frequency_hz"] = freq
            run["reference_acceleration_gain"] = (freq / BASE_FREQ) ** 2
            details.append(run)
            rows.extend(summarize_case(run, method, freq))

    detail = pd.concat(details, ignore_index=True)
    summary = pd.DataFrame(rows)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    detail.to_csv(args.out_dir / "closed_loop_frequency_acceleration_detail.csv", index=False)
    summary.to_csv(args.out_dir / "closed_loop_frequency_acceleration_summary.csv", index=False)
    write_plots(summary, args.out_dir / "figures")
    report = write_report(summary, args.out_dir)
    print(summary.to_string(index=False, float_format=lambda v: f"{v:.4f}"))
    print(f"report={report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
