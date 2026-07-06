#!/usr/bin/env python3
"""Analyze mutual hip-knee coupling torques with MuJoCo inverse dynamics.

This script intentionally excludes externally injected torques. It estimates
what the hip motion contributes to the knee channel and what the knee motion
contributes to the hip channel along measured trajectories.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd
import yaml

from report_paths import analysis_report_dir, repo_relpath


ROOT = Path(__file__).resolve().parents[1]
HIP = 1
KNEE = 2
DEFAULT_WINDOWS = {
    "steady_3_7p4": (3.0, 7.4),
    "early_3_4": (3.0, 4.0),
    "middle_4_5p4": (4.0, 5.4),
    "late_5p4_7p4": (5.4, 7.4),
}


@dataclass(frozen=True)
class JointDof:
    qposadr: int
    dofadr: int


@dataclass(frozen=True)
class Plant:
    Jeff: float
    b: float
    gravityA: float
    gravityB: float
    tau0: float


@dataclass(frozen=True)
class RunMeta:
    method: str
    repeat: str
    log_path: Path
    config_path: Path
    condition_id: str


class HipKneeInverseDynamics:
    def __init__(self, xml_path: Path) -> None:
        import mujoco

        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        self.qpos0 = self._base_qpos()
        self.hip = self._joint("right_hip_pitch_joint")
        self.knee = self._joint("right_knee_joint")

    def _joint(self, name: str) -> JointDof:
        joint_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise RuntimeError(f"MuJoCo joint not found: {name}")
        return JointDof(
            qposadr=int(self.model.jnt_qposadr[joint_id]),
            dofadr=int(self.model.jnt_dofadr[joint_id]),
        )

    def _base_qpos(self) -> np.ndarray:
        if self.model.nkey > 0:
            qpos = np.array(self.model.key_qpos[0], dtype=float)
        else:
            qpos = np.array(self.model.qpos0, dtype=float)
        if qpos.size >= 7:
            qpos[2] = max(qpos[2], 0.98)
            qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        return qpos

    def tau_pair(
        self,
        qh: float,
        qk: float,
        dqh: float,
        dqk: float,
        ddqh: float,
        ddqk: float,
    ) -> tuple[float, float]:
        self.data.qpos[:] = self.qpos0
        self.data.qvel[:] = 0.0
        self.data.qacc[:] = 0.0
        self.data.qpos[self.hip.qposadr] = qh
        self.data.qpos[self.knee.qposadr] = qk
        self.data.qvel[self.hip.dofadr] = dqh
        self.data.qvel[self.knee.dofadr] = dqk
        self.data.qacc[self.hip.dofadr] = ddqh
        self.data.qacc[self.knee.dofadr] = ddqk
        self.mujoco.mj_inverse(self.model, self.data)
        return (
            float(self.data.qfrc_inverse[self.hip.dofadr]),
            float(self.data.qfrc_inverse[self.knee.dofadr]),
        )


def read_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def yaml_joint(cfg: dict, joint_id: int) -> dict:
    joints = cfg["controller"]["joints"]
    return joints.get(joint_id, joints.get(str(joint_id)))


def load_plant(config_path: Path, joint_id: int) -> Plant:
    plant = yaml_joint(read_yaml(config_path), joint_id)["plant"]
    return Plant(
        Jeff=float(plant["Jeff"]),
        b=float(plant["b"]),
        gravityA=float(plant["gravityA"]),
        gravityB=float(plant["gravityB"]),
        tau0=float(plant["tau0"]),
    )


def local_tau(q: np.ndarray, dq: np.ndarray, qdd: np.ndarray, plant: Plant) -> np.ndarray:
    return (
        plant.Jeff * qdd
        + plant.b * dq
        + plant.gravityA * np.sin(q)
        + plant.gravityB * np.cos(q)
        + plant.tau0
    )


def local_gravity(q: np.ndarray, plant: Plant) -> np.ndarray:
    return plant.gravityA * np.sin(q) + plant.gravityB * np.cos(q) + plant.tau0


def finite_rms(values: np.ndarray) -> float:
    x = np.asarray(values, dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(x * x)))


def finite_peak(values: np.ndarray) -> float:
    x = np.asarray(values, dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float("nan")
    return float(np.max(np.abs(x)))


def finite_mean(values: np.ndarray) -> float:
    x = np.asarray(values, dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float("nan")
    return float(np.mean(x))


def odd_window(n: int, dt: float, window_ms: float, polyorder: int) -> int:
    if n <= polyorder + 2:
        return 0
    samples = max(polyorder + 2, int(round(window_ms / 1000.0 / max(dt, 1e-6))))
    if samples % 2 == 0:
        samples += 1
    samples = min(samples, n if n % 2 == 1 else n - 1)
    return samples if samples > polyorder else 0


def estimate_qdd(dq: np.ndarray, t: np.ndarray, window_ms: float, polyorder: int) -> np.ndarray:
    from scipy.signal import savgol_filter

    dt = float(np.median(np.diff(t))) if t.size > 2 else 0.002
    win = odd_window(dq.size, dt, window_ms, polyorder)
    if win:
        return savgol_filter(dq, window_length=win, polyorder=polyorder, deriv=1, delta=dt, mode="interp")
    return np.gradient(dq, t)


def discover_logs(data_dir: Path, condition_substring: str | None) -> list[RunMeta]:
    metas: list[RunMeta] = []
    paths = sorted(data_dir.glob("20260623_19*/h1_real_p2_anti_hip_knee_*_log.csv"))
    for path in paths:
        with path.open("r", encoding="utf-8", newline="") as f:
            first = next(csv.DictReader(f), None)
        if not first:
            continue
        condition = str(first["condition_id"])
        if condition_substring and condition_substring not in condition:
            continue
        method = "eid" if "_EID_" in condition else "pd" if "_PD_" in condition else "unknown"
        if method not in {"pd", "eid"}:
            continue
        metas.append(
            RunMeta(
                method=method,
                repeat=str(first["repeat_id"]),
                log_path=path,
                config_path=ROOT / str(first["config_path"]),
                condition_id=condition,
            )
        )
    return metas


def pivot_run(path: Path) -> pd.DataFrame:
    by_cycle: dict[int, dict[int, dict[str, str]]] = {}
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            joint_id = int(row["joint_id"])
            if joint_id not in (HIP, KNEE):
                continue
            by_cycle.setdefault(int(row["cycle"]), {})[joint_id] = row

    records = []
    for cycle in sorted(by_cycle):
        pair = by_cycle[cycle]
        if HIP not in pair or KNEE not in pair:
            continue
        h = pair[HIP]
        k = pair[KNEE]
        records.append(
            {
                "cycle": cycle,
                "t": float(h["t"]),
                "q_h": float(h["q"]),
                "q_k": float(k["q"]),
                "dq_h": float(h["dq"]),
                "dq_k": float(k["dq"]),
                "tau_est_h": float(h["tau_est"]),
                "tau_est_k": float(k["tau_est"]),
            }
        )
    out = pd.DataFrame.from_records(records)
    out["t_rel"] = out["t"] - float(out["t"].iloc[0])
    return out


def decompose_run(
    meta: RunMeta,
    inv: HipKneeInverseDynamics,
    window_ms: float,
    polyorder: int,
    max_time: float,
) -> pd.DataFrame:
    hip_plant = load_plant(meta.config_path, HIP)
    knee_plant = load_plant(meta.config_path, KNEE)
    run = pivot_run(meta.log_path)
    run = run[run["t_rel"] <= max_time].copy()

    t = run["t_rel"].to_numpy(dtype=float)
    qh = run["q_h"].to_numpy(dtype=float)
    qk = run["q_k"].to_numpy(dtype=float)
    dqh = run["dq_h"].to_numpy(dtype=float)
    dqk = run["dq_k"].to_numpy(dtype=float)
    ddqh = estimate_qdd(dqh, t, window_ms, polyorder)
    ddqk = estimate_qdd(dqk, t, window_ms, polyorder)
    run["qdd_h"] = ddqh
    run["qdd_k"] = ddqk

    n = qh.size
    names = [
        "full",
        "g",
        "hacc",
        "kacc",
        "vel_full",
        "vel_honly",
        "vel_konly",
    ]
    tau_h = {name: np.empty(n, dtype=float) for name in names}
    tau_k = {name: np.empty(n, dtype=float) for name in names}

    for i in range(n):
        tau_h["full"][i], tau_k["full"][i] = inv.tau_pair(qh[i], qk[i], dqh[i], dqk[i], ddqh[i], ddqk[i])
        tau_h["g"][i], tau_k["g"][i] = inv.tau_pair(qh[i], qk[i], 0.0, 0.0, 0.0, 0.0)
        tau_h["hacc"][i], tau_k["hacc"][i] = inv.tau_pair(qh[i], qk[i], 0.0, 0.0, ddqh[i], 0.0)
        tau_h["kacc"][i], tau_k["kacc"][i] = inv.tau_pair(qh[i], qk[i], 0.0, 0.0, 0.0, ddqk[i])
        tau_h["vel_full"][i], tau_k["vel_full"][i] = inv.tau_pair(qh[i], qk[i], dqh[i], dqk[i], 0.0, 0.0)
        tau_h["vel_honly"][i], tau_k["vel_honly"][i] = inv.tau_pair(qh[i], qk[i], dqh[i], 0.0, 0.0, 0.0)
        tau_h["vel_konly"][i], tau_k["vel_konly"][i] = inv.tau_pair(qh[i], qk[i], 0.0, dqk[i], 0.0, 0.0)

    tau_loc_h = local_tau(qh, dqh, ddqh, hip_plant)
    tau_loc_k = local_tau(qk, dqk, ddqk, knee_plant)
    g_loc_h = local_gravity(qh, hip_plant)
    g_loc_k = local_gravity(qk, knee_plant)

    run["tau_full_h"] = tau_h["full"]
    run["tau_full_k"] = tau_k["full"]
    run["tau_loc_h"] = tau_loc_h
    run["tau_loc_k"] = tau_loc_k

    run["d_h_total"] = tau_h["full"] - tau_loc_h
    run["d_k_total"] = tau_k["full"] - tau_loc_k

    run["d_h_config"] = tau_h["g"] - g_loc_h
    run["d_k_config"] = tau_k["g"] - g_loc_k

    run["d_h_from_knee_acc"] = tau_h["kacc"] - tau_h["g"]
    run["d_k_from_hip_acc"] = tau_k["hacc"] - tau_k["g"]

    run["d_h_self_acc_err"] = (tau_h["hacc"] - tau_h["g"]) - hip_plant.Jeff * ddqh
    run["d_k_self_acc_err"] = (tau_k["kacc"] - tau_k["g"]) - knee_plant.Jeff * ddqk

    run["d_h_from_knee_vel"] = tau_h["vel_full"] - tau_h["vel_honly"]
    run["d_k_from_hip_vel"] = tau_k["vel_full"] - tau_k["vel_konly"]

    run["d_h_vel_residual"] = (tau_h["vel_full"] - tau_h["g"]) - hip_plant.b * dqh
    run["d_k_vel_residual"] = (tau_k["vel_full"] - tau_k["g"]) - knee_plant.b * dqk

    run["d_h_from_knee_dynamic"] = run["d_h_from_knee_acc"] + run["d_h_from_knee_vel"]
    run["d_k_from_hip_dynamic"] = run["d_k_from_hip_acc"] + run["d_k_from_hip_vel"]

    run["d_h_recon"] = run["d_h_config"] + run["d_h_from_knee_acc"] + run["d_h_self_acc_err"] + run["d_h_vel_residual"]
    run["d_k_recon"] = run["d_k_config"] + run["d_k_from_hip_acc"] + run["d_k_self_acc_err"] + run["d_k_vel_residual"]
    run["d_h_recon_err"] = run["d_h_recon"] - run["d_h_total"]
    run["d_k_recon_err"] = run["d_k_recon"] - run["d_k_total"]

    run["method"] = meta.method
    run["repeat"] = meta.repeat
    run["condition_id"] = meta.condition_id
    run["log_path"] = str(meta.log_path.relative_to(ROOT))
    run["config_path"] = str(meta.config_path.relative_to(ROOT))
    return run


def summarize(detail: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    components = [
        "d_k_total",
        "d_k_config",
        "d_k_from_hip_acc",
        "d_k_from_hip_vel",
        "d_k_from_hip_dynamic",
        "d_k_self_acc_err",
        "d_k_vel_residual",
        "d_k_recon_err",
        "d_h_total",
        "d_h_config",
        "d_h_from_knee_acc",
        "d_h_from_knee_vel",
        "d_h_from_knee_dynamic",
        "d_h_self_acc_err",
        "d_h_vel_residual",
        "d_h_recon_err",
    ]
    rows = []
    for (method, repeat, log_path), g in detail.groupby(["method", "repeat", "log_path"], sort=True):
        dt = float(np.median(np.diff(g["t_rel"]))) if len(g) > 2 else 0.002
        for window_name, (start, stop) in DEFAULT_WINDOWS.items():
            w = g[(g["t_rel"] >= start) & (g["t_rel"] < stop)]
            if w.empty:
                continue
            row = {
                "method": method,
                "repeat": repeat,
                "log_path": log_path,
                "window": window_name,
                "n": int(len(w)),
                "dt_median": dt,
            }
            for comp in components:
                values = w[comp].to_numpy(dtype=float)
                row[f"{comp}_rms"] = finite_rms(values)
                row[f"{comp}_peak_abs"] = finite_peak(values)
                row[f"{comp}_mean"] = finite_mean(values)
            rows.append(row)
    summary = pd.DataFrame(rows)
    numeric = [c for c in summary.select_dtypes(include=[np.number]).columns if c not in {"n"}]
    agg = summary.groupby(["method", "window"], dropna=False)[numeric].mean().reset_index()
    counts = summary.groupby(["method", "window"], dropna=False)["repeat"].nunique().reset_index(name="n_runs")
    return summary, counts.merge(agg, on=["method", "window"], how="left")


def mean_band(detail: pd.DataFrame, method: str, column: str, grid: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    curves = []
    for _, run in detail[detail["method"] == method].groupby(["repeat", "log_path"], sort=True):
        x = run["t_rel"].to_numpy(dtype=float)
        y = run[column].to_numpy(dtype=float)
        mask = np.isfinite(x) & np.isfinite(y)
        if mask.sum() < 2:
            continue
        lo = max(float(np.min(x[mask])), float(grid[0]))
        hi = min(float(np.max(x[mask])), float(grid[-1]))
        valid_grid = (grid >= lo) & (grid <= hi)
        yi = np.full_like(grid, np.nan, dtype=float)
        yi[valid_grid] = np.interp(grid[valid_grid], x[mask], y[mask])
        curves.append(yi)
    if not curves:
        return np.full_like(grid, np.nan), np.full_like(grid, np.nan)
    arr = np.vstack(curves)
    return np.nanmean(arr, axis=0), np.nanstd(arr, axis=0)


def setup_matplotlib() -> None:
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
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
        }
    )


def save_figure(fig, path_base: Path) -> list[Path]:
    paths = []
    for suffix in [".png", ".pdf"]:
        path = path_base.with_suffix(suffix)
        fig.savefig(path)
        paths.append(path)
    return paths


def write_mechanism_diagram(out_dir: Path) -> list[Path]:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Arc, Circle

    setup_matplotlib()
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(6.85, 2.9))
    ax.set_aspect("equal")
    ax.axis("off")

    hip = np.array([0.0, 0.0])
    knee = np.array([2.25, -1.2])
    shank = np.array([3.35, -2.25])
    blue = "#0072B2"
    orange = "#D55E00"
    green = "#009E73"
    gray = "#666666"

    ax.plot([hip[0], knee[0]], [hip[1], knee[1]], color=gray, lw=5, solid_capstyle="round")
    ax.plot([knee[0], shank[0]], [knee[1], shank[1]], color="#9AA0A6", lw=5, solid_capstyle="round")
    for point, label in [(hip, "hip pitch"), (knee, "knee pitch")]:
        ax.add_patch(Circle(point, 0.13, facecolor="white", edgecolor="black", lw=1.2))
        ax.text(point[0], point[1] + 0.22, label, ha="center", va="bottom")

    mid = (hip + knee) / 2
    ax.annotate(
        r"$r_{hk}\approx0.4\,m$",
        xy=mid,
        xytext=(mid[0] - 0.15, mid[1] + 0.38),
        ha="center",
        arrowprops={"arrowstyle": "-", "lw": 0.8, "color": gray},
    )

    ax.annotate("", xy=knee + np.array([0.0, 0.85]), xytext=knee, arrowprops={"arrowstyle": "->", "lw": 1.8, "color": blue})
    ax.text(knee[0] + 0.12, knee[1] + 0.78, r"$a_{O_h}$ carried motion", color=blue, va="center")
    ax.annotate("", xy=knee + np.array([0.85, 0.48]), xytext=knee, arrowprops={"arrowstyle": "->", "lw": 1.8, "color": orange})
    ax.text(knee[0] + 0.78, knee[1] + 0.62, r"$\alpha_h\times r_{hk}$", color=orange, va="center")
    ax.annotate("", xy=knee + np.array([-0.55, -0.42]), xytext=knee, arrowprops={"arrowstyle": "->", "lw": 1.8, "color": green})
    ax.text(knee[0] - 1.58, knee[1] - 0.48, r"$\omega_h\times(\omega_h\times r_{hk})$", color=green, va="center")

    ax.add_patch(Arc(hip, 0.78, 0.78, angle=0, theta1=-60, theta2=80, color=orange, lw=1.5))
    ax.annotate("", xy=(0.36, 0.34), xytext=(0.28, 0.18), arrowprops={"arrowstyle": "->", "lw": 1.4, "color": orange})
    ax.text(-0.78, 0.42, "hip angular\nmotion", color=orange, ha="center")

    ax.text(
        4.2,
        -0.05,
        "The knee origin is moving.\nA single-joint knee model\nmisses these parent-link\nacceleration inputs.",
        ha="left",
        va="center",
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "#F7F7F7", "edgecolor": "#CCCCCC"},
    )
    ax.set_xlim(-1.25, 6.15)
    ax.set_ylim(-2.75, 1.35)
    paths = save_figure(fig, fig_dir / "hip_knee_non_inertial_mechanism")
    plt.close(fig)
    return paths


def write_component_share_plot(aggregate: pd.DataFrame, out_dir: Path) -> list[Path]:
    import matplotlib.pyplot as plt

    setup_matplotlib()
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    steady = aggregate[aggregate["window"] == "steady_3_7p4"].copy()
    rows = []
    specs = [
        ("Knee channel", "d_k_config_rms", "d_k_from_hip_acc_rms", "d_k_vel_residual_rms", "d_k_self_acc_err_rms"),
        ("Hip channel", "d_h_config_rms", "d_h_from_knee_acc_rms", "d_h_vel_residual_rms", "d_h_self_acc_err_rms"),
    ]
    for method in ["pd", "eid"]:
        src = steady[steady["method"] == method]
        if src.empty:
            continue
        row = src.iloc[0]
        for channel, config_col, acc_col, vel_col, self_col in specs:
            values = {
                "configuration": float(row[config_col]),
                "other-joint acceleration": float(row[acc_col]),
                "velocity/residual": float(row[vel_col]),
                "self inertia mismatch": float(row[self_col]),
            }
            total = sum(abs(v) for v in values.values())
            rows.append((f"{channel} {method.upper()}", values, total))

    colors = {
        "configuration": "#E69F00",
        "other-joint acceleration": "#0072B2",
        "velocity/residual": "#009E73",
        "self inertia mismatch": "#CC79A7",
    }
    fig, ax = plt.subplots(figsize=(6.85, 2.6))
    y = np.arange(len(rows))
    for yi, (label, values, total) in enumerate(rows):
        left = 0.0
        for key in ["configuration", "other-joint acceleration", "velocity/residual", "self inertia mismatch"]:
            pct = 100.0 * abs(values[key]) / total if total > 0 else 0.0
            ax.barh(yi, pct, left=left, height=0.62, color=colors[key], label=key if yi == 0 else None)
            if pct >= 8.0:
                ax.text(left + pct / 2, yi, f"{pct:.0f}%", ha="center", va="center", color="white", fontsize=7.2)
            left += pct
    ax.set_yticks(y)
    ax.set_yticklabels([label for label, _, _ in rows])
    ax.set_xlabel("share of summed component RMS [%]")
    ax.set_xlim(0, 100)
    ax.legend(frameon=False, ncol=2, loc="lower center", bbox_to_anchor=(0.5, 1.0))
    paths = save_figure(fig, fig_dir / "coupling_component_share")
    plt.close(fig)
    return paths


def write_acceleration_link_plots(detail: pd.DataFrame, out_dir: Path) -> list[Path]:
    import matplotlib.pyplot as plt

    setup_matplotlib()
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    colors = {"pd": "#9AA0A6", "eid": "#0072B2"}
    labels = {"pd": "PD", "eid": "EID"}
    t0, t1, dt = 3.0, 7.4, 0.01
    grid = np.arange(t0, t1 + dt / 2, dt)

    specs = [
        (
            "hip_acc_to_knee_torque_link",
            [
                ("qdd_h", r"$\ddot q_h$" + "\n" + r"[rad/s$^2$]"),
                ("d_k_from_hip_acc", "hip acc -> knee\n[N m]"),
                ("d_k_total", "knee residual\n[N m]"),
            ],
        ),
        (
            "knee_acc_to_hip_torque_link",
            [
                ("qdd_k", r"$\ddot q_k$" + "\n" + r"[rad/s$^2$]"),
                ("d_h_from_knee_acc", "knee acc -> hip\n[N m]"),
                ("d_h_total", "hip residual\n[N m]"),
            ],
        ),
    ]
    paths: list[Path] = []
    for filename, panels in specs:
        fig, axes = plt.subplots(3, 1, figsize=(6.85, 4.9), sharex=True)
        for ax, (col, ylabel) in zip(axes, panels):
            for method in ["pd", "eid"]:
                mean, std = mean_band(detail, method, col, grid)
                ax.plot(grid, mean, color=colors[method], lw=1.35, label=labels[method])
                ax.fill_between(grid, mean - std, mean + std, color=colors[method], alpha=0.14, linewidth=0)
            ax.axhline(0.0, color="black", lw=0.6)
            ax.set_ylabel(ylabel)
        axes[-1].set_xlabel("time from log start [s]")
        axes[0].legend(frameon=False, ncol=2, loc="upper right")
        fig.subplots_adjust(left=0.14, right=0.98, top=0.96, bottom=0.1, hspace=0.45)
        paths.extend(save_figure(fig, fig_dir / filename))
        plt.close(fig)
    return paths


def write_plots(detail: pd.DataFrame, aggregate: pd.DataFrame, out_dir: Path) -> list[Path]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    setup_matplotlib()
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    paths: list[Path] = []
    paths.extend(write_mechanism_diagram(out_dir))
    colors = {"pd": "#9AA0A6", "eid": "#0072B2"}
    labels = {"pd": "PD", "eid": "EID"}

    t0, t1, dt = 3.0, 7.4, 0.01
    grid = np.arange(t0, t1 + dt / 2, dt)

    fig, axes = plt.subplots(2, 1, figsize=(6.85, 4.3), sharex=True)
    panels = [
        ("d_k_from_hip_dynamic", "hip -> knee\n[N m]"),
        ("d_h_from_knee_dynamic", "knee -> hip\n[N m]"),
    ]
    for ax, (col, ylabel) in zip(axes, panels):
        for method in ["pd", "eid"]:
            mean, std = mean_band(detail, method, col, grid)
            ax.plot(grid, mean, color=colors[method], lw=1.4, label=labels[method])
            ax.fill_between(grid, mean - std, mean + std, color=colors[method], alpha=0.16, linewidth=0)
        ax.axhline(0.0, color="black", lw=0.6)
        ax.set_ylabel(ylabel)
    axes[-1].set_xlabel("time from log start [s]")
    axes[0].legend(frameon=False, ncol=2)
    paths.extend(save_figure(fig, fig_dir / "mutual_dynamic_coupling_timeseries"))
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(6.85, 4.3), sharex=True)
    panels = [
        ("d_k_total", "knee residual\n[N m]"),
        ("d_h_total", "hip residual\n[N m]"),
    ]
    for ax, (col, ylabel) in zip(axes, panels):
        for method in ["pd", "eid"]:
            mean, std = mean_band(detail, method, col, grid)
            ax.plot(grid, mean, color=colors[method], lw=1.4, label=labels[method])
            ax.fill_between(grid, mean - std, mean + std, color=colors[method], alpha=0.16, linewidth=0)
        ax.axhline(0.0, color="black", lw=0.6)
        ax.set_ylabel(ylabel)
    axes[-1].set_xlabel("time from log start [s]")
    axes[0].legend(frameon=False, ncol=2)
    paths.extend(save_figure(fig, fig_dir / "local_model_residual_timeseries"))
    plt.close(fig)

    steady = aggregate[aggregate["window"] == "steady_3_7p4"].copy()
    comps = [
        ("d_k_from_hip_acc_rms", "hip acc\n-> knee"),
        ("d_k_from_hip_vel_rms", "hip vel\n-> knee"),
        ("d_k_total_rms", "knee total\nresidual"),
        ("d_h_from_knee_acc_rms", "knee acc\n-> hip"),
        ("d_h_from_knee_vel_rms", "knee vel\n-> hip"),
        ("d_h_total_rms", "hip total\nresidual"),
    ]
    x = np.arange(len(comps))
    width = 0.34
    fig, ax = plt.subplots(figsize=(6.85, 2.6))
    for i, method in enumerate(["pd", "eid"]):
        row = steady[steady["method"] == method]
        if row.empty:
            continue
        vals = [float(row[c].iloc[0]) for c, _ in comps]
        offset = (i - 0.5) * width
        ax.bar(x + offset, vals, width=width, color=colors[method], label=labels[method])
    ax.set_xticks(x)
    ax.set_xticklabels([label for _, label in comps])
    ax.set_ylabel("RMS [N m]")
    ax.legend(frameon=False, ncol=2)
    paths.extend(save_figure(fig, fig_dir / "mutual_coupling_rms"))
    plt.close(fig)

    paths.extend(write_component_share_plot(aggregate, out_dir))
    paths.extend(write_acceleration_link_plots(detail, out_dir))

    return paths


def markdown_table(df: pd.DataFrame, cols: list[str], floatfmt: str = ".4g") -> str:
    if df.empty:
        return ""
    header = "| " + " | ".join(cols) + " |"
    sep = "| " + " | ".join(["---"] * len(cols)) + " |"
    rows = []
    for _, row in df[cols].iterrows():
        cells = []
        for col in cols:
            val = row[col]
            if isinstance(val, float) or isinstance(val, np.floating):
                cells.append(format(float(val), floatfmt) if np.isfinite(val) else "")
            else:
                cells.append(str(val))
        rows.append("| " + " | ".join(cells) + " |")
    return "\n".join([header, sep, *rows])


def write_report(aggregate: pd.DataFrame, out_dir: Path, plot_paths: Iterable[Path]) -> Path:
    report_dir = analysis_report_dir(out_dir)
    report_dir.mkdir(parents=True, exist_ok=True)
    path = report_dir / "hip_knee_mutual_coupling_report.md"
    steady = aggregate[aggregate["window"] == "steady_3_7p4"].copy()
    cols = [
        "method",
        "n_runs",
        "d_k_from_hip_acc_rms",
        "d_k_from_hip_vel_rms",
        "d_k_from_hip_dynamic_rms",
        "d_k_total_rms",
        "d_h_from_knee_acc_rms",
        "d_h_from_knee_vel_rms",
        "d_h_from_knee_dynamic_rms",
        "d_h_total_rms",
        "d_k_recon_err_rms",
        "d_h_recon_err_rms",
    ]
    lines = [
        "# Hip-Knee Mutual Coupling Analysis",
        "",
        "This report excludes externally injected torques.",
        "Coupling terms are computed from MuJoCo inverse dynamics along measured right hip pitch and right knee trajectories.",
        "",
        "## Steady Window RMS",
        "",
        markdown_table(steady, cols),
        "",
        "## Figures",
        "",
    ]
    for p in plot_paths:
        lines.append(f"- `{repo_relpath(p)}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, default=ROOT / "data")
    parser.add_argument("--xml", type=Path, default=ROOT / "h1_official_mujoco" / "h1.xml")
    parser.add_argument("--out-dir", type=Path, default=ROOT / "analysis_artifacts" / "hip_knee_mutual_coupling")
    parser.add_argument("--condition-substring", default="software_disturbance")
    parser.add_argument("--max-time", type=float, default=8.0)
    parser.add_argument("--window-ms", type=float, default=81.0)
    parser.add_argument("--polyorder", type=int, default=3)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    metas = discover_logs(args.data_dir, args.condition_substring)
    if not metas:
        raise SystemExit("No matching right hip-knee logs found.")
    args.out_dir.mkdir(parents=True, exist_ok=True)

    inv = HipKneeInverseDynamics(args.xml)
    detail_parts = []
    for idx, meta in enumerate(metas, start=1):
        print(f"[{idx}/{len(metas)}] {meta.method} {meta.repeat} {meta.log_path}")
        detail_parts.append(decompose_run(meta, inv, args.window_ms, args.polyorder, args.max_time))

    detail = pd.concat(detail_parts, ignore_index=True)
    summary, aggregate = summarize(detail)
    detail.to_csv(args.out_dir / "hip_knee_mutual_coupling_timeseries.csv", index=False)
    summary.to_csv(args.out_dir / "hip_knee_mutual_coupling_summary.csv", index=False)
    aggregate.to_csv(args.out_dir / "hip_knee_mutual_coupling_aggregate.csv", index=False)
    plot_paths = write_plots(detail, aggregate, args.out_dir)
    report = write_report(aggregate, args.out_dir, plot_paths)
    print(f"Wrote {args.out_dir}")
    print(f"Wrote {report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
