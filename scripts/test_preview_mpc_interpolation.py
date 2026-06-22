#!/usr/bin/env python3
"""Preview-MPC interpolation experiment for 1, 2, and 3 future policy points.

This is a pure kinematic trajectory generator. It does not use robot dynamics,
inertia, gravity, torque limits, or a plant model.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import matplotlib.pyplot as plt
import numpy as np


@dataclass(frozen=True)
class MpcWeights:
    preview_q: float = 2.0e7
    path_v: float = 3.0e-3
    path_a: float = 8.0e-5
    jerk: float = 2.0e-9
    terminal_v: float = 2.0e-1
    terminal_a: float = 2.0e-3
    ridge: float = 1.0e-10


@dataclass(frozen=True)
class ExperimentConfig:
    dt: float = 0.002
    policy_dt: float = 0.05
    duration: float = 3.0
    weights: MpcWeights = MpcWeights()


@dataclass
class Trajectory:
    t: np.ndarray
    q: np.ndarray
    dq: np.ndarray
    ddq: np.ndarray
    jerk: np.ndarray


def target_sine(t: np.ndarray | float) -> np.ndarray | float:
    return 0.75 + 0.16 * np.sin(2.0 * np.pi * 0.85 * np.asarray(t) - 0.6)


def target_reversal(t: np.ndarray | float) -> np.ndarray | float:
    x = np.asarray(t)
    return (
        0.75
        + 0.13 * np.sin(2.0 * np.pi * 0.55 * x + 0.2)
        + 0.055 * np.sin(2.0 * np.pi * 1.65 * x - 0.4)
    )


def target_step_hold(t: np.ndarray | float) -> np.ndarray | float:
    x = np.asarray(t)
    eps = 1.0e-12
    return np.select(
        [
            x < 0.40 - eps,
            x < 1.00 - eps,
            x < 1.55 - eps,
            x < 2.20 - eps,
        ],
        [
            0.75,
            0.95,
            0.66,
            0.88,
        ],
        default=0.74,
    )


SCENARIOS: dict[str, Callable[[np.ndarray | float], np.ndarray | float]] = {
    "smooth_sine": target_sine,
    "multi_reversal": target_reversal,
    "step_hold": target_step_hold,
}


def build_kinematic_matrices(steps: int, dt: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    aq = np.zeros((steps, steps))
    av = np.zeros((steps, steps))
    aa = np.zeros((steps, steps))
    for k in range(1, steps + 1):
        for i in range(k):
            r = k - i
            aa[k - 1, i] = dt
            av[k - 1, i] = 0.5 * dt * dt * (r * r - (r - 1) * (r - 1))
            aq[k - 1, i] = (dt**3 / 6.0) * (r**3 - (r - 1) ** 3)
    return aq, av, aa


def base_trajectory(q0: float, dq0: float, ddq0: float, steps: int, dt: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    t = dt * np.arange(1, steps + 1)
    q = q0 + dq0 * t + 0.5 * ddq0 * t * t
    dq = dq0 + ddq0 * t
    ddq = np.full(steps, ddq0)
    return q, dq, ddq


def solve_preview_mpc(
    q0: float,
    dq0: float,
    ddq0: float,
    preview_q: np.ndarray,
    cfg: ExperimentConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    policy_steps = round(cfg.policy_dt / cfg.dt)
    horizon_steps = policy_steps * len(preview_q)
    weights = cfg.weights

    aq, av, aa = build_kinematic_matrices(horizon_steps, cfg.dt)
    q_base, dq_base, ddq_base = base_trajectory(q0, dq0, ddq0, horizon_steps, cfg.dt)

    q_mat = weights.path_v * (av.T @ av)
    q_mat += weights.path_a * (aa.T @ aa)
    q_mat += weights.jerk * np.eye(horizon_steps)
    q_mat += weights.terminal_v * np.outer(av[-1], av[-1])
    q_mat += weights.terminal_a * np.outer(aa[-1], aa[-1])
    q_mat += weights.ridge * np.eye(horizon_steps)

    c_vec = weights.path_v * (av.T @ dq_base)
    c_vec += weights.path_a * (aa.T @ ddq_base)
    c_vec += weights.terminal_v * av[-1] * dq_base[-1]
    c_vec += weights.terminal_a * aa[-1] * ddq_base[-1]

    for m, q_target in enumerate(preview_q[1:], start=2):
        row = aq[m * policy_steps - 1]
        err = q_base[m * policy_steps - 1] - q_target
        q_mat += weights.preview_q * np.outer(row, row)
        c_vec += weights.preview_q * row * err

    # Hard constraint: exactly hit the first policy point at one policy period.
    ce = aq[policy_steps - 1 : policy_steps]
    be = np.array([preview_q[0] - q_base[policy_steps - 1]])
    kkt = np.block(
        [
            [q_mat, ce.T],
            [ce, np.zeros((1, 1))],
        ]
    )
    rhs = np.concatenate([-c_vec, be])
    try:
        sol = np.linalg.solve(kkt, rhs)
    except np.linalg.LinAlgError:
        sol = np.linalg.lstsq(kkt, rhs, rcond=None)[0]
    jerk = sol[:horizon_steps]

    q = q_base + aq @ jerk
    dq = dq_base + av @ jerk
    ddq = ddq_base + aa @ jerk
    return q, dq, ddq, jerk


def eval_quintic(
    q0: float,
    dq0: float,
    ddq0: float,
    q1: float,
    dq1: float,
    ddq1: float,
    duration: float,
    tau: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    a0 = q0
    a1 = dq0
    a2 = 0.5 * ddq0
    t2 = duration * duration
    t3 = t2 * duration
    b0 = q1 - (a0 + a1 * duration + a2 * t2)
    b1 = dq1 - (a1 + 2.0 * a2 * duration)
    b2 = ddq1 - 2.0 * a2
    c0 = b0 / t3
    c1 = b1 / t2
    c2 = b2 / duration
    a3 = 10.0 * c0 - 4.0 * c1 + 0.5 * c2
    a4 = (-15.0 * c0 + 7.0 * c1 - c2) / duration
    a5 = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / t2

    q = a0 + a1 * tau + a2 * tau**2 + a3 * tau**3 + a4 * tau**4 + a5 * tau**5
    dq = a1 + 2.0 * a2 * tau + 3.0 * a3 * tau**2 + 4.0 * a4 * tau**3 + 5.0 * a5 * tau**4
    ddq = 2.0 * a2 + 6.0 * a3 * tau + 12.0 * a4 * tau**2 + 20.0 * a5 * tau**3
    return q, dq, ddq


def simulate_mpc(
    target: Callable[[np.ndarray | float], np.ndarray | float],
    preview_count: int,
    cfg: ExperimentConfig,
) -> Trajectory:
    policy_steps = round(cfg.policy_dt / cfg.dt)
    segments = round(cfg.duration / cfg.policy_dt)
    t_values = [0.0]
    q_values = [float(target(0.0))]
    dq_values = [0.0]
    ddq_values = [0.0]
    jerk_values = [0.0]

    q0 = q_values[-1]
    dq0 = 0.0
    ddq0 = 0.0
    for seg in range(segments):
        preview_times = cfg.policy_dt * (seg + 1 + np.arange(preview_count))
        preview_q = np.asarray(target(preview_times), dtype=float)
        q, dq, ddq, jerk = solve_preview_mpc(q0, dq0, ddq0, preview_q, cfg)

        start = seg * cfg.policy_dt
        for i in range(policy_steps):
            t_values.append(start + (i + 1) * cfg.dt)
            q_values.append(q[i])
            dq_values.append(dq[i])
            ddq_values.append(ddq[i])
            jerk_values.append(jerk[i])

        q0 = q[policy_steps - 1]
        dq0 = dq[policy_steps - 1]
        ddq0 = ddq[policy_steps - 1]

    return Trajectory(
        np.asarray(t_values),
        np.asarray(q_values),
        np.asarray(dq_values),
        np.asarray(ddq_values),
        np.asarray(jerk_values),
    )


def simulate_quintic_stop(
    target: Callable[[np.ndarray | float], np.ndarray | float],
    cfg: ExperimentConfig,
) -> Trajectory:
    policy_steps = round(cfg.policy_dt / cfg.dt)
    segments = round(cfg.duration / cfg.policy_dt)
    tau = cfg.dt * np.arange(1, policy_steps + 1)
    t_values = [0.0]
    q_values = [float(target(0.0))]
    dq_values = [0.0]
    ddq_values = [0.0]
    jerk_values = [0.0]
    q0 = q_values[-1]

    for seg in range(segments):
        q1 = float(target((seg + 1) * cfg.policy_dt))
        q, dq, ddq = eval_quintic(q0, 0.0, 0.0, q1, 0.0, 0.0, cfg.policy_dt, tau)
        jerk = np.gradient(ddq, cfg.dt)
        start = seg * cfg.policy_dt
        for i in range(policy_steps):
            t_values.append(start + (i + 1) * cfg.dt)
            q_values.append(q[i])
            dq_values.append(dq[i])
            ddq_values.append(ddq[i])
            jerk_values.append(jerk[i])
        q0 = q1

    return Trajectory(
        np.asarray(t_values),
        np.asarray(q_values),
        np.asarray(dq_values),
        np.asarray(ddq_values),
        np.asarray(jerk_values),
    )


def trajectory_metrics(
    target: Callable[[np.ndarray | float], np.ndarray | float],
    traj: Trajectory,
    cfg: ExperimentConfig,
) -> dict[str, float]:
    policy_steps = round(cfg.policy_dt / cfg.dt)
    boundary = np.arange(policy_steps, len(traj.t), policy_steps)
    target_q = np.asarray(target(traj.t), dtype=float)
    first_point_err = traj.q[boundary] - np.asarray(target(traj.t[boundary]), dtype=float)
    return {
        "dense_q_rmse": float(np.sqrt(np.mean((traj.q - target_q) ** 2))),
        "first_point_error_abs_max": float(np.max(np.abs(first_point_err))),
        "boundary_dq_abs_mean": float(np.mean(np.abs(traj.dq[boundary]))),
        "dq_abs_max": float(np.max(np.abs(traj.dq))),
        "dq_rms": float(np.sqrt(np.mean(traj.dq**2))),
        "ddq_abs_max": float(np.max(np.abs(traj.ddq))),
        "ddq_rms": float(np.sqrt(np.mean(traj.ddq**2))),
        "jerk_abs_max": float(np.max(np.abs(traj.jerk))),
        "jerk_rms": float(np.sqrt(np.mean(traj.jerk**2))),
    }


def run_all(cfg: ExperimentConfig) -> dict[str, dict[str, Trajectory]]:
    results: dict[str, dict[str, Trajectory]] = {}
    for scenario, target in SCENARIOS.items():
        results[scenario] = {
            "quintic_stop": simulate_quintic_stop(target, cfg),
            "mpc_1": simulate_mpc(target, 1, cfg),
            "mpc_2": simulate_mpc(target, 2, cfg),
            "mpc_3": simulate_mpc(target, 3, cfg),
        }
    return results


def write_metrics(results: dict[str, dict[str, Trajectory]], cfg: ExperimentConfig, out_path: Path) -> None:
    rows = []
    for scenario, methods in results.items():
        target = SCENARIOS[scenario]
        for method, traj in methods.items():
            row = {"scenario": scenario, "method": method}
            row.update(trajectory_metrics(target, traj, cfg))
            rows.append(row)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_timeseries(results: dict[str, dict[str, Trajectory]], cfg: ExperimentConfig, out_path: Path) -> None:
    colors = {
        "quintic_stop": "#333333",
        "mpc_1": "#d95f02",
        "mpc_2": "#7570b3",
        "mpc_3": "#1b9e77",
    }
    labels = {
        "quintic_stop": "quintic stop",
        "mpc_1": "MPC 1 point",
        "mpc_2": "MPC 2 points",
        "mpc_3": "MPC 3 points",
    }
    fig, axes = plt.subplots(len(SCENARIOS), 3, figsize=(17, 10), sharex="row")
    for row, (scenario, methods) in enumerate(results.items()):
        target = SCENARIOS[scenario]
        t = next(iter(methods.values())).t
        target_q = np.asarray(target(t), dtype=float)
        policy_t = np.arange(0.0, cfg.duration + cfg.policy_dt * 0.5, cfg.policy_dt)
        policy_q = np.asarray(target(policy_t), dtype=float)

        axes[row, 0].plot(t, target_q, color="#9a9a9a", linewidth=1.0, label="dense target")
        axes[row, 0].scatter(policy_t, policy_q, s=10, color="#555555", label="policy points", zorder=3)
        for method, traj in methods.items():
            linestyle = "--" if method == "quintic_stop" else "-"
            axes[row, 0].plot(traj.t, traj.q, color=colors[method], linewidth=1.0, linestyle=linestyle, label=labels[method])
            axes[row, 1].plot(traj.t, traj.dq, color=colors[method], linewidth=1.0, linestyle=linestyle)
            axes[row, 2].plot(traj.t, traj.ddq, color=colors[method], linewidth=1.0, linestyle=linestyle)

        axes[row, 0].set_ylabel(f"{scenario}\nq [rad]")
        axes[row, 1].set_ylabel("dq [rad/s]")
        axes[row, 2].set_ylabel("ddq [rad/s^2]")
        for ax in axes[row]:
            ax.grid(True, alpha=0.25)

    axes[0, 0].legend(loc="upper center", bbox_to_anchor=(1.65, 1.42), ncol=6, frameon=False)
    for ax in axes[-1]:
        ax.set_xlabel("time [s]")
    fig.suptitle("Pure kinematic preview-MPC interpolation, hard q(T)=first policy point", y=0.98)
    fig.subplots_adjust(top=0.87, left=0.07, right=0.98, bottom=0.06, hspace=0.28, wspace=0.24)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def plot_metrics(metrics_path: Path, out_path: Path) -> None:
    with metrics_path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    methods = ["quintic_stop", "mpc_1", "mpc_2", "mpc_3"]
    labels = ["quintic\nstop", "MPC\n1 point", "MPC\n2 points", "MPC\n3 points"]
    colors = ["#333333", "#d95f02", "#7570b3", "#1b9e77"]
    specs = [
        ("dense_q_rmse", "Dense target RMSE [rad]", "linear"),
        ("boundary_dq_abs_mean", "Mean boundary |dq| [rad/s]", "linear"),
        ("ddq_rms", "Acceleration RMS [rad/s^2]", "log"),
        ("jerk_rms", "Jerk RMS [rad/s^3]", "log"),
    ]
    by_scenario = {
        scenario: {row["method"]: row for row in rows if row["scenario"] == scenario}
        for scenario in SCENARIOS
    }
    fig, axes = plt.subplots(len(SCENARIOS), len(specs), figsize=(16, 10))
    for row, scenario in enumerate(SCENARIOS):
        sub = by_scenario[scenario]
        for col, (metric, title, scale) in enumerate(specs):
            ax = axes[row, col]
            vals = [float(sub[m][metric]) for m in methods]
            ax.bar(labels, vals, color=colors, width=0.7)
            ax.set_title(title if row == 0 else "")
            ax.set_yscale(scale)
            ax.grid(True, axis="y", alpha=0.25)
            if col == 0:
                ax.set_ylabel(scenario)
            for i, val in enumerate(vals):
                ax.text(i, val, f"{val:.2g}", ha="center", va="bottom", fontsize=8)
    fig.suptitle("Preview-MPC interpolation metrics", y=0.98)
    fig.subplots_adjust(top=0.90, left=0.07, right=0.98, bottom=0.08, hspace=0.35, wspace=0.28)
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def plot_zoom(results: dict[str, dict[str, Trajectory]], cfg: ExperimentConfig, out_path: Path) -> None:
    scenario = "smooth_sine"
    target = SCENARIOS[scenario]
    methods = results[scenario]
    colors = {
        "quintic_stop": "#333333",
        "mpc_1": "#d95f02",
        "mpc_2": "#7570b3",
        "mpc_3": "#1b9e77",
    }
    labels = {
        "quintic_stop": "quintic stop",
        "mpc_1": "MPC 1 point",
        "mpc_2": "MPC 2 points",
        "mpc_3": "MPC 3 points",
    }
    fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    t_dense = np.linspace(0.30, 0.62, 400)
    axes[0].plot(t_dense, np.asarray(target(t_dense), dtype=float), color="#9a9a9a", label="dense target")
    policy_t = np.arange(0.30, 0.62 + cfg.policy_dt, cfg.policy_dt)
    axes[0].scatter(policy_t, np.asarray(target(policy_t), dtype=float), color="#555555", s=24, zorder=3, label="policy points")
    for method, traj in methods.items():
        mask = (traj.t >= 0.30) & (traj.t <= 0.62)
        linestyle = "--" if method == "quintic_stop" else "-"
        axes[0].plot(traj.t[mask], traj.q[mask], color=colors[method], linestyle=linestyle, linewidth=1.4, label=labels[method])
        axes[1].plot(traj.t[mask], traj.dq[mask], color=colors[method], linestyle=linestyle, linewidth=1.4)
    axes[0].set_ylabel("q [rad]")
    axes[1].set_ylabel("dq [rad/s]")
    axes[1].set_xlabel("time [s]")
    axes[0].legend(loc="upper center", bbox_to_anchor=(0.5, 1.32), ncol=6, frameon=False)
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle("Zoom: future points change the velocity through the first target", y=0.98)
    fig.subplots_adjust(top=0.78, left=0.08, right=0.98, bottom=0.10, hspace=0.08)
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=Path("analysis_artifacts/preview_mpc_interpolation"))
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--policy-dt", type=float, default=0.05)
    args = parser.parse_args()

    cfg = ExperimentConfig(dt=args.dt, policy_dt=args.policy_dt, duration=args.duration)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    fig_dir = args.out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)

    results = run_all(cfg)
    metrics_path = args.out_dir / "preview_mpc_metrics.csv"
    write_metrics(results, cfg, metrics_path)
    plot_timeseries(results, cfg, fig_dir / "preview_mpc_timeseries.png")
    plot_metrics(metrics_path, fig_dir / "preview_mpc_metrics.png")
    plot_zoom(results, cfg, fig_dir / "preview_mpc_zoom.png")

    print(f"metrics={metrics_path}")
    print(f"figure={fig_dir / 'preview_mpc_timeseries.png'}")
    print(f"figure={fig_dir / 'preview_mpc_metrics.png'}")
    print(f"figure={fig_dir / 'preview_mpc_zoom.png'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
