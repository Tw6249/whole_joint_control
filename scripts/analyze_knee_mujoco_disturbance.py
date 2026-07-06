#!/usr/bin/env python3
"""Decompose right-knee equivalent input disturbance with MuJoCo inverse dynamics."""

from __future__ import annotations

import argparse
import csv
import math
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
WINDOWS = {
    "pre": (3.0, 4.0),
    "disturbance": (4.0, 5.4),
    "plateau": (4.2, 5.2),
    "recovery": (5.4, 7.4),
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


class KneeInverseDynamics:
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

    def knee_tau(
        self,
        qh: float,
        qk: float,
        dqh: float,
        dqk: float,
        ddqh: float,
        ddqk: float,
    ) -> float:
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
        return float(self.data.qfrc_inverse[self.knee.dofadr])


def read_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def yaml_joint(cfg: dict, joint_id: int) -> dict:
    joints = cfg["controller"]["joints"]
    return joints.get(joint_id, joints.get(str(joint_id)))


def load_knee_plant(config_path: Path) -> Plant:
    cfg = read_yaml(config_path)
    plant = yaml_joint(cfg, KNEE)["plant"]
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


def correlation(a: np.ndarray, b: np.ndarray) -> float:
    aa = np.asarray(a, dtype=float)
    bb = np.asarray(b, dtype=float)
    mask = np.isfinite(aa) & np.isfinite(bb)
    aa = aa[mask]
    bb = bb[mask]
    if aa.size < 3:
        return float("nan")
    aa = aa - np.mean(aa)
    bb = bb - np.mean(bb)
    den = float(np.linalg.norm(aa) * np.linalg.norm(bb))
    if den <= 1e-12:
        return float("nan")
    return float(np.dot(aa, bb) / den)


def best_lag_corr(a: np.ndarray, b: np.ndarray, dt: float, max_lag_ms: float) -> tuple[float, float]:
    if not np.isfinite(dt) or dt <= 0:
        return float("nan"), float("nan")
    max_lag = int(round(max_lag_ms / 1000.0 / dt))
    best_corr = float("nan")
    best_lag = 0
    for lag in range(-max_lag, max_lag + 1):
        if lag < 0:
            aa = a[-lag:]
            bb = b[: lag or None]
        elif lag > 0:
            aa = a[:-lag]
            bb = b[lag:]
        else:
            aa = a
            bb = b
        c = correlation(aa, bb)
        if np.isfinite(c) and (not np.isfinite(best_corr) or abs(c) > abs(best_corr)):
            best_corr = c
            best_lag = lag
    return best_corr, float(best_lag * dt * 1000.0)


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


def discover_p2d_logs(data_dir: Path, methods: set[str]) -> list[RunMeta]:
    metas: list[RunMeta] = []
    paths = sorted(data_dir.glob("20260623_19*/h1_real_p2_anti_hip_knee_*_log.csv"))
    for path in paths:
        with path.open("r", encoding="utf-8", newline="") as f:
            reader = csv.DictReader(f)
            first = next(reader, None)
        if not first:
            continue
        condition = str(first["condition_id"])
        if "P2D_" not in condition or "software_disturbance" not in condition:
            continue
        method = "eid" if "_EID_" in condition else "pd" if "_PD_" in condition else "unknown"
        if method not in methods:
            continue
        config = ROOT / str(first["config_path"])
        metas.append(
            RunMeta(
                method=method,
                repeat=str(first["repeat_id"]),
                log_path=path,
                config_path=config,
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
            cycle = int(row["cycle"])
            by_cycle.setdefault(cycle, {})[joint_id] = row

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
                "tau_est_k": float(k["tau_est"]),
                "tau_cmd_k": float(k["tau_cmd"]),
                "u_feedback_k": float(k["debug_7"]),
                "xbar_q_k": float(k["debug_15"]),
                "eta_u_k": float(k["debug_32"]),
                "tau_sw_k": float(k["tau_dist"]),
                "disturbance_scale": float(k["disturbance_scale"]),
            }
        )

    out = pd.DataFrame.from_records(records)
    out["t_rel"] = out["t"] - float(out["t"].iloc[0])
    return out


def decompose_run(
    meta: RunMeta,
    inv: KneeInverseDynamics,
    window_ms: float,
    polyorder: int,
    max_time: float,
) -> pd.DataFrame:
    plant = load_knee_plant(meta.config_path)
    run = pivot_run(meta.log_path)
    run = run[run["t_rel"] <= max_time].copy()
    t = run["t_rel"].to_numpy(dtype=float)
    run["qdd_h"] = estimate_qdd(run["dq_h"].to_numpy(dtype=float), t, window_ms, polyorder)
    run["qdd_k"] = estimate_qdd(run["dq_k"].to_numpy(dtype=float), t, window_ms, polyorder)

    qh = run["q_h"].to_numpy(dtype=float)
    qk = run["q_k"].to_numpy(dtype=float)
    dqh = run["dq_h"].to_numpy(dtype=float)
    dqk = run["dq_k"].to_numpy(dtype=float)
    ddqh = run["qdd_h"].to_numpy(dtype=float)
    ddqk = run["qdd_k"].to_numpy(dtype=float)

    tau_full = np.empty_like(qk)
    tau_g_full = np.empty_like(qk)
    tau_hacc = np.empty_like(qk)
    tau_kacc = np.empty_like(qk)
    tau_vel = np.empty_like(qk)

    for i in range(qk.size):
        tau_full[i] = inv.knee_tau(qh[i], qk[i], dqh[i], dqk[i], ddqh[i], ddqk[i])
        tau_g_full[i] = inv.knee_tau(qh[i], qk[i], 0.0, 0.0, 0.0, 0.0)
        tau_hacc[i] = inv.knee_tau(qh[i], qk[i], 0.0, 0.0, ddqh[i], 0.0)
        tau_kacc[i] = inv.knee_tau(qh[i], qk[i], 0.0, 0.0, 0.0, ddqk[i])
        tau_vel[i] = inv.knee_tau(qh[i], qk[i], dqh[i], dqk[i], 0.0, 0.0)

    tau_loc = local_tau(qk, dqk, ddqk, plant)
    g_loc = local_gravity(qk, plant)
    run["tau_full"] = tau_full
    run["tau_loc"] = tau_loc
    run["d_model"] = tau_full - tau_loc
    run["d_g"] = tau_g_full - g_loc
    run["d_hacc"] = tau_hacc - tau_g_full
    run["d_kacc"] = (tau_kacc - tau_g_full) - plant.Jeff * ddqk
    run["d_vel"] = (tau_vel - tau_g_full) - plant.b * dqk
    run["d_recon"] = run["d_g"] + run["d_hacc"] + run["d_kacc"] + run["d_vel"]
    run["d_recon_err"] = run["d_recon"] - run["d_model"]
    run["d_in"] = run["tau_sw_k"] + run["d_model"]
    run["eid_comp_k"] = -run["eta_u_k"]
    run["method"] = meta.method
    run["repeat"] = meta.repeat
    run["condition_id"] = meta.condition_id
    run["log_path"] = str(meta.log_path.relative_to(ROOT))
    run["config_path"] = str(meta.config_path.relative_to(ROOT))
    return run


def summarize_runs(detail: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    components = ["tau_sw_k", "d_g", "d_hacc", "d_kacc", "d_vel", "d_model", "d_in", "eid_comp_k"]
    rows = []
    align_rows = []
    for (method, repeat, log_path), g in detail.groupby(["method", "repeat", "log_path"], sort=True):
        dt = float(np.median(np.diff(g["t_rel"]))) if len(g) > 2 else 0.002
        for window_name, (start, stop) in WINDOWS.items():
            w = g[(g["t_rel"] >= start) & (g["t_rel"] < stop)].copy()
            if w.empty:
                continue
            base = {
                "method": method,
                "repeat": repeat,
                "log_path": log_path,
                "window": window_name,
                "n": int(len(w)),
                "dt_median": dt,
            }
            row = dict(base)
            for comp in components:
                values = w[comp].to_numpy(dtype=float)
                row[f"{comp}_rms"] = finite_rms(values)
                row[f"{comp}_peak_abs"] = finite_peak(values)
                row[f"{comp}_mean"] = finite_mean(values)
            denom = row["d_in_rms"]
            for comp in ["tau_sw_k", "d_g", "d_hacc", "d_kacc", "d_vel", "d_model"]:
                row[f"{comp}_rms_over_d_in"] = row[f"{comp}_rms"] / denom if denom and np.isfinite(denom) else float("nan")
            row["recon_err_rms"] = finite_rms(w["d_recon_err"].to_numpy(dtype=float))
            row["recon_err_over_model_rms"] = (
                row["recon_err_rms"] / row["d_model_rms"] if row["d_model_rms"] and np.isfinite(row["d_model_rms"]) else float("nan")
            )
            rows.append(row)

            for target in ["tau_sw_k", "d_g", "d_hacc", "d_kacc", "d_vel", "d_model", "d_in"]:
                comp = w["eid_comp_k"].to_numpy(dtype=float)
                tgt = w[target].to_numpy(dtype=float)
                best, lag_ms = best_lag_corr(comp, tgt, dt, 80.0)
                align_rows.append(
                    {
                        **base,
                        "source": "eid_comp_k",
                        "target": target,
                        "corr_zero_lag": correlation(comp, tgt),
                        "best_corr_abs_lag": best,
                        "best_lag_ms": lag_ms,
                    }
                )
    return pd.DataFrame(rows), pd.DataFrame(align_rows)


def aggregate_summary(summary: pd.DataFrame) -> pd.DataFrame:
    numeric = summary.select_dtypes(include=[np.number]).columns.tolist()
    keep = [c for c in numeric if c not in {"n"}]
    agg = summary.groupby(["method", "window"], dropna=False)[keep].mean().reset_index()
    counts = summary.groupby(["method", "window"], dropna=False)["repeat"].nunique().reset_index(name="n_runs")
    return counts.merge(agg, on=["method", "window"], how="left")


def write_plots(detail: pd.DataFrame, summary: pd.DataFrame, align: pd.DataFrame, out_dir: Path) -> list[Path]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    paths: list[Path] = []

    eid = detail[detail["method"] == "eid"]
    if not eid.empty:
        first_key = sorted(eid["repeat"].unique())[0]
        w = eid[(eid["repeat"] == first_key) & (eid["t_rel"] >= 3.8) & (eid["t_rel"] <= 5.7)]
        fig, ax = plt.subplots(figsize=(9.5, 5.0))
        for col, label in [
            ("tau_sw_k", "software pulse"),
            ("d_hacc", "hip acceleration coupling"),
            ("d_vel", "velocity coupling"),
            ("d_g", "gravity/config residual"),
            ("d_in", "composite input residual"),
            ("eid_comp_k", "-eta_u"),
        ]:
            ax.plot(w["t_rel"], w[col], lw=1.1, label=label)
        ax.axvspan(4.0, 5.4, color="#D9D9D9", alpha=0.35, lw=0)
        ax.set_xlabel("t_rel [s]")
        ax.set_ylabel("knee torque component [N m]")
        ax.set_title(f"P2D EID {first_key}: MuJoCo knee disturbance decomposition")
        ax.grid(True, alpha=0.25)
        ax.legend(ncol=2, fontsize=8)
        fig.tight_layout()
        path = fig_dir / "knee_mujoco_decomposition_timeseries_eid_r01.png"
        fig.savefig(path, dpi=180)
        plt.close(fig)
        paths.append(path)

    dist = summary[summary["window"] == "disturbance"].copy()
    if not dist.empty:
        comps = ["tau_sw_k_rms", "d_g_rms", "d_hacc_rms", "d_kacc_rms", "d_vel_rms", "d_in_rms"]
        labels = ["software", "gravity", "hip acc", "knee inertia err", "velocity", "composite"]
        methods = sorted(dist["method"].unique())
        x = np.arange(len(labels))
        width = 0.35 if len(methods) > 1 else 0.5
        fig, ax = plt.subplots(figsize=(9.5, 4.8))
        for idx, method in enumerate(methods):
            sub = dist[dist["method"] == method].groupby("method").mean(numeric_only=True)
            vals = [float(sub[c].iloc[0]) for c in comps]
            offset = (idx - (len(methods) - 1) / 2.0) * width
            ax.bar(x + offset, vals, width=width, label=method.upper())
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=20, ha="right")
        ax.set_ylabel("RMS [N m]")
        ax.set_title("P2D disturbance-window knee input residual components")
        ax.grid(True, axis="y", alpha=0.25)
        ax.legend()
        fig.tight_layout()
        path = fig_dir / "knee_mujoco_component_rms.png"
        fig.savefig(path, dpi=180)
        plt.close(fig)
        paths.append(path)

    eid_align = align[(align["method"] == "eid") & (align["window"] == "disturbance")].copy()
    if not eid_align.empty:
        order = ["tau_sw_k", "d_g", "d_hacc", "d_kacc", "d_vel", "d_model", "d_in"]
        grouped = eid_align.groupby("target")["best_corr_abs_lag"].mean()
        vals = [grouped.get(k, np.nan) for k in order]
        fig, ax = plt.subplots(figsize=(8.5, 4.5))
        ax.bar(np.arange(len(order)), vals, color="#4C78A8")
        ax.axhline(0.0, color="black", lw=0.8)
        ax.set_xticks(np.arange(len(order)))
        ax.set_xticklabels(order, rotation=25, ha="right")
        ax.set_ylabel("mean best correlation")
        ax.set_title("EID compensation alignment targets")
        ax.grid(True, axis="y", alpha=0.25)
        fig.tight_layout()
        path = fig_dir / "knee_mujoco_eid_alignment.png"
        fig.savefig(path, dpi=180)
        plt.close(fig)
        paths.append(path)

    return paths


def write_report(agg: pd.DataFrame, align: pd.DataFrame, out_dir: Path, plot_paths: Iterable[Path]) -> Path:
    def markdown_table(df: pd.DataFrame, cols: list[str], floatfmt: str = ".4g") -> str:
        if df.empty:
            return ""
        table = df[cols].copy()
        header = "| " + " | ".join(cols) + " |"
        sep = "| " + " | ".join(["---"] * len(cols)) + " |"
        rows = []
        for _, row in table.iterrows():
            cells = []
            for col in cols:
                val = row[col]
                if isinstance(val, float) or isinstance(val, np.floating):
                    cells.append(format(float(val), floatfmt) if np.isfinite(val) else "")
                else:
                    cells.append(str(val))
            rows.append("| " + " | ".join(cells) + " |")
        return "\n".join([header, sep, *rows])

    report_dir = analysis_report_dir(out_dir)
    report_dir.mkdir(parents=True, exist_ok=True)
    path = report_dir / "knee_mujoco_disturbance_report.md"
    dist = agg[agg["window"] == "disturbance"].copy()
    lines = [
        "# MuJoCo Knee Disturbance Decomposition",
        "",
        "This report decomposes the right-knee input residual against the controller's local single-joint plant.",
        "",
        "## Disturbance Window RMS",
        "",
    ]
    if dist.empty:
        lines.append("No disturbance-window rows were produced.")
    else:
        cols = [
            "method",
            "n_runs",
            "tau_sw_k_rms",
            "d_g_rms",
            "d_hacc_rms",
            "d_kacc_rms",
            "d_vel_rms",
            "d_model_rms",
            "d_in_rms",
            "recon_err_over_model_rms",
        ]
        lines.append(markdown_table(dist, cols))
    lines.extend(["", "## EID Compensation Alignment", ""])
    eid_align = align[(align["method"] == "eid") & (align["window"] == "disturbance")]
    if eid_align.empty:
        lines.append("No EID alignment rows were produced.")
    else:
        table = (
            eid_align.groupby("target")[["corr_zero_lag", "best_corr_abs_lag", "best_lag_ms"]]
            .mean(numeric_only=True)
            .reset_index()
        )
        lines.append(markdown_table(table, table.columns.tolist()))
    if plot_paths:
        lines.extend(["", "## Figures", ""])
        for p in plot_paths:
            lines.append(f"- `{repo_relpath(p)}`")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, default=ROOT / "data")
    parser.add_argument("--xml", type=Path, default=ROOT / "h1_official_mujoco" / "h1.xml")
    parser.add_argument("--out-dir", type=Path, default=ROOT / "analysis_artifacts" / "knee_mujoco_disturbance")
    parser.add_argument("--methods", nargs="+", choices=["pd", "eid"], default=["pd", "eid"])
    parser.add_argument("--window-ms", type=float, default=81.0)
    parser.add_argument("--polyorder", type=int, default=3)
    parser.add_argument("--max-time", type=float, default=7.4)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    logs = discover_p2d_logs(args.data_dir, set(args.methods))
    if not logs:
        raise SystemExit("No P2D software-disturbance logs found.")

    inv = KneeInverseDynamics(args.xml)
    details = []
    for meta in logs:
        print(f"processing {meta.method.upper()} {meta.repeat}: {meta.log_path}")
        details.append(decompose_run(meta, inv, args.window_ms, args.polyorder, args.max_time))

    detail = pd.concat(details, ignore_index=True)
    detail_path = out_dir / "knee_mujoco_disturbance_timeseries.csv"
    detail.to_csv(detail_path, index=False)

    summary, align = summarize_runs(detail)
    agg = aggregate_summary(summary)
    summary_path = out_dir / "knee_mujoco_disturbance_summary.csv"
    align_path = out_dir / "knee_mujoco_disturbance_alignment.csv"
    agg_path = out_dir / "knee_mujoco_disturbance_aggregate.csv"
    summary.to_csv(summary_path, index=False)
    align.to_csv(align_path, index=False)
    agg.to_csv(agg_path, index=False)

    plot_paths = write_plots(detail, agg, align, out_dir)
    report_path = write_report(agg, align, out_dir, plot_paths)

    print(f"timeseries={detail_path}")
    print(f"summary={summary_path}")
    print(f"aggregate={agg_path}")
    print(f"alignment={align_path}")
    print(f"report={report_path}")
    for path in plot_paths:
        print(f"figure={path}")


if __name__ == "__main__":
    main()
