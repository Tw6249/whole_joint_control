from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
SUMMARY_CSV = ROOT / "analysis_artifacts" / "real_p2d_p4d_summary" / "p2d_p4d_summary_by_condition.csv"
RUN_METRICS_CSV = ROOT / "analysis_artifacts" / "real_p2d_p4d_summary" / "p2d_p4d_run_metrics.csv"
OUT_DIR = ROOT / "analysis_artifacts" / "bychen_answer"
FIG_DIR = OUT_DIR / "figures"

WINDOWS = {
    "pre": (3.0, 4.0),
    "disturbance": (4.0, 5.4),
    "plateau": (4.2, 5.2),
    "post": (5.4, 7.4),
}


@dataclass(frozen=True)
class JointPlant:
    name: str
    joint_id: int
    q0: float
    jeff: float
    b: float
    gravity_a: float
    gravity_b: float


PLANTS = [
    JointPlant("Hip", 1, -0.3, 1.00508532, 1.0, 15.7100627, 2.79723089),
    JointPlant("Knee", 2, 0.5, 0.2501484, 1.0, 4.14117407, -2.09365203),
]

OKABE = {
    "blue": "#0072B2",
    "sky": "#56B4E9",
    "green": "#009E73",
    "orange": "#E69F00",
    "vermillion": "#D55E00",
    "purple": "#CC79A7",
    "gray": "#6B7280",
    "black": "#111827",
}


def ensure_dirs() -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)


def configure_matplotlib() -> None:
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "DejaVu Serif"],
        "font.size": 9.0,
        "axes.labelsize": 9.2,
        "axes.titlesize": 9.4,
        "xtick.labelsize": 8.1,
        "ytick.labelsize": 8.1,
        "legend.fontsize": 8.0,
        "figure.dpi": 300,
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.grid": True,
        "grid.alpha": 0.18,
        "grid.linewidth": 0.55,
    })


def save_figure(fig: plt.Figure, out: Path) -> None:
    fig.savefig(out, bbox_inches="tight")
    fig.savefig(out.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(fig)


def panel_label(ax: plt.Axes, label: str) -> None:
    ax.text(
        -0.12,
        1.04,
        label,
        transform=ax.transAxes,
        fontweight="bold",
        va="bottom",
        ha="left",
    )


def nice_window_labels(values: list[str]) -> list[str]:
    names = {"pre": "Pre", "disturbance": "Disturb.", "post": "Post", "plateau": "Plateau"}
    return [names.get(v, v) for v in values]


def rel(path: Path) -> str:
    return str(path.relative_to(ROOT))


def load_summary() -> pd.DataFrame:
    df = pd.read_csv(SUMMARY_CSV)
    df["mean_delta_tau_est_rms"] = (
        df["hip_tau_est_diff_rms_nm_per_sample_mean"]
        + df["knee_tau_est_diff_rms_nm_per_sample_mean"]
    ) / 2.0
    df["mean_tau_est_rms"] = (
        df["hip_tau_est_rms_nm_mean"] + df["knee_tau_est_rms_nm_mean"]
    ) / 2.0
    df["mean_eta_u_rms"] = (
        df["hip_eta_u_rms_nm_mean"] + df["knee_eta_u_rms_nm_mean"]
    ) / 2.0
    df["candidate"] = df.apply(candidate_label, axis=1)
    return df


def load_run_metrics() -> pd.DataFrame:
    return pd.read_csv(RUN_METRICS_CSV)


def candidate_label(row: pd.Series) -> str:
    if row["family"] == "P2D":
        return f"P2D-{str(row['method']).upper()}"
    return str(row["group"]).upper()


def read_log(path: str | Path) -> pd.DataFrame:
    df = pd.read_csv(ROOT / path if not Path(path).is_absolute() else path)
    df["t_rel"] = df["t"] - df["t"].min()
    return df


def window_slice(df: pd.DataFrame, joint_id: int, window: str) -> pd.DataFrame:
    start, stop = WINDOWS[window]
    out = df[(df["joint_id"] == joint_id) & (df["t_rel"] >= start) & (df["t_rel"] < stop)].copy()
    return out.sort_values("t_rel")


def rms(x: pd.Series | np.ndarray) -> float:
    arr = np.asarray(x, dtype=float)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(arr * arr)))


def spectrum(signal: np.ndarray, dt: float, f_grid: np.ndarray) -> np.ndarray:
    y = np.asarray(signal, dtype=float)
    y = y[np.isfinite(y)]
    if y.size < 16 or not np.isfinite(dt) or dt <= 0:
        return np.full_like(f_grid, np.nan, dtype=float)
    y = y - np.mean(y)
    win = np.hanning(y.size)
    freq = np.fft.rfftfreq(y.size, d=dt)
    amp = 2.0 * np.abs(np.fft.rfft(y * win)) / max(np.sum(win), 1.0)
    return np.interp(f_grid, freq, amp, left=np.nan, right=np.nan)


def make_all_repeat_spectrum(run_metrics: pd.DataFrame) -> list[Path]:
    p2d = run_metrics[
        (run_metrics["family"] == "P2D")
        & (run_metrics["window"] == "disturbance")
        & (run_metrics["window_complete"])
    ].copy()
    logs = p2d[["method", "repeat", "log_path"]].drop_duplicates()
    f_grid = np.linspace(0.5, 100.0, 240)
    records = []

    for _, meta in logs.iterrows():
        df = read_log(meta["log_path"])
        for joint_id, joint_name in [(1, "Hip"), (2, "Knee")]:
            w = window_slice(df, joint_id, "disturbance")
            if w.empty:
                continue
            t = w["t_rel"].to_numpy()
            dt = float(np.median(np.diff(t))) if t.size > 2 else 0.002
            for sig_name, values in [
                ("tau_est", w["tau_est"].to_numpy()),
                ("delta_tau_est", np.diff(w["tau_est"].to_numpy())),
                ("eta_u", w.get("debug_32", pd.Series(np.zeros(len(w)))).to_numpy()),
            ]:
                amp = spectrum(values, dt, f_grid)
                for f, a in zip(f_grid, amp):
                    records.append(
                        {
                            "method": meta["method"],
                            "repeat": meta["repeat"],
                            "joint": joint_name,
                            "signal": sig_name,
                            "freq_hz": f,
                            "amp": a,
                        }
                    )

    spec = pd.DataFrame(records)
    spec.to_csv(OUT_DIR / "bychen_p2d_all_repeat_spectrum.csv", index=False)
    peak_rows = []
    grouped = spec.groupby(["method", "joint", "signal", "freq_hz"], as_index=False)["amp"].mean()

    for (method, joint, signal), g in grouped.groupby(["method", "joint", "signal"]):
        g = g.dropna()
        if not g.empty:
            idx = g["amp"].idxmax()
            peak_rows.append(
                {
                    "method": method,
                    "joint": joint,
                    "signal": signal,
                    "peak_freq_hz": float(g.loc[idx, "freq_hz"]),
                    "mean_peak_amp": float(g.loc[idx, "amp"]),
                }
            )
    pd.DataFrame(peak_rows).to_csv(OUT_DIR / "bychen_p2d_all_repeat_spectrum_peaks.csv", index=False)

    paths: list[Path] = []
    fig, axes = plt.subplots(2, 2, figsize=(8.6, 5.6), sharex=True)
    colors = {"pd": OKABE["gray"], "eid": OKABE["blue"]}
    for r, joint in enumerate(["Hip", "Knee"]):
        for c, signal in enumerate(["tau_est", "delta_tau_est"]):
            ax = axes[r, c]
            for method in ["pd", "eid"]:
                sub = spec[(spec["joint"] == joint) & (spec["signal"] == signal) & (spec["method"] == method)]
                pivot = sub.pivot_table(index="freq_hz", columns="repeat", values="amp")
                mean = pivot.mean(axis=1)
                std = pivot.std(axis=1)
                lower = np.maximum(mean - std, 0.0)
                ax.semilogy(mean.index, mean + 1e-12, color=colors[method], label=method.upper(), lw=1.35)
                ax.fill_between(mean.index, lower + 1e-12, mean + std + 1e-12, color=colors[method], alpha=0.13, linewidth=0)
            title = rf"{joint}: $\tau_{{est}}$" if signal == "tau_est" else rf"{joint}: $\Delta\tau_{{est}}$"
            ax.set_title(title)
            ax.set_ylabel("Amplitude")
            ax.set_xlim(0.5, 100.0)
    for ax in axes[-1, :]:
        ax.set_xlabel("Frequency (Hz)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.995), ncol=2, frameon=False)
    for idx, ax in enumerate(axes.ravel()):
        panel_label(ax, f"({chr(ord('a') + idx)})")
    fig.subplots_adjust(left=0.10, right=0.99, bottom=0.10, top=0.88, hspace=0.45, wspace=0.30)
    out = FIG_DIR / "bychen_p2d_all_repeat_spectrum.png"
    save_figure(fig, out)
    paths.append(out)
    return paths


def make_p4d_candidate_spectrum(run_metrics: pd.DataFrame) -> list[Path]:
    p4d = run_metrics[
        (run_metrics["family"] == "P4D")
        & (run_metrics["window"] == "disturbance")
        & (run_metrics["window_complete"])
        & (
            ((run_metrics["scan"] == "ko") & (run_metrics["group"].isin(["o4", "o5"])))
            | ((run_metrics["scan"] == "ku") & (run_metrics["group"].isin(["u1", "u4"])))
        )
    ].copy()
    logs = p4d[["scan", "group", "repeat", "log_path"]].drop_duplicates()
    f_grid = np.linspace(0.5, 100.0, 240)
    records = []

    for _, meta in logs.iterrows():
        df = read_log(meta["log_path"])
        label = str(meta["group"]).upper()
        for joint_id, joint_name in [(1, "Hip"), (2, "Knee")]:
            w = window_slice(df, joint_id, "disturbance")
            if w.empty:
                continue
            t = w["t_rel"].to_numpy()
            dt = float(np.median(np.diff(t))) if t.size > 2 else 0.002
            for sig_name, values in [
                ("tau_est", w["tau_est"].to_numpy()),
                ("delta_tau_est", np.diff(w["tau_est"].to_numpy())),
                ("eta_u", w.get("debug_32", pd.Series(np.zeros(len(w)))).to_numpy()),
            ]:
                amp = spectrum(values, dt, f_grid)
                for f, a in zip(f_grid, amp):
                    records.append(
                        {
                            "candidate": label,
                            "scan": meta["scan"],
                            "group": meta["group"],
                            "repeat": meta["repeat"],
                            "joint": joint_name,
                            "signal": sig_name,
                            "freq_hz": f,
                            "amp": a,
                        }
                    )

        w_all = df[
            (df["joint_id"].isin([1, 2]))
            & (df["t_rel"] >= WINDOWS["disturbance"][0])
            & (df["t_rel"] < WINDOWS["disturbance"][1])
        ].copy()
        pivot = w_all.pivot_table(index="cycle", columns="joint_id", values=["t_rel", "q", "debug_0"], aggfunc="first").dropna()
        if not pivot.empty and (1 in pivot["q"].columns) and (2 in pivot["q"].columns):
            t = pivot[("t_rel", 1)].to_numpy(dtype=float)
            dt = float(np.median(np.diff(t))) if t.size > 2 else 0.002
            e_hip = pivot[("debug_0", 1)].to_numpy(dtype=float) - pivot[("q", 1)].to_numpy(dtype=float)
            e_knee = pivot[("debug_0", 2)].to_numpy(dtype=float) - pivot[("q", 2)].to_numpy(dtype=float)
            coord = e_hip - e_knee
            amp = spectrum(coord, dt, f_grid)
            for f, a in zip(f_grid, amp):
                records.append(
                    {
                        "candidate": label,
                        "scan": meta["scan"],
                        "group": meta["group"],
                        "repeat": meta["repeat"],
                        "joint": "Coord",
                        "signal": "coord_error",
                        "freq_hz": f,
                        "amp": a,
                    }
                )

    spec = pd.DataFrame(records)
    spec.to_csv(OUT_DIR / "bychen_p4d_candidate_spectrum.csv", index=False)
    grouped = spec.groupby(["candidate", "joint", "signal", "freq_hz"], as_index=False)["amp"].mean()
    peak_rows = []
    for (candidate, joint, signal), g in grouped.groupby(["candidate", "joint", "signal"]):
        g = g.dropna()
        if g.empty:
            continue
        idx = g["amp"].idxmax()
        peak_rows.append(
            {
                "candidate": candidate,
                "joint": joint,
                "signal": signal,
                "peak_freq_hz": float(g.loc[idx, "freq_hz"]),
                "mean_peak_amp": float(g.loc[idx, "amp"]),
            }
        )
    pd.DataFrame(peak_rows).to_csv(OUT_DIR / "bychen_p4d_candidate_spectrum_peaks.csv", index=False)

    fig, axes = plt.subplots(2, 2, figsize=(8.8, 5.8), sharex=True)
    colors = {"O4": OKABE["blue"], "O5": OKABE["vermillion"], "U1": OKABE["green"], "U4": OKABE["purple"]}
    plot_specs = [
        ("Knee", "delta_tau_est", r"Knee: $\Delta\tau_{est}$"),
        ("Knee", "eta_u", r"Knee: $\eta_u$"),
        ("Hip", "delta_tau_est", r"Hip: $\Delta\tau_{est}$"),
        ("Coord", "coord_error", "Coordination error"),
    ]
    for ax, (joint, sig_name, title) in zip(axes.ravel(), plot_specs):
        for candidate in ["O4", "O5", "U1", "U4"]:
            sub = spec[(spec["candidate"] == candidate) & (spec["joint"] == joint) & (spec["signal"] == sig_name)]
            if sub.empty:
                continue
            pivot = sub.pivot_table(index="freq_hz", columns="repeat", values="amp")
            mean = pivot.mean(axis=1)
            ax.semilogy(mean.index, mean + 1e-12, color=colors[candidate], label=candidate, lw=1.35)
        ax.set_title(title)
        ax.set_ylabel("Amplitude")
        ax.set_xlim(0.5, 100.0)
    for ax in axes[-1, :]:
        ax.set_xlabel("Frequency (Hz)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.995), ncol=4, frameon=False)
    for idx, ax in enumerate(axes.ravel()):
        panel_label(ax, f"({chr(ord('a') + idx)})")
    fig.subplots_adjust(left=0.10, right=0.99, bottom=0.10, top=0.88, hspace=0.45, wspace=0.30)
    out = FIG_DIR / "bychen_p4d_candidate_spectrum.png"
    save_figure(fig, out)
    return [out]


def make_window_recovery_figures(summary: pd.DataFrame) -> list[Path]:
    paths: list[Path] = []
    windows = ["pre", "disturbance", "post"]
    p2d = summary[(summary["family"] == "P2D") & (summary["window"].isin(windows))].copy()
    p2d["label"] = p2d["method"].str.upper()

    fig, axes = plt.subplots(1, 2, figsize=(8.4, 3.4), sharex=True)
    p2d_colors = {"PD": OKABE["gray"], "EID": OKABE["blue"]}
    x = np.arange(len(windows))
    for metric, ax, ylabel in [
        ("coord_rmse_rad_mean", axes[0], "Coordination RMSE (rad)"),
        ("mean_delta_tau_est_rms", axes[1], r"Mean $\Delta\tau_{est}$ RMS"),
    ]:
        pivot = p2d.pivot_table(index="window", columns="label", values=metric).reindex(windows)
        for label in ["PD", "EID"]:
            if label in pivot:
                ax.plot(x, pivot[label].to_numpy(dtype=float), marker="o", lw=1.7, color=p2d_colors[label], label=label)
        ax.set_xticks(x)
        ax.set_xticklabels(nice_window_labels(windows))
        ax.set_ylabel(ylabel)
        ax.margins(y=0.18)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 1.02), ncol=2, frameon=False)
    for idx, ax in enumerate(axes):
        panel_label(ax, f"({chr(ord('a') + idx)})")
    fig.subplots_adjust(left=0.10, right=0.99, bottom=0.17, top=0.82, wspace=0.30)
    out = FIG_DIR / "bychen_p2d_window_recovery.png"
    save_figure(fig, out)
    paths.append(out)

    candidates = summary[
        (summary["family"] == "P4D")
        & (summary["window"].isin(windows))
        & (
            ((summary["scan"] == "ko") & (summary["group"].isin(["o4", "o5"])))
            | ((summary["scan"] == "ku") & (summary["group"].isin(["u1", "u4"])))
        )
    ].copy()
    candidates["label"] = candidates["group"].str.upper()
    fig, axes = plt.subplots(1, 2, figsize=(8.6, 3.5), sharex=True)
    cand_colors = {"O4": OKABE["blue"], "O5": OKABE["vermillion"], "U1": OKABE["green"], "U4": OKABE["purple"]}
    x = np.arange(len(windows))
    for metric, ax, ylabel in [
        ("coord_rmse_rad_mean", axes[0], "Coordination RMSE (rad)"),
        ("mean_delta_tau_est_rms", axes[1], r"Mean $\Delta\tau_{est}$ RMS"),
    ]:
        pivot = candidates.pivot_table(index="window", columns="label", values=metric).reindex(windows)
        for label in ["O4", "O5", "U1", "U4"]:
            if label in pivot:
                ax.plot(x, pivot[label].to_numpy(dtype=float), marker="o", lw=1.6, color=cand_colors[label], label=label)
        ax.set_xticks(x)
        ax.set_xticklabels(nice_window_labels(windows))
        ax.set_ylabel(ylabel)
        ax.margins(y=0.18)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 1.02), ncol=4, frameon=False)
    for idx, ax in enumerate(axes):
        panel_label(ax, f"({chr(ord('a') + idx)})")
    fig.subplots_adjust(left=0.10, right=0.99, bottom=0.17, top=0.82, wspace=0.30)
    out = FIG_DIR / "bychen_p4d_candidate_windows.png"
    save_figure(fig, out)
    paths.append(out)
    return paths


def make_coord_decomposition(run_metrics: pd.DataFrame) -> list[Path]:
    rows = []
    meta = run_metrics[
        (run_metrics["family"] == "P2D")
        & (run_metrics["window"] == "disturbance")
        & (run_metrics["window_complete"])
    ][["method", "repeat", "log_path"]].drop_duplicates()

    for _, m in meta.iterrows():
        df = read_log(m["log_path"])
        hip = window_slice(df, 1, "disturbance")[["cycle", "debug_0", "q"]].rename(
            columns={"debug_0": "q_ref_hip", "q": "q_hip"}
        )
        knee = window_slice(df, 2, "disturbance")[["cycle", "debug_0", "q"]].rename(
            columns={"debug_0": "q_ref_knee", "q": "q_knee"}
        )
        merged = hip.merge(knee, on="cycle", how="inner")
        if merged.empty:
            continue
        e_hip = merged["q_ref_hip"].to_numpy(dtype=float) - merged["q_hip"].to_numpy(dtype=float)
        e_knee = merged["q_ref_knee"].to_numpy(dtype=float) - merged["q_knee"].to_numpy(dtype=float)
        hip_ms = float(np.mean(e_hip * e_hip))
        knee_ms = float(np.mean(e_knee * e_knee))
        covariance_term = float(-2.0 * np.mean(e_hip * e_knee))
        coord_ms = float(np.mean((e_hip - e_knee) ** 2))
        reconstructed_ms = hip_ms + knee_ms + covariance_term
        rows.append(
            {
                "method": str(m["method"]).upper(),
                "repeat": m["repeat"],
                "n_samples": int(len(merged)),
                "hip_ms": hip_ms,
                "knee_ms": knee_ms,
                "covariance_term": covariance_term,
                "coord_ms_direct": coord_ms,
                "coord_ms_reconstructed": reconstructed_ms,
                "coord_rmse_direct": float(np.sqrt(coord_ms)),
                "coord_rmse_reconstructed": float(np.sqrt(max(reconstructed_ms, 0.0))),
                "reconstruction_error": reconstructed_ms - coord_ms,
            }
        )

    detail = pd.DataFrame(rows)
    detail.to_csv(OUT_DIR / "bychen_coord_decomposition_runs.csv", index=False)
    summary = (
        detail.groupby("method")
        .agg(
            n_runs=("repeat", "count"),
            n_samples_mean=("n_samples", "mean"),
            hip_ms=("hip_ms", "mean"),
            knee_ms=("knee_ms", "mean"),
            covariance_term=("covariance_term", "mean"),
            coord_ms_direct=("coord_ms_direct", "mean"),
            coord_ms_reconstructed=("coord_ms_reconstructed", "mean"),
            coord_rmse_direct=("coord_rmse_direct", "mean"),
            max_abs_reconstruction_error=("reconstruction_error", lambda x: float(np.max(np.abs(x)))),
        )
        .reset_index()
    )
    summary.to_csv(OUT_DIR / "bychen_coord_decomposition.csv", index=False)

    plot = summary.set_index("method").reindex(["PD", "EID"]).reset_index()
    components = [
        ("hip_ms", r"$E[e_{hip}^2]$", OKABE["blue"]),
        ("knee_ms", r"$E[e_{knee}^2]$", OKABE["orange"]),
        ("covariance_term", r"$-2E[e_{hip}e_{knee}]$", OKABE["green"]),
    ]
    x = np.arange(len(plot))
    fig, ax = plt.subplots(figsize=(6.8, 3.6))
    positive_bottom = np.zeros(len(plot))
    negative_bottom = np.zeros(len(plot))
    for col, label, color in components:
        values = plot[col].to_numpy(dtype=float)
        bottoms = np.where(values >= 0.0, positive_bottom, negative_bottom)
        ax.bar(x, values, bottom=bottoms, width=0.52, label=label, color=color, alpha=0.92)
        positive_bottom += np.where(values >= 0.0, values, 0.0)
        negative_bottom += np.where(values < 0.0, values, 0.0)
    ax.scatter(
        x,
        plot["coord_ms_direct"].to_numpy(dtype=float),
        color=OKABE["black"],
        marker="D",
        s=34,
        label=r"direct $E[e_{coord}^2]$",
        zorder=4,
    )
    ax.set_xticks(x)
    ax.set_xticklabels(plot["method"])
    ax.set_ylabel(r"Mean-square error (rad$^2$)")
    ax.axhline(0.0, color="#111827", lw=0.8)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, 1.18), ncol=2, frameon=False)
    ax.margins(y=0.20)
    fig.subplots_adjust(left=0.12, right=0.99, bottom=0.16, top=0.78)
    out = FIG_DIR / "bychen_coord_decomposition.png"
    save_figure(fig, out)
    return [out]


def corr_at_lag(x: np.ndarray, y: np.ndarray, lag: int) -> float:
    if lag > 0:
        xs, ys = x[:-lag], y[lag:]
    elif lag < 0:
        xs, ys = x[-lag:], y[:lag]
    else:
        xs, ys = x, y
    if xs.size < 8 or ys.size < 8 or np.std(xs) < 1e-12 or np.std(ys) < 1e-12:
        return float("nan")
    return float(np.corrcoef(xs, ys)[0, 1])


def best_lag_correlation(x: np.ndarray, y: np.ndarray, dt: float, max_lag_s: float = 0.25) -> tuple[float, float]:
    max_lag = int(max_lag_s / max(dt, 1e-9))
    best_corr = float("nan")
    best_lag = 0
    for lag in range(-max_lag, max_lag + 1):
        c = corr_at_lag(x, y, lag)
        if np.isfinite(c) and (not np.isfinite(best_corr) or abs(c) > abs(best_corr)):
            best_corr = c
            best_lag = lag
    return best_corr, 1000.0 * best_lag * dt


def make_eta_disturbance_alignment(run_metrics: pd.DataFrame) -> list[Path]:
    rows = []
    meta = run_metrics[
        (run_metrics["method"] == "eid")
        & (run_metrics["window"] == "disturbance")
        & (run_metrics["window_complete"])
    ][["family", "scan", "group", "ko_scale", "ku_scale", "repeat", "log_path"]].drop_duplicates()

    for _, m in meta.iterrows():
        df = read_log(m["log_path"])
        for joint_id, joint_name in [(1, "Hip"), (2, "Knee")]:
            w = window_slice(df, joint_id, "disturbance")
            if w.empty or "debug_32" not in w:
                continue
            t = w["t_rel"].to_numpy()
            dt = float(np.median(np.diff(t))) if t.size > 2 else 0.002
            eta_u = w["debug_32"].to_numpy(dtype=float)
            comp = -eta_u
            tau_dist = w["tau_dist"].to_numpy(dtype=float)
            zero_corr_eta = corr_at_lag(eta_u, tau_dist, 0)
            zero_corr_comp = corr_at_lag(comp, tau_dist, 0)
            best_corr, best_lag_ms = best_lag_correlation(comp, tau_dist, dt)
            rows.append(
                {
                    **{k: m[k] for k in ["family", "scan", "group", "ko_scale", "ku_scale", "repeat", "log_path"]},
                    "joint": joint_name,
                    "eta_u_rms": rms(eta_u),
                    "comp_rms": rms(comp),
                    "tau_dist_rms": rms(tau_dist),
                    "eta_to_dist_rms_ratio": rms(eta_u) / rms(tau_dist) if rms(tau_dist) > 1e-12 else np.nan,
                    "corr_eta_u_tau_dist_zero_lag": zero_corr_eta,
                    "corr_minus_eta_u_tau_dist_zero_lag": zero_corr_comp,
                    "best_corr_minus_eta_u_tau_dist": best_corr,
                    "best_lag_ms_minus_eta_u_tau_dist": best_lag_ms,
                }
            )

    detail = pd.DataFrame(rows)
    detail.to_csv(OUT_DIR / "bychen_eta_disturbance_alignment_runs.csv", index=False)
    summary = detail.groupby(["family", "scan", "group", "joint"], dropna=False).agg(
        n=("repeat", "nunique"),
        eta_to_dist_rms_ratio_mean=("eta_to_dist_rms_ratio", "mean"),
        corr_comp_zero_lag_mean=("corr_minus_eta_u_tau_dist_zero_lag", "mean"),
        best_corr_comp_mean=("best_corr_minus_eta_u_tau_dist", "mean"),
        best_lag_ms_mean=("best_lag_ms_minus_eta_u_tau_dist", "mean"),
    ).reset_index()
    summary["label"] = summary.apply(lambda r: "P2D" if r["family"] == "P2D" else str(r["group"]).upper(), axis=1)
    summary.to_csv(OUT_DIR / "bychen_eta_disturbance_alignment_summary.csv", index=False)

    paths: list[Path] = []
    plot_data = summary[
        ((summary["family"] == "P2D"))
        | ((summary["family"] == "P4D") & (summary["scan"].isin(["ko", "ku"])))
    ].copy()
    order = ["P2D", "O1", "O2", "O3", "O4", "O5", "U1", "U2", "U3", "U4"]
    plot_data["label"] = pd.Categorical(plot_data["label"], categories=order, ordered=True)
    plot_data = plot_data.sort_values(["label", "joint"])

    labels = [label for label in order if label in set(plot_data["label"].astype(str))]
    x = np.arange(len(labels))
    fig, axes = plt.subplots(2, 1, figsize=(8.8, 5.2), sharex=True)
    for joint, marker, color in [("Hip", "o", OKABE["blue"]), ("Knee", "s", OKABE["vermillion"])]:
        sub = plot_data[plot_data["joint"] == joint].copy()
        ratio = sub.set_index(sub["label"].astype(str))["eta_to_dist_rms_ratio_mean"].reindex(labels)
        corr = sub.set_index(sub["label"].astype(str))["corr_comp_zero_lag_mean"].reindex(labels)
        axes[0].plot(x, ratio.to_numpy(dtype=float), marker=marker, color=color, lw=1.6, label=joint)
        axes[1].plot(x, corr.to_numpy(dtype=float), marker=marker, color=color, lw=1.6, label=joint)
    axes[0].axhline(1.0, color="#9CA3AF", lw=0.8, ls="--")
    axes[1].axhline(0.0, color="#9CA3AF", lw=0.8, ls="--")
    axes[0].set_ylabel(r"RMS($\eta_u$) / RMS($\tau_d$)")
    axes[1].set_ylabel(r"corr($-\eta_u$, $\tau_d$)")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(labels)
    for ax in axes:
        ax.margins(x=0.02, y=0.16)
    handles, labels_leg = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels_leg, loc="upper center", bbox_to_anchor=(0.5, 1.01), ncol=2, frameon=False)
    for idx, ax in enumerate(axes):
        panel_label(ax, f"({chr(ord('a') + idx)})")
    fig.subplots_adjust(left=0.12, right=0.99, bottom=0.13, top=0.86, hspace=0.22)
    out = FIG_DIR / "bychen_eta_disturbance_alignment.png"
    save_figure(fig, out)
    paths.append(out)
    return paths


def make_force_decomposition(run_metrics: pd.DataFrame) -> list[Path]:
    rows = []
    meta = run_metrics[
        (run_metrics["method"] == "eid")
        & (run_metrics["window"] == "disturbance")
        & (run_metrics["window_complete"])
    ][["family", "scan", "group", "ko_scale", "ku_scale", "repeat", "log_path"]].drop_duplicates()

    for _, m in meta.iterrows():
        df = read_log(m["log_path"])
        for joint_id, joint_name in [(1, "Hip"), (2, "Knee")]:
            w = window_slice(df, joint_id, "disturbance")
            if w.empty:
                continue
            rows.append(
                {
                    **{k: m[k] for k in ["family", "scan", "group", "ko_scale", "ku_scale", "repeat", "log_path"]},
                    "joint": joint_name,
                    "u_star_rms": rms(w["debug_6"]),
                    "u_feedback_rms": rms(w["debug_7"]),
                    "u_eid_comp_rms": rms(-w["debug_32"]),
                    "u_raw_rms": rms(w["debug_25"]),
                    "u_t_rms": rms(w["debug_8"]),
                    "tau_cmd_rms": rms(w["tau_cmd"]),
                    "tau_est_rms": rms(w["tau_est"]),
                    "tau_dist_rms": rms(w["tau_dist"]),
                }
            )

    detail = pd.DataFrame(rows)
    detail.to_csv(OUT_DIR / "bychen_force_decomposition_runs.csv", index=False)
    summary = detail.groupby(["family", "scan", "group", "joint"], dropna=False).mean(numeric_only=True).reset_index()
    summary["label"] = summary.apply(lambda r: "P2D" if r["family"] == "P2D" else str(r["group"]).upper(), axis=1)
    summary.to_csv(OUT_DIR / "bychen_force_decomposition_summary.csv", index=False)

    selected = summary[
        (summary["label"].isin(["P2D", "O4", "O5", "U1", "U4"]))
    ].copy()
    label_order = {"O4": 0, "O5": 1, "P2D": 2, "U1": 3, "U4": 4}
    joint_order = {"Hip": 0, "Knee": 1}
    selected["label_order"] = selected["label"].map(label_order)
    selected["joint_order"] = selected["joint"].map(joint_order)
    selected["label_joint"] = selected["label"] + "\n" + selected["joint"]
    selected = selected.sort_values(["label_order", "joint_order"])
    x = np.arange(len(selected))
    width = 0.24
    fig, axes = plt.subplots(
        2,
        1,
        figsize=(9.2, 5.8),
        sharex=True,
        gridspec_kw={"height_ratios": [2.2, 1.0]},
    )
    for offset, col, label, color in [
        (-1.0, "u_star_rms", r"$u^\star$", OKABE["blue"]),
        (0.0, "u_feedback_rms", r"$u_{fb}$", OKABE["green"]),
        (1.0, "u_t_rms", r"$u_t$", OKABE["purple"]),
    ]:
        axes[0].bar(x + offset * width, selected[col], width=width, label=label, color=color, alpha=0.92)
    axes[1].bar(x, selected["u_eid_comp_rms"], width=0.42, label=r"$-\eta_u$", color=OKABE["vermillion"], alpha=0.90)
    axes[0].set_ylabel("Main-term RMS (N m)")
    axes[1].set_ylabel(r"$-\eta_u$ RMS (N m)")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(selected["label_joint"], rotation=0)
    axes[0].legend(loc="upper center", bbox_to_anchor=(0.5, 1.16), ncol=3, frameon=False)
    axes[1].legend(loc="upper right", frameon=False)
    for idx, ax in enumerate(axes):
        panel_label(ax, f"({chr(ord('a') + idx)})")
        ax.margins(x=0.02, y=0.14)
    fig.subplots_adjust(left=0.10, right=0.99, bottom=0.12, top=0.88, hspace=0.18)
    out = FIG_DIR / "bychen_force_decomposition.png"
    save_figure(fig, out)
    return [out]


def make_multiobjective(summary: pd.DataFrame) -> list[Path]:
    dist = summary[(summary["window"] == "disturbance") & (summary["n_complete_window"] > 0)].copy()
    dist = dist[~((dist["family"] == "P4D") & (dist["group"] == "o0"))].copy()
    dist["label"] = dist["candidate"]
    metrics = {
        "hip_rmse_rad_mean": "hip_rmse_norm",
        "knee_rmse_rad_mean": "knee_rmse_norm",
        "coord_rmse_rad_mean": "coord_rmse_norm",
        "mean_tau_est_rms": "tau_rms_norm",
        "mean_delta_tau_est_rms": "delta_tau_norm",
        "mean_eta_u_rms": "eta_u_norm",
    }
    scales = {}
    p2d_pd = dist[(dist["family"] == "P2D") & (dist["method"] == "pd")]
    for src, dst in metrics.items():
        if src == "mean_eta_u_rms":
            scales[src] = max(float(dist[src].max()), 1e-9)
        elif not p2d_pd.empty and np.isfinite(float(p2d_pd.iloc[0][src])) and float(p2d_pd.iloc[0][src]) > 1e-12:
            scales[src] = float(p2d_pd.iloc[0][src])
        else:
            scales[src] = max(float(dist[src].median()), 1e-9)
        dist[dst] = dist[src] / scales[src]

    schemes = {
        "conservative": {
            "hip_rmse_norm": 0.10,
            "knee_rmse_norm": 0.10,
            "coord_rmse_norm": 0.20,
            "tau_rms_norm": 0.20,
            "delta_tau_norm": 0.30,
            "eta_u_norm": 0.10,
        },
        "error_first": {
            "hip_rmse_norm": 0.25,
            "knee_rmse_norm": 0.25,
            "coord_rmse_norm": 0.30,
            "tau_rms_norm": 0.10,
            "delta_tau_norm": 0.05,
            "eta_u_norm": 0.05,
        },
        "coord_first": {
            "hip_rmse_norm": 0.10,
            "knee_rmse_norm": 0.10,
            "coord_rmse_norm": 0.50,
            "tau_rms_norm": 0.10,
            "delta_tau_norm": 0.15,
            "eta_u_norm": 0.05,
        },
    }
    for name, weights in schemes.items():
        dist[f"J_{name}"] = sum(dist[col] * w for col, w in weights.items())
        dist[f"rank_{name}"] = dist[f"J_{name}"].rank(method="min")

    keep = [
        "label",
        "family",
        "method",
        "scan",
        "group",
        "ko_scale",
        "ku_scale",
        "J_conservative",
        "rank_conservative",
        "J_error_first",
        "rank_error_first",
        "J_coord_first",
        "rank_coord_first",
    ]
    dist[keep].sort_values("J_conservative").to_csv(OUT_DIR / "bychen_multiobjective_ranking.csv", index=False)
    pd.DataFrame([{"metric": k, "normalization_scale": v} for k, v in scales.items()]).to_csv(
        OUT_DIR / "bychen_multiobjective_normalization.csv", index=False
    )

    plot = dist.set_index("label")[["J_conservative", "J_error_first", "J_coord_first"]].copy()
    plot = plot.sort_values("J_conservative", ascending=False)
    fig, ax = plt.subplots(figsize=(8.6, 5.2))
    y = np.arange(len(plot))
    height = 0.22
    colors = [OKABE["blue"], OKABE["orange"], OKABE["green"]]
    for offset, col, label, color in [
        (-height, "J_conservative", "Conservative", colors[0]),
        (0.0, "J_error_first", "Error first", colors[1]),
        (height, "J_coord_first", "Coord first", colors[2]),
    ]:
        ax.barh(y + offset, plot[col].to_numpy(dtype=float), height=height, label=label, color=color, alpha=0.90)
    ax.set_yticks(y)
    ax.set_yticklabels(plot.index)
    ax.set_xlabel("Normalized weighted cost, lower is better")
    ax.set_ylabel("Candidate")
    ax.grid(True, axis="x", alpha=0.18)
    ax.grid(False, axis="y")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, 1.08), ncol=3, frameon=False)
    fig.subplots_adjust(left=0.16, right=0.99, bottom=0.10, top=0.87)
    out = FIG_DIR / "bychen_multiobjective_sensitivity.png"
    save_figure(fig, out)
    return [out]


def linearized_ab(plant: JointPlant, dt: float) -> tuple[np.ndarray, np.ndarray]:
    gq = plant.gravity_a * np.cos(plant.q0) - plant.gravity_b * np.sin(plant.q0)
    j = plant.jeff
    b = plant.b
    a = np.array(
        [
            [1.0 - dt * dt * gq / j, dt - dt * dt * b / j],
            [-dt * gq / j, 1.0 - dt * b / j],
        ],
        dtype=float,
    )
    bmat = np.array([[dt * dt / j], [dt / j]], dtype=float)
    return a, bmat


def augmented_mats(
    plant: JointPlant,
    ko_scale: float,
    ku_scale: float,
    dt: float = 0.002,
    kp: float = 120.0,
    kd: float = 5.0,
    alpha: float = 0.85,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    a, b = linearized_ab(plant, dt)
    an, bn = a.copy(), b.copy()
    k = np.array([[kp, kd]], dtype=float)
    ko = np.diag([0.8 * ko_scale, 0.2 * ko_scale])
    ku = np.array([[12.0 * ku_scale, 1.0 * ku_scale]], dtype=float)
    acl = np.block(
        [
            [a, -b @ k, -b @ (k + ku)],
            [np.zeros((2, 2)), an - bn @ k, an - bn @ (k + ku)],
            [alpha * ko, -alpha * ko, (1.0 - alpha) * np.eye(2) - alpha * ko],
        ]
    )
    bd = np.vstack([b, np.zeros((2, 1)), np.zeros((2, 1))])
    bv = np.vstack([np.zeros((2, 2)), np.zeros((2, 2)), alpha * ko])
    ceq = np.array([[-1.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
    cu = np.hstack([np.zeros((1, 2)), -k, -(k + ku)])
    return acl, bd, bv, ceq, cu


def freq_response_mag(
    acl: np.ndarray,
    bmat: np.ndarray,
    cmat: np.ndarray,
    freqs: np.ndarray,
    dt: float,
    input_norm: bool = False,
) -> np.ndarray:
    eye = np.eye(acl.shape[0])
    out = []
    for f in freqs:
        z = np.exp(1j * 2.0 * np.pi * f * dt)
        h = cmat @ np.linalg.solve(z * eye - acl, bmat)
        if input_norm:
            out.append(float(np.linalg.norm(h, ord=2)))
        else:
            out.append(float(np.abs(h).squeeze()))
    return np.asarray(out)


def make_augmented_closed_loop_analysis() -> list[Path]:
    paths: list[Path] = []
    dt = 0.002
    ko_grid = np.linspace(0.0, 1.5, 61)
    ku_grid = np.linspace(0.0, 1.5, 61)

    fig, axes = plt.subplots(1, 2, figsize=(10.0, 4.2), dpi=160)
    for ax, plant in zip(axes, PLANTS):
        zmax = np.zeros((len(ku_grid), len(ko_grid)))
        for i, ku in enumerate(ku_grid):
            for j, ko in enumerate(ko_grid):
                acl, *_ = augmented_mats(plant, ko, ku, dt=dt)
                eig = np.linalg.eigvals(acl)
                zmax[i, j] = np.max(np.abs(eig))
        im = ax.imshow(
            zmax,
            origin="lower",
            extent=[ko_grid.min(), ko_grid.max(), ku_grid.min(), ku_grid.max()],
            aspect="auto",
            cmap="magma",
            vmin=np.nanmin(zmax),
            vmax=min(np.nanmax(zmax), 1.5),
        )
        ax.contour(ko_grid, ku_grid, zmax, levels=[1.0], colors="cyan", linewidths=1.1)
        ax.scatter([1.0, 1.25], [0.5, 0.5], c=["white", "lime"], edgecolor="black", label="O4/O5 + U1")
        ax.set_title(f"{plant.name}: max |closed-loop pole|")
        ax.set_xlabel("Ko scale")
        ax.set_ylabel("Ku scale")
        ax.legend(fontsize=8)
    cbar = fig.colorbar(im, ax=axes.ravel().tolist(), fraction=0.035, pad=0.02)
    cbar.set_label("max pole magnitude")
    out = FIG_DIR / "bychen_augmented_pole_grid.png"
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    paths.append(out)

    rows = []
    for plant in PLANTS:
        for label, ko, ku in [("O4+U1", 1.0, 0.5), ("O5+U1", 1.25, 0.5), ("O4+U3", 1.0, 1.0)]:
            acl, *_ = augmented_mats(plant, ko, ku, dt=dt)
            eig = np.linalg.eigvals(acl)
            rows.append(
                {
                    "joint": plant.name,
                    "candidate": label,
                    "ko_scale": ko,
                    "ku_scale": ku,
                    "max_pole_magnitude": float(np.max(np.abs(eig))),
                    "dominant_pole_real": float(eig[np.argmax(np.abs(eig))].real),
                    "dominant_pole_imag": float(eig[np.argmax(np.abs(eig))].imag),
                }
            )
    pd.DataFrame(rows).to_csv(OUT_DIR / "bychen_augmented_poles.csv", index=False)

    freqs = np.linspace(0.1, 100.0, 260)
    fig, axes = plt.subplots(2, 2, figsize=(10.2, 7.0), dpi=160, sharex=True)
    colors = {"O4+U1": "#2563eb", "O5+U1": "#dc2626", "O4+U3": "#16a34a"}
    for r, plant in enumerate(PLANTS):
        for label, ko, ku in [("O4+U1", 1.0, 0.5), ("O5+U1", 1.25, 0.5), ("O4+U3", 1.0, 1.0)]:
            acl, bd, bv, ceq, cu = augmented_mats(plant, ko, ku, dt=dt)
            axes[r, 0].semilogy(freqs, freq_response_mag(acl, bd, ceq, freqs, dt) + 1e-12, color=colors[label], label=label)
            axes[r, 1].semilogy(freqs, freq_response_mag(acl, bv, cu, freqs, dt, input_norm=True) + 1e-12, color=colors[label], label=label)
        axes[r, 0].set_title(f"{plant.name}: input disturbance -> q error")
        axes[r, 1].set_title(f"{plant.name}: measurement noise -> u")
        axes[r, 0].set_ylabel("Magnitude")
        axes[r, 1].set_ylabel("Magnitude")
        axes[r, 0].grid(True, alpha=0.28)
        axes[r, 1].grid(True, alpha=0.28)
    for ax in axes[-1, :]:
        ax.set_xlabel("Frequency (Hz)")
    axes[0, 0].legend(fontsize=8)
    axes[0, 1].legend(fontsize=8)
    fig.suptitle("Simplified augmented closed-loop frequency response", y=1.02)
    fig.tight_layout()
    out = FIG_DIR / "bychen_augmented_frequency_response.png"
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    paths.append(out)
    return paths


def main() -> None:
    configure_matplotlib()
    ensure_dirs()
    summary = load_summary()
    run_metrics = load_run_metrics()
    generated: list[Path] = []
    generated += make_all_repeat_spectrum(run_metrics)
    generated += make_p4d_candidate_spectrum(run_metrics)
    generated += make_window_recovery_figures(summary)
    generated += make_coord_decomposition(run_metrics)
    generated += make_eta_disturbance_alignment(run_metrics)
    generated += make_force_decomposition(run_metrics)
    generated += make_multiobjective(summary)
    generated += make_augmented_closed_loop_analysis()

    print("Generated figures:")
    for path in generated:
        print(rel(path))
    print("Generated tables:")
    for path in sorted(OUT_DIR.glob("bychen_*csv")):
        print(rel(path))


if __name__ == "__main__":
    main()
