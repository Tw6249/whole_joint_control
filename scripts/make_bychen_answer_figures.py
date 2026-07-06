from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
SUMMARY_CSV = ROOT / "analysis_artifacts" / "real_p2d_p4d_summary" / "p2d_p4d_summary_by_condition.csv"
OUT_DIR = ROOT / "analysis_artifacts" / "bychen_answer" / "figures"
TABLE_DIR = ROOT / "analysis_artifacts" / "bychen_answer"

P2D_PD_LOG = ROOT / "data" / "20260623_191336" / "h1_real_p2_anti_hip_knee_pd_log.csv"
P2D_EID_LOG = ROOT / "data" / "20260623_191351" / "h1_real_p2_anti_hip_knee_eid_log.csv"

OKABE = {
    "blue": "#0072B2",
    "sky": "#56B4E9",
    "green": "#009E73",
    "orange": "#E69F00",
    "vermillion": "#D55E00",
    "purple": "#CC79A7",
    "gray": "#6B7280",
}


def ensure_dirs() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    TABLE_DIR.mkdir(parents=True, exist_ok=True)


def configure_matplotlib() -> None:
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "DejaVu Serif"],
        "font.size": 9.0,
        "axes.labelsize": 9.2,
        "axes.titlesize": 9.4,
        "xtick.labelsize": 8.2,
        "ytick.labelsize": 8.2,
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


def pad_limits(values: pd.Series, pad_ratio: float = 0.08) -> tuple[float, float]:
    arr = values.to_numpy(dtype=float)
    arr = arr[np.isfinite(arr)]
    lo, hi = float(np.min(arr)), float(np.max(arr))
    pad = max((hi - lo) * pad_ratio, 1e-6)
    return lo - pad, hi + pad


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


def load_disturbance_summary() -> pd.DataFrame:
    df = pd.read_csv(SUMMARY_CSV)
    df = df[df["window"] == "disturbance"].copy()
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
    df["plot_label"] = df.apply(make_label, axis=1)
    df["valid_window"] = df["n_complete_window"].fillna(0) > 0
    return df


def make_label(row: pd.Series) -> str:
    if row["family"] == "P2D":
        return f"P2D {str(row['method']).upper()}"
    return str(row["group"]).upper()


def plot_tradeoff(df: pd.DataFrame) -> Path:
    fig, ax = plt.subplots(figsize=(8.4, 5.3))
    valid = df[df["valid_window"]].copy()

    p2d = valid[valid["family"] == "P2D"]
    ko = valid[(valid["family"] == "P4D") & (valid["scan"] == "ko")]
    ku = valid[(valid["family"] == "P4D") & (valid["scan"] == "ku")]
    invalid = df[~df["valid_window"]]

    def size(series: pd.Series) -> np.ndarray:
        return 58.0 + 1450.0 * series.fillna(0.0).to_numpy()

    if not p2d.empty:
        colors = [OKABE["gray"] if method == "pd" else OKABE["blue"] for method in p2d["method"]]
        ax.scatter(
            p2d["coord_rmse_rad_mean"],
            p2d["mean_delta_tau_est_rms"],
            s=size(p2d["mean_eta_u_rms"]),
            c=colors,
            marker="s",
            edgecolor="black",
            linewidth=0.7,
            label="P2D PD/EID",
            zorder=3,
        )

    if not ko.empty:
        sc_ko = ax.scatter(
            ko["coord_rmse_rad_mean"],
            ko["mean_delta_tau_est_rms"],
            s=size(ko["mean_eta_u_rms"]),
            c=ko["ko_scale"],
            cmap="viridis",
            marker="o",
            edgecolor="black",
            linewidth=0.6,
            label="P4D Ko scan",
            zorder=3,
        )
        cbar = fig.colorbar(sc_ko, ax=ax, fraction=0.04, pad=0.02)
        cbar.set_label("Ko scale")

    if not ku.empty:
        ax.scatter(
            ku["coord_rmse_rad_mean"],
            ku["mean_delta_tau_est_rms"],
            s=size(ku["mean_eta_u_rms"]),
            c=OKABE["orange"],
            marker="^",
            edgecolor="black",
            linewidth=0.6,
            label="P4D Ku scan",
            zorder=3,
        )

    offsets = {
        "P2D PD": (8, 8),
        "P2D EID": (18, -18),
        "O1": (8, 5),
        "O2": (12, -13),
        "O3": (20, -2),
        "O4": (14, -22),
        "O5": (-30, 14),
        "U1": (20, -25),
        "U2": (-32, -7),
        "U3": (-30, 26),
        "U4": (10, 8),
    }
    for _, row in valid.iterrows():
        label = row["plot_label"]
        xytext = offsets.get(label, (6, 5))
        ax.annotate(
            label,
            (row["coord_rmse_rad_mean"], row["mean_delta_tau_est_rms"]),
            textcoords="offset points",
            xytext=xytext,
            fontsize=7.5,
            arrowprops=(
                {"arrowstyle": "-", "color": "#9CA3AF", "lw": 0.45, "shrinkA": 0, "shrinkB": 4}
                if abs(xytext[0]) > 12 or abs(xytext[1]) > 10
                else None
            ),
        )

    ax.set_xlim(*pad_limits(valid["coord_rmse_rad_mean"], 0.12))
    ax.set_ylim(*pad_limits(valid["mean_delta_tau_est_rms"], 0.16))
    ax.set_xlabel("Coordination RMSE (rad), lower is better")
    ax.set_ylabel(r"Mean $\Delta\tau_{est}$ RMS (N m / sample), lower is better")
    handles, labels = ax.get_legend_handles_labels()
    handles.append(Line2D([0], [0], marker="x", color="none", markeredgecolor="#9CA3AF", markersize=7, lw=0))
    labels.append("Incomplete O0 omitted from scale")
    ax.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 1.12), ncol=4, frameon=False)

    if not invalid.empty:
        inset = ax.inset_axes([0.66, 0.09, 0.29, 0.29])
        inset.scatter(valid["coord_rmse_rad_mean"], valid["mean_delta_tau_est_rms"], s=14, c="#6B7280", alpha=0.55)
        inset.scatter(
            invalid["coord_rmse_rad_mean"],
            invalid["mean_delta_tau_est_rms"],
            s=36,
            c="#9CA3AF",
            marker="x",
            linewidth=1.4,
        )
        for _, row in invalid.iterrows():
            inset.annotate(str(row["plot_label"]), (row["coord_rmse_rad_mean"], row["mean_delta_tau_est_rms"]), xytext=(4, 3), textcoords="offset points", fontsize=7)
        inset.set_title("All points", fontsize=7.5, pad=2)
        inset.tick_params(labelsize=6.5, pad=1)
        inset.grid(True, alpha=0.14)

    fig.subplots_adjust(left=0.10, right=0.92, bottom=0.13, top=0.86)

    out = OUT_DIR / "bychen_tradeoff_pareto.png"
    save_figure(fig, out)

    export_cols = [
        "family",
        "method",
        "scan",
        "group",
        "ko_scale",
        "ku_scale",
        "n_runs",
        "n_complete_window",
        "coord_rmse_rad_mean",
        "mean_delta_tau_est_rms",
        "mean_tau_est_rms",
        "mean_eta_u_rms",
    ]
    df[export_cols].to_csv(TABLE_DIR / "bychen_tradeoff_points.csv", index=False)
    return out


def plot_p4d_scans(df: pd.DataFrame) -> Path:
    p4d = df[(df["family"] == "P4D") & (df["valid_window"])].copy()
    ko = p4d[p4d["scan"] == "ko"].sort_values("ko_scale")
    ku = p4d[p4d["scan"] == "ku"].sort_values("ku_scale")

    fig, axes = plt.subplots(2, 2, figsize=(8.4, 5.6), sharex=False)
    ax = axes[0, 0]
    ax.plot(ko["ko_scale"], ko["coord_rmse_rad_mean"], marker="o", color=OKABE["blue"], lw=1.7)
    ax.set_title(r"$K_o$ scan: coordination")
    ax.set_xlabel(r"$K_o$ scale")
    ax.set_ylabel("Coordination RMSE (rad)")

    ax = axes[0, 1]
    ax.plot(ko["ko_scale"], ko["mean_delta_tau_est_rms"], marker="o", color=OKABE["purple"], lw=1.7)
    ax.set_title(r"$K_o$ scan: torque variation")
    ax.set_xlabel(r"$K_o$ scale")
    ax.set_ylabel(r"Mean $\Delta\tau_{est}$ RMS")

    ax = axes[1, 0]
    ax.plot(ku["ku_scale"], ku["coord_rmse_rad_mean"], marker="o", color=OKABE["orange"], lw=1.7)
    ax.set_title(r"$K_u$ scan: coordination")
    ax.set_xlabel(r"$K_u$ scale")
    ax.set_ylabel("Coordination RMSE (rad)")

    ax = axes[1, 1]
    ax.plot(ku["ku_scale"], ku["mean_eta_u_rms"], marker="o", color=OKABE["vermillion"], lw=1.7)
    ax.set_title(r"$K_u$ scan: compensation")
    ax.set_xlabel(r"$K_u$ scale")
    ax.set_ylabel(r"Mean $\eta_u$ RMS (N m)")

    for idx, ax in enumerate(axes.ravel()):
        panel_label(ax, f"({chr(ord('a') + idx)})")
        ax.margins(x=0.08, y=0.16)
    fig.subplots_adjust(left=0.10, right=0.99, bottom=0.10, top=0.92, hspace=0.50, wspace=0.35)
    out = OUT_DIR / "bychen_p4d_gain_scans.png"
    save_figure(fig, out)
    return out


def spectrum(signal: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    y = np.asarray(signal, dtype=float)
    y = y[np.isfinite(y)]
    if y.size < 8:
        return np.array([]), np.array([])
    y = y - np.mean(y)
    window = np.hanning(y.size)
    freq = np.fft.rfftfreq(y.size, d=dt)
    amp = 2.0 * np.abs(np.fft.rfft(y * window)) / max(np.sum(window), 1.0)
    return freq, amp


def load_joint_window(path: Path, joint_id: int, t0: float, t1: float) -> tuple[np.ndarray, np.ndarray, float]:
    df = pd.read_csv(path)
    jdf = df[df["joint_id"] == joint_id].copy()
    jdf["t_rel"] = jdf["t"] - df["t"].min()
    jdf = jdf[(jdf["t_rel"] >= t0) & (jdf["t_rel"] < t1)].sort_values("t_rel")
    t = jdf["t_rel"].to_numpy()
    tau = jdf["tau_est"].to_numpy()
    dt = float(np.median(np.diff(t))) if t.size > 2 else 0.002
    return t, tau, dt


def plot_p2d_spectrum() -> Path:
    fig, axes = plt.subplots(2, 2, figsize=(8.6, 5.6), sharex=True)
    runs = [("PD", P2D_PD_LOG, OKABE["gray"]), ("EID", P2D_EID_LOG, OKABE["blue"])]
    joints = [(1, "Hip"), (2, "Knee")]
    peak_rows = []

    for row, (joint_id, joint_name) in enumerate(joints):
        for method, path, color in runs:
            t, tau, dt = load_joint_window(path, joint_id, 4.0, 5.4)
            f_tau, a_tau = spectrum(tau, dt)
            dtau = np.diff(tau)
            f_dtau, a_dtau = spectrum(dtau, dt)

            keep_tau = (f_tau >= 0.5) & (f_tau <= 100.0)
            keep_dtau = (f_dtau >= 0.5) & (f_dtau <= 100.0)
            axes[row, 0].semilogy(f_tau[keep_tau], a_tau[keep_tau] + 1e-12, color=color, label=method, lw=1.35)
            axes[row, 1].semilogy(f_dtau[keep_dtau], a_dtau[keep_dtau] + 1e-12, color=color, label=method, lw=1.35)

            if np.any(keep_tau):
                idx = np.argmax(a_tau[keep_tau])
                peak_rows.append(
                    {
                        "method": method,
                        "joint": joint_name,
                        "signal": "tau_est",
                        "peak_freq_hz": float(f_tau[keep_tau][idx]),
                        "peak_amp": float(a_tau[keep_tau][idx]),
                    }
                )
            if np.any(keep_dtau):
                idx = np.argmax(a_dtau[keep_dtau])
                peak_rows.append(
                    {
                        "method": method,
                        "joint": joint_name,
                        "signal": "delta_tau_est",
                        "peak_freq_hz": float(f_dtau[keep_dtau][idx]),
                        "peak_amp": float(a_dtau[keep_dtau][idx]),
                    }
                )

        axes[row, 0].set_title(rf"{joint_name}: $\tau_{{est}}$")
        axes[row, 1].set_title(rf"{joint_name}: $\Delta\tau_{{est}}$")
        axes[row, 0].set_ylabel("Amplitude")
        axes[row, 1].set_ylabel("Amplitude")
        axes[row, 0].set_xlim(0.5, 100.0)
        axes[row, 1].set_xlim(0.5, 100.0)

    for ax in axes[-1, :]:
        ax.set_xlabel("Frequency (Hz)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.995), ncol=2, frameon=False)
    for idx, ax in enumerate(axes.ravel()):
        panel_label(ax, f"({chr(ord('a') + idx)})")
    fig.subplots_adjust(left=0.10, right=0.99, bottom=0.10, top=0.88, hspace=0.45, wspace=0.30)

    out = OUT_DIR / "bychen_p2d_disturbance_spectrum.png"
    save_figure(fig, out)
    pd.DataFrame(peak_rows).to_csv(TABLE_DIR / "bychen_p2d_spectrum_peaks.csv", index=False)
    return out


def main() -> None:
    configure_matplotlib()
    ensure_dirs()
    df = load_disturbance_summary()
    paths = [
        plot_tradeoff(df),
        plot_p4d_scans(df),
        plot_p2d_spectrum(),
    ]
    print("Generated:")
    for path in paths:
        print(path.relative_to(ROOT))
    print((TABLE_DIR / "bychen_tradeoff_points.csv").relative_to(ROOT))
    print((TABLE_DIR / "bychen_p2d_spectrum_peaks.csv").relative_to(ROOT))


if __name__ == "__main__":
    main()
