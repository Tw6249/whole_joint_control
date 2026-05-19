#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""H1 Joint Control 鈥?Experiment Analysis Dashboard."""

from __future__ import annotations

import math
import re
import sqlite3
from pathlib import Path

import altair as alt
import numpy as np
import pandas as pd
import streamlit as st

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DB = REPO_ROOT / "data" / "experiments.db"
DEFAULT_DATA = REPO_ROOT / "data"

JOINT_NAMES: dict[int, str] = {
    0: "RightHipRoll", 1: "RightHipPitch", 2: "RightKnee",
    3: "LeftHipRoll", 4: "LeftHipPitch", 5: "LeftKnee",
    6: "WaistYaw", 7: "LeftHipYaw", 8: "RightHipYaw",
    10: "LeftAnkle", 11: "RightAnkle",
    12: "RightShoulderPitch", 13: "RightShoulderRoll",
    14: "RightShoulderYaw", 15: "RightElbow",
    16: "LeftShoulderPitch", 17: "LeftShoulderRoll",
    18: "LeftShoulderYaw", 19: "LeftElbow",
}

LOWER_BODY = {0, 1, 2, 3, 4, 5, 7, 8, 10, 11}
UPPER_BODY = {12, 13, 14, 15, 16, 17, 18, 19}

# Color palette
C = {
    "bg": "#f8f9fb",
    "card": "#ffffff",
    "eid": "#3b82f6",
    "pd": "#ef4444",
    "ref": "#6b7280",
    "good": "#10b981",
    "warn": "#f59e0b",
    "bad": "#ef4444",
    "text": "#1f2937",
    "muted": "#9ca3af",
    "border": "#e5e7eb",
    "accent": "#6366f1",
}

# Joint groups for coloring
JOINT_GROUPS = {
    **{j: "Hip Roll" for j in [0, 3]},
    **{j: "Hip Pitch" for j in [1, 4]},
    **{j: "Knee" for j in [2, 5]},
    **{j: "Hip Yaw" for j in [7, 8]},
    6: "Waist",
    **{j: "Ankle" for j in [10, 11]},
    **{j: "Shoulder Pitch" for j in [12, 16]},
    **{j: "Shoulder Roll" for j in [13, 17]},
    **{j: "Shoulder Yaw" for j in [14, 18]},
    **{j: "Elbow" for j in [15, 19]},
}

GROUP_COLORS = {
    "Knee": "#3b82f6", "Hip Pitch": "#6366f1", "Hip Roll": "#8b5cf6",
    "Hip Yaw": "#06b6d4", "Ankle": "#10b981",
    "Shoulder Pitch": "#f59e0b", "Shoulder Roll": "#ef4444",
    "Shoulder Yaw": "#ec4899", "Elbow": "#f97316", "Waist": "#6b7280",
}

SIGNAL_LABELS = {
    "eid_q": "EID position", "eid_dq": "EID velocity",
    "eid_q_ref": "EID reference", "eid_q_error": "EID error",
    "eid_tau_cmd": "EID torque",
    "pd_q": "PD position", "pd_dq": "PD velocity",
    "pd_q_ref": "PD reference", "pd_q_error": "PD error",
    "pd_tau_cmd": "PD torque",
    "disturb_tau": "Disturbance",
}

SIGNAL_COLORS = {
    "eid_q": "#3b82f6", "eid_dq": "#60a5fa", "eid_q_ref": "#93c5fd",
    "eid_q_error": "#1d4ed8", "eid_tau_cmd": "#2563eb",
    "pd_q": "#ef4444", "pd_dq": "#f87171", "pd_q_ref": "#fca5a5",
    "pd_q_error": "#dc2626", "pd_tau_cmd": "#b91c1c",
    "disturb_tau": "#6b7280",
}

SIGNAL_CATEGORIES = {
    "Position": ["q_actual", "q_ref_shaped", "q_ref_raw", "q_ref_shaped_next",
                  "q_error_shaped", "q_error_raw", "motor_q"],
    "Velocity": ["dq_actual", "dq_ref_shaped", "dq_ref_raw", "dq_ref_shaped_next",
                  "dq_error_shaped", "dq_error_raw", "motor_dq"],
    "Torque": ["u_t", "u_raw", "u_star", "u_feedback", "tau_cmd",
               "observer_tau_applied", "motor_tau"],
    "Observer": ["eta_q", "eta_dq", "x_hat_q", "x_hat_dq", "x_bar_q", "x_bar_dq",
                  "observer_qacc"],
    "EID Reference": ["r_d_q", "r_d_dq", "e_q", "e_dq"],
    "Inverse Model": ["rho_q", "rho_dq"],
    "Command": ["kp_cmd", "kd_cmd", "motor_kp", "motor_kd"],
}


# ---------------------------------------------------------------------------
# DB helpers
# ---------------------------------------------------------------------------

@st.cache_data(ttl=30, show_spinner=False)
def _query(sql: str, params: tuple = (), db_path: str = "") -> pd.DataFrame:
    conn = sqlite3.connect(db_path or str(DEFAULT_DB))
    try:
        return pd.read_sql_query(sql, conn, params=params)
    finally:
        conn.close()


def load_exps(db: str) -> pd.DataFrame:
    return _query("""
        SELECT experiment_id, run_id, timestamp, object_type,
               controller_method, duration_s, run_dir
        FROM experiments ORDER BY timestamp DESC
    """, db_path=db)


def load_comps(db: str) -> pd.DataFrame:
    return _query("""
        SELECT e.experiment_id, e.run_id, e.timestamp, e.controller_method,
               jc.joint_id, jc.joint_name, jc.kp, jc.kd, jc.eid_tau_limit,
               jc.policy_interpolation, jc.policy_source,
               jc.observer_gain_q, jc.observer_gain_dq, jc.filter_alpha,
               cr.eid_rmse, cr.pd_rmse, cr.rmse_ratio,
               cr.eid_max_error, cr.pd_max_error,
               cr.eid_mean_abs_tau, cr.pd_mean_abs_tau,
               cr.disturb_type, cr.pd_kp_used, cr.pd_kd_used
        FROM comparison_results cr
        JOIN joint_configs jc ON cr.experiment_id = jc.experiment_id AND cr.joint_id = jc.joint_id
        JOIN experiments e ON cr.experiment_id = e.experiment_id
        ORDER BY e.timestamp DESC, jc.joint_id
    """, db_path=db)


def load_summaries(db: str) -> pd.DataFrame:
    """Load per-controller joint_summaries joined with experiments and joint_configs."""
    return _query("""
        SELECT e.experiment_id, e.run_id, e.timestamp, e.controller_method,
               e.disturb_type,
               js.joint_id, jc.joint_name,
               js.q_rmse, js.q_max_error, js.tau_abs_max, js.tau_mean_abs,
               jc.kp, jc.kd, jc.eid_tau_limit,
               jc.observer_gain_q, jc.observer_gain_dq, jc.filter_alpha
        FROM experiments e
        JOIN joint_summaries js ON e.experiment_id = js.experiment_id
        JOIN joint_configs jc ON e.experiment_id = jc.experiment_id
            AND js.joint_id = jc.joint_id
        WHERE e.controller_method IN ('EID', 'PD')
        ORDER BY e.timestamp DESC, js.joint_id
    """, db_path=db)


def load_pairs(db: str) -> pd.DataFrame:
    """Load comparison_pairs with EID and PD experiment metadata."""
    return _query("""
        SELECT cp.pair_id,
               eid.experiment_id AS eid_exp_id, eid.run_id AS eid_run_id,
               eid.timestamp AS eid_timestamp,
               pd.experiment_id AS pd_exp_id, pd.run_id AS pd_run_id,
               pd.timestamp AS pd_timestamp,
               COALESCE(eid.disturb_type, cp.disturb_type, 'none') AS disturb_type
        FROM comparison_pairs cp
        JOIN experiments eid ON cp.eid_experiment_id = eid.experiment_id
        JOIN experiments pd ON cp.pd_experiment_id = pd.experiment_id
        ORDER BY eid.timestamp DESC
    """, db_path=db)


def load_paired_comparison(db: str) -> pd.DataFrame:
    """Load comparison metrics by joining paired experiments' joint_summaries."""
    return _query("""
        SELECT cp.pair_id,
               cp.eid_experiment_id, cp.pd_experiment_id,
               eid.run_id AS eid_run_id, pd.run_id AS pd_run_id,
               eid.disturb_type AS disturb_type,
               es.joint_id, ejc.joint_name,
               es.q_rmse AS eid_rmse, es.q_max_error AS eid_max_error,
               es.tau_mean_abs AS eid_mean_abs_tau,
               ps.q_rmse AS pd_rmse, ps.q_max_error AS pd_max_error,
               ps.tau_mean_abs AS pd_mean_abs_tau,
               ps.q_rmse / NULLIF(es.q_rmse, 0) AS rmse_ratio,
               ejc.kp, ejc.kd, ejc.eid_tau_limit,
               ejc.observer_gain_q, ejc.observer_gain_dq, ejc.filter_alpha
        FROM comparison_pairs cp
        JOIN experiments eid ON cp.eid_experiment_id = eid.experiment_id
        JOIN experiments pd ON cp.pd_experiment_id = pd.experiment_id
        JOIN joint_summaries es ON cp.eid_experiment_id = es.experiment_id
        JOIN joint_summaries ps ON cp.pd_experiment_id = ps.experiment_id
            AND es.joint_id = ps.joint_id
        JOIN joint_configs ejc ON cp.eid_experiment_id = ejc.experiment_id
            AND es.joint_id = ejc.joint_id
        WHERE es.q_rmse IS NOT NULL AND ps.q_rmse IS NOT NULL
        ORDER BY eid.timestamp DESC, es.joint_id
    """, db_path=db)


def load_ts_runs(repo_root: Path, db: str) -> pd.DataFrame:
    """Return experiments that have any timeseries CSV available."""
    df = load_exps(db)
    if df.empty:
        return df
    rows = []
    ts_files = [
        "mujoco_closed_loop_log.csv",
        "eid_mujoco_closed_loop_log.csv",
        "pd_mujoco_closed_loop_log.csv",
        "eid_vs_pd_timeseries.csv",
    ]
    for _, r in df.iterrows():
        run_dir = Path(str(r["run_dir"]))
        if not run_dir.is_absolute():
            run_dir = repo_root / run_dir
        for fname in ts_files:
            if (run_dir / fname).exists():
                rows.append(r.to_dict())
                break
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# UI components
# ---------------------------------------------------------------------------

def inject_css() -> None:
    st.markdown("""<style>
    @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap');

    * { font-family: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif; }

    .main .block-container { padding: 2rem 3rem; max-width: 1400px; }

    /* KPI cards */
    .kpi-grid { display: flex; gap: 1rem; flex-wrap: wrap; margin-bottom: 1.5rem; }
    .kpi-card {
        flex: 1; min-width: 140px;
        background: #fff; border: 1px solid #e5e7eb; border-radius: 12px;
        padding: 16px 20px; box-shadow: 0 1px 3px rgba(0,0,0,.04);
    }
    .kpi-card .label { font-size: 0.75rem; color: #9ca3af; text-transform: uppercase; letter-spacing: .05em; }
    .kpi-card .value { font-size: 1.6rem; font-weight: 700; color: #1f2937; margin-top: 2px; }

    /* Section cards */
    .section-card {
        background: #fff; border: 1px solid #e5e7eb; border-radius: 12px;
        padding: 20px 24px; margin-bottom: 1.25rem;
        box-shadow: 0 1px 3px rgba(0,0,0,.04);
    }

    /* Hide Streamlit chrome */
    #MainMenu, footer, header[data-testid="stHeader"] { display: none; }
    section[data-testid="stSidebar"] { background: #f8f9fb; }
    section[data-testid="stSidebar"] .block-container { padding: 1.5rem 1rem; }

    /* Tab styling */
    .stTabs [data-baseweb="tab-list"] { gap: 4px; background: transparent; }
    .stTabs [data-baseweb="tab"] {
        border-radius: 8px 8px 0 0; padding: 10px 20px;
        font-weight: 500; font-size: 0.9rem; border: none;
        color: #6b7280; background: transparent;
    }
    .stTabs [aria-selected="true"] {
        color: #3b82f6; background: #fff;
        border-bottom: 2px solid #3b82f6; border-radius: 8px 8px 0 0;
    }

    /* Buttons */
    .stButton > button {
        border-radius: 8px; font-weight: 500; border: 1px solid #d1d5db;
        background: #fff; color: #374151; padding: 6px 14px;
    }
    .stButton > button:hover { border-color: #3b82f6; color: #3b82f6; }

    /* Multiselect tag styling */
    span[data-baseweb="tag"] { border-radius: 6px !important; }

    /* Metric adjustments */
    div[data-testid="stMetric"] {
        background: #fff; border: 1px solid #e5e7eb; border-radius: 10px;
        padding: 12px 16px;
    }
    </style>""", unsafe_allow_html=True)


def kpi_row(metrics: list[tuple[str, str | int | float, str | None]]) -> None:
    """Render a row of KPI cards. Each tuple: (label, value, delta)."""
    html = '<div class="kpi-grid">'
    for label, value, delta in metrics:
        d = f'<span style="color:#10b981;font-size:.8rem;">{delta}</span>' if delta else ""
        html += f'<div class="kpi-card"><div class="label">{label}</div><div class="value">{value}</div>{d}</div>'
    html += '</div>'
    st.markdown(html, unsafe_allow_html=True)


def section(title: str):
    """Return a context manager for a card section."""
    return st.container()


def ratio_badge(val: float | None) -> str:
    if val is None or not math.isfinite(val):
        return ""
    if val < 0.5:
        return "background-color:#d1fae5;color:#065f46;padding:2px 8px;border-radius:6px;font-weight:600"
    if val < 1.0:
        return "background-color:#e5e7eb;color:#374151;padding:2px 8px;border-radius:6px"
    return "background-color:#fee2e2;color:#991b1b;padding:2px 8px;border-radius:6px;font-weight:600"


def compact_altair_chart(chart: alt.Chart) -> None:
    """Render altair chart with clean config."""
    st.altair_chart(
        chart.configure_axis(
            gridColor="#f3f4f6", labelFontSize=11, titleFontSize=12, titleFontWeight=500,
        ).configure_legend(
            titleFontSize=12, labelFontSize=11, padding=8, cornerRadius=6,
        ).configure_view(
            strokeWidth=0,
        ),
        use_container_width=True,
    )


# ---------------------------------------------------------------------------
# Pages
# ---------------------------------------------------------------------------

def page_overview(db: str) -> None:
    exps = load_exps(db)
    summaries = load_summaries(db)
    pairs = load_pairs(db)
    legacy = load_comps(db)

    if exps.empty:
        st.info("No data yet. Run: `python scripts/db_manager.py import-all`")
        return

    # Filter out legacy experiments for display
    display_exps = exps[~exps["controller_method"].str.contains("legacy", na=False)].copy()

    # KPI row
    n_eid = int((display_exps["controller_method"] == "EID").sum())
    n_pd = int((display_exps["controller_method"] == "PD").sum())
    n_pairs = len(pairs) if not pairs.empty else 0
    n_summaries = len(summaries) if not summaries.empty else 0
    best_eid_rmse = summaries[summaries["controller_method"] == "EID"]["q_rmse"].min() if not summaries.empty else 0

    kpi_row([
        ("EID Experiments", n_eid, None),
        ("PD Experiments", n_pd, None),
        ("Comparison Pairs", n_pairs, None),
        ("Best EID RMSE", f"{best_eid_rmse:.5f}" if best_eid_rmse else "-", None),
    ])

    # Experiment table
    st.markdown("### Experiments")
    cols = st.columns(3)
    with cols[0]:
        methods = display_exps["controller_method"].dropna().unique().tolist()
        method_f = st.multiselect("Method", sorted(methods), key="ov_method")
    with cols[1]:
        if not summaries.empty:
            joint_f = st.multiselect("Joint", sorted(summaries["joint_name"].dropna().unique()), key="ov_joint")
        else:
            joint_f = []
    with cols[2]:
        disturbs = display_exps["disturb_type"].dropna().unique().tolist() if "disturb_type" in display_exps.columns else []
        disturb_f = st.multiselect("Disturbance", sorted(disturbs) if disturbs else ["none"], key="ov_disturb")

    filtered = display_exps.copy()
    if method_f:
        filtered = filtered[filtered["controller_method"].isin(method_f)]
    if disturb_f and "disturb_type" in filtered.columns:
        filtered = filtered[filtered["disturb_type"].isin(disturb_f)]

    display = filtered[["run_id", "timestamp", "controller_method", "duration_s"]].copy()
    display.columns = ["Run", "Time", "Method", "Duration (s)"]
    st.dataframe(display, use_container_width=True, hide_index=True, height=350)

    # Show pairs table
    if not pairs.empty:
        st.markdown("### Comparison Pairs")
        pair_display = pairs[["eid_run_id", "pd_run_id", "disturb_type"]].copy()
        pair_display.columns = ["EID Run", "PD Run", "Disturbance"]
        st.dataframe(pair_display, use_container_width=True, hide_index=True, height=250)


def page_joint_analysis(db: str) -> None:
    st.markdown("### Joint Performance")

    df = load_paired_comparison(db)
    if df.empty:
        # Fallback: try legacy comparison_results
        df = load_comps(db)
        if df.empty:
            st.info("No comparison data. Create comparison pairs or import experiments with summaries.")
            return
        st.caption("Using legacy comparison_results data.")

    joints = sorted(df["joint_name"].unique())
    disturbs = sorted(df["disturb_type"].dropna().unique())

    col1, col2, col3 = st.columns(3)
    with col1:
        sel_joints = st.multiselect("Joints", joints, default=["RightKnee", "RightHipPitch", "RightElbow", "WaistYaw"],
                                     key="ja_joints")
    with col2:
        sel_disturb = st.selectbox("Disturbance", ["all"] + disturbs, key="ja_disturb")
    with col3:
        sort_by = st.selectbox("Sort by", ["RMSE Ratio", "EID RMSE", "PD RMSE"], key="ja_sort")

    filt = df[df["joint_name"].isin(sel_joints)] if sel_joints else df
    if sel_disturb != "all":
        filt = filt[filt["disturb_type"] == sel_disturb]

    if filt.empty:
        st.info("No data matching filters.")
        return

    # If using legacy data, columns are already eid_rmse/pd_rmse
    eid_col = "eid_rmse"
    pd_col = "pd_rmse"
    ratio_col = "rmse_ratio"

    agg = filt.groupby("joint_name").agg(
        eid=(eid_col, "mean"), pd=(pd_col, "mean"),
        ratio=(ratio_col, "mean"), n=("pair_id" if "pair_id" in filt.columns else "experiment_id", "count"),
    ).reset_index()

    if sort_by == "EID RMSE":
        agg = agg.sort_values("eid")
    elif sort_by == "PD RMSE":
        agg = agg.sort_values("pd")
    else:
        agg = agg.sort_values("ratio")

    chart_df = agg.melt(id_vars="joint_name", value_vars=["eid", "pd"],
                         var_name="Controller", value_name="RMSE (rad)")
    chart_df["Controller"] = chart_df["Controller"].replace({"eid": "EID", "pd": "PD"})

    bar = alt.Chart(chart_df).mark_bar(
        cornerRadiusTopLeft=4, cornerRadiusTopRight=4, width=20,
    ).encode(
        x=alt.X("joint_name:N", title=None, sort=agg["joint_name"].tolist(), axis=alt.Axis(labelAngle=-30)),
        y=alt.Y("RMSE (rad):Q", title=None),
        color=alt.Color("Controller:N", scale=alt.Scale(domain=["EID", "PD"], range=[C["eid"], C["pd"]])),
        xOffset="Controller:N",
        tooltip=["joint_name", "Controller", alt.Tooltip("RMSE (rad):Q", format=".6f")],
    ).properties(height=320)
    compact_altair_chart(bar)

    st.markdown("#### RMSE Ratio  (PD / EID)")
    st.caption("< 0.5 EID dominates | 0.5-1.0 EID better | > 1.0 PD better")

    def _style(sdf):
        s = sdf.style.format({
            "EID RMSE": "{:.5f}", "PD RMSE": "{:.5f}", "Ratio": "{:.4f}",
        }, na_rep="-")
        if "Ratio" in sdf.columns:
            def _c(v):
                if not isinstance(v, (int, float)) or not math.isfinite(v):
                    return ""
                if v < 0.5:
                    return "background-color:#d1fae5;color:#065f46;font-weight:600"
                if v < 1.0:
                    return "background-color:#f3f4f6;color:#374151"
                return "background-color:#fee2e2;color:#991b1b;font-weight:600"
            s = s.map(_c, subset=["Ratio"])
        return s

    table = agg[["joint_name", "eid", "pd", "ratio", "n"]].copy()
    table.columns = ["Joint", "EID RMSE", "PD RMSE", "Ratio", "Runs"]
    st.dataframe(_style(table), use_container_width=True, hide_index=True)


def page_timeseries(db: str) -> None:
    st.markdown("### Timeseries Viewer")

    repo_root = Path(db).parent.parent if Path(db).exists() else REPO_ROOT
    runs_df = load_ts_runs(repo_root, db)

    if runs_df.empty:
        st.info("No timeseries CSVs yet.")
        return

    # --- Multi-experiment selection ---
    all_runs = runs_df["run_id"].tolist()
    run_labels = {}
    for _, r in runs_df.iterrows():
        run_labels[r["run_id"]] = (
            f"{r['run_id']}  [{r.get('controller_method', '?')}]  {r.get('timestamp', '')}"
        )

    selected_runs: list[str] = st.multiselect(
        "Experiments (select 1 or more to compare)",
        all_runs,
        default=all_runs,  # select all by default
        format_func=lambda rid: run_labels.get(rid, rid),
        key="ts_runs",
    )

    if not selected_runs:
        st.info("Select at least one experiment to view timeseries.")
        return

    # --- Load all selected experiments' timeseries ---
    @st.cache_data(show_spinner="Loading...")
    def _find_ts_file(run_dir_str: str, repo_root_str: str) -> Path | None:
        root = Path(repo_root_str)
        d = Path(run_dir_str)
        if not d.is_absolute():
            d = root / d
        for fname in ["mujoco_closed_loop_log.csv",
                       "eid_mujoco_closed_loop_log.csv",
                       "pd_mujoco_closed_loop_log.csv",
                       "eid_vs_pd_timeseries.csv"]:
            p = d / fname
            if p.exists():
                return p
        return None

    @st.cache_data(show_spinner="Loading...")
    def _load_ts(path_str: str) -> pd.DataFrame:
        df = pd.read_csv(path_str)
        if "t" in df.columns:
            df["t"] = pd.to_numeric(df["t"], errors="coerce")
        return df

    # Load all selected dataframes, merge with exp_prefix
    COMMON_COLS = ["t", "cycle", "joint_id"]
    all_dfs: list[pd.DataFrame] = []
    exp_colors: dict[str, str] = {}
    color_palette = ["#3b82f6", "#ef4444", "#10b981", "#f59e0b",
                     "#8b5cf6", "#ec4899", "#06b6d4", "#f97316"]

    for i, run_id in enumerate(selected_runs):
        row = runs_df.set_index("run_id").loc[run_id]
        run_dir = Path(str(row["run_dir"]))
        if not run_dir.is_absolute():
            run_dir = repo_root / run_dir
        ts_path = _find_ts_file(str(run_dir), str(repo_root))
        if ts_path is None:
            st.warning(f"No timeseries CSV for {run_id}")
            continue

        df = _load_ts(str(ts_path))
        prefix = f"exp{i}_"
        data_cols = [c for c in df.columns if c not in COMMON_COLS]
        rename_map = {c: f"{prefix}{c}" for c in data_cols}
        df_renamed = df.rename(columns=rename_map)
        all_dfs.append(df_renamed)
        exp_colors[prefix] = color_palette[i % len(color_palette)]

    if not all_dfs:
        st.error("Could not load any timeseries data.")
        return

    # Merge all experiments on common cols
    merged = all_dfs[0]
    for df in all_dfs[1:]:
        on_cols = [c for c in COMMON_COLS if c in merged.columns and c in df.columns]
        merged = pd.merge(merged, df, on=on_cols, how="outer")

    # --- Signal helpers ---
    def base_signal_name(sig: str) -> str:
        """Strip experiment prefix to get base signal name."""
        parts = sig.split("_", 1)
        return parts[1] if len(parts) > 1 and parts[0].startswith("exp") else sig

    def signal_category(sig: str) -> str:
        base = base_signal_name(sig)
        for cat, members in SIGNAL_CATEGORIES.items():
            if base in members:
                return cat
        return "Other"

    # All signal columns (with exp prefix), excluding metadata and unused debug slots
    _META_SIGNALS = {"flags", "joint_flags"}
    all_signals = [c for c in merged.columns
                   if c not in COMMON_COLS and base_signal_name(c) not in _META_SIGNALS]

    joints = sorted(merged["joint_id"].unique())
    jlabels = {j: f"J{j} {JOINT_NAMES.get(j, '')}" for j in joints}

    # --- Panel management ---
    if "ts_panels" not in st.session_state:
        st.session_state["ts_panels"] = 2  # default 2 panels

    panel_count = st.session_state["ts_panels"]

    col_add, col_rem, _ = st.columns([1, 1, 8])
    with col_add:
        if st.button("+ Panel", key="ts_add_panel"):
            st.session_state["ts_panels"] += 1
            st.rerun()
    with col_rem:
        if st.button("- Panel", key="ts_rem_panel") and panel_count > 1:
            st.session_state["ts_panels"] -= 1
            st.rerun()

    st.markdown(f"**{len(selected_runs)} experiment(s), {len(all_signals)} signals, {panel_count} panel(s)**")

    # Joint selection (shared across panels)
    sel_joints = st.multiselect(
        "Joints", joints,
        default=[j for j in [2, 1, 15] if j in joints],
        format_func=lambda j: jlabels[j], key="ts_joints",
    )

    if not sel_joints:
        st.info("Select joints to plot.")
        return

    # Build signal selection per panel
    # Group by category, then by base name -> list of prefixed signals
    sig_by_cat: dict[str, dict[str, list[str]]] = {}
    for sig in all_signals:
        base = base_signal_name(sig)
        # Skip internal debug_N slots (unused PD slots)
        if re.match(r"^debug_\d+$", base):
            continue
        cat = signal_category(sig)
        sig_by_cat.setdefault(cat, {}).setdefault(base, []).append(sig)

    MAX_POINTS = 8000

    # Default signal selections per panel (base signal names)
    PANEL_DEFAULTS: list[dict[str, set[str]]] = [
        # Panel 0: Position + Torque, key signals
        {
            "Position": {"q_actual", "q_ref_shaped", "q_error_shaped"},
            "Torque": {"u_t", "u_star"},
        },
        # Panel 1: Observer + EID Reference + Inverse Model, key signals
        {
            "Observer": {"eta_q", "x_bar_q", "x_hat_q"},
            "EID Reference": {"e_q", "r_d_q"},
            "Inverse Model": {"rho_q"},
        },
    ]

    for panel_idx in range(panel_count):
        with st.expander(f"Panel {panel_idx + 1}", expanded=(panel_idx == 0)):
            left, right = st.columns([1, 4])

            with left:
                st.caption("**Signal categories**")
                panel_selected: list[str] = []

                # Get defaults for this panel
                panel_default = PANEL_DEFAULTS[panel_idx] if panel_idx < len(PANEL_DEFAULTS) else {}
                default_cats = set(panel_default.keys())

                # "All on/off" quick toggle (off by default)
                toggle_all = st.checkbox("Toggle all categories", value=False,
                                         key=f"ts_toggle_all_{panel_idx}")

                for cat in sorted(sig_by_cat.keys()):
                    with st.container():
                        cat_key = f"ts_cat_{panel_idx}_{cat}"
                        cat_selected = st.checkbox(
                            f"**{cat}**  ({len(sig_by_cat[cat])} signals)",
                            value=(cat in default_cats) if not toggle_all else toggle_all,
                            key=cat_key,
                        )
                        if cat_selected:
                            cat_defaults = panel_default.get(cat, set())
                            for base, sigs in sorted(sig_by_cat[cat].items()):
                                # Default-checked if in panel_default or toggle_all is on
                                checked = toggle_all or (base in cat_defaults)
                                if len(sigs) == 1:
                                    sig = sigs[0]
                                    exp_tag = sig.split("_", 1)[0]
                                    label = f"{base.replace('_', ' ')}  [{exp_tag}]"
                                    if st.checkbox(label, value=checked,
                                                   key=f"ts_{panel_idx}_{sig}"):
                                        panel_selected.append(sig)
                                else:
                                    if st.checkbox(base.replace("_", " "), value=checked,
                                                   key=f"ts_{panel_idx}_{cat}_{base}"):
                                        panel_selected.extend(sigs)

            with right:
                if not panel_selected:
                    st.caption("Select signals on the left.")
                    continue

                ts_filtered = merged[merged["joint_id"].isin(sel_joints)].copy()
                total_rows = len(ts_filtered)
                if total_rows > MAX_POINTS:
                    ts_filtered = ts_filtered.iloc[::max(1, total_rows // MAX_POINTS)]

                # Build long-form dataframe for Altair
                records = []
                for _, r in ts_filtered.iterrows():
                    for sig in panel_selected:
                        if sig in ts_filtered.columns:
                            v = r[sig]
                            if pd.notna(v):
                                # Extract experiment label from signal prefix
                                exp_prefix = sig.split("_", 1)[0]
                                exp_idx = int(exp_prefix[3:]) if exp_prefix.startswith("exp") else 0
                                exp_label = selected_runs[exp_idx] if exp_idx < len(selected_runs) else "?"
                                records.append({
                                    "t": r["t"],
                                    "Joint": jlabels[int(r["joint_id"])],
                                    "Signal": sig,
                                    "Experiment": exp_label,
                                    "value": float(v),
                                })
                lf = pd.DataFrame(records)

                if lf.empty:
                    st.caption("No data points.")
                    continue

                # Color: signal determines color, experiment determines shade
                # Group signals by base name for consistent coloring
                base_names = sorted(set(base_signal_name(s) for s in panel_selected))
                base_colors = {}
                for j, bn in enumerate(base_names):
                    base_colors[bn] = color_palette[j % len(color_palette)]

                def _sig_color_2(sig: str) -> str:
                    return base_colors.get(base_signal_name(sig), C["muted"])

                domain = sorted(panel_selected)
                range_ = [_sig_color_2(s) for s in domain]

                n_joints = len(sel_joints)
                charts = []
                for jid in sel_joints:
                    jname = jlabels[jid]
                    jlf = lf[lf["Joint"] == jname]
                    if jlf.empty:
                        continue
                    # Build compound label: Experiment + Signal
                    jlf = jlf.copy()
                    jlf["Label"] = jlf["Experiment"] + " | " + jlf["Signal"].map(
                        lambda s: base_signal_name(s).replace("_", " ")
                    )

                    ch = alt.Chart(jlf).mark_line(point=False, strokeWidth=1.5).encode(
                        x=alt.X("t:Q", title="time (s)"),
                        y=alt.Y("value:Q", title=None),
                        color=alt.Color("Signal:N",
                                        scale=alt.Scale(domain=domain, range=range_),
                                        legend=alt.Legend(title=None, orient="top", labelLimit=300)),
                        strokeDash=alt.StrokeDash("Experiment:N",
                                                   legend=alt.Legend(title=None, orient="top")),
                        tooltip=["t:Q", "Experiment:N", "Signal:N",
                                 alt.Tooltip("value:Q", format=".5f")],
                    ).properties(
                        height=max(120, min(280, 600 // n_joints)),
                        title=jname,
                    )
                    charts.append(ch)

                if charts:
                    combined = alt.vconcat(*charts).resolve_scale(color="shared")
                    compact_altair_chart(combined)

    # --- Per-joint stats ---
    with st.expander("Per-joint statistics", expanded=False):
        rows = []
        for jid in sel_joints:
            jdf = merged[merged["joint_id"] == jid]
            r = {"Joint": JOINT_NAMES.get(jid, str(jid))}
            for sig in all_signals:
                s = jdf[sig].dropna() if sig in jdf.columns else pd.Series(dtype=float)
                if len(s) > 0:
                    r[f"{sig}_rms"] = f"{math.sqrt((s**2).mean()):.4f}"
            rows.append(r)
        if rows:
            st.dataframe(pd.DataFrame(rows), use_container_width=True, hide_index=True)


def page_config(db: str) -> None:
    st.markdown("### Config Inspector")

    exps = _query("SELECT experiment_id, run_id, timestamp, controller_method FROM experiments ORDER BY timestamp DESC", db_path=db)
    if exps.empty:
        st.info("No data.")
        return

    run_id = st.selectbox("Experiment", exps["run_id"].tolist(), key="cfg_run",
                           format_func=lambda r: f"{r}  ({exps.set_index('run_id').loc[r, 'timestamp']})")
    exp_id = int(exps.set_index("run_id").loc[run_id, "experiment_id"])
    method = str(exps.set_index("run_id").loc[run_id, "controller_method"])

    configs = _query("""
        SELECT joint_id, joint_name, enabled, kp, kd, observer_gain_q, observer_gain_dq,
               filter_alpha, policy_interpolation, policy_source,
               policy_center, policy_amplitude, policy_frequency_hz, eid_tau_limit,
               plant_Jeff, plant_b, plant_tau_max
        FROM joint_configs WHERE experiment_id = ? ORDER BY joint_id
    """, (exp_id,), db_path=db)

    summaries = _query("""
        SELECT jc.joint_name, js.q_rmse, js.q_max_error, js.tau_abs_max, js.tau_mean_abs
        FROM joint_summaries js
        JOIN joint_configs jc ON js.experiment_id = jc.experiment_id AND js.joint_id = jc.joint_id
        WHERE js.experiment_id = ? ORDER BY jc.joint_id
    """, (exp_id,), db_path=db)

    # Check for paired experiment
    pair_info = _query("""
        SELECT eid_experiment_id, pd_experiment_id,
               (SELECT run_id FROM experiments WHERE experiment_id = cp.eid_experiment_id) AS eid_run,
               (SELECT run_id FROM experiments WHERE experiment_id = cp.pd_experiment_id) AS pd_run
        FROM comparison_pairs cp
        WHERE cp.eid_experiment_id = ? OR cp.pd_experiment_id = ?
    """, (exp_id, exp_id), db_path=db)

    col1, col2 = st.columns(2)
    with col1:
        st.caption("#### Controller Parameters")
        st.dataframe(configs, use_container_width=True, hide_index=True,
                     column_config={"kp": "%.1f", "kd": "%.1f", "eid_tau_limit": "%.1f"})
    with col2:
        st.caption(f"#### Results ({method})")
        if not summaries.empty:
            st.dataframe(summaries, use_container_width=True, hide_index=True,
                         column_config={"q_rmse": "%.6f", "q_max_error": "%.6f",
                                        "tau_abs_max": "%.4f", "tau_mean_abs": "%.4f"})

    if not pair_info.empty:
        st.caption("#### Paired Experiment")
        for _, p in pair_info.iterrows():
            if p["eid_experiment_id"] == exp_id:
                st.info(f"PD counterpart: **{p['pd_run']}**")
            else:
                st.info(f"EID counterpart: **{p['eid_run']}**")


def page_query(db: str) -> None:
    st.markdown("### SQL Console")
    st.caption("Tables: `experiments` `joint_configs` `joint_summaries` `comparison_results` `ablation_configs`")

    sql = st.text_area("Query", height=140, key="sql_query", value="""SELECT
    jc.joint_name,
    cr.disturb_type,
    AVG(cr.eid_rmse) AS eid_rmse,
    AVG(cr.pd_rmse)  AS pd_rmse,
    AVG(cr.rmse_ratio) AS ratio,
    COUNT(*) AS n
FROM comparison_results cr
JOIN joint_configs jc USING (experiment_id, joint_id)
WHERE cr.eid_rmse IS NOT NULL
GROUP BY jc.joint_name, cr.disturb_type
ORDER BY AVG(cr.rmse_ratio)""")

    if st.button("Run", type="primary"):
        try:
            result = _query(sql, db_path=db)
            st.dataframe(result, use_container_width=True, hide_index=True)
            st.caption(f"{len(result)} rows")
            st.download_button("Download CSV", result.to_csv(index=False).encode(), "result.csv")
        except Exception as e:
            st.error(str(e))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    st.set_page_config(page_title="H1 Control Analysis", page_icon="馃", layout="wide")
    inject_css()

    # Sidebar
    with st.sidebar:
        st.markdown("## 馃 H1 Control DB")

        db_path = st.text_input("Database", str(DEFAULT_DB), key="db_path")

        if not Path(db_path).exists():
            st.error(f"Not found: {db_path}")
            st.info("Run: `python scripts/db_manager.py import-all`")
            return

        # Quick stats
        stats = _query("""
            SELECT (SELECT COUNT(*) FROM experiments) AS n_exps,
                   (SELECT COUNT(*) FROM comparison_results) AS n_comps
        """, db_path=db_path)
        if not stats.empty:
            s = stats.iloc[0]
            st.metric("Experiments", int(s["n_exps"]))
            st.metric("Comparisons", int(s["n_comps"]))

        st.divider()

        # Quick reimport
        if st.button("馃攧 Re-import all runs", use_container_width=True):
            import subprocess, sys
            result = subprocess.run(
                [sys.executable, str(REPO_ROOT / "scripts" / "db_manager.py"), "rebuild",
                 "--db", db_path],
                capture_output=True, text=True, timeout=60,
            )
            if result.returncode == 0:
                st.success("Done 鈥?refresh the page")
                _query.clear()
                load_exps.clear()
                load_comps.clear()
            else:
                st.error(result.stderr[-300:] if result.stderr else "Unknown error")

    # Main tabs
    tab1, tab2, tab3, tab4 = st.tabs([
        "Overview", "Joint Analysis", "Timeseries", "SQL",
    ])

    with tab1:
        page_overview(db_path)
    with tab2:
        page_joint_analysis(db_path)
    with tab3:
        page_timeseries(db_path)
    with tab4:
        c1, c2 = st.tabs(["SQL Console", "Config Inspector"])
        with c1:
            page_query(db_path)
        with c2:
            page_config(db_path)


if __name__ == "__main__":
    main()
