#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""H1 Joint Control 閳?Experiment Analysis Dashboard."""

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
               "tau_command", "observer_tau_applied", "motor_tau"],
    "Observer": ["eta_q", "eta_dq", "x_hat_q", "x_hat_dq", "x_bar_q", "x_bar_dq",
                  "observer_qacc"],
    "EID Reference": ["r_d_q", "r_d_dq", "e_q", "e_dq"],
    "Inverse Model": ["rho_q", "rho_dq"],
    "Command": ["kp_cmd", "kd_cmd", "motor_kp", "motor_kd"],
    "Safety": ["saturation"],
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
               controller_method, duration_s, run_dir, disturb_type
        FROM experiments ORDER BY timestamp DESC
    """, db_path=db)


def table_exists(db: str, table: str) -> bool:
    df = _query(
        "SELECT name FROM sqlite_master WHERE type='table' AND name=?",
        (table,), db_path=db,
    )
    return not df.empty


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
               ps.q_rmse / NULLIF(es.q_rmse, 0) AS eid_improvement_factor,
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


def load_control_metrics(db: str) -> pd.DataFrame:
    if not table_exists(db, "control_metrics"):
        return pd.DataFrame()
    return _query("""
        SELECT e.experiment_id, e.run_id, e.timestamp, e.controller_method,
               e.disturb_type, cm.joint_id, jc.joint_name,
               cm.metric_name, cm.value, cm.unit,
               cm.window_start_s, cm.window_end_s, cm.source
        FROM control_metrics cm
        JOIN experiments e ON cm.experiment_id = e.experiment_id
        LEFT JOIN joint_configs jc ON cm.experiment_id = jc.experiment_id
             AND cm.joint_id = jc.joint_id
        WHERE e.controller_method IN ('EID', 'PD')
        ORDER BY e.timestamp DESC, cm.joint_id, cm.metric_name
    """, db_path=db)


def load_paired_metric(metric_name: str, db: str) -> pd.DataFrame:
    if not table_exists(db, "control_metrics"):
        return pd.DataFrame()
    return _query("""
        SELECT cp.pair_id,
               eid.run_id AS eid_run_id, pd.run_id AS pd_run_id,
               COALESCE(eid.disturb_type, cp.disturb_type, 'none') AS disturb_type,
               em.joint_id, ejc.joint_name,
               em.value AS eid_value, pm.value AS pd_value,
               pm.value / NULLIF(em.value, 0) AS pd_over_eid
        FROM comparison_pairs cp
        JOIN experiments eid ON cp.eid_experiment_id = eid.experiment_id
        JOIN experiments pd ON cp.pd_experiment_id = pd.experiment_id
        JOIN control_metrics em ON em.experiment_id = cp.eid_experiment_id
             AND em.metric_name = ?
        JOIN control_metrics pm ON pm.experiment_id = cp.pd_experiment_id
             AND pm.metric_name = em.metric_name
             AND pm.joint_id = em.joint_id
        LEFT JOIN joint_configs ejc ON ejc.experiment_id = cp.eid_experiment_id
             AND ejc.joint_id = em.joint_id
        ORDER BY eid.timestamp DESC, em.joint_id
    """, (metric_name,), db_path=db)


def load_dashboard_stats(db: str) -> dict[str, float]:
    exps = load_exps(db)
    stats: dict[str, float] = {
        "experiments": float(len(exps)),
        "timeseries_files": 0.0,
        "parquet_coverage": 0.0,
        "avg_saturation": math.nan,
        "flagged_joints": 0.0,
    }
    if table_exists(db, "timeseries_files"):
        ts = _query("SELECT COUNT(*) AS n FROM timeseries_files", db_path=db)
        stats["timeseries_files"] = float(ts.iloc[0]["n"]) if not ts.empty else 0.0
    if len(exps):
        stats["parquet_coverage"] = 100.0 * stats["timeseries_files"] / len(exps)
    if table_exists(db, "control_metrics"):
        sat = _query("""
            SELECT AVG(value) AS avg_sat FROM control_metrics
            WHERE metric_name='tau_saturation_duty'
        """, db_path=db)
        flags = _query("""
            SELECT COUNT(*) AS n FROM control_metrics
            WHERE metric_name='joint_flag_any' AND value != 0
        """, db_path=db)
        if not sat.empty and pd.notna(sat.iloc[0]["avg_sat"]):
            stats["avg_saturation"] = float(sat.iloc[0]["avg_sat"])
        if not flags.empty:
            stats["flagged_joints"] = float(flags.iloc[0]["n"])
    return stats


def load_ts_runs(repo_root: Path, db: str) -> pd.DataFrame:
    """Return experiments that have a Parquet or CSV timeseries available."""
    df = load_exps(db)
    if df.empty:
        return df
    if table_exists(db, "timeseries_files"):
        ts = _query("""
            SELECT experiment_id, path AS timeseries_path, format AS timeseries_format
            FROM timeseries_files
        """, db_path=db)
        if not ts.empty:
            df = df.merge(ts, on="experiment_id", how="left")
        else:
            df["timeseries_path"] = None
            df["timeseries_format"] = None
    else:
        df["timeseries_path"] = None
        df["timeseries_format"] = None
    rows = []
    ts_files = [
        "mujoco_closed_loop_log.csv",
        "eid_mujoco_closed_loop_log.csv",
        "pd_mujoco_closed_loop_log.csv",
        "eid_vs_pd_timeseries.csv",
    ]
    for _, r in df.iterrows():
        if pd.notna(r.get("timeseries_path")):
            p = Path(str(r["timeseries_path"]))
            if not p.is_absolute():
                p = repo_root / p
            if p.exists():
                rows.append(r.to_dict())
                continue
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

    /* Use Streamlit default fonts */

    .main .block-container { padding: 2rem 3rem; max-width: 1400px; }

    /* KPI cards with smooth premium transition */
    .kpi-grid { display: flex; gap: 1rem; flex-wrap: wrap; margin-bottom: 2rem; }
    .kpi-card {
        flex: 1; min-width: 150px;
        background: #ffffff; border: 1px solid #f1f5f9; border-radius: 16px;
        padding: 18px 24px; box-shadow: 0 4px 6px -1px rgba(0,0,0,0.05), 0 2px 4px -1px rgba(0,0,0,0.03);
        transition: transform 0.22s cubic-bezier(0.4, 0, 0.2, 1), box-shadow 0.22s cubic-bezier(0.4, 0, 0.2, 1);
    }
    .kpi-card:hover {
        transform: translateY(-3px);
        box-shadow: 0 10px 20px -3px rgba(0,0,0,0.06), 0 4px 10px -2px rgba(0,0,0,0.03);
        border-color: #3b82f6;
    }
    .kpi-card .label { font-size: 0.78rem; font-weight: 600; color: #64748b; text-transform: uppercase; letter-spacing: .06em; }
    .kpi-card .value { font-size: 1.75rem; font-weight: 700; color: #0f172a; margin-top: 4px; }

    /* Section cards */
    .section-card {
        background: #fff; border: 1px solid #f1f5f9; border-radius: 16px;
        padding: 24px 28px; margin-bottom: 1.5rem;
        box-shadow: 0 4px 6px -1px rgba(0,0,0,0.03);
    }

    /* Hide Streamlit chrome */
    #MainMenu, footer, header[data-testid="stHeader"] { display: none; }
    section[data-testid="stSidebar"] { background: #f8fafc; border-right: 1px solid #f1f5f9; }
    section[data-testid="stSidebar"] .block-container { padding: 2rem 1.25rem; }

    /* Premium Tab styling */
    .stTabs [data-baseweb="tab-list"] { gap: 8px; background: #f1f5f9; padding: 6px; border-radius: 12px; margin-bottom: 1.5rem; }
    .stTabs [data-baseweb="tab"] {
        border-radius: 8px; padding: 8px 16px;
        font-weight: 600; font-size: 0.88rem; border: none;
        color: #475569; background: transparent;
        transition: all 0.2s ease;
    }
    .stTabs [aria-selected="true"] {
        color: #1e40af !important; background: #ffffff !important;
        box-shadow: 0 4px 6px -1px rgba(0,0,0,0.05);
    }

    /* Buttons */
    .stButton > button {
        border-radius: 10px; font-weight: 600; border: 1px solid #e2e8f0;
        background: #fff; color: #334155; padding: 8px 18px;
        transition: all 0.2s ease;
    }
    .stButton > button:hover { border-color: #3b82f6; color: #3b82f6; transform: translateY(-1px); }

    /* Multiselect tag styling */
    span[data-baseweb="tag"] { border-radius: 8px !important; font-weight: 500; }

    /* Metric adjustments */
    div[data-testid="stMetric"] {
        background: #fff; border: 1px solid #f1f5f9; border-radius: 14px;
        padding: 14px 18px;
        box-shadow: 0 2px 4px -1px rgba(0,0,0,0.02);
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
    if val > 1.5:
        return "background-color:#d1fae5;color:#065f46;padding:2px 8px;border-radius:6px;font-weight:600"
    if val > 1.0:
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

@st.fragment
def page_overview(db: str) -> None:
    exps = load_exps(db)
    summaries = load_summaries(db)
    pairs = load_pairs(db)
    stats = load_dashboard_stats(db)

    if exps.empty:
        st.info("No data yet. Run: `python scripts/db_manager.py import-all`")
        return

    # Filter out legacy experiments for display
    display_exps = exps[~exps["controller_method"].str.contains("legacy", na=False)].copy()

    # KPI row
    n_eid = int((display_exps["controller_method"] == "EID").sum())
    n_pd = int((display_exps["controller_method"] == "PD").sum())
    n_pairs = len(pairs) if not pairs.empty else 0
    best_eid_rmse = summaries[summaries["controller_method"] == "EID"]["q_rmse"].min() if not summaries.empty else 0
    avg_sat = stats["avg_saturation"]

    kpi_row([
        ("EID Experiments", n_eid, None),
        ("PD Experiments", n_pd, None),
        ("Comparison Pairs", n_pairs, None),
        ("Parquet Coverage", f"{stats['parquet_coverage']:.0f}%", None),
        ("Avg Saturation", f"{100.0 * avg_sat:.1f}%" if math.isfinite(avg_sat) else "-", None),
        ("Flagged Joints", int(stats["flagged_joints"]), None),
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

    display = filtered[["run_id", "timestamp", "controller_method", "disturb_type", "duration_s"]].copy()
    display.columns = ["Run", "Time", "Method", "Disturbance", "Duration (s)"]
    st.dataframe(display, use_container_width=True, hide_index=True, height=350)

    # Show pairs table
    if not pairs.empty:
        st.markdown("### Comparison Pairs")
        pair_display = pairs[["eid_run_id", "pd_run_id", "disturb_type"]].copy()
        pair_display.columns = ["EID Run", "PD Run", "Disturbance"]
        st.dataframe(pair_display, use_container_width=True, hide_index=True, height=250)


@st.fragment
def page_joint_analysis(db: str) -> None:
    st.markdown("### Joint Performance")

    df = load_paired_comparison(db)
    if df.empty:
        # Fallback: try legacy comparison_results
        df = load_comps(db)
        if df.empty:
            st.info("No comparison data. Create comparison pairs or import experiments with summaries.")
            return
        if "eid_improvement_factor" not in df.columns and "rmse_ratio" in df.columns:
            df["eid_improvement_factor"] = df["rmse_ratio"]
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
        sort_by = st.selectbox("Sort by", ["EID Improvement", "EID RMSE", "PD RMSE"], key="ja_sort")

    filt = df[df["joint_name"].isin(sel_joints)] if sel_joints else df
    if sel_disturb != "all":
        filt = filt[filt["disturb_type"] == sel_disturb]

    if filt.empty:
        st.info("No data matching filters.")
        return

    # If using legacy data, columns are already eid_rmse/pd_rmse
    eid_col = "eid_rmse"
    pd_col = "pd_rmse"
    ratio_col = "eid_improvement_factor"

    agg = filt.groupby("joint_name").agg(
        eid=(eid_col, "mean"), pd=(pd_col, "mean"),
        improvement=(ratio_col, "mean"), n=("pair_id" if "pair_id" in filt.columns else "experiment_id", "count"),
    ).reset_index()
    agg["winner"] = np.where(agg["improvement"] > 1.0, "EID", "PD")

    if sort_by == "EID RMSE":
        agg = agg.sort_values("eid")
    elif sort_by == "PD RMSE":
        agg = agg.sort_values("pd")
    else:
        agg = agg.sort_values("improvement", ascending=False)

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

    st.markdown("#### EID Improvement Factor  (PD RMSE / EID RMSE)")
    st.caption("> 1.0 means EID has lower RMSE; < 1.0 means PD has lower RMSE.")

    def _style(sdf):
        s = sdf.style.format({
            "EID RMSE": "{:.5f}", "PD RMSE": "{:.5f}", "EID Improvement": "{:.4f}",
        }, na_rep="-")
        if "EID Improvement" in sdf.columns:
            def _c(v):
                if not isinstance(v, (int, float)) or not math.isfinite(v):
                    return ""
                if v > 1.5:
                    return "background-color:#d1fae5;color:#065f46;font-weight:600"
                if v > 1.0:
                    return "background-color:#f3f4f6;color:#374151"
                return "background-color:#fee2e2;color:#991b1b;font-weight:600"
            s = s.map(_c, subset=["EID Improvement"])
        return s

    table = agg[["joint_name", "eid", "pd", "improvement", "winner", "n"]].copy()
    table.columns = ["Joint", "EID RMSE", "PD RMSE", "EID Improvement", "Winner", "Runs"]
    st.dataframe(_style(table), use_container_width=True, hide_index=True)


@st.fragment
def page_control_metrics(db: str) -> None:
    st.markdown("### Control Metrics")
    df = load_control_metrics(db)
    if df.empty:
        st.info("No control metrics yet. Run: `python scripts/db_manager.py rebuild`.")
        return

    metric_options = [
        "q_rmse", "q_iae", "q_mae", "q_max_abs_error",
        "tau_energy", "tau_saturation_duty", "tau_rms", "tau_abs_max",
    ]
    available = [m for m in metric_options if m in set(df["metric_name"])]
    col1, col2, col3 = st.columns(3)
    with col1:
        metric = st.selectbox("Metric", available, index=0, key="cm_metric")
    with col2:
        methods = sorted(df["controller_method"].dropna().unique())
        method = st.selectbox("Controller", ["all"] + methods, key="cm_method")
    with col3:
        joints = sorted(df["joint_name"].dropna().unique())
        sel_joints = st.multiselect("Joints", joints, default=[j for j in ["RightKnee", "RightHipPitch", "RightElbow"] if j in joints],
                                    key="cm_joints")

    filt = df[df["metric_name"] == metric].copy()
    if method != "all":
        filt = filt[filt["controller_method"] == method]
    if sel_joints:
        filt = filt[filt["joint_name"].isin(sel_joints)]
    if filt.empty:
        st.info("No metrics matching filters.")
        return

    agg = filt.groupby(["joint_name", "controller_method"], as_index=False).agg(
        value=("value", "mean"), n=("value", "count"),
    )
    chart = alt.Chart(agg).mark_bar(cornerRadiusTopLeft=4, cornerRadiusTopRight=4).encode(
        x=alt.X("joint_name:N", title=None, axis=alt.Axis(labelAngle=-30)),
        y=alt.Y("value:Q", title=metric),
        color=alt.Color("controller_method:N", title=None,
                        scale=alt.Scale(domain=["EID", "PD"], range=[C["eid"], C["pd"]])),
        xOffset="controller_method:N",
        tooltip=["joint_name", "controller_method", alt.Tooltip("value:Q", format=".5f"), "n"],
    ).properties(height=330)
    compact_altair_chart(chart)

    table = agg.pivot(index="joint_name", columns="controller_method", values="value").reset_index()
    table.columns.name = None
    st.dataframe(table, use_container_width=True, hide_index=True)


@st.fragment
def page_sine_tracking(db: str) -> None:
    st.markdown("### Sine Tracking Diagnostics")
    df = load_control_metrics(db)
    if df.empty:
        st.info("No control metrics yet. Run: `python scripts/db_manager.py rebuild`.")
        return

    sine_metrics = ["tracking_gain", "phase_lag_deg", "amplitude_error", "bias_error"]
    df = df[df["metric_name"].isin(sine_metrics)].copy()
    if df.empty:
        st.info("No sine tracking metrics found. Zero-amplitude or non-sine joints are intentionally skipped.")
        return

    col1, col2 = st.columns(2)
    with col1:
        metric = st.selectbox("Metric", sine_metrics, key="st_metric")
    with col2:
        joints = sorted(df["joint_name"].dropna().unique())
        sel_joints = st.multiselect("Joints", joints, default=[j for j in ["RightKnee", "RightHipPitch", "LeftHipPitch"] if j in joints],
                                    key="st_joints")

    filt = df[df["metric_name"] == metric]
    if sel_joints:
        filt = filt[filt["joint_name"].isin(sel_joints)]
    if filt.empty:
        st.info("No sine metrics matching filters.")
        return

    agg = filt.groupby(["joint_name", "controller_method"], as_index=False).agg(
        value=("value", "mean"), n=("value", "count"),
    )
    chart = alt.Chart(agg).mark_bar(cornerRadiusTopLeft=4, cornerRadiusTopRight=4).encode(
        x=alt.X("joint_name:N", title=None, axis=alt.Axis(labelAngle=-30)),
        y=alt.Y("value:Q", title=metric),
        color=alt.Color("controller_method:N", title=None,
                        scale=alt.Scale(domain=["EID", "PD"], range=[C["eid"], C["pd"]])),
        xOffset="controller_method:N",
        tooltip=["joint_name", "controller_method", alt.Tooltip("value:Q", format=".5f"), "n"],
    ).properties(height=320)
    compact_altair_chart(chart)

    paired = load_paired_metric(metric, db)
    if not paired.empty:
        if sel_joints:
            paired = paired[paired["joint_name"].isin(sel_joints)]
        if not paired.empty:
            st.markdown("#### Paired EID vs PD")
            paired_display = paired[["joint_name", "eid_value", "pd_value", "pd_over_eid"]].copy()
            paired_display.columns = ["Joint", "EID", "PD", "PD / EID"]
            st.dataframe(
                paired_display.style.format({"EID": "{:.5f}", "PD": "{:.5f}", "PD / EID": "{:.4f}"}),
                use_container_width=True, hide_index=True,
            )


@st.fragment
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
        default=[],  # select nothing by default
        format_func=lambda rid: run_labels.get(rid, rid),
        key="ts_runs",
    )

    if not selected_runs:
        st.info("Select at least one experiment to view timeseries.")
        return

    # --- Load all selected experiments' timeseries ---
    @st.cache_data(show_spinner="Loading...")
    def _find_ts_file(run_dir_str: str, repo_root_str: str, preferred_path: str = "") -> Path | None:
        root = Path(repo_root_str)
        if preferred_path:
            p = Path(preferred_path)
            if not p.is_absolute():
                p = root / p
            if p.exists():
                return p
        d = Path(run_dir_str)
        if not d.is_absolute():
            d = root / d
        for fname in ["timeseries.parquet",
                       "mujoco_closed_loop_log.csv",
                       "eid_mujoco_closed_loop_log.csv",
                       "pd_mujoco_closed_loop_log.csv",
                       "eid_vs_pd_timeseries.csv"]:
            p = d / fname
            if p.exists():
                return p
        return None

    @st.cache_data(show_spinner="Loading...")
    def _load_ts(path_str: str) -> pd.DataFrame:
        path = Path(path_str)
        if path.suffix.lower() == ".parquet":
            df = pd.read_parquet(path)
        else:
            df = pd.read_csv(path)
        if "t" in df.columns:
            df["t"] = pd.to_numeric(df["t"], errors="coerce")
        tau_col = next((c for c in ["u_t", "tau_cmd", "motor_tau"] if c in df.columns), None)
        if tau_col and "tau_command" not in df.columns:
            df["tau_command"] = pd.to_numeric(df[tau_col], errors="coerce")
        if "saturation" not in df.columns and "joint_flags" in df.columns:
            df["saturation"] = (pd.to_numeric(df["joint_flags"], errors="coerce").fillna(0) != 0).astype(int)
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
        preferred = str(row.get("timeseries_path") or "")
        ts_path = _find_ts_file(str(run_dir), str(repo_root), preferred)
        if ts_path is None:
            st.warning(f"No timeseries data for {run_id}")
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

    _META_SIGNALS = {"flags", "joint_flags"}

    # Collect all signals and joints across all dataframes without outer merging
    all_signals_set = set()
    joints_set = set()
    for df in all_dfs:
        joints_set.update(df["joint_id"].unique())
        for c in df.columns:
            if c not in COMMON_COLS and base_signal_name(c) not in _META_SIGNALS:
                all_signals_set.add(c)

    all_signals = sorted(list(all_signals_set))
    joints = sorted(list(joints_set))
    jlabels = {j: f"J{j} {JOINT_NAMES.get(j, '')}" for j in joints}

    # --- Panel management ---
    if "ts_panels" not in st.session_state:
        st.session_state["ts_panels"] = 1  # default 1 panel

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
        default=[],
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
    PANEL_DEFAULTS: list[dict[str, set[str]]] = []

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

                # Build long-form dataframe for Altair without outer merge
                records = []
                for df in all_dfs:
                    ts_filtered = df[df["joint_id"].isin(sel_joints)].copy()
                    if ts_filtered.empty:
                        continue

                    total_rows = len(ts_filtered)
                    if total_rows > MAX_POINTS:
                        ts_filtered = ts_filtered.iloc[::max(1, total_rows // MAX_POINTS)]

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

                import plotly.graph_objects as go

                # Group signals by base name for consistent coloring
                base_names = sorted(set(base_signal_name(s) for s in panel_selected))
                base_colors = {}
                for j, bn in enumerate(base_names):
                    base_colors[bn] = color_palette[j % len(color_palette)]

                n_joints = len(sel_joints)

                # Render each joint as a beautifully styled, high-performance Plotly trace
                for jid in sel_joints:
                    jname = jlabels[jid]
                    jlf = lf[lf["Joint"] == jname]
                    if jlf.empty:
                        continue

                    fig = go.Figure()

                    experiments = jlf["Experiment"].unique()
                    signals = jlf["Signal"].unique()

                    # Distinguish different experiments using dashes
                    dash_styles = ['solid', 'dash', 'dot', 'dashdot', 'longdash']
                    exp_dash = {exp: dash_styles[i % len(dash_styles)] for i, exp in enumerate(experiments)}

                    for sig in signals:
                        for exp in experiments:
                            sub_df = jlf[(jlf["Signal"] == sig) & (jlf["Experiment"] == exp)].sort_values("t")
                            if sub_df.empty:
                                continue

                            base_sig = base_signal_name(sig)
                            display_name = f"{exp} | {base_sig.replace('_', ' ')}"

                            fig.add_trace(go.Scatter(
                                x=sub_df["t"],
                                y=sub_df["value"],
                                name=display_name,
                                mode="lines",
                                line=dict(
                                    color=base_colors.get(base_sig, C["muted"]),
                                    width=2.0,
                                    dash=exp_dash[exp]
                                ),
                                hovertemplate="%{y:.5f}<extra></extra>"
                            ))

                    fig.update_layout(
                        title=dict(
                            text=f"<b>{jname}</b>",
                            font=dict(size=14, color=C["text"])
                        ),
                        height=max(220, min(340, 700 // n_joints)),
                        margin=dict(l=15, r=15, t=45, b=15),
                        hovermode="x unified",
                        plot_bgcolor="rgba(0,0,0,0)",
                        paper_bgcolor="rgba(0,0,0,0)",
                        legend=dict(
                            orientation="h",
                            yanchor="bottom",
                            y=1.02,
                            xanchor="left",
                            x=0,
                            font=dict(size=10),
                            bgcolor="rgba(255,255,255,0.7)"
                        ),
                        xaxis=dict(
                            title="Time (s)" if jid == sel_joints[-1] else "",
                            showgrid=True,
                            gridcolor="#f1f5f9",
                            linecolor="#e2e8f0",
                            tickfont=dict(size=10)
                        ),
                        yaxis=dict(
                            showgrid=True,
                            gridcolor="#f1f5f9",
                            linecolor="#e2e8f0",
                            tickfont=dict(size=10)
                        )
                    )
                    st.plotly_chart(fig, use_container_width=True)

    # --- Per-joint stats ---
    with st.expander("Per-joint statistics", expanded=False):
        rows = []
        for jid in sel_joints:
            r = {"Joint": JOINT_NAMES.get(jid, str(jid))}
            for df in all_dfs:
                jdf = df[df["joint_id"] == jid]
                for sig in all_signals:
                    if sig in jdf.columns:
                        s = jdf[sig].dropna()
                        if len(s) > 0:
                            r[f"{sig}_rms"] = f"{math.sqrt((s**2).mean()):.4f}"
            rows.append(r)
        if rows:
            st.dataframe(pd.DataFrame(rows), use_container_width=True, hide_index=True)


@st.fragment
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


@st.fragment
def page_query(db: str) -> None:
    st.markdown("### SQL Console")
    st.caption("Tables: `experiments` `joint_configs` `joint_summaries` `comparison_results` `ablation_configs`")

    sql = st.text_area("Query", height=140, key="sql_query", value="""SELECT
    ejc.joint_name,
    COALESCE(eid.disturb_type, cp.disturb_type, 'none') AS disturb_type,
    AVG(em.value) AS eid_rmse,
    AVG(pm.value) AS pd_rmse,
    AVG(pm.value / NULLIF(em.value, 0)) AS eid_improvement_factor,
    COUNT(*) AS n
FROM comparison_pairs cp
JOIN experiments eid ON cp.eid_experiment_id = eid.experiment_id
JOIN experiments pd ON cp.pd_experiment_id = pd.experiment_id
JOIN control_metrics em ON em.experiment_id = cp.eid_experiment_id
    AND em.metric_name = 'q_rmse'
JOIN control_metrics pm ON pm.experiment_id = cp.pd_experiment_id
    AND pm.metric_name = em.metric_name AND pm.joint_id = em.joint_id
JOIN joint_configs ejc ON ejc.experiment_id = cp.eid_experiment_id
    AND ejc.joint_id = em.joint_id
GROUP BY ejc.joint_name, disturb_type
ORDER BY eid_improvement_factor DESC""")

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

    # Premium Gradient Title Block
    st.markdown("""
    <div style="background: linear-gradient(135deg, #1e3a8a 0%, #3b82f6 50%, #4f46e5 100%); padding: 2rem 2.5rem; border-radius: 18px; margin-bottom: 2rem; box-shadow: 0 10px 25px -5px rgba(59, 130, 246, 0.12), 0 8px 10px -6px rgba(59, 130, 246, 0.12);">
        <h1 style="color: white; margin: 0; font-size: 2.2rem; font-weight: 800; letter-spacing: -0.025em; line-height: 1.2; font-family: 'Inter', sans-serif;">馃 H1 Joint Control & Diagnostics Dashboard</h1>
        <p style="color: rgba(255, 255, 255, 0.85); margin: 0.6rem 0 0 0; font-size: 1.05rem; font-weight: 400; font-family: 'Inter', sans-serif;">Advanced real-time visualization and performance validation for EID & PD controllers</p>
    </div>
    """, unsafe_allow_html=True)

    # Sidebar
    with st.sidebar:
        st.markdown("## 棣冾樆 H1 Control DB")

        db_path = st.text_input("Database", str(DEFAULT_DB), key="db_path")

        if not Path(db_path).exists():
            st.error(f"Not found: {db_path}")
            st.info("Run: `python scripts/db_manager.py import-all`")
            return

        # Quick stats
        stats = _query("""
            SELECT (SELECT COUNT(*) FROM experiments) AS n_exps,
                   (SELECT COUNT(*) FROM comparison_pairs) AS n_comps
        """, db_path=db_path)
        if not stats.empty:
            s = stats.iloc[0]
            st.metric("Experiments", int(s["n_exps"]))
            st.metric("Comparisons", int(s["n_comps"]))

        st.divider()

        # Quick reimport
        if st.button("棣冩敡 Re-import all runs", use_container_width=True):
            import subprocess, sys
            result = subprocess.run(
                [sys.executable, str(REPO_ROOT / "scripts" / "db_manager.py"), "rebuild",
                 "--db", db_path],
                capture_output=True, text=True, timeout=60,
            )
            if result.returncode == 0:
                st.success("Done 閳?refresh the page")
                _query.clear()
                load_exps.clear()
                load_comps.clear()
            else:
                st.error(result.stderr[-300:] if result.stderr else "Unknown error")

    # Main tabs
    tab1, tab2, tab3, tab4, tab5, tab6 = st.tabs([
        "Overview", "Joint Analysis", "Control Metrics", "Sine Tracking", "Timeseries", "SQL",
    ])

    with tab1:
        page_overview(db_path)
    with tab2:
        page_joint_analysis(db_path)
    with tab3:
        page_control_metrics(db_path)
    with tab4:
        page_sine_tracking(db_path)
    with tab5:
        page_timeseries(db_path)
    with tab6:
        c1, c2 = st.tabs(["SQL Console", "Config Inspector"])
        with c1:
            page_query(db_path)
        with c2:
            page_config(db_path)


if __name__ == "__main__":
    main()
