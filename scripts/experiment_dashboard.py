#!/usr/bin/env python3
"""Local Streamlit dashboard for browsing H1 joint-control experiments."""

from __future__ import annotations

import math
import sys
from pathlib import Path

try:
    import altair as alt
    import pandas as pd
    import streamlit as st
except ImportError as exc:  # pragma: no cover - shown when run without deps.
    print(
        "Missing dashboard dependency. Install with:\n"
        "  python -m pip install -r requirements-dashboard.txt\n\n"
        f"Import error: {exc}",
        file=sys.stderr,
    )
    raise SystemExit(1)

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from index_runs import build_index, write_index  # noqa: E402


DEFAULT_DATA_ROOT = REPO_ROOT / "data"
DEFAULT_INDEX_PATH = DEFAULT_DATA_ROOT / "runs_index.csv"

DERIVED_FIELDS = {
    "time_s": lambda df: df["t"] - df["t"].iloc[0],
    "q_ref": lambda df: df["debug_0"],
    "dq_ref": lambda df: df["debug_1"],
    "q_error": lambda df: df["debug_0"] - df["q"],
    "dq_error": lambda df: df["debug_1"] - df["dq"],
    "u_raw": lambda df: df["debug_25"],
    "q_ref_raw": lambda df: df["debug_26"],
    "dq_ref_raw": lambda df: df["debug_27"],
}

PRESETS = {
    "Tracking": ["q_ref", "q", "q_error"],
    "Velocity": ["dq_ref", "dq", "dq_error"],
    "Torque": ["tau_cmd", "u_raw", "tau_est"],
    "Safety": ["lowstate_age", "flags", "dt"],
}

TABLE_COLUMNS = [
    "run_id",
    "timestamp",
    "object_type",
    "joint_name",
    "controller_method",
    "reference_mode",
    "interpolation",
    "trajectory",
    "duration_s",
    "q_error_rmse",
    "tau_cmd_abs_max",
    "flags",
]


def repo_path(path_text: str) -> Path:
    path = Path(path_text)
    return path if path.is_absolute() else REPO_ROOT / path


def ensure_index(index_path: Path, data_root: Path) -> None:
    if not index_path.exists():
        rows = build_index(data_root)
        write_index(rows, index_path)


@st.cache_data(show_spinner=False)
def load_index(index_path_text: str) -> pd.DataFrame:
    df = pd.read_csv(index_path_text, keep_default_na=False)
    numeric_cols = [
        "joint_id",
        "samples",
        "duration_s",
        "q_error_rmse",
        "q_min",
        "q_max",
        "q_ref_min",
        "q_ref_max",
        "tau_cmd_abs_max",
        "lowstate_age_max",
        "file_size_bytes",
    ]
    for col in numeric_cols:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    return df.sort_values("timestamp", ascending=False, kind="stable")


@st.cache_data(show_spinner="Loading log...")
def load_log_sampled(log_path_text: str, max_points: int) -> pd.DataFrame:
    path = repo_path(log_path_text)
    df = pd.read_csv(path)
    for name, builder in DERIVED_FIELDS.items():
        try:
            df[name] = builder(df)
        except KeyError:
            pass
    if len(df) > max_points:
        step = max(1, math.ceil(len(df) / max_points))
        df = df.iloc[::step].copy()
    return df


def css() -> None:
    st.markdown(
        """
        <style>
        .block-container { padding-top: 1.2rem; }
        div[data-testid="stMetric"] {
            border: 1px solid #e5e7eb;
            border-radius: 8px;
            padding: 10px 12px;
            background: #fafafa;
        }
        .stTabs [data-baseweb="tab-list"] { gap: 8px; }
        .stTabs [data-baseweb="tab"] {
            border: 1px solid #e5e7eb;
            border-radius: 8px;
            padding: 8px 14px;
        }
        </style>
        """,
        unsafe_allow_html=True,
    )


def option_values(df: pd.DataFrame, column: str) -> list[str]:
    return sorted(value for value in df[column].dropna().astype(str).unique() if value)


def choose_many(label: str, values: list[str], default: list[str] | None = None) -> list[str]:
    return st.multiselect(label, values, default=default or [], placeholder="All")


def filter_panel(df: pd.DataFrame) -> dict[str, object]:
    with st.sidebar.form("filters"):
        st.caption("Filters apply only when you press Apply.")
        search = st.text_input("Search", placeholder="RightKnee closed_loop quintic")
        filters = {
            "search": search,
            "object_type": choose_many("Object", option_values(df, "object_type")),
            "joint_name": choose_many("Joint", option_values(df, "joint_name")),
            "controller_method": choose_many("Controller", option_values(df, "controller_method")),
            "reference_mode": choose_many("Reference mode", option_values(df, "reference_mode")),
            "interpolation": choose_many("Interpolation", option_values(df, "interpolation")),
            "trajectory": choose_many("Trajectory", option_values(df, "trajectory")),
            "only_clean": st.checkbox("Hide rows with safety flags", value=False),
        }
        st.form_submit_button("Apply filters", use_container_width=True)
    return filters


def apply_filters(df: pd.DataFrame, filters: dict[str, object]) -> pd.DataFrame:
    filtered = df.copy()
    search = str(filters.get("search", "")).strip()
    if search:
        text = filtered.fillna("").astype(str).agg(" ".join, axis=1).str.lower()
        mask = pd.Series(True, index=filtered.index)
        for token in search.lower().split():
            mask &= text.str.contains(token, regex=False)
        filtered = filtered[mask]

    for column in [
        "object_type",
        "joint_name",
        "controller_method",
        "reference_mode",
        "interpolation",
        "trajectory",
    ]:
        selected = filters.get(column) or []
        if selected:
            filtered = filtered[filtered[column].astype(str).isin(selected)]

    if filters.get("only_clean"):
        filtered = filtered[filtered["flags"].astype(str).isin(["", "0"])]
    return filtered


def label_for(row: pd.Series) -> str:
    return (
        f"{row['run_id']} | {row['joint_name']} | {row['controller_method']} | "
        f"{row['reference_mode']} | {row['interpolation']}"
    )


def fields_for(df: pd.DataFrame) -> list[str]:
    numeric = [col for col in df.columns if pd.api.types.is_numeric_dtype(df[col])]
    preferred = [
        "q_ref",
        "q",
        "q_error",
        "dq_ref",
        "dq",
        "dq_error",
        "tau_cmd",
        "u_raw",
        "tau_est",
        "lowstate_age",
        "dt",
        "flags",
    ]
    ordered = [col for col in preferred if col in numeric]
    ordered.extend(col for col in numeric if col not in ordered and col not in {"cycle", "time_s"})
    return ordered


def line_chart(df: pd.DataFrame, fields: list[str], title: str) -> None:
    if not fields:
        st.info("Select at least one field.")
        return
    if "time_s" not in df.columns:
        st.warning("This log does not contain a usable time column.")
        return

    chart_df = df[["time_s", *fields]].melt("time_s", var_name="field", value_name="value")
    chart = (
        alt.Chart(chart_df)
        .mark_line(point=False)
        .encode(
            x=alt.X("time_s:Q", title="time (s)"),
            y=alt.Y("value:Q", title=None),
            color=alt.Color("field:N", title=None),
            tooltip=[
                alt.Tooltip("time_s:Q", format=".3f"),
                alt.Tooltip("field:N"),
                alt.Tooltip("value:Q", format=".5f"),
            ],
        )
        .properties(height=380, title=title)
        .interactive()
    )
    st.altair_chart(chart, use_container_width=True)


def show_metrics(df: pd.DataFrame, filtered: pd.DataFrame) -> None:
    clean = filtered[filtered["flags"].astype(str).isin(["", "0"])]
    c1, c2, c3, c4 = st.columns(4)
    c1.metric("Runs", len(filtered), delta=f"{len(df)} total")
    c2.metric("Clean", len(clean))
    c3.metric("Best RMSE", f"{filtered['q_error_rmse'].min():.5f}" if len(filtered) else "-")
    c4.metric(
        "Max |tau|",
        f"{filtered['tau_cmd_abs_max'].max():.2f}" if len(filtered) else "-",
    )


def overview_tab(filtered: pd.DataFrame) -> None:
    visible = [col for col in TABLE_COLUMNS if col in filtered.columns]
    st.dataframe(
        filtered[visible],
        use_container_width=True,
        hide_index=True,
        column_config={
            "q_error_rmse": st.column_config.NumberColumn("q RMSE", format="%.5f"),
            "tau_cmd_abs_max": st.column_config.NumberColumn("max |tau|", format="%.2f"),
            "duration_s": st.column_config.NumberColumn("duration", format="%.2f s"),
        },
    )
    st.download_button(
        "Download filtered index",
        filtered.to_csv(index=False).encode("utf-8"),
        "filtered_runs.csv",
        "text/csv",
    )


def inspect_tab(filtered: pd.DataFrame, max_points: int) -> None:
    labels = [label_for(row) for _, row in filtered.iterrows()]
    label_map = {label: idx for label, idx in zip(labels, filtered.index)}
    if not labels:
        st.info("No runs to inspect.")
        return
    selected = st.selectbox("Run", labels, index=0)
    if selected is None:
        st.info("Select a run to inspect.")
        return
    row = filtered.loc[label_map[selected]]

    c1, c2, c3, c4 = st.columns(4)
    c1.metric("Joint", str(row["joint_name"]))
    c2.metric("Mode", str(row["reference_mode"]))
    c3.metric("Interpolation", str(row["interpolation"]))
    c4.metric("RMSE", f"{row['q_error_rmse']:.5f}")

    log_df = load_log_sampled(str(row["log_path"]), max_points)
    all_fields = fields_for(log_df)
    if not all_fields:
        st.warning("This log has no numeric fields available for plotting.")
        return
    preset = st.segmented_control("Preset", list(PRESETS), default="Tracking") or "Tracking"
    default_fields = [field for field in PRESETS[preset] if field in all_fields]

    field_query = st.text_input("Field search", placeholder="debug_25, tau, q_error")
    choices = all_fields
    if field_query.strip():
        needle = field_query.strip().lower()
        choices = [field for field in all_fields if needle in field.lower()]
    fields = st.multiselect("Fields", choices, default=default_fields)
    line_chart(log_df, fields, row["run_id"])

    left, right = st.columns([1, 1])
    with left.expander("Run metadata", expanded=False):
        st.json({key: row[key] for key in row.index})
    with right.expander("Sampled data preview", expanded=False):
        st.dataframe(log_df.head(300), use_container_width=True, hide_index=True)


def compare_tab(filtered: pd.DataFrame, max_points: int) -> None:
    labels = [label_for(row) for _, row in filtered.iterrows()]
    label_map = {label: idx for label, idx in zip(labels, filtered.index)}
    selected = st.multiselect("Runs to compare", labels, default=labels[: min(3, len(labels))])
    if not selected:
        st.info("Select at least one run.")
        return
    if len(selected) > 8:
        st.warning("Showing the first 8 selected runs to keep the chart responsive.")
        selected = selected[:8]

    first = filtered.loc[label_map[selected[0]]]
    first_log = load_log_sampled(str(first["log_path"]), max_points)
    options = fields_for(first_log)
    if not options:
        st.warning("The selected run has no numeric fields available for plotting.")
        return
    preferred = [field for field in ["q_error", "q", "q_ref", "tau_cmd", "u_raw"] if field in options]
    field = st.selectbox("Compare field", preferred + [f for f in options if f not in preferred])
    if field is None:
        st.info("Select a field to compare.")
        return

    wide = []
    selected_rows = []
    for label in selected:
        row = filtered.loc[label_map[label]]
        log_df = load_log_sampled(str(row["log_path"]), max_points)
        if field not in log_df.columns:
            continue
        series = log_df[["time_s", field]].rename(columns={field: row["run_id"]})
        wide.append(series.set_index("time_s"))
        selected_rows.append(row)

    if wide:
        chart_df = pd.concat(wide, axis=1).reset_index().melt("time_s", var_name="run", value_name=field)
        chart = (
            alt.Chart(chart_df)
            .mark_line(point=False)
            .encode(
                x=alt.X("time_s:Q", title="time (s)"),
                y=alt.Y(f"{field}:Q", title=field),
                color=alt.Color("run:N", title=None),
                tooltip=[
                    alt.Tooltip("time_s:Q", format=".3f"),
                    alt.Tooltip("run:N"),
                    alt.Tooltip(f"{field}:Q", format=".5f"),
                ],
            )
            .properties(height=430, title=f"Compare {field}")
            .interactive()
        )
        st.altair_chart(chart, use_container_width=True)

    if selected_rows:
        st.dataframe(pd.DataFrame(selected_rows)[TABLE_COLUMNS], use_container_width=True, hide_index=True)


def files_tab(filtered: pd.DataFrame) -> None:
    cols = ["run_id", "log_path", "config_path", "run_yaml_path", "run_dir", "file_size_bytes"]
    st.dataframe(filtered[[col for col in cols if col in filtered.columns]], use_container_width=True, hide_index=True)


def main() -> None:
    st.set_page_config(page_title="H1 Experiment Browser", layout="wide")
    css()

    st.title("H1 Experiment Browser")

    data_root = Path(st.sidebar.text_input("Data root", str(DEFAULT_DATA_ROOT)))
    index_path = Path(st.sidebar.text_input("Index file", str(DEFAULT_INDEX_PATH)))
    if st.sidebar.button("Rescan data", use_container_width=True):
        rows = build_index(data_root)
        write_index(rows, index_path)
        load_index.clear()
        load_log_sampled.clear()
        st.sidebar.success(f"Indexed {len(rows)} logs")

    ensure_index(index_path, data_root)
    index_df = load_index(str(index_path))
    filters = filter_panel(index_df)
    filtered = apply_filters(index_df, filters)

    max_points = st.sidebar.slider("Chart max points per run", 500, 12000, 2500, step=500)
    show_metrics(index_df, filtered)

    if filtered.empty:
        st.info("No runs match the current filters.")
        return

    tab_overview, tab_inspect, tab_compare, tab_files = st.tabs(
        ["Overview", "Inspect", "Compare", "Files"]
    )
    with tab_overview:
        overview_tab(filtered)
    with tab_inspect:
        inspect_tab(filtered, max_points)
    with tab_compare:
        compare_tab(filtered, max_points)
    with tab_files:
        files_tab(filtered)


if __name__ == "__main__":
    main()
