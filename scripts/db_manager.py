#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""H1 experiment database manager 鈥?SQLite + Parquet hybrid storage.

Usage:
  python scripts/db_manager.py init                        # Create tables
  python scripts/db_manager.py import-all                  # Import all runs under data/
  python scripts/db_manager.py import <run_dir>            # Import a single run
  python scripts/db_manager.py rebuild                     # Drop + recreate + reimport all
  python scripts/db_manager.py stats                       # Print summary statistics
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sqlite3
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DB_PATH = REPO_ROOT / "data" / "experiments.db"
DEFAULT_DATA_ROOT = REPO_ROOT / "data"

JOINT_NAMES: dict[int, str] = {
    0: "RightHipRoll", 1: "RightHipPitch", 2: "RightKnee",
    3: "LeftHipRoll", 4: "LeftHipPitch", 5: "LeftKnee",
    6: "WaistYaw", 7: "LeftHipYaw", 8: "RightHipYaw",
    9: "NotUsedJoint", 10: "LeftAnkle", 11: "RightAnkle",
    12: "RightShoulderPitch", 13: "RightShoulderRoll",
    14: "RightShoulderYaw", 15: "RightElbow",
    16: "LeftShoulderPitch", 17: "LeftShoulderRoll",
    18: "LeftShoulderYaw", 19: "LeftElbow",
}

# DDL statements
TABLE_DEFS = [
    """CREATE TABLE IF NOT EXISTS experiments (
        experiment_id     INTEGER PRIMARY KEY AUTOINCREMENT,
        run_id            TEXT NOT NULL UNIQUE,
        timestamp         DATETIME NOT NULL,
        object_type       TEXT NOT NULL CHECK (object_type IN ('mujoco', 'mock', 'real')),
        controller_method TEXT NOT NULL,
        duration_s        REAL,
        control_dt        REAL NOT NULL DEFAULT 0.002,
        config_path       TEXT,
        config_snapshot   TEXT,
        run_dir           TEXT,
        notes             TEXT,
        created_at        DATETIME DEFAULT CURRENT_TIMESTAMP
    )""",
    """CREATE TABLE IF NOT EXISTS joint_configs (
        config_id       INTEGER PRIMARY KEY AUTOINCREMENT,
        experiment_id   INTEGER NOT NULL REFERENCES experiments(experiment_id),
        joint_id        INTEGER NOT NULL CHECK (joint_id >= 0 AND joint_id < 20),
        joint_name      TEXT NOT NULL,
        enabled         INTEGER NOT NULL DEFAULT 1,
        kp                  REAL,
        kd                  REAL,
        observer_gain_q     REAL,
        observer_gain_dq    REAL,
        filter_alpha        REAL,
        policy_interpolation      TEXT,
        policy_source    TEXT,
        policy_center          REAL,
        policy_amplitude       REAL,
        policy_frequency_hz       REAL,
        policy_phase_rad           REAL,
        policy_step_time_s       REAL,
        startup_blend_duration_s REAL,
        eid_tau_limit       REAL,
        eid_tau_slew_rate   REAL,
        torque_safe_kp      REAL,
        torque_safe_kd      REAL,
        inverse_q_weight    REAL,
        inverse_dq_weight   REAL,
        plant_Jeff      REAL,
        plant_b         REAL,
        plant_gravityA  REAL,
        plant_gravityB  REAL,
        plant_tau0      REAL,
        plant_q_min     REAL,
        plant_q_max     REAL,
        plant_tau_max   REAL,
        UNIQUE(experiment_id, joint_id)
    )""",
    """CREATE TABLE IF NOT EXISTS joint_summaries (
        summary_id      INTEGER PRIMARY KEY AUTOINCREMENT,
        experiment_id   INTEGER NOT NULL REFERENCES experiments(experiment_id),
        joint_id        INTEGER NOT NULL,
        q_rmse          REAL,
        q_max_error     REAL,
        dq_rmse         REAL,
        q_error_ref_type TEXT,
        tau_abs_max     REAL,
        tau_mean_abs    REAL,
        tau_rms         REAL,
        safety_flags    INTEGER DEFAULT 0,
        q_min_actual    REAL,
        q_max_actual    REAL,
        dq_max_actual   REAL,
        lowstate_age_max REAL,
        UNIQUE(experiment_id, joint_id),
        FOREIGN KEY (experiment_id) REFERENCES experiments(experiment_id)
    )""",
    """CREATE TABLE IF NOT EXISTS comparison_results (
        comparison_id   INTEGER PRIMARY KEY AUTOINCREMENT,
        experiment_id   INTEGER NOT NULL REFERENCES experiments(experiment_id),
        joint_id        INTEGER NOT NULL,
        eid_rmse        REAL,
        pd_rmse         REAL,
        rmse_ratio      REAL,
        eid_max_error   REAL,
        pd_max_error    REAL,
        eid_mean_abs_tau REAL,
        pd_mean_abs_tau REAL,
        disturb_type        TEXT DEFAULT 'none',
        disturb_magnitude   REAL,
        disturb_frequency   REAL,
        pd_kp_used      REAL,
        pd_kd_used      REAL,
        UNIQUE(experiment_id, joint_id),
        FOREIGN KEY (experiment_id) REFERENCES experiments(experiment_id)
    )""",
    """CREATE TABLE IF NOT EXISTS ablation_configs (
        ablation_id     INTEGER PRIMARY KEY AUTOINCREMENT,
        experiment_id   INTEGER NOT NULL REFERENCES experiments(experiment_id),
        no_feedforward  INTEGER DEFAULT 0,
        no_observer     INTEGER DEFAULT 0,
        no_ref_mod      INTEGER DEFAULT 0,
        no_feedback     INTEGER DEFAULT 0,
        UNIQUE(experiment_id)
    )""",
    """CREATE TABLE IF NOT EXISTS comparison_pairs (
        pair_id           INTEGER PRIMARY KEY AUTOINCREMENT,
        eid_experiment_id INTEGER NOT NULL REFERENCES experiments(experiment_id),
        pd_experiment_id  INTEGER NOT NULL REFERENCES experiments(experiment_id),
        disturb_type      TEXT DEFAULT 'none',
        notes             TEXT,
        created_at        DATETIME DEFAULT CURRENT_TIMESTAMP,
        UNIQUE(eid_experiment_id, pd_experiment_id)
    )""",
]

# Flexible summary CSV column mapping: CSV column name -> DB column name
SUMMARY_COLUMN_MAP: dict[str, str] = {
    "q_rmse": "q_rmse",
    "q_max_abs_error": "q_max_error",
    "tau_cmd_abs_max": "tau_abs_max",
    "tau_mean_abs": "tau_mean_abs",
    "rmse": "q_rmse",
    "max_error": "q_max_error",
    "max_torque": "tau_abs_max",
    "mean_abs_torque": "tau_mean_abs",
}

# Columns in the C++ controller stepper CSV log (raw format before transformation)
RAW_LOG_COLUMNS = {
    "cycle", "t", "dt", "lowstate_age",
    "joint_id", "q", "dq", "tau_est",
    "q_cmd", "dq_cmd", "kp_cmd", "kd_cmd", "tau_cmd", "flags",
}

# Debug data column mapping: debug_joint.data[index] -> parquet column name
DEBUG_COLUMN_MAP: dict[int, tuple[str, str]] = {
    0:  ("q_ref", "float32"),
    1:  ("dq_ref", "float32"),
    2:  ("q", "float32"),
    3:  ("dq", "float32"),
    4:  ("q_error_shaped", "float32"),
    5:  ("dq_error_shaped", "float32"),
    6:  ("u_star", "float32"),
    7:  ("u_feedback", "float32"),
    8:  ("u_t", "float32"),
    9:  ("eta_q", "float32"),
    10: ("eta_dq", "float32"),
    11: ("x_hat_q", "float32"),
    12: ("x_hat_dq", "float32"),
    13: ("rho_q", "float32"),
    14: ("rho_dq", "float32"),
    15: ("x_bar_q", "float32"),
    16: ("q_ref_next", "float32"),
    17: ("dq_ref_next", "float32"),
    18: ("x_bar_dq", "float32"),
    19: ("r_d_q", "float32"),
    20: ("r_d_dq", "float32"),
    21: ("e_q", "float32"),
    22: ("e_dq", "float32"),
    23: ("observer_qacc", "float32"),
    24: ("observer_tau_applied", "float32"),
    25: ("u_raw", "float32"),
    26: ("q_ref_raw", "float32"),
    27: ("dq_ref_raw", "float32"),
    28: ("q_error_raw", "float32"),
    29: ("dq_error_raw", "float32"),
    30: ("q_error_shaped2", "float32"),
    31: ("dq_error_shaped2", "float32"),
}


# ---------------------------------------------------------------------------
# YAML config parser (handles the project's indentation-based YAML subset)
# ---------------------------------------------------------------------------

KEY_VALUE_RE = re.compile(r"^\s+([A-Za-z0-9_]+):\s*([^#]*?)(?:\s*#.*)?$")
NUMERIC_KEY_RE = re.compile(r"^\s*([0-9]+):\s*(?:#.*)?$")


def parse_yaml_config(config_path: Path) -> dict[str, Any]:
    """Parse the project's YAML config subset into nested dicts.

    Returns a dict with keys like:
      'robot', 'control_dt', 'controller', 'safe_hold',
      'controller': {'kind': str, 'defaults': {}, 'joints': {joint_id: {...}}},
      'joint_limits': {joint_id: {field: value}}
    """
    text = config_path.read_text(encoding="utf-8")

    result: dict[str, Any] = {
        "controller": {"kind": "eid", "defaults": {}, "joints": {}},
        "safe_hold": {},
        "joint_limits": {},
    }

    section: str | None = None
    controller_scope: str | None = None
    current_controller_joint: int | None = None
    current_limit_joint: int | None = None
    in_plant: bool = False

    for raw_line in text.splitlines():
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#"):
            continue

        indent = len(raw_line) - len(raw_line.lstrip(" "))

        # Top-level section header
        if indent == 0 and stripped.endswith(":") and ":" not in stripped.rstrip(":"):
            section = stripped.rstrip(":")
            controller_scope = None
            current_controller_joint = None
            current_limit_joint = None
            in_plant = False
            continue

        # Check for numeric joint header (e.g. "  0:") before general key:value
        num_match = NUMERIC_KEY_RE.match(raw_line)
        if num_match and indent == 4 and section == "controller" and controller_scope == "joints":
            current_controller_joint = int(num_match.group(1))
            if current_controller_joint not in result["controller"]["joints"]:
                result["controller"]["joints"][current_controller_joint] = {}
            in_plant = False
            continue
        if num_match and indent == 2 and section == "joint_limits":
            current_limit_joint = int(num_match.group(1))
            result["joint_limits"][current_limit_joint] = {}
            continue

        # Parse key: value
        m = KEY_VALUE_RE.match(raw_line)
        if not m:
            continue

        key = m.group(1).strip()
        value = m.group(2).strip().strip('"').strip("'")

        # Top-level key
        if indent == 0 and section is None:
            result[key] = _coerce_value(value)
            continue

        # Section-level keys
        if section == "safe_hold":
            result[section][key] = _coerce_value(value)
        elif section == "controller":
            if indent == 2 and key == "kind":
                result["controller"]["kind"] = value
            elif indent == 2 and key in ("defaults", "joints") and not value:
                controller_scope = key
                current_controller_joint = None
                in_plant = False
            elif indent == 4 and controller_scope == "defaults":
                result["controller"]["defaults"][key] = _coerce_value(value)
            elif indent == 6 and controller_scope == "joints" and current_controller_joint is not None:
                if key == "plant" and not value:
                    in_plant = True
                    result["controller"]["joints"][current_controller_joint].setdefault("plant", {})
                    continue
                in_plant = False
                result["controller"]["joints"][current_controller_joint][key] = _coerce_value(value)
            elif indent == 8 and controller_scope == "joints" and in_plant and current_controller_joint is not None:
                result["controller"]["joints"][current_controller_joint].setdefault("plant", {})
                result["controller"]["joints"][current_controller_joint]["plant"][key] = _coerce_value(value)
        elif section == "joint_limits" and current_limit_joint is not None:
            result["joint_limits"][current_limit_joint][key] = _coerce_value(value)

    return result


def _coerce_value(value: str) -> Any:
    """Try to coerce a YAML string value to bool/int/float, falling back to str."""
    v = value.strip()
    if v.lower() in ("true", "yes", "on"):
        return True
    if v.lower() in ("false", "no", "off"):
        return False
    try:
        if "." in v or "e" in v.lower():
            return float(v)
        return int(v)
    except ValueError:
        try:
            return float(v)
        except ValueError:
            return v


# ---------------------------------------------------------------------------
# Metadata extraction from directory / file names
# ---------------------------------------------------------------------------

_TIMESTAMP_RE = re.compile(r"^(\d{8})_(\d{6})")


def extract_timestamp(dirname: str) -> datetime | None:
    m = _TIMESTAMP_RE.match(dirname)
    if not m:
        return None
    try:
        return datetime.strptime(f"{m.group(1)}_{m.group(2)}", "%Y%m%d_%H%M%S")
    except ValueError:
        return None


def extract_metadata_from_dirname(dirname: str) -> dict[str, Any]:
    """Infer experiment metadata from directory name conventions."""
    name = dirname.lower().replace("-", "_")

    meta: dict[str, Any] = {
        "object_type": "mujoco",
        "controller_method": "EID",
    }

    # Controller method — single-controller only
    if name == "pd" or ("_pd" in name and "eid_vs_pd" not in name):
        meta["controller_method"] = "PD"
    elif name == "eid" or "_eid" in name or "mujoco" in name:
        meta["controller_method"] = "EID"
    elif "position_pd" in name:
        meta["controller_method"] = "PD"

    # Disturbance type
    if "random_impulse" in name:
        meta["disturb_type"] = "random_impulse"
    elif "random_walk" in name:
        meta["disturb_type"] = "random_walk"
    elif "sinusoidal" in name:
        meta["disturb_type"] = "sinusoidal"
    elif "step" in name:
        meta["disturb_type"] = "step"
    else:
        meta["disturb_type"] = "none"

    # Legacy: detect combined EID_vs_PD (handled specially in import)
    if "eid_vs_pd" in name or "eid_vs_pid" in name:
        meta["controller_method"] = "EID_vs_PD"
        meta["legacy_combined"] = True

    return meta


# ---------------------------------------------------------------------------
# Database manager class
# ---------------------------------------------------------------------------

class ExperimentDB:
    def __init__(self, db_path: Path = DEFAULT_DB_PATH, data_root: Path = DEFAULT_DATA_ROOT):
        self.db_path = Path(db_path)
        self.data_root = Path(data_root)

    # ---- Connection helpers ----

    def _connect(self) -> sqlite3.Connection:
        conn = sqlite3.connect(str(self.db_path))
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA foreign_keys=ON")
        return conn

    # ---- Init ----

    def init_db(self) -> None:
        """Create all tables if they don't exist, then apply migrations."""
        conn = self._connect()
        try:
            for ddl in TABLE_DEFS:
                conn.execute(ddl)
            conn.commit()
        finally:
            conn.close()
        self._migrate_schema()

    def _migrate_schema(self) -> None:
        """Add columns/tables that may be missing from older databases."""
        conn = self._connect()
        try:
            # Ensure comparison_pairs table exists
            conn.execute(
                """CREATE TABLE IF NOT EXISTS comparison_pairs (
                    pair_id           INTEGER PRIMARY KEY AUTOINCREMENT,
                    eid_experiment_id INTEGER NOT NULL REFERENCES experiments(experiment_id),
                    pd_experiment_id  INTEGER NOT NULL REFERENCES experiments(experiment_id),
                    disturb_type      TEXT DEFAULT 'none',
                    notes             TEXT,
                    created_at        DATETIME DEFAULT CURRENT_TIMESTAMP,
                    UNIQUE(eid_experiment_id, pd_experiment_id)
                )"""
            )
            # Add disturb columns to experiments if missing
            cur = conn.execute("PRAGMA table_info(experiments)")
            cols = {row[1] for row in cur.fetchall()}
            for col, col_type in [
                ("disturb_type", "TEXT DEFAULT 'none'"),
                ("disturb_magnitude", "REAL"),
                ("disturb_frequency", "REAL"),
            ]:
                if col not in cols:
                    conn.execute(f"ALTER TABLE experiments ADD COLUMN {col} {col_type}")
            conn.commit()
        finally:
            conn.close()

    # ---- Import a single run directory ----

    def import_experiment(self, run_dir: Path) -> int | None:
        """Import one run directory. Returns experiment_id or None if skipped."""
        run_dir = Path(run_dir)
        if not run_dir.is_dir():
            print(f"  SKIP: not a directory: {run_dir}")
            return None

        # Detect config file
        config_path = self._find_file(run_dir, ["input_config.yaml", "config.yaml", "config.yml"])
        if config_path is None:
            print(f"  SKIP: no config YAML found in {run_dir}")
            return None

        # Parse config
        try:
            cfg = parse_yaml_config(config_path)
        except Exception as exc:
            print(f"  SKIP: YAML parse error in {config_path}: {exc}")
            return None

        # Build run_id and metadata
        run_id = run_dir.name
        ts = extract_timestamp(run_id)
        inferred = extract_metadata_from_dirname(run_id)

        if ts is None:
            ts = datetime.fromtimestamp(run_dir.stat().st_mtime)

        # Determine controller method from config if available
        controller_method = str(inferred.get("controller_method", "EID"))
        # If this is from compare_eid_pd_mujoco, it's EID_vs_PD
        summary_files = self._find_summary_files(run_dir)

        if "eid_vs_pd_summary" in summary_files:
            controller_method = "EID_vs_PD"

        control_dt = float(cfg.get("control_dt", 0.002))
        duration_s = self._estimate_duration(run_dir, summary_files)

        conn = self._connect()
        try:
            # Insert experiment with disturb metadata
            conn.execute(
                """INSERT OR REPLACE INTO experiments
                   (run_id, timestamp, object_type, controller_method,
                    duration_s, control_dt, config_path, config_snapshot, run_dir, notes,
                    disturb_type, disturb_magnitude, disturb_frequency)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                (
                    run_id,
                    ts.strftime("%Y-%m-%d %H:%M:%S"),
                    inferred.get("object_type", "mujoco"),
                    controller_method,
                    duration_s,
                    control_dt,
                    self._relpath(config_path),
                    json.dumps(self._serialize_config(cfg), ensure_ascii=False),
                    self._relpath(run_dir),
                    inferred.get("notes", ""),
                    inferred.get("disturb_type", "none"),
                    inferred.get("disturb_magnitude"),
                    inferred.get("disturb_frequency"),
                ),
            )
            exp_id = conn.execute("SELECT last_insert_rowid()").fetchone()[0]

            # Insert joint configs
            self._import_joint_configs(conn, exp_id, cfg, run_id)

            # Handle summary files
            if "eid_vs_pd_summary" in summary_files:
                # Legacy combined file: split into two experiments
                eid_exp_id, pd_exp_id = self._import_legacy_comparison(
                    conn, exp_id, run_id, ts, cfg, summary_files["eid_vs_pd_summary"], inferred
                )
                conn.commit()
                return exp_id  # return legacy experiment id

            if "controller_summary" in summary_files:
                self._import_summary_csv(conn, exp_id, summary_files["controller_summary"])

            conn.commit()
            return exp_id
        except Exception:
            conn.rollback()
            raise
        finally:
            conn.close()

    def _import_joint_configs(self, conn: sqlite3.Connection, exp_id: int,
                               cfg: dict, run_id: str) -> None:
        """Extract per-joint controller configs from parsed YAML and insert."""
        controller_cfg = cfg.get("controller", {})
        controller_defaults = controller_cfg.get("defaults", {})
        controller_joints = controller_cfg.get("joints", {})
        joint_limits = cfg.get("joint_limits", {})

        if not controller_joints:
            print(f"  WARNING: no controller.joints section in config for {run_id}")
            return

        for joint_id, jc in controller_joints.items():
            jid = int(joint_id)
            jlim = joint_limits.get(joint_id, {})
            plant = jc.get("plant", {})

            enabled = jc.get("enabled", True)
            if isinstance(enabled, str):
                enabled = enabled.lower() in ("true", "1", "yes", "on")
            enabled_int = 1 if enabled else 0

            name = jc.get("name", JOINT_NAMES.get(jid, f"Joint{jid}"))

            def _get(d: dict, key: str, default: Any = None) -> Any:
                return d.get(key, controller_defaults.get(key, default))

            conn.execute(
                """INSERT OR REPLACE INTO joint_configs
                   (experiment_id, joint_id, joint_name, enabled,
                    kp, kd, observer_gain_q, observer_gain_dq, filter_alpha,
                    policy_interpolation, policy_source,
                    policy_center, policy_amplitude, policy_frequency_hz, policy_phase_rad,
                    policy_step_time_s, startup_blend_duration_s,
                    eid_tau_limit, eid_tau_slew_rate,
                    torque_safe_kp, torque_safe_kd,
                    inverse_q_weight, inverse_dq_weight,
                    plant_Jeff, plant_b, plant_gravityA, plant_gravityB,
                    plant_tau0, plant_q_min, plant_q_max, plant_tau_max)
                   VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
                (
                    exp_id, jid, name, enabled_int,
                    _get(jc, "kp"), _get(jc, "kd"),
                    _get(jc, "observer_gain_q"), _get(jc, "observer_gain_dq"),
                    _get(jc, "filter_alpha"),
                    _get(jc, "policy_interpolation"), _get(jc, "policy_source"),
                    _get(jc, "policy_center"), _get(jc, "policy_amplitude"),
                    _get(jc, "policy_frequency_hz"), _get(jc, "policy_phase_rad"),
                    _get(jc, "policy_step_time_s"),
                    _get(jc, "startup_blend_duration_s"),
                    _get(jc, "tau_limit"), _get(jc, "tau_slew_rate"),
                    _get(jc, "torque_safe_kp"), _get(jc, "torque_safe_kd"),
                    _get(jc, "inverse_q_weight"), _get(jc, "inverse_dq_weight"),
                    plant.get("Jeff"), plant.get("b"),
                    plant.get("gravityA"), plant.get("gravityB"),
                    plant.get("tau0"),
                    plant.get("q_min", jlim.get("q_min")),
                    plant.get("q_max", jlim.get("q_max")),
                    plant.get("tau_max", jlim.get("tau_max")),
                ),
            )

    def _import_summary_csv(self, conn: sqlite3.Connection, exp_id: int,
                             csv_path: Path) -> None:
        """Import a controller summary CSV (generic format) into joint_summaries."""
        with csv_path.open(newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            fieldnames = set(reader.fieldnames or [])
            for row in reader:
                joint_id = int(row.get("joint_id", -1))
                conn.execute(
                    """INSERT OR REPLACE INTO joint_summaries
                       (experiment_id, joint_id,
                        q_rmse, q_max_error, tau_abs_max, tau_mean_abs)
                       VALUES (?,?, ?,?, ?,?)""",
                    (
                        exp_id, joint_id,
                        _float_or_none(row.get("q_rmse") or row.get("rmse")),
                        _float_or_none(row.get("q_max_abs_error") or row.get("max_error")),
                        _float_or_none(row.get("tau_cmd_abs_max") or row.get("max_torque")),
                        _float_or_none(row.get("tau_mean_abs") or row.get("mean_abs_torque")),
                    ),
                )

    def _import_summary_csv_legacy(self, conn: sqlite3.Connection, exp_id: int,
                                    csv_path: Path) -> None:
        """Import a legacy h1_eid_summary.csv (old column names)."""
        self._import_summary_csv(conn, exp_id, csv_path)

    def _import_comparison_csv(self, conn: sqlite3.Connection, exp_id: int,
                                csv_path: Path, inferred: dict) -> None:
        """Legacy: import eid_vs_pd_summary.csv into comparison_results table."""
        with csv_path.open(newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                joint_id = int(row.get("joint_id", -1))
                conn.execute(
                    """INSERT OR REPLACE INTO comparison_results
                       (experiment_id, joint_id,
                        eid_rmse, pd_rmse, rmse_ratio,
                        eid_max_error, pd_max_error,
                        eid_mean_abs_tau, pd_mean_abs_tau,
                        disturb_type, disturb_magnitude, disturb_frequency,
                        pd_kp_used, pd_kd_used)
                       VALUES (?,?, ?,?,?, ?,?, ?,?, ?,?,?, ?,?)""",
                    (
                        exp_id, joint_id,
                        _float_or_none(row.get("eid_rmse")),
                        _float_or_none(row.get("pd_rmse")),
                        _float_or_none(row.get("rmse_ratio")),
                        _float_or_none(row.get("eid_max_error")),
                        _float_or_none(row.get("pd_max_error")),
                        _float_or_none(row.get("eid_mean_abs_tau")),
                        _float_or_none(row.get("pd_mean_abs_tau")),
                        inferred.get("disturb_type", "none"),
                        None,
                        None,
                        _float_or_none(row.get("pd_kp_used")),
                        _float_or_none(row.get("pd_kd_used")),
                    ),
                )

    def _import_legacy_comparison(self, conn: sqlite3.Connection, legacy_exp_id: int,
                                   run_id: str, ts: datetime, cfg: dict,
                                   csv_path: Path, inferred: dict) -> tuple[int, int]:
        """Split legacy eid_vs_pd_summary into two experiments + comparison_pairs."""
        # Import the comparison CSV into comparison_results for backward compat
        self._import_comparison_csv(conn, legacy_exp_id, csv_path, inferred)

        # Mark original experiment as legacy
        conn.execute(
            "UPDATE experiments SET controller_method=?, notes=? WHERE experiment_id=?",
            ("EID_vs_PD (legacy)", f"Legacy combined experiment. Split into EID/PD companions.", legacy_exp_id),
        )

        # Read comparison data for splitting
        eid_data: dict[int, dict[str, float | None]] = {}
        pd_data: dict[int, dict[str, float | None]] = {}
        with csv_path.open(newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                joint_id = int(row.get("joint_id", -1))
                eid_data[joint_id] = {
                    "q_rmse": _float_or_none(row.get("eid_rmse")),
                    "q_max_error": _float_or_none(row.get("eid_max_error")),
                    "tau_mean_abs": _float_or_none(row.get("eid_mean_abs_tau")),
                }
                pd_data[joint_id] = {
                    "q_rmse": _float_or_none(row.get("pd_rmse")),
                    "q_max_error": _float_or_none(row.get("pd_max_error")),
                    "tau_mean_abs": _float_or_none(row.get("pd_mean_abs_tau")),
                }

        config_snapshot = json.dumps(self._serialize_config(cfg), ensure_ascii=False)
        config_rel = self._relpath(self._find_file(Path(run_id) if Path(run_id).is_absolute() else Path("data") / run_id,
                                                    ["input_config.yaml"]) or Path(""))
        run_dir_rel = self._relpath(Path("data") / run_id if not Path(run_id).is_absolute() else Path(run_id))

        # Create EID companion experiment
        eid_run_id = f"{run_id}__EID"
        conn.execute(
            """INSERT OR REPLACE INTO experiments
               (run_id, timestamp, object_type, controller_method,
                duration_s, control_dt, config_path, config_snapshot, run_dir, notes,
                disturb_type)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
            (eid_run_id, ts.strftime("%Y-%m-%d %H:%M:%S"),
             inferred.get("object_type", "mujoco"), "EID",
             None, 0.002, config_rel, config_snapshot, run_dir_rel,
             f"Split from legacy {run_id}", inferred.get("disturb_type", "none")),
        )
        eid_exp_id = conn.execute("SELECT last_insert_rowid()").fetchone()[0]

        # Create PD companion experiment
        pd_run_id = f"{run_id}__PD"
        conn.execute(
            """INSERT OR REPLACE INTO experiments
               (run_id, timestamp, object_type, controller_method,
                duration_s, control_dt, config_path, config_snapshot, run_dir, notes,
                disturb_type)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
            (pd_run_id, ts.strftime("%Y-%m-%d %H:%M:%S"),
             inferred.get("object_type", "mujoco"), "PD",
             None, 0.002, config_rel, config_snapshot, run_dir_rel,
             f"Split from legacy {run_id}", inferred.get("disturb_type", "none")),
        )
        pd_exp_id = conn.execute("SELECT last_insert_rowid()").fetchone()[0]

        # Copy joint_configs to both
        self._import_joint_configs(conn, eid_exp_id, cfg, run_id)
        self._import_joint_configs(conn, pd_exp_id, cfg, run_id)

        # Insert joint_summaries for both
        for joint_id, metrics in eid_data.items():
            conn.execute(
                """INSERT OR REPLACE INTO joint_summaries
                   (experiment_id, joint_id, q_rmse, q_max_error, tau_mean_abs)
                   VALUES (?,?,?,?,?)""",
                (eid_exp_id, joint_id, metrics["q_rmse"], metrics["q_max_error"], metrics["tau_mean_abs"]),
            )
        for joint_id, metrics in pd_data.items():
            conn.execute(
                """INSERT OR REPLACE INTO joint_summaries
                   (experiment_id, joint_id, q_rmse, q_max_error, tau_mean_abs)
                   VALUES (?,?,?,?,?)""",
                (pd_exp_id, joint_id, metrics["q_rmse"], metrics["q_max_error"], metrics["tau_mean_abs"]),
            )

        # Create comparison_pair
        conn.execute(
            """INSERT OR REPLACE INTO comparison_pairs
               (eid_experiment_id, pd_experiment_id, disturb_type, notes)
               VALUES (?,?,?,?)""",
            (eid_exp_id, pd_exp_id, inferred.get("disturb_type", "none"),
             f"Auto-created from legacy {run_id}"),
        )

        print(f"  Legacy split: {run_id} -> EID={eid_exp_id}, PD={pd_exp_id}")
        return eid_exp_id, pd_exp_id

    # ---- Import all runs ----

    def import_all_runs(self) -> int:
        """Scan data_root recursively for run directories and import them."""
        self.init_db()
        run_dirs = self._discover_run_dirs()
        print(f"Found {len(run_dirs)} potential run directories")

        imported = 0
        for run_dir in run_dirs:
            print(f"Importing: {run_dir.name}")
            exp_id = self.import_experiment(run_dir)
            if exp_id is not None:
                imported += 1
                print(f"  -> experiment_id={exp_id}")

        print(f"\nImported {imported}/{len(run_dirs)} runs")
        return imported

    # ---- Rebuild ----

    def rebuild(self) -> int:
        """Drop all tables, recreate, and reimport all runs."""
        conn = self._connect()
        try:
            conn.execute("DROP TABLE IF EXISTS ablation_configs")
            conn.execute("DROP TABLE IF EXISTS comparison_results")
            conn.execute("DROP TABLE IF EXISTS comparison_pairs")
            conn.execute("DROP TABLE IF EXISTS joint_summaries")
            conn.execute("DROP TABLE IF EXISTS joint_configs")
            conn.execute("DROP TABLE IF EXISTS experiments")
            conn.commit()
        finally:
            conn.close()
        print("Dropped all tables. Reinitializing...")
        return self.import_all_runs()

    # ---- Migrate legacy data ----

    def migrate_legacy(self) -> None:
        """Convert all EID_vs_PD experiments into separate EID/PD experiments.

        For each legacy experiment with comparison_results data, creates EID and PD
        companion experiments, populates joint_summaries, and creates comparison_pairs.
        """
        self._migrate_schema()
        conn = self._connect()
        try:
            legacy_exps = conn.execute(
                "SELECT experiment_id, run_id, timestamp, config_snapshot, run_dir, notes "
                "FROM experiments WHERE controller_method = 'EID_vs_PD'"
            ).fetchall()

            if not legacy_exps:
                print("No legacy EID_vs_PD experiments to migrate.")
                return

            print(f"Migrating {len(legacy_exps)} legacy experiments...")

            for exp_row in legacy_exps:
                legacy_id, run_id, ts, config_snapshot, run_dir, notes = exp_row

                # Check if already has companion experiments
                existing = conn.execute(
                    "SELECT pair_id FROM comparison_pairs WHERE "
                    "eid_experiment_id IN (SELECT experiment_id FROM experiments WHERE run_id=?) "
                    "OR pd_experiment_id IN (SELECT experiment_id FROM experiments WHERE run_id=?)",
                    (f"{run_id}__EID", f"{run_id}__PD")
                ).fetchone()
                if existing:
                    print(f"  SKIP {run_id}: already migrated (pair_id={existing[0]})")
                    continue

                # Get comparison results
                comps = conn.execute(
                    "SELECT * FROM comparison_results WHERE experiment_id=?",
                    (legacy_id,)
                ).fetchall()

                if not comps:
                    print(f"  SKIP {run_id}: no comparison_results found")
                    continue

                # Create EID companion
                eid_run_id = f"{run_id}__EID"
                conn.execute(
                    """INSERT OR REPLACE INTO experiments
                       (run_id, timestamp, object_type, controller_method,
                        control_dt, config_path, config_snapshot, run_dir, notes, disturb_type)
                       VALUES (?, ?, 'mujoco', 'EID', 0.002, '', ?, ?, ?, '')""",
                    (eid_run_id, ts, config_snapshot, run_dir or "",
                     f"Migrated from legacy {run_id}"),
                )
                eid_exp_id = conn.execute("SELECT last_insert_rowid()").fetchone()[0]

                # Create PD companion
                pd_run_id = f"{run_id}__PD"
                conn.execute(
                    """INSERT OR REPLACE INTO experiments
                       (run_id, timestamp, object_type, controller_method,
                        control_dt, config_path, config_snapshot, run_dir, notes, disturb_type)
                       VALUES (?, ?, 'mujoco', 'PD', 0.002, '', ?, ?, ?, '')""",
                    (pd_run_id, ts, config_snapshot, run_dir or "",
                     f"Migrated from legacy {run_id}"),
                )
                pd_exp_id = conn.execute("SELECT last_insert_rowid()").fetchone()[0]

                # Copy joint_configs
                configs = conn.execute(
                    "SELECT * FROM joint_configs WHERE experiment_id=?", (legacy_id,)
                ).fetchall()
                col_names = [d[1] for d in conn.execute("PRAGMA table_info(joint_configs)").fetchall()
                            if d[1] != "config_id"]
                placeholders = ",".join(["?"] * len(col_names))
                cols_str = ",".join(col_names)
                for cfg_row in configs:
                    vals = list(cfg_row[2:])  # skip config_id, experiment_id
                    conn.execute(
                        f"INSERT OR REPLACE INTO joint_configs ({cols_str}) VALUES ({placeholders})",
                        [eid_exp_id] + vals,
                    )
                    conn.execute(
                        f"INSERT OR REPLACE INTO joint_configs ({cols_str}) VALUES ({placeholders})",
                        [pd_exp_id] + vals,
                    )

                # Populate joint_summaries from comparison_results
                for comp in comps:
                    comp_dict = {}
                    comp_cols = [d[1] for d in conn.execute("PRAGMA table_info(comparison_results)").fetchall()]
                    for i, col in enumerate(comp_cols):
                        comp_dict[col] = comp[i]

                    joint_id = comp_dict["joint_id"]
                    eid_rmse = comp_dict.get("eid_rmse")
                    pd_rmse = comp_dict.get("pd_rmse")
                    eid_max = comp_dict.get("eid_max_error")
                    pd_max = comp_dict.get("pd_max_error")
                    eid_tau = comp_dict.get("eid_mean_abs_tau")
                    pd_tau = comp_dict.get("pd_mean_abs_tau")
                    disturb_type = comp_dict.get("disturb_type", "none")

                    conn.execute(
                        """INSERT OR REPLACE INTO joint_summaries
                           (experiment_id, joint_id, q_rmse, q_max_error, tau_mean_abs)
                           VALUES (?,?,?,?,?)""",
                        (eid_exp_id, joint_id, eid_rmse, eid_max, eid_tau),
                    )
                    conn.execute(
                        """INSERT OR REPLACE INTO joint_summaries
                           (experiment_id, joint_id, q_rmse, q_max_error, tau_mean_abs)
                           VALUES (?,?,?,?,?)""",
                        (pd_exp_id, joint_id, pd_rmse, pd_max, pd_tau),
                    )

                    # Update disturb_type on both experiments
                    if disturb_type:
                        conn.execute(
                            "UPDATE experiments SET disturb_type=? WHERE experiment_id=?",
                            (disturb_type, eid_exp_id),
                        )
                        conn.execute(
                            "UPDATE experiments SET disturb_type=? WHERE experiment_id=?",
                            (disturb_type, pd_exp_id),
                        )

                # Create comparison_pair; use disturb_type from first comparison row
                first_disturb = comps[0] if comps else None
                comp_cols2 = [d[1] for d in conn.execute("PRAGMA table_info(comparison_results)").fetchall()]
                disturb_col_idx = comp_cols2.index("disturb_type") if "disturb_type" in comp_cols2 else -1
                disturb_val = first_disturb[disturb_col_idx] if first_disturb and disturb_col_idx >= 0 else "none"
                conn.execute(
                    """INSERT OR REPLACE INTO comparison_pairs
                       (eid_experiment_id, pd_experiment_id, disturb_type, notes)
                       VALUES (?,?,?,?)""",
                    (eid_exp_id, pd_exp_id, disturb_val or "none",
                     f"Migrated from legacy {run_id}"),
                )

                # Mark legacy experiment
                conn.execute(
                    "UPDATE experiments SET controller_method=?, notes=? WHERE experiment_id=?",
                    ("EID_vs_PD (legacy)",
                     f"Migrated to EID={eid_exp_id}, PD={pd_exp_id}",
                     legacy_id),
                )

                print(f"  {run_id} -> EID={eid_exp_id}, PD={pd_exp_id}")

            conn.commit()
            print(f"Migration complete.")
        except Exception:
            conn.rollback()
            raise
        finally:
            conn.close()

    # ---- Pair experiments manually ----

    def pair_experiments(self, eid_exp_id: int, pd_exp_id: int,
                          label: str = "", notes: str = "") -> int:
        """Create a comparison_pairs entry linking two experiments."""
        self._migrate_schema()
        conn = self._connect()
        try:
            cur = conn.execute(
                """INSERT OR REPLACE INTO comparison_pairs
                   (eid_experiment_id, pd_experiment_id, disturb_type, notes)
                   VALUES (?,?,?,?)""",
                (eid_exp_id, pd_exp_id, label, notes),
            )
            conn.commit()
            pair_id = cur.lastrowid
            print(f"Pair created: pair_id={pair_id}  EID={eid_exp_id}  PD={pd_exp_id}")
            return pair_id
        finally:
            conn.close()

    # ---- Stats ----

    def print_stats(self) -> None:
        """Print summary statistics about the database."""
        conn = self._connect()
        try:
            n_exps = conn.execute("SELECT COUNT(*) FROM experiments").fetchone()[0]
            n_configs = conn.execute("SELECT COUNT(*) FROM joint_configs").fetchone()[0]
            n_summaries = conn.execute("SELECT COUNT(*) FROM joint_summaries").fetchone()[0]
            n_comparisons = conn.execute("SELECT COUNT(*) FROM comparison_results").fetchone()[0]
            n_ablations = conn.execute("SELECT COUNT(*) FROM ablation_configs").fetchone()[0]
            n_pairs = conn.execute("SELECT COUNT(*) FROM comparison_pairs").fetchone()[0]

            print(f"Database: {self.db_path}")
            print(f"  experiments:        {n_exps}")
            print(f"  joint_configs:      {n_configs}")
            print(f"  joint_summaries:    {n_summaries}")
            print(f"  comparison_results: {n_comparisons}")
            print(f"  comparison_pairs:   {n_pairs}")
            print(f"  ablation_configs:   {n_ablations}")

            if n_exps > 0:
                print("\nBy controller method:")
                for row in conn.execute(
                    "SELECT controller_method, COUNT(*) FROM experiments GROUP BY controller_method"
                ):
                    print(f"  {row[0]:20s} {row[1]}")

                print("\nBy object type:")
                for row in conn.execute(
                    "SELECT object_type, COUNT(*) FROM experiments GROUP BY object_type"
                ):
                    print(f"  {row[0]:20s} {row[1]}")

                print("\nRecent runs:")
                for row in conn.execute(
                    "SELECT run_id, controller_method, timestamp FROM experiments "
                    "ORDER BY timestamp DESC LIMIT 10"
                ):
                    print(f"  {row[0]:50s} {row[1]:15s} {row[2]}")
        finally:
            conn.close()

    # ---- Helpers ----

    def _discover_run_dirs(self) -> list[Path]:
        """Find all directories that look like experiment runs under data_root."""
        run_dirs: list[Path] = []
        for pattern in [
            "mujoco_fit/*/",
            "mujoco_fit/runs/*/",
            "*/runs/*/",
        ]:
            for d in sorted(self.data_root.glob(pattern)):
                if d.is_dir() and d.name != "runs":
                    # Check for minimal required files
                    has_config = any(d.glob("*.yaml")) or any(d.glob("*.yml"))
                    has_csv = any(d.glob("*.csv"))
                    if has_config or has_csv:
                        if d not in run_dirs:
                            run_dirs.append(d)
        return run_dirs

    def _find_file(self, run_dir: Path, candidates: list[str]) -> Path | None:
        for name in candidates:
            p = run_dir / name
            if p.exists():
                return p
        return None

    def _find_summary_files(self, run_dir: Path) -> dict[str, Path]:
        """Map summary type -> path for CSV files in the run directory."""
        found: dict[str, Path] = {}
        for csv_file in sorted(run_dir.glob("*.csv")):
            name = csv_file.name.lower().replace("-", "_")
            if "eid_vs_pd_summary" in name:
                found["eid_vs_pd_summary"] = csv_file
            elif "summary" in name and "timeseries" not in name and "position_pd" not in name:
                found["controller_summary"] = csv_file
        return found

    def _estimate_duration(self, run_dir: Path, summary_files: dict) -> float | None:
        """Estimate experiment duration from available files."""
        # Try reading from summary CSV first (count rows * dt approximation)
        for csv_path in summary_files.values():
            try:
                with csv_path.open(newline="", encoding="utf-8") as f:
                    reader = csv.DictReader(f)
                    if "duration_s" in (reader.fieldnames or []):
                        for row in reader:
                            d = _float_or_none(row.get("duration_s"))
                            if d is not None:
                                return d
            except Exception:
                pass
        return None

    def _relpath(self, path: Path) -> str:
        try:
            return path.resolve().relative_to(self.data_root.resolve().parent).as_posix()
        except ValueError:
            return path.as_posix()

    def _serialize_config(self, cfg: dict) -> dict:
        """Convert parsed config to JSON-serializable form."""
        serialized: dict[str, Any] = {}
        for key, value in cfg.items():
            if key == "joint_limits":
                d: dict[str, Any] = {}
                for k, v in value.items():
                    d[str(k)] = v
                serialized[key] = d
            elif key == "controller" and isinstance(value, dict):
                controller = dict(value)
                if isinstance(controller.get("joints"), dict):
                    controller["joints"] = {str(k): v for k, v in controller["joints"].items()}
                serialized[key] = controller
            elif isinstance(value, dict):
                serialized[key] = value
            elif isinstance(value, (bool, type(None))):
                serialized[key] = value
            else:
                serialized[key] = value
        return serialized


# ---------------------------------------------------------------------------
# Convert raw CSV log -> Parquet
# ---------------------------------------------------------------------------

def csv_to_parquet(csv_path: Path, parquet_path: Path, chunk_size: int = 50000) -> None:
    """Convert a C++ stepper raw CSV log to Parquet tall format.

    The input CSV has columns: cycle, t, dt, lowstate_age, joint_id,
    q, dq, tau_est, q_cmd, dq_cmd, kp_cmd, kd_cmd, tau_cmd, flags,
    debug_0 ... debug_31

    Output Parquet uses tall format: one (cycle, joint_id) row per record.
    """
    try:
        import pandas as pd
    except ImportError:
        print("  WARNING: pandas not available, skipping Parquet conversion")
        return

    parquet_path.parent.mkdir(parents=True, exist_ok=True)

    # Read CSV in chunks and convert
    chunks: list[pd.DataFrame] = []
    total_rows = 0

    for chunk in pd.read_csv(csv_path, chunksize=chunk_size):
        # The raw log has one row per (cycle, joint), so we can use it directly
        # but rename columns to our standard names

        # Map debug_N columns to meaningful names
        renames = {}
        for idx, (col_name, _) in DEBUG_COLUMN_MAP.items():
            old = f"debug_{idx}"
            if old in chunk.columns:
                renames[old] = col_name

        if renames:
            chunk = chunk.rename(columns=renames)

        # Select columns we want to keep
        keep_cols = [
            "cycle", "t", "dt", "lowstate_age", "joint_id",
            "q", "dq", "tau_est", "tau_cmd", "flags",
        ]
        keep_cols += [name for name, _ in DEBUG_COLUMN_MAP.values()]
        keep_cols = [c for c in keep_cols if c in chunk.columns]

        chunks.append(chunk[keep_cols])
        total_rows += len(chunk)

    if not chunks:
        print("  WARNING: no data in CSV")
        return

    df = pd.concat(chunks, ignore_index=True)

    # Downcast numeric columns for efficiency
    for col in df.columns:
        if col in ("cycle", "joint_id", "flags"):
            continue
        if df[col].dtype == "float64":
            df[col] = df[col].astype("float32")

    df.to_parquet(parquet_path, index=False, compression="zstd")
    print(f"  Parquet: {total_rows} rows -> {parquet_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _float_or_none(value: Any) -> float | None:
    if value is None:
        return None
    try:
        v = float(value)
        return v if math.isfinite(v) else None
    except (ValueError, TypeError):
        return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", nargs="?", default="stats",
                        choices=["init", "import", "import-all", "rebuild", "stats",
                                 "migrate", "pair", "csv-to-parquet"],
                        help="Action to perform")
    parser.add_argument("target", nargs="?", default=None,
                        help="Run directory (for import) or CSV path (for csv-to-parquet)")
    parser.add_argument("target2", nargs="?", default=None,
                        help="Second experiment_id (for pair command: pd experiment)")
    parser.add_argument("--db", type=Path, default=DEFAULT_DB_PATH,
                        help=f"SQLite database path (default: {DEFAULT_DB_PATH})")
    parser.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT,
                        help=f"Data root directory (default: {DEFAULT_DATA_ROOT})")
    parser.add_argument("--parquet-out", type=Path, default=None,
                        help="Output path for csv-to-parquet")
    parser.add_argument("--label", type=str, default="",
                        help="Label for pair command (disturb_type)")

    args = parser.parse_args()
    db = ExperimentDB(db_path=args.db, data_root=args.data_root)

    if args.command == "init":
        db.init_db()
        print(f"Database initialized: {args.db}")

    elif args.command == "import":
        if not args.target:
            print("ERROR: 'import' requires a run directory path", file=sys.stderr)
            return 1
        db.init_db()
        run_dir = Path(args.target)
        exp_id = db.import_experiment(run_dir)
        if exp_id is not None:
            print(f"Imported: {run_dir.name} -> experiment_id={exp_id}")

    elif args.command == "import-all":
        db.import_all_runs()

    elif args.command == "rebuild":
        db.rebuild()

    elif args.command == "stats":
        db.print_stats()

    elif args.command == "migrate":
        db.migrate_legacy()

    elif args.command == "pair":
        if not args.target or not args.target2:
            print("ERROR: 'pair' requires two experiment IDs: pair <eid_exp_id> <pd_exp_id>",
                  file=sys.stderr)
            return 1
        db.pair_experiments(int(args.target), int(args.target2),
                             label=args.label)

    elif args.command == "csv-to-parquet":
        if not args.target:
            print("ERROR: 'csv-to-parquet' requires a CSV file path", file=sys.stderr)
            return 1
        csv_path = Path(args.target)
        if args.parquet_out:
            parquet_path = args.parquet_out
        else:
            parquet_path = csv_path.with_suffix(".parquet")
        csv_to_parquet(csv_path, parquet_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
