#!/usr/bin/env python3
"""Build a searchable index for H1 joint-control experiment logs."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import re
from pathlib import Path


RAW_LOG_COLUMNS = {
    "cycle",
    "t",
    "dt",
    "lowstate_age",
    "joint_id",
    "q",
    "dq",
    "tau_est",
    "q_cmd",
    "dq_cmd",
    "kp_cmd",
    "kd_cmd",
    "tau_cmd",
    "flags",
}

JOINT_NAMES = {
    0: "RightHipRoll",
    1: "RightHipPitch",
    2: "RightKnee",
    3: "LeftHipRoll",
    4: "LeftHipPitch",
    5: "LeftKnee",
    6: "WaistYaw",
    7: "LeftHipYaw",
    8: "RightHipYaw",
    9: "NotUsedJoint",
    10: "LeftAnkle",
    11: "RightAnkle",
    12: "RightShoulderPitch",
    13: "RightShoulderRoll",
    14: "RightShoulderYaw",
    15: "RightElbow",
    16: "LeftShoulderPitch",
    17: "LeftShoulderRoll",
    18: "LeftShoulderYaw",
    19: "LeftElbow",
}

INDEX_COLUMNS = [
    "run_id",
    "timestamp",
    "run_dir",
    "log_path",
    "run_yaml_path",
    "config_path",
    "object_type",
    "robot",
    "joint_id",
    "joint_name",
    "controller_method",
    "policy_interpolation",
    "interpolation",
    "trajectory",
    "kp",
    "kd",
    "policy_center",
    "policy_amplitude",
    "policy_frequency_hz",
    "samples",
    "duration_s",
    "q_error_rmse",
    "q_error_reference",
    "q_min",
    "q_max",
    "q_ref_min",
    "q_ref_max",
    "tau_cmd_abs_max",
    "lowstate_age_max",
    "flags",
    "file_size_bytes",
    "modified_time",
    "notes",
]


def finite_float(value: object, default: float = math.nan) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def maybe_int(value: object) -> int | None:
    number = finite_float(value)
    if not math.isfinite(number):
        return None
    return int(number)


def relpath(path: Path, root: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return path.as_posix()


def detect_raw_log(path: Path) -> bool:
    if path.name == "runs_index.csv" or path.name.endswith("_clean.csv"):
        return False
    try:
        with path.open(newline="") as f:
            reader = csv.reader(f)
            header = next(reader, [])
    except (OSError, UnicodeDecodeError):
        return False
    return RAW_LOG_COLUMNS.issubset(set(header))


def parse_timestamp_from_dir(path: Path) -> str:
    match = re.search(r"(\d{8})_(\d{6})", path.name)
    if not match:
        return ""
    try:
        parsed = dt.datetime.strptime("_".join(match.groups()), "%Y%m%d_%H%M%S")
    except ValueError:
        return ""
    return parsed.isoformat(sep=" ")


def parse_simple_yaml(path: Path) -> dict[str, str]:
    """Parse the small key/value YAML subset used by this repo.

    This is intentionally conservative. It supports top-level keys and one level
    of nested maps such as controller.policy_interpolation or target.joint_id.
    """
    if not path.exists():
        return {}

    values: dict[str, str] = {}
    section = ""
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except UnicodeDecodeError:
        lines = path.read_text(errors="ignore").splitlines()

    for raw_line in lines:
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip() or ":" not in line:
            continue
        indent = len(line) - len(line.lstrip(" "))
        key, value = line.strip().split(":", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if indent == 0 and not value:
            section = key
            continue
        if indent == 0:
            section = ""
            values[key] = value
        elif section and value:
            values[f"{section}.{key}"] = value
    return values


def find_sidecar(run_dir: Path, names: list[str]) -> Path | None:
    for name in names:
        candidate = run_dir / name
        if candidate.exists():
            return candidate
    return None


def log_key(log_path: Path) -> str:
    key = log_path.stem.lower().replace("-", "_")
    for prefix in ["h1_mock_", "h1_", "mock_"]:
        if key.startswith(prefix):
            key = key[len(prefix):]
    for suffix in ["_log", "_closed", "_open"]:
        if key.endswith(suffix):
            key = key[: -len(suffix)]
    key = re.sub(r"_tau_0p\d+$", "", key)
    if key == "open":
        return "open_loop"
    if key == "closed":
        return "closed_loop"
    return key


def find_config_for_log(data_root: Path, log_path: Path) -> Path | None:
    run_dir = log_path.parent
    sidecar = find_sidecar(run_dir, ["config.yaml", "config.yml"])
    if sidecar:
        return sidecar

    repo_root = data_root.parent
    key = log_key(log_path)
    candidates = [
        repo_root / "config" / f"{key}.yaml",
    ]

    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def infer_from_text(log_path: Path) -> dict[str, str]:
    text = f"{log_path.parent.name} {log_path.stem}".lower().replace("-", "_")
    inferred = {
        "object_type": "unspecified",
        "controller_method": "EID",
        "policy_interpolation": "unspecified",
        "interpolation": "unspecified",
        "trajectory": "sine",
    }

    if "mock" in text or "mujoco" in text or "virtual" in text:
        inferred["object_type"] = "mock"
    elif "real" in text or "direct" in text or "h1_direct" in text:
        inferred["object_type"] = "real"

    if "pid" in text:
        inferred["controller_method"] = "PID"
    elif re.search(r"(^|_)pd($|_)", text):
        inferred["controller_method"] = "PD"
    elif "eid" in text or "mock" in text:
        inferred["controller_method"] = "EID"

    if "closed_loop" in text or re.search(r"(^|_)closed($|_)", text):
        inferred["policy_interpolation"] = "closed_loop"
    elif "open_loop" in text or re.search(r"(^|_)open($|_)", text):
        inferred["policy_interpolation"] = "open_loop"

    if "quintic" in text:
        inferred["interpolation"] = "quintic"

    if "sine" in text:
        inferred["trajectory"] = "sine"

    return inferred


def summarize_log(path: Path) -> dict[str, object]:
    samples = 0
    t0 = math.nan
    t_last = math.nan
    joint_id: int | None = None
    flags: set[int] = set()
    q_error_shaped_sse = 0.0
    q_error_shaped_count = 0
    q_error_raw_sse = 0.0
    q_error_raw_count = 0
    q_min = math.inf
    q_max = -math.inf
    q_ref_shaped_min = math.inf
    q_ref_shaped_max = -math.inf
    q_ref_raw_min = math.inf
    q_ref_raw_max = -math.inf
    tau_abs_max = 0.0
    lowstate_age_max = 0.0

    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = finite_float(row.get("t"))
            q = finite_float(row.get("q"))
            q_ref = finite_float(row.get("debug_0"))
            q_ref_raw = finite_float(row.get("debug_26"))
            tau_cmd = finite_float(row.get("tau_cmd"))
            lowstate_age = finite_float(row.get("lowstate_age"))
            flag = maybe_int(row.get("flags"))

            if samples == 0:
                t0 = t
                joint_id = maybe_int(row.get("joint_id"))
            if math.isfinite(t):
                t_last = t
            if math.isfinite(q):
                q_min = min(q_min, q)
                q_max = max(q_max, q)
            if math.isfinite(q_ref):
                q_ref_shaped_min = min(q_ref_shaped_min, q_ref)
                q_ref_shaped_max = max(q_ref_shaped_max, q_ref)
            if math.isfinite(q_ref_raw):
                q_ref_raw_min = min(q_ref_raw_min, q_ref_raw)
                q_ref_raw_max = max(q_ref_raw_max, q_ref_raw)
            q_error_shaped = math.nan
            if math.isfinite(q) and math.isfinite(q_ref):
                q_error_shaped = q_ref - q
            if math.isfinite(q_error_shaped):
                q_error_shaped_sse += q_error_shaped * q_error_shaped
                q_error_shaped_count += 1
            q_error_raw = math.nan
            if math.isfinite(q) and math.isfinite(q_ref_raw):
                q_error_raw = q_ref_raw - q
            if math.isfinite(q_error_raw):
                q_error_raw_sse += q_error_raw * q_error_raw
                q_error_raw_count += 1
            if math.isfinite(tau_cmd):
                tau_abs_max = max(tau_abs_max, abs(tau_cmd))
            if math.isfinite(lowstate_age):
                lowstate_age_max = max(lowstate_age_max, lowstate_age)
            if flag is not None:
                flags.add(flag)
            samples += 1

    duration = t_last - t0 if math.isfinite(t0) and math.isfinite(t_last) else math.nan
    shaped_rmse = (
        math.sqrt(q_error_shaped_sse / q_error_shaped_count)
        if q_error_shaped_count
        else math.nan
    )
    raw_rmse = math.sqrt(q_error_raw_sse / q_error_raw_count) if q_error_raw_count else math.nan
    use_raw_error = (
        q_error_raw_count > 0
        and math.isfinite(shaped_rmse)
        and math.isfinite(raw_rmse)
        and shaped_rmse < 1.0e-9
        and raw_rmse > 1.0e-9
    )
    rmse = raw_rmse if use_raw_error else shaped_rmse
    q_ref_min = q_ref_raw_min if use_raw_error else q_ref_shaped_min
    q_ref_max = q_ref_raw_max if use_raw_error else q_ref_shaped_max

    def clean(value: float) -> float:
        return value if math.isfinite(value) else math.nan

    return {
        "joint_id": joint_id,
        "samples": samples,
        "duration_s": duration,
        "q_error_rmse": rmse,
        "q_error_reference": "raw" if use_raw_error else "shaped",
        "q_error_shaped_rmse": clean(shaped_rmse),
        "q_error_raw_rmse": clean(raw_rmse),
        "q_min": clean(q_min),
        "q_max": clean(q_max),
        "q_ref_min": clean(q_ref_min),
        "q_ref_max": clean(q_ref_max),
        "tau_cmd_abs_max": tau_abs_max,
        "lowstate_age_max": lowstate_age_max,
        "flags": "|".join(str(flag) for flag in sorted(flags)),
    }


def choose_run_id(log_path: Path, raw_logs_in_dir: dict[Path, int]) -> str:
    base = log_path.parent.name
    if raw_logs_in_dir.get(log_path.parent, 0) > 1:
        return f"{base}__{log_path.stem}"
    return base


def build_index(data_root: Path) -> list[dict[str, object]]:
    csv_paths = sorted(data_root.rglob("*.csv"))
    raw_logs = [path for path in csv_paths if detect_raw_log(path)]
    raw_count_by_dir: dict[Path, int] = {}
    for path in raw_logs:
        raw_count_by_dir[path.parent] = raw_count_by_dir.get(path.parent, 0) + 1

    rows: list[dict[str, object]] = []
    for log_path in raw_logs:
        run_dir = log_path.parent
        run_yaml = find_sidecar(run_dir, ["run.yaml", "run.yml"])
        config_yaml = find_config_for_log(data_root, log_path)
        run_meta = parse_simple_yaml(run_yaml) if run_yaml else {}
        config_meta = parse_simple_yaml(config_yaml) if config_yaml else {}
        inferred = infer_from_text(log_path)
        summary = summarize_log(log_path)

        cfg_policy_interpolation = config_meta.get("controller.policy_interpolation", "")
        cfg_interpolation = ""
        if cfg_policy_interpolation in {"open_loop", "closed_loop"}:
            cfg_interpolation = "quintic"

        data_policy_interpolation = ""
        shaped_rmse = finite_float(summary.get("q_error_shaped_rmse"))
        if math.isfinite(shaped_rmse):
            data_policy_interpolation = "closed_loop" if shaped_rmse < 1.0e-9 else "open_loop"

        joint_id = maybe_int(run_meta.get("target.joint_id"))
        if joint_id is None:
            joint_id = maybe_int(config_meta.get("controller.target_joint"))
        if joint_id is None:
            joint_id = summary.get("joint_id")
        joint_name = run_meta.get("target.joint_name") or (
            JOINT_NAMES.get(joint_id, "") if isinstance(joint_id, int) else ""
        )

        stat = log_path.stat()
        row = {
            "run_id": run_meta.get("run_id") or choose_run_id(log_path, raw_count_by_dir),
            "timestamp": run_meta.get("time") or parse_timestamp_from_dir(run_dir),
            "run_dir": relpath(run_dir, data_root.parent),
            "log_path": relpath(log_path, data_root.parent),
            "run_yaml_path": relpath(run_yaml, data_root.parent) if run_yaml else "",
            "config_path": relpath(config_yaml, data_root.parent) if config_yaml else "",
            "object_type": run_meta.get("target.object_type") or inferred["object_type"],
            "robot": run_meta.get("target.robot") or config_meta.get("robot", ""),
            "joint_id": joint_id if joint_id is not None else "",
            "joint_name": joint_name,
            "controller_method": run_meta.get("controller.method") or inferred["controller_method"],
            "policy_interpolation": (
                run_meta.get("reference.mode")
                or cfg_policy_interpolation
                or (inferred["policy_interpolation"] if inferred["policy_interpolation"] != "unspecified" else "")
                or data_policy_interpolation
                or "unspecified"
            ),
            "interpolation": (
                run_meta.get("reference.interpolation")
                or (inferred["interpolation"] if inferred["interpolation"] != "unspecified" else "")
                or cfg_interpolation
                or "quintic"
            ),
            "trajectory": run_meta.get("reference.trajectory") or inferred["trajectory"],
            "kp": run_meta.get("controller.kp") or config_meta.get("controller.kp", ""),
            "kd": run_meta.get("controller.kd") or config_meta.get("controller.kd", ""),
            "policy_center": run_meta.get("reference.center") or config_meta.get("controller.policy_center", ""),
            "policy_amplitude": run_meta.get("reference.amplitude")
            or config_meta.get("controller.policy_amplitude", ""),
            "policy_frequency_hz": run_meta.get("reference.frequency")
            or config_meta.get("controller.policy_frequency_hz", ""),
            "file_size_bytes": stat.st_size,
            "modified_time": dt.datetime.fromtimestamp(stat.st_mtime).isoformat(sep=" "),
            "notes": run_meta.get("notes", ""),
        }
        row.update(summary)
        rows.append(row)
    return rows


def write_index(rows: list[dict[str, object]], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=INDEX_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def main() -> int:
    parser = argparse.ArgumentParser(description="Index H1 experiment CSV logs.")
    parser.add_argument("--data-root", default="data", type=Path)
    parser.add_argument("--out", default=Path("data") / "runs_index.csv", type=Path)
    args = parser.parse_args()

    rows = build_index(args.data_root)
    write_index(rows, args.out)
    print(f"indexed_runs={len(rows)}")
    print(f"index={args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
