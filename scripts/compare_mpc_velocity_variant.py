#!/usr/bin/env python3
"""Compare the 3-ref Preview-MPC and the 4-ref velocity-target Preview-MPC."""

from __future__ import annotations

import argparse
import csv
import math
import re
import shutil
import subprocess
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import numpy as np


KEY_VALUE_RE = re.compile(r"^\s+([A-Za-z0-9_]+):\s*([^#]*?)(?:\s*#.*)?$")
JOINT_HEADER_RE = re.compile(r"^\s+([0-9]+):\s*(?:#.*)?$")


@dataclass(frozen=True)
class Case:
    label: str
    interpolation: str
    reference_points: int
    condition: str


@dataclass(frozen=True)
class JointPolicy:
    joint_id: int
    source: str
    center: float
    amplitude: float
    frequency_hz: float
    phase_rad: float
    step_time_s: float
    policy_dt: float


CASES = [
    Case(
        label="preview_mpc_3ref",
        interpolation="preview_mpc",
        reference_points=3,
        condition="P3_PD_selected_mpc_3ref_same_phase",
    ),
    Case(
        label="preview_mpc_velocity_4ref",
        interpolation="preview_mpc_velocity",
        reference_points=4,
        condition="P3_PD_velocity_mpc_4ref_same_phase",
    ),
]


def repo_rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(Path.cwd().resolve())).replace("\\", "/")
    except ValueError:
        return str(path)


def replace_scalar(text: str, key: str, value: str) -> str:
    pattern = re.compile(rf"^(\s*{re.escape(key)}:\s*).*$", re.MULTILINE)
    if not pattern.search(text):
        raise RuntimeError(f"Cannot find key {key!r} in baseline config")
    return pattern.sub(rf"\g<1>{value}", text, count=1)


def replace_all_scalars(text: str, key: str, value: str) -> str:
    pattern = re.compile(rf"^(\s*{re.escape(key)}:\s*).*$", re.MULTILINE)
    if not pattern.search(text):
        raise RuntimeError(f"Cannot find key {key!r} in baseline config")
    return pattern.sub(rf"\g<1>{value}", text)


def write_case_configs(baseline_config: Path, out_dir: Path) -> dict[str, Path]:
    config_dir = out_dir / "configs"
    config_dir.mkdir(parents=True, exist_ok=True)
    base_text = baseline_config.read_text(encoding="utf-8")
    result: dict[str, Path] = {}
    for case in CASES:
        text = base_text
        text = replace_scalar(text, "condition", case.condition)
        text = replace_scalar(text, "log_path", f"data/{case.label}_log.csv")
        text = replace_all_scalars(text, "policy_interpolation", case.interpolation)
        text = replace_all_scalars(text, "policy_reference_points", str(case.reference_points))
        path = config_dir / f"{case.label}.yaml"
        path.write_text(text, encoding="utf-8")
        result[case.label] = path
    return result


def parse_config_references(config: Path) -> dict[int, JointPolicy]:
    section: str | None = None
    controller_scope: str | None = None
    current_joint: int | None = None
    defaults: dict[str, str] = {}
    joints: dict[int, dict[str, str]] = {}
    keys = {
        "policy_source",
        "policy_center",
        "policy_amplitude",
        "policy_frequency_hz",
        "policy_phase_rad",
        "policy_step_time_s",
        "policy_dt",
    }
    for raw_line in config.read_text(encoding="utf-8").splitlines():
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if not raw_line.startswith(" "):
            section = stripped.rstrip(":") if stripped.endswith(":") else None
            controller_scope = None
            current_joint = None
            continue
        if section != "controller":
            continue
        indent = len(raw_line) - len(raw_line.lstrip(" "))
        match = KEY_VALUE_RE.match(raw_line)
        if indent == 2 and match and match.group(1) in {"defaults", "joints"}:
            controller_scope = match.group(1)
            current_joint = None
            continue
        if controller_scope == "defaults" and indent == 4 and match and match.group(1) in keys:
            defaults[match.group(1)] = match.group(2).strip()
            continue
        if controller_scope == "joints" and indent == 4:
            header = JOINT_HEADER_RE.match(raw_line)
            if header:
                current_joint = int(header.group(1))
                joints.setdefault(current_joint, {})
            continue
        if controller_scope == "joints" and current_joint is not None and indent == 6 and match:
            if match.group(1) in keys:
                joints[current_joint][match.group(1)] = match.group(2).strip()

    policies: dict[int, JointPolicy] = {}
    for joint_id, overrides in sorted(joints.items()):
        merged = dict(defaults)
        merged.update(overrides)
        policies[joint_id] = JointPolicy(
            joint_id=joint_id,
            source=merged.get("policy_source", "sine").lower(),
            center=float(merged["policy_center"]),
            amplitude=float(merged["policy_amplitude"]),
            frequency_hz=float(merged.get("policy_frequency_hz", "0.05")),
            phase_rad=float(merged.get("policy_phase_rad", "0.0")),
            step_time_s=float(merged.get("policy_step_time_s", "1.0")),
            policy_dt=float(merged.get("policy_dt", "0.05")),
        )
    if not policies:
        raise RuntimeError(f"{config}: no joint policy settings found")
    return policies


def policy_position(policy: JointPolicy, t: np.ndarray | float) -> np.ndarray | float:
    if policy.source == "hold":
        return policy.center + np.zeros_like(t, dtype=float) if isinstance(t, np.ndarray) else policy.center
    if policy.source == "step":
        return np.where(np.asarray(t) < policy.step_time_s, policy.center, policy.center + policy.amplitude)
    omega = 2.0 * math.pi * policy.frequency_hz
    return policy.center + policy.amplitude * np.sin(omega * np.asarray(t) + policy.phase_rad)


def finite_rms(values: np.ndarray) -> float:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return float(math.sqrt(np.mean(values * values))) if values.size else float("nan")


def escape_svg(text: object) -> str:
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def svg_text(x: float, y: float, text: object, size: int = 12, anchor: str = "middle",
             weight: str = "400", rotate: float | None = None) -> str:
    transform = f' transform="rotate({rotate:.1f} {x:.1f} {y:.1f})"' if rotate is not None else ""
    return (
        f'<text x="{x:.1f}" y="{y:.1f}" font-family="Arial, sans-serif" '
        f'font-size="{size}" text-anchor="{anchor}" font-weight="{weight}"{transform}>'
        f'{escape_svg(text)}</text>'
    )


def svg_line(x1: float, y1: float, x2: float, y2: float, color: str = "#333333",
             width: float = 1.0, dash: str | None = None) -> str:
    dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
    return (
        f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
        f'stroke="{color}" stroke-width="{width:.2f}"{dash_attr}/>'
    )


def svg_rect(x: float, y: float, width: float, height: float, fill: str,
             stroke: str = "none") -> str:
    return (
        f'<rect x="{x:.1f}" y="{y:.1f}" width="{max(0.0, width):.1f}" '
        f'height="{max(0.0, height):.1f}" fill="{fill}" stroke="{stroke}"/>'
    )


def svg_circle(x: float, y: float, radius: float, stroke: str, fill: str = "#ffffff",
               width: float = 1.8) -> str:
    return (
        f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{radius:.1f}" '
        f'fill="{fill}" stroke="{stroke}" stroke-width="{width:.2f}"/>'
    )


def svg_diamond(x: float, y: float, radius: float, stroke: str, fill: str = "#ffffff",
                width: float = 1.8) -> str:
    points = [
        (x, y - radius),
        (x + radius, y),
        (x, y + radius),
        (x - radius, y),
    ]
    point_text = " ".join(f"{px:.1f},{py:.1f}" for px, py in points)
    return (
        f'<polygon points="{point_text}" fill="{fill}" stroke="{stroke}" '
        f'stroke-width="{width:.2f}"/>'
    )


def svg_path(points: list[tuple[float, float]], color: str, width: float = 1.4,
             dash: str | None = None, opacity: float | None = None) -> str:
    if not points:
        return ""
    command = "M " + " L ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
    opacity_attr = f' opacity="{opacity:.2f}"' if opacity is not None else ""
    return (
        f'<path d="{command}" fill="none" stroke="{color}" '
        f'stroke-width="{width:.2f}"{dash_attr}{opacity_attr}/>'
    )


def scale_points(t: np.ndarray, y: np.ndarray, rect: tuple[float, float, float, float],
                 xlim: tuple[float, float], ylim: tuple[float, float],
                 max_points: int = 700) -> list[tuple[float, float]]:
    x0, y0, width, height = rect
    xmin, xmax = xlim
    ymin, ymax = ylim
    if xmax <= xmin:
        xmax = xmin + 1.0
    if ymax <= ymin:
        pad = max(1.0e-6, abs(ymin) * 0.05)
        ymin -= pad
        ymax += pad
    step = max(1, int(math.ceil(len(t) / max_points)))
    points: list[tuple[float, float]] = []
    for tx, yy in zip(t[::step], y[::step]):
        if not np.isfinite(tx) or not np.isfinite(yy):
            continue
        px = x0 + width * (float(tx) - xmin) / (xmax - xmin)
        py = y0 + height * (1.0 - (float(yy) - ymin) / (ymax - ymin))
        points.append((px, py))
    return points


def scale_xy(tx: float, yy: float, rect: tuple[float, float, float, float],
             xlim: tuple[float, float], ylim: tuple[float, float]) -> tuple[float, float]:
    x0, y0, width, height = rect
    xmin, xmax = xlim
    ymin, ymax = ylim
    if xmax <= xmin:
        xmax = xmin + 1.0
    if ymax <= ymin:
        pad = max(1.0e-6, abs(ymin) * 0.05)
        ymin -= pad
        ymax += pad
    px = x0 + width * (float(tx) - xmin) / (xmax - xmin)
    py = y0 + height * (1.0 - (float(yy) - ymin) / (ymax - ymin))
    return px, py


def spike_indices(y: np.ndarray, count: int = 3, min_spacing: int = 35) -> list[int]:
    """Return local high-frequency spike indices ranked by second-difference size."""
    values = np.asarray(y, dtype=float)
    if len(values) < 3:
        return []
    score = np.full(len(values), np.nan)
    score[1:-1] = np.abs(values[1:-1] - 0.5 * (values[:-2] + values[2:]))
    order = np.argsort(np.nan_to_num(score, nan=-1.0))[::-1]
    selected: list[int] = []
    for idx in order:
        if not np.isfinite(score[idx]) or score[idx] <= 0.0:
            continue
        if all(abs(int(idx) - prev) >= min_spacing for prev in selected):
            selected.append(int(idx))
            if len(selected) >= count:
                break
    return sorted(selected)


def write_svg(path: Path, width: int, height: int, elements: list[str]) -> Path:
    path.write_text(
        "\n".join([
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
            f'viewBox="0 0 {width} {height}">',
            '<rect width="100%" height="100%" fill="#ffffff"/>',
            *elements,
            "</svg>",
        ]),
        encoding="utf-8",
    )
    return path


def derivative_rms(t: np.ndarray, y: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    if len(t) < 2:
        return float("nan"), np.array([]), np.array([])
    dy = np.diff(y)
    dt = np.diff(t)
    valid = np.abs(dt) > 1.0e-12
    deriv = dy[valid] / dt[valid]
    return finite_rms(deriv), t[1:][valid], deriv


def read_log_rows(log_path: Path) -> list[dict[str, float | int]]:
    rows: list[dict[str, float | int]] = []
    with log_path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fieldnames = set(reader.fieldnames or [])
        for raw in reader:
            joint_id = int(float(raw["joint_id"]))
            if {"q_ref_shaped", "dq_ref_shaped", "q_actual", "dq_actual", "tau_applied"} <= fieldnames:
                row: dict[str, float | int] = {
                    "t": float(raw["t"]),
                    "joint_id": joint_id,
                    "q_ref_shaped": float(raw["q_ref_shaped"]),
                    "dq_ref_shaped": float(raw["dq_ref_shaped"]),
                    "q_actual": float(raw["q_actual"]),
                    "dq_actual": float(raw["dq_actual"]),
                    "tau_applied": float(raw["tau_applied"]),
                }
            elif {"q", "dq", "q_cmd", "dq_cmd", "kp_cmd", "kd_cmd", "tau_cmd", "debug_0", "debug_1"} <= fieldnames:
                q = float(raw["q"])
                dq = float(raw["dq"])
                q_cmd = float(raw["q_cmd"])
                dq_cmd = float(raw["dq_cmd"])
                kp = float(raw["kp_cmd"])
                kd = float(raw["kd_cmd"])
                tau_cmd = float(raw["tau_cmd"])
                row = {
                    "t": float(raw["t"]),
                    "joint_id": joint_id,
                    "q_ref_shaped": float(raw["debug_0"]),
                    "dq_ref_shaped": float(raw["debug_1"]),
                    "q_actual": q,
                    "dq_actual": dq,
                    "tau_applied": kp * (q_cmd - q) + kd * (dq_cmd - dq) + tau_cmd,
                }
            else:
                missing = "q_ref_shaped/dq_ref_shaped or debug_0/debug_1"
                raise RuntimeError(f"{log_path}: unsupported log schema, missing {missing}")
            rows.append(row)
    return rows


def write_csv_rows(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def summarize_log(label: str, config: Path, log_path: Path, warmup_s: float) -> list[dict[str, object]]:
    policies = parse_config_references(config)
    data = [row for row in read_log_rows(log_path) if float(row["t"]) >= warmup_s]
    if not data:
        raise RuntimeError(f"{log_path}: no samples after warmup_s={warmup_s}")

    by_joint: dict[int, list[dict[str, float | int]]] = defaultdict(list)
    for row in data:
        by_joint[int(row["joint_id"])].append(row)

    rows: list[dict[str, object]] = []
    for joint_id in sorted(by_joint):
        policy = policies[joint_id]
        joint_rows = sorted(by_joint[joint_id], key=lambda r: float(r["t"]))
        t = np.asarray([float(row["t"]) for row in joint_rows], dtype=float)
        q_ref = np.asarray([float(row["q_ref_shaped"]) for row in joint_rows], dtype=float)
        dq_ref = np.asarray([float(row["dq_ref_shaped"]) for row in joint_rows], dtype=float)
        q_actual = np.asarray([float(row["q_actual"]) for row in joint_rows], dtype=float)
        dq_actual = np.asarray([float(row["dq_actual"]) for row in joint_rows], dtype=float)
        tau = np.asarray([float(row["tau_applied"]) for row in joint_rows], dtype=float)

        ref_ddq_rms, ddq_t, ref_ddq = derivative_rms(t, dq_ref)
        ref_jerk_rms, _, _ = derivative_rms(ddq_t, ref_ddq)
        tau_step_rms, _, tau_rate = derivative_rms(t, tau)

        node_index = np.rint(t / policy.policy_dt)
        node_t = node_index * policy.policy_dt
        median_dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.002
        is_node = (node_index >= 1.0) & (np.abs(t - node_t) <= max(1.0e-9, 0.55 * median_dt))
        policy_time = node_t[is_node] - policy.policy_dt
        if policy_time.size:
            q_target = np.asarray(policy_position(policy, policy_time), dtype=float)
            dq_target = (
                np.asarray(policy_position(policy, policy_time + policy.policy_dt), dtype=float) -
                q_target
            ) / policy.policy_dt
            node_q_error_rms = finite_rms(q_ref[is_node] - q_target)
            node_dq_error_rms = finite_rms(dq_ref[is_node] - dq_target)
        else:
            node_q_error_rms = float("nan")
            node_dq_error_rms = float("nan")

        q_error = q_ref - q_actual
        dq_error = dq_ref - dq_actual
        rows.append(
            {
                "method": label,
                "joint_id": joint_id,
                "samples": len(joint_rows),
                "q_rmse": finite_rms(q_error),
                "q_max_abs_error": float(np.max(np.abs(q_error))) if len(q_error) else float("nan"),
                "dq_rmse": finite_rms(dq_error),
                "dq_max_abs_error": float(np.max(np.abs(dq_error))) if len(dq_error) else float("nan"),
                "ref_dq_rms": finite_rms(dq_ref),
                "ref_ddq_rms": ref_ddq_rms,
                "ref_jerk_rms": ref_jerk_rms,
                "policy_node_q_error_rms": node_q_error_rms,
                "policy_node_dq_error_rms": node_dq_error_rms,
                "tau_applied_rms": finite_rms(tau),
                "tau_applied_abs_max": float(np.max(np.abs(tau))) if len(tau) else float("nan"),
                "tau_rate_rms": finite_rms(tau_rate),
            }
        )
    return rows


def run_mujoco_case(args: argparse.Namespace, label: str, config: Path, out_dir: Path) -> None:
    cmd = [
        sys.executable,
        str(args.run_mujoco),
        "--scene",
        str(args.scene),
        "--config",
        str(config),
        "--stepper",
        str(args.stepper),
        "--out-dir",
        str(out_dir),
        "--duration",
        str(args.duration),
        "--dt",
        str(args.dt),
        "--log-every-step",
        "--export-summary",
    ]
    print(f"[{label}] {' '.join(cmd)}")
    subprocess.run(cmd, check=True)


def run_mock_case(args: argparse.Namespace, label: str, config: Path, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [str(args.mock_runner), str(config), str(args.duration)]
    print(f"[{label}] {' '.join(cmd)}")
    result = subprocess.run(cmd, check=True, text=True, capture_output=True)
    print(result.stdout)
    log_path: Path | None = None
    for line in result.stdout.splitlines():
        if "log_path=" in line:
            log_path = Path(line.split("log_path=", 1)[1].strip())
            break
    if log_path is None or not log_path.exists():
        raise RuntimeError(f"mock runner did not report a valid log_path for {label}")
    target_log = out_dir / "mujoco_closed_loop_log.csv"
    shutil.copyfile(log_path, target_log)
    shutil.copyfile(config, out_dir / "input_config.yaml")
    (out_dir / "mujoco_closed_loop_manifest.txt").write_text(
        "\n".join([
            "C++ mock closed-loop fallback",
            f"config={config}",
            f"mock_runner={args.mock_runner}",
            f"source_log={log_path}",
            f"csv={target_log}",
            f"duration={args.duration}",
            f"dt={args.dt}",
        ]) + "\n",
        encoding="utf-8",
    )


def aggregate_metrics(metrics: list[dict[str, object]]) -> list[dict[str, object]]:
    by_method: dict[str, list[dict[str, object]]] = defaultdict(list)
    for row in metrics:
        by_method[str(row["method"])].append(row)
    aggregates: list[dict[str, object]] = []
    for method in [case.label for case in CASES]:
        rows = by_method[method]
        out: dict[str, object] = {"method": method}
        keys = [key for key, value in rows[0].items() if key != "method" and isinstance(value, (int, float))]
        for key in keys:
            values = np.asarray([float(row[key]) for row in rows], dtype=float)
            values = values[np.isfinite(values)]
            out[key] = float(np.mean(values)) if values.size else float("nan")
        aggregates.append(out)
    return aggregates


def metric_value(rows: list[dict[str, object]], method: str, metric: str, joint_id: int | None = None) -> float:
    for row in rows:
        if row.get("method") != method:
            continue
        if joint_id is not None and int(row.get("joint_id", -1)) != joint_id:
            continue
        return float(row[metric])
    return float("nan")


def write_metric_plots(metrics: list[dict[str, object]], aggregate: list[dict[str, object]],
                       logs: dict[str, Path], out_dir: Path, warmup_s: float) -> list[Path]:
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    paths: list[Path] = []

    key_metrics = [
        "q_rmse",
        "dq_rmse",
        "ref_ddq_rms",
        "ref_jerk_rms",
        "policy_node_dq_error_rms",
        "tau_rate_rms",
    ]
    ratios: list[dict[str, object]] = []
    for row in aggregate:
        item: dict[str, object] = {"method": row["method"]}
        for metric in key_metrics:
            base = metric_value(aggregate, CASES[0].label, metric)
            item[metric] = float(row[metric]) / base if abs(base) > 1.0e-12 else float("nan")
        ratios.append(item)

    colors = {"preview_mpc_3ref": "#3B82F6", "preview_mpc_velocity_4ref": "#F97316"}

    width_px, height_px = 980, 500
    plot = (75.0, 70.0, 860.0, 310.0)
    metric_labels = ["q RMSE", "dq RMSE", "ref ddq", "ref jerk", "node dq err", "tau rate"]
    elements = [
        svg_text(width_px / 2, 32, "Aggregate comparison, lower is better", 18, weight="700"),
        svg_line(plot[0], plot[1] + plot[3], plot[0] + plot[2], plot[1] + plot[3], "#222222"),
        svg_line(plot[0], plot[1], plot[0], plot[1] + plot[3], "#222222"),
    ]
    ratio_values = np.asarray([[float(row[m]) for m in key_metrics] for row in ratios], dtype=float)
    ratio_values = ratio_values[np.isfinite(ratio_values)]
    y_max = max(1.1, float(np.max(ratio_values)) * 1.15) if ratio_values.size else 1.1
    for tick in np.linspace(0.0, y_max, 5):
        y = plot[1] + plot[3] * (1.0 - tick / y_max)
        elements.append(svg_line(plot[0], y, plot[0] + plot[2], y, "#dddddd", 0.8))
        elements.append(svg_text(plot[0] - 10, y + 4, f"{tick:.2g}", 11, anchor="end"))
    one_y = plot[1] + plot[3] * (1.0 - 1.0 / y_max)
    elements.append(svg_line(plot[0], one_y, plot[0] + plot[2], one_y, "#666666", 1.0, "5 4"))
    group_w = plot[2] / len(key_metrics)
    bar_w = group_w * 0.28
    for metric_idx, metric in enumerate(key_metrics):
        cx = plot[0] + group_w * (metric_idx + 0.5)
        for case_idx, row in enumerate(ratios):
            value = float(row[metric])
            bar_h = plot[3] * value / y_max if np.isfinite(value) else 0.0
            x = cx + (case_idx - 0.5) * bar_w * 1.25 - bar_w / 2
            y = plot[1] + plot[3] - bar_h
            elements.append(svg_rect(x, y, bar_w, bar_h, colors.get(row["method"], "#888888")))
        elements.append(svg_text(cx, plot[1] + plot[3] + 28, metric_labels[metric_idx], 11))
    for idx, case in enumerate(CASES):
        lx = plot[0] + 30 + idx * 260
        elements.append(svg_rect(lx, height_px - 58, 18, 12, colors[case.label]))
        elements.append(svg_text(lx + 24, height_px - 48, case.label, 12, anchor="start"))
    path = fig_dir / "mpc_velocity_compare_metric_ratios.svg"
    paths.append(write_svg(path, width_px, height_px, elements))

    width_px, height_px = 1020, 450
    elements = [svg_text(width_px / 2, 30, "Reference smoothness by joint", 18, weight="700")]
    smooth_specs = [
        ("ref_ddq_rms", "Reference acceleration RMS"),
        ("ref_jerk_rms", "Reference jerk RMS"),
    ]
    for panel_idx, (metric, title) in enumerate(smooth_specs):
        px = 65.0 + panel_idx * 500.0
        py = 70.0
        pw = 390.0
        ph = 260.0
        y_max_panel = float(np.nanmax(np.asarray([float(row[metric]) for row in metrics], dtype=float))) * 1.15
        y_max_panel = y_max_panel if y_max_panel > 0.0 else 1.0
        elements.append(svg_text(px + pw / 2, py - 18, title, 14, weight="700"))
        elements.append(svg_line(px, py + ph, px + pw, py + ph, "#222222"))
        elements.append(svg_line(px, py, px, py + ph, "#222222"))
        for tick in np.linspace(0.0, y_max_panel, 4):
            y = py + ph * (1.0 - tick / y_max_panel)
            elements.append(svg_line(px, y, px + pw, y, "#dddddd", 0.8))
            elements.append(svg_text(px - 8, y + 4, f"{tick:.2g}", 10, anchor="end"))
        joint_ids = sorted({int(row["joint_id"]) for row in metrics})
        group_w = pw / len(joint_ids)
        bar_w = group_w * 0.28
        for joint_idx, joint_id in enumerate(joint_ids):
            cx = px + group_w * (joint_idx + 0.5)
            for case_idx, case in enumerate(CASES):
                value = metric_value(metrics, case.label, metric, joint_id)
                bar_h = ph * value / y_max_panel if np.isfinite(value) else 0.0
                x = cx + (case_idx - 0.5) * bar_w * 1.25 - bar_w / 2
                y = py + ph - bar_h
                elements.append(svg_rect(x, y, bar_w, bar_h, colors[case.label]))
            elements.append(svg_text(cx, py + ph + 24, str(joint_id), 11))
        elements.append(svg_text(px + pw / 2, py + ph + 48, "joint_id", 11))
    for idx, case in enumerate(CASES):
        lx = 240 + idx * 270
        elements.append(svg_rect(lx, height_px - 42, 18, 12, colors[case.label]))
        elements.append(svg_text(lx + 24, height_px - 32, case.label, 12, anchor="start"))
    path = fig_dir / "mpc_velocity_compare_smoothness.svg"
    paths.append(write_svg(path, width_px, height_px, elements))

    loaded = {label: read_log_rows(path) for label, path in logs.items()}
    sample = next(iter(loaded.values()))
    joint_ids = sorted({int(row["joint_id"]) for row in sample})
    t_max = min(max(float(row["t"]) for row in sample), warmup_s + 1.5)
    ts_styles = {
        "preview_mpc_3ref": {
            "actual": "#0072B2",
            "ref": "#56B4E9",
            "error": "#0072B2",
            "ref_dash": "9 4",
            "error_dash": None,
            "actual_width": 2.35,
            "ref_width": 1.55,
            "error_width": 2.25,
            "marker": "circle",
        },
        "preview_mpc_velocity_4ref": {
            "actual": "#D55E00",
            "ref": "#E69F00",
            "error": "#D55E00",
            "ref_dash": "2 5",
            "error_dash": "8 4",
            "actual_width": 2.15,
            "ref_width": 1.45,
            "error_width": 2.25,
            "marker": "diamond",
        },
    }
    width_px = 1360
    row_h = 430
    height_px = 110 + row_h * len(joint_ids)
    elements = [
        svg_text(width_px / 2, 30, "Closed-loop position and velocity after warmup", 18, weight="700"),
        svg_text(width_px / 2, 52, "Error panels mark the largest local spikes on each method curve", 11),
    ]
    for row_idx, joint_id in enumerate(joint_ids):
        row_y = 70 + row_idx * row_h
        panels = [
            (80.0, row_y + 38.0, 540.0, 125.0, "q tracking", "rad"),
            (760.0, row_y + 38.0, 520.0, 125.0, "dq tracking", "rad/s"),
            (80.0, row_y + 245.0, 540.0, 125.0, "q error: ref - actual", "rad"),
            (760.0, row_y + 245.0, 520.0, 125.0, "dq error: ref - actual", "rad/s"),
        ]
        traces: list[tuple[int, str, str, str, str | None, float, np.ndarray, np.ndarray]] = []
        for case in CASES:
            view = [
                row for row in loaded[case.label]
                if int(row["joint_id"]) == joint_id and warmup_s <= float(row["t"]) <= t_max
            ]
            style = ts_styles[case.label]
            t = np.asarray([float(row["t"]) for row in view], dtype=float)
            q_actual = np.asarray([float(row["q_actual"]) for row in view], dtype=float)
            q_ref = np.asarray([float(row["q_ref_shaped"]) for row in view], dtype=float)
            dq_actual = np.asarray([float(row["dq_actual"]) for row in view], dtype=float)
            dq_ref = np.asarray([float(row["dq_ref_shaped"]) for row in view], dtype=float)
            traces.append((0, case.label, "actual", style["actual"], None,
                           float(style["actual_width"]), t, q_actual))
            traces.append((0, case.label, "ref", style["ref"], str(style["ref_dash"]),
                           float(style["ref_width"]), t, q_ref))
            traces.append((1, case.label, "actual", style["actual"], None,
                           float(style["actual_width"]), t, dq_actual))
            traces.append((1, case.label, "ref", style["ref"], str(style["ref_dash"]),
                           float(style["ref_width"]), t, dq_ref))
            traces.append((2, case.label, "q_error", style["error"], style["error_dash"],
                           float(style["error_width"]), t, q_ref - q_actual))
            traces.append((3, case.label, "dq_error", style["error"], style["error_dash"],
                           float(style["error_width"]), t, dq_ref - dq_actual))
        elements.append(svg_text(width_px / 2, row_y + 14, f"joint {joint_id}", 14, weight="700"))
        for panel_idx, (px, py, pw, ph, title, ylabel) in enumerate(panels):
            panel_traces = [tr for tr in traces if tr[0] == panel_idx]
            y_values = np.concatenate([tr[7] for tr in panel_traces if len(tr[7])]) if panel_traces else np.array([0.0])
            ymin = float(np.nanmin(y_values))
            ymax = float(np.nanmax(y_values))
            pad = max(1.0e-6, 0.08 * (ymax - ymin if ymax > ymin else max(abs(ymax), 1.0)))
            ylim = (ymin - pad, ymax + pad)
            elements.append(svg_text(px + pw / 2, py - 12, title, 12, weight="700"))
            elements.append(svg_line(px, py + ph, px + pw, py + ph, "#222222"))
            elements.append(svg_line(px, py, px, py + ph, "#222222"))
            for tick in np.linspace(ylim[0], ylim[1], 4):
                y = py + ph * (1.0 - (tick - ylim[0]) / (ylim[1] - ylim[0]))
                elements.append(svg_line(px, y, px + pw, y, "#dddddd", 0.8))
                elements.append(svg_text(px - 8, y + 4, f"{tick:.3g}", 10, anchor="end"))
            for _, method, role, color, dash, line_width, t, y in panel_traces:
                points = scale_points(t, y, (px, py, pw, ph), (warmup_s, t_max), ylim)
                opacity = 0.92 if role == "ref" else None
                elements.append(svg_path(points, color, line_width, dash, opacity))
            if panel_idx in {2, 3}:
                for _, method, role, color, _, _, t, y in panel_traces:
                    if role not in {"q_error", "dq_error"}:
                        continue
                    style = ts_styles[method]
                    for idx in spike_indices(y, count=6, min_spacing=22):
                        mx, my = scale_xy(float(t[idx]), float(y[idx]), (px, py, pw, ph), (warmup_s, t_max), ylim)
                        mx += -4.0 if method == CASES[0].label else 4.0
                        if style["marker"] == "diamond":
                            elements.append(svg_diamond(mx, my, 3.9, color, "#ffffff", 1.8))
                        else:
                            elements.append(svg_circle(mx, my, 3.7, color, "#ffffff", 1.8))
            elements.append(svg_text(px + pw / 2, py + ph + 32, "t [s]", 11))
            elements.append(svg_text(px - 52, py + ph / 2, ylabel, 11, rotate=-90))
    legend_y = height_px - 28
    legend_items = [
        ("3ref actual", "preview_mpc_3ref", "actual", None),
        ("3ref ref", "preview_mpc_3ref", "ref", None),
        ("3ref error spike", "preview_mpc_3ref", "error", "circle"),
        ("4ref actual", "preview_mpc_velocity_4ref", "actual", None),
        ("4ref ref", "preview_mpc_velocity_4ref", "ref", None),
        ("4ref error spike", "preview_mpc_velocity_4ref", "error", "diamond"),
    ]
    lx = 80.0
    for label, method, role, marker in legend_items:
        style = ts_styles[method]
        if role == "actual":
            color, dash, line_width = style["actual"], None, style["actual_width"]
        elif role == "ref":
            color, dash, line_width = style["ref"], style["ref_dash"], style["ref_width"]
        else:
            color, dash, line_width = style["error"], style["error_dash"], style["error_width"]
        elements.append(svg_line(lx, legend_y - 4, lx + 25, legend_y - 4, color, line_width, dash))
        if marker == "diamond":
            elements.append(svg_diamond(lx + 12.5, legend_y - 4, 3.8, color, "#ffffff", 1.8))
        elif marker == "circle":
            elements.append(svg_circle(lx + 12.5, legend_y - 4, 3.7, color, "#ffffff", 1.8))
        elements.append(svg_text(lx + 32, legend_y, label, 10, anchor="start"))
        lx += 205
    path = fig_dir / "mpc_velocity_compare_tracking_timeseries.svg"
    paths.append(write_svg(path, width_px, height_px, elements))

    return paths


def markdown_table(rows: list[dict[str, object]]) -> str:
    columns: list[str] = []
    for row in rows:
        for key in row:
            if key not in columns:
                columns.append(key)
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join("---" for _ in columns) + " |",
    ]
    for row in rows:
        cells = []
        for col in columns:
            value = row.get(col, "")
            if isinstance(value, (float, np.floating)):
                cells.append(f"{value:.6g}")
            else:
                cells.append(str(value))
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def comparison_change(comparison: list[dict[str, object]], metric: str) -> float:
    for row in comparison:
        if row["metric"] == metric:
            return float(row["change_percent"])
    return float("nan")


def write_report(metrics: list[dict[str, object]], aggregate: list[dict[str, object]],
                 comparison: list[dict[str, object]],
                 plot_paths: list[Path], out_dir: Path) -> Path:
    report = out_dir / "summary.md"
    q_change = comparison_change(comparison, "q_rmse")
    ddq_change = comparison_change(comparison, "ref_ddq_rms")
    jerk_change = comparison_change(comparison, "ref_jerk_rms")
    node_dq_change = comparison_change(comparison, "policy_node_dq_error_rms")
    verdict = [
        "# MPC velocity-target variant comparison",
        "",
        "Lower values are better for the metrics below. The comparison discards the startup warmup window.",
        "",
        "## Aggregate change of 4-ref velocity variant vs 3-ref baseline",
        "",
        f"- q tracking RMSE: {q_change:+.2f}%",
        f"- reference acceleration RMS: {ddq_change:+.2f}%",
        f"- reference jerk RMS: {jerk_change:+.2f}%",
        f"- policy-node velocity error RMS: {node_dq_change:+.2f}%",
        "",
        "## CSV artifacts",
        "",
        f"- `{repo_rel(out_dir / 'metrics_by_joint.csv')}`",
        f"- `{repo_rel(out_dir / 'aggregate_metrics.csv')}`",
        f"- `{repo_rel(out_dir / 'comparison_vs_preview_mpc_3ref.csv')}`",
        "",
        "## Figures",
        "",
    ]
    for path in plot_paths:
        verdict.append(f"- `{repo_rel(path)}`")
    verdict.extend([
        "",
        "## Aggregate table",
        "",
        markdown_table(aggregate),
        "",
    ])
    report.write_text("\n".join(verdict), encoding="utf-8")
    return report


def build_comparison(aggregate: list[dict[str, object]]) -> list[dict[str, object]]:
    baseline = next(row for row in aggregate if row["method"] == CASES[0].label)
    variant = next(row for row in aggregate if row["method"] == CASES[1].label)
    metrics = [
        "q_rmse",
        "dq_rmse",
        "ref_dq_rms",
        "ref_ddq_rms",
        "ref_jerk_rms",
        "policy_node_q_error_rms",
        "policy_node_dq_error_rms",
        "tau_applied_rms",
        "tau_rate_rms",
    ]
    rows = []
    for metric in metrics:
        base = float(baseline[metric])
        new = float(variant[metric])
        rows.append({
            "metric": metric,
            "preview_mpc_3ref": base,
            "preview_mpc_velocity_4ref": new,
            "change_percent": 100.0 * (new / base - 1.0) if abs(base) > 1.0e-12 else float("nan"),
        })
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-config", type=Path,
                        default=Path("config/h1_real_p3_selected_mpc_hip_knee_pd.yaml"))
    parser.add_argument("--out-dir", type=Path,
                        default=Path("analysis_artifacts/mpc_velocity_compare"))
    parser.add_argument("--run-mujoco", type=Path, default=Path("scripts/run_mujoco.py"))
    parser.add_argument("--scene", type=Path, default=Path("h1_official_mujoco/scene.xml"))
    parser.add_argument("--stepper", type=Path, default=Path("build-codex/Debug/h1_controller_stepper.exe"))
    parser.add_argument("--mock-runner", type=Path, default=Path("build-codex/Debug/h1_mock_closed_loop.exe"))
    parser.add_argument("--backend", choices=["auto", "mujoco", "mock"], default="auto")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--warmup-s", type=float, default=3.2)
    parser.add_argument("--skip-run", action="store_true",
                        help="Reuse existing logs in the output directory.")
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    configs = write_case_configs(args.baseline_config, args.out_dir)

    logs: dict[str, Path] = {}
    if not args.skip_run:
        selected_backend = args.backend
        if selected_backend in {"auto", "mujoco"}:
            try:
                for case in CASES:
                    case_dir = args.out_dir / "runs" / case.label
                    run_mujoco_case(args, case.label, configs[case.label], case_dir)
                selected_backend = "mujoco"
            except subprocess.CalledProcessError as ex:
                if args.backend == "mujoco":
                    raise
                print(f"MuJoCo backend failed with exit code {ex.returncode}; falling back to mock backend.")
                selected_backend = "mock"
        if selected_backend == "mock":
            for case in CASES:
                case_dir = args.out_dir / "runs" / case.label
                run_mock_case(args, case.label, configs[case.label], case_dir)

    for case in CASES:
        case_dir = args.out_dir / "runs" / case.label
        log_path = case_dir / "mujoco_closed_loop_log.csv"
        if not log_path.exists():
            raise RuntimeError(f"missing log: {log_path}")
        logs[case.label] = log_path

    metrics: list[dict[str, object]] = []
    for case in CASES:
        metrics.extend(summarize_log(case.label, configs[case.label], logs[case.label], args.warmup_s))
    metrics_path = args.out_dir / "metrics_by_joint.csv"
    write_csv_rows(metrics_path, metrics)

    aggregate = aggregate_metrics(metrics)
    aggregate_path = args.out_dir / "aggregate_metrics.csv"
    write_csv_rows(aggregate_path, aggregate)

    comparison = build_comparison(aggregate)
    comparison_path = args.out_dir / "comparison_vs_preview_mpc_3ref.csv"
    write_csv_rows(comparison_path, comparison)

    plot_paths = write_metric_plots(metrics, aggregate, logs, args.out_dir, args.warmup_s)
    report_path = write_report(metrics, aggregate, comparison, plot_paths, args.out_dir)

    print(f"metrics={metrics_path}")
    print(f"aggregate={aggregate_path}")
    print(f"comparison={comparison_path}")
    print(f"report={report_path}")
    for path in plot_paths:
        print(f"figure={path}")
    print(markdown_table(comparison))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
