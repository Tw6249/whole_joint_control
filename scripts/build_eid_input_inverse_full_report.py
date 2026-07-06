#!/usr/bin/env python3
"""Build the full EID input-inverse explanation report with data-direct PNG figures."""

from __future__ import annotations

import csv
import math
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path
from typing import Iterable

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from report_paths import markdown_relpath, repo_relpath


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "analysis_artifacts" / "eid_input_inverse_full"
FIG_DIR = OUT_DIR / "figures"
MUJOCO_OUT_DIR = ROOT / "analysis_artifacts" / "eid_input_inverse_mujoco"
MUJOCO_FIG_DIR = MUJOCO_OUT_DIR / "figures"
REPORT_DIR = ROOT / "docs" / "reports" / "analysis" / "eid_input_inverse_full"
REPORT_PATH = REPORT_DIR / "eid_input_inverse_full_report.md"

TARGET_LOG = ROOT / "data" / "20260623_170746" / "h1_real_p4_ku_u3_hip_knee_eid_log.csv"
CONFIG_SUMMARY = ROOT / "analysis_artifacts" / "eid_input_inverse" / "eid_input_inverse_config_summary.csv"
LOG_SUMMARY = ROOT / "analysis_artifacts" / "eid_input_inverse" / "eid_input_inverse_log_summary.csv"
MUJOCO_COMPARISON = ROOT / "analysis_artifacts" / "eid_input_inverse_mujoco" / "mujoco_equiv_ku_comparison.csv"
MUJOCO_ORIGINAL_LOG = MUJOCO_OUT_DIR / "runs" / "original_ku" / "mujoco_closed_loop_log.csv"
MUJOCO_EQUIV_KU_LOG = MUJOCO_OUT_DIR / "runs" / "input_inverse_equiv_ku" / "mujoco_closed_loop_log.csv"
PYTHON314 = Path(r"C:\Python314\python.exe")

JOINT_LABEL = {1: "右髋俯仰关节", 2: "右膝"}
JOINT_SHORT = {1: "髋", 2: "膝"}
JOINT_LABEL_EN = {1: "Right hip pitch", 2: "Right knee"}
LEGACY_OUTPUTS = [
    OUT_DIR / "p4_tracking_scan_summary.csv",
    OUT_DIR / "p4_ku_tracking_repeat_summary.csv",
    OUT_DIR / "p4_ku_tracking_relative_summary.csv",
    OUT_DIR / "target_ku_u3_tracking_metrics.csv",
    OUT_DIR / "anti_phase_tracking_scan_summary.csv",
    OUT_DIR / "input_gain_tracking_repeat_summary.csv",
    OUT_DIR / "input_gain_tracking_relative_summary.csv",
    MUJOCO_FIG_DIR / "mujoco_equiv_ku_q_error_timeseries.png",
    MUJOCO_FIG_DIR / "mujoco_equiv_ku_eta_u_timeseries.png",
    MUJOCO_FIG_DIR / "data_direct_q_error_timeseries.png",
    MUJOCO_FIG_DIR / "data_direct_eta_u_timeseries.png",
]
KO_PARAMS = {
    "Ko_o0": (0.0, 0.0),
    "Ko_o1": (0.2, 0.05),
    "Ko_o2": (0.4, 0.1),
    "Ko_o3": (0.6, 0.15),
    "Ko_o4": (0.8, 0.2),
    "Ko_o5": (1.0, 0.25),
}
KU_PARAMS = {
    "Ku_u1": (6.0, 0.5),
    "Ku_u2": (9.0, 0.75),
    "Ku_u3": (12.0, 1.0),
    "Ku_u4": (15.0, 1.25),
}


def cleanup_legacy_outputs() -> None:
    for path in LEGACY_OUTPUTS:
        if path.exists():
            path.unlink()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys()) if rows else []
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def f(row: dict[str, str], key: str) -> float:
    return float(row[key])


def rms(values: Iterable[float]) -> float:
    x = np.asarray(list(values), dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(x * x)))


def peak_abs(values: Iterable[float]) -> float:
    x = np.asarray(list(values), dtype=float)
    x = np.abs(x[np.isfinite(x)])
    if x.size == 0:
        return float("nan")
    return float(np.max(x))


def quantile_abs(values: Iterable[float], q: float) -> float:
    x = np.asarray(list(values), dtype=float)
    x = np.abs(x[np.isfinite(x)])
    if x.size == 0:
        return float("nan")
    return float(np.quantile(x, q))


def fmt(value: object, digits: int = 4) -> str:
    if isinstance(value, (float, np.floating)):
        x = float(value)
        if not math.isfinite(x):
            return "nan"
        if abs(x) != 0.0 and (abs(x) < 1.0e-3 or abs(x) >= 1.0e4):
            return f"{x:.{digits}e}"
        return f"{x:.{digits}g}"
    return str(value)


def markdown_table(rows: list[dict[str, object]], columns: list[str], labels: dict[str, str] | None = None) -> str:
    labels = labels or {}
    if not rows:
        return "_No rows._"
    header_names = [labels.get(c, c) for c in columns]
    header = "| " + " | ".join(header_names) + " |"
    sep = "| " + " | ".join("---" for _ in columns) + " |"
    body = []
    for row in rows:
        body.append("| " + " | ".join(fmt(row.get(c, "")) for c in columns) + " |")
    return "\n".join([header, sep, *body])


def case_label(case: object) -> str:
    name = str(case)
    if name in KO_PARAMS:
        ko_q, ko_dq = KO_PARAMS[name]
        return f"$K_o=({ko_q:g},{ko_dq:g}), K_u=(12,1)$"
    if name in KU_PARAMS:
        ku_q, ku_dq = KU_PARAMS[name]
        return f"$K_o=(0.8,0.2), K_u=({ku_q:g},{ku_dq:g})$"
    return name


def case_short_label(case: object) -> str:
    name = str(case)
    if name in KO_PARAMS:
        ko_q, ko_dq = KO_PARAMS[name]
        return f"Ko({ko_q:g},{ko_dq:g})"
    if name in KU_PARAMS:
        ku_q, ku_dq = KU_PARAMS[name]
        return f"Ku({ku_q:g},{ku_dq:g})"
    return name


def with_case_label(rows: list[dict[str, object]], column: str = "experiment") -> list[dict[str, object]]:
    out = []
    for row in rows:
        new_row = dict(row)
        new_row[column] = case_label(row.get("case", ""))
        out.append(new_row)
    return out


def target_tracking_metrics(path: Path) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    rows = read_csv(path)
    t0 = f(rows[0], "t")
    out = []
    for window, start, stop in [("after3s", 3.0, None), ("4to6s", 4.0, 6.0)]:
        for joint_id in (1, 2):
            selected = []
            for row in rows:
                if int(float(row["joint_id"])) != joint_id:
                    continue
                t_rel = f(row, "t") - t0
                if t_rel < start:
                    continue
                if stop is not None and t_rel >= stop:
                    continue
                selected.append(row)
            out.append(
                {
                    "window": window,
                    "joint_id": joint_id,
                    "joint_name": JOINT_LABEL[joint_id],
                    "n": len(selected),
                    "q_rmse": rms(f(r, "debug_30") for r in selected),
                    "q_p95_abs": quantile_abs((f(r, "debug_30") for r in selected), 0.95),
                    "q_peak_abs": peak_abs(f(r, "debug_30") for r in selected),
                    "dq_rmse": rms(f(r, "debug_31") for r in selected),
                    "eta_u_rms": rms(f(r, "debug_32") for r in selected),
                    "tau_cmd_rms": rms(f(r, "tau_cmd") for r in selected),
                }
            )
    return out, rows


def tracking_scan_summary() -> list[dict[str, object]]:
    patterns = [
        "data/20260623_17*/h1_real_p4_ko_o*_hip_knee_eid_log.csv",
        "data/20260623_17*/h1_real_p4_ku_u*_hip_knee_eid_log.csv",
    ]
    paths: list[Path] = []
    for pattern in patterns:
        paths.extend(sorted(ROOT.glob(pattern)))
    by_case: dict[tuple[str, int], list[dict[str, float]]] = defaultdict(list)
    for path in sorted(set(paths)):
        rows = read_csv(path)
        if not rows:
            continue
        condition = rows[0].get("condition_id", "")
        if "no_disturbance" not in condition:
            continue
        match = re.search(r"(Ko_o\d+|Ku_u\d+)", condition)
        if not match:
            continue
        label = match.group(1)
        t0 = f(rows[0], "t")
        for joint_id in (1, 2):
            selected = [
                r
                for r in rows
                if int(float(r["joint_id"])) == joint_id and f(r, "t") - t0 >= 3.0
            ]
            if not selected:
                continue
            by_case[(label, joint_id)].append(
                {
                    "q_rmse": rms(f(r, "debug_30") for r in selected),
                    "q_p95_abs": quantile_abs((f(r, "debug_30") for r in selected), 0.95),
                    "q_peak_abs": peak_abs(f(r, "debug_30") for r in selected),
                    "dq_rmse": rms(f(r, "debug_31") for r in selected),
                    "eta_u_rms": rms(f(r, "debug_32") for r in selected),
                    "tau_cmd_rms": rms(f(r, "tau_cmd") for r in selected),
                }
            )

    order = {f"Ko_o{i}": i for i in range(6)}
    order.update({f"Ku_u{i}": 10 + i for i in range(1, 5)})
    rows = []
    for (label, joint_id), vals in sorted(by_case.items(), key=lambda kv: (order.get(kv[0][0], 99), kv[0][1])):
        rows.append(
            {
                "case": label,
                "joint_id": joint_id,
                "joint_name": JOINT_LABEL[joint_id],
                "repeats": len(vals),
                "q_rmse_mean": float(np.mean([v["q_rmse"] for v in vals])),
                "q_rmse_min": float(np.min([v["q_rmse"] for v in vals])),
                "q_rmse_max": float(np.max([v["q_rmse"] for v in vals])),
                "q_p95_abs_mean": float(np.mean([v["q_p95_abs"] for v in vals])),
                "q_peak_abs_mean": float(np.mean([v["q_peak_abs"] for v in vals])),
                "dq_rmse_mean": float(np.mean([v["dq_rmse"] for v in vals])),
                "eta_u_rms_mean": float(np.mean([v["eta_u_rms"] for v in vals])),
                "tau_cmd_rms_mean": float(np.mean([v["tau_cmd_rms"] for v in vals])),
            }
        )
    return rows


def ku_repeat_summary() -> list[dict[str, object]]:
    paths = sorted(ROOT.glob("data/20260623_17*/h1_real_p4_ku_u*_hip_knee_eid_log.csv"))
    rows_out: list[dict[str, object]] = []
    for path in paths:
        rows = read_csv(path)
        if not rows:
            continue
        condition = rows[0].get("condition_id", "")
        if "no_disturbance" not in condition:
            continue
        match = re.search(r"(Ku_u\d+)", condition)
        if not match:
            continue
        label = match.group(1)
        repeat = rows[0].get("repeat_id", "")
        t0 = f(rows[0], "t")
        for joint_id in (1, 2):
            selected = [
                r
                for r in rows
                if int(float(r["joint_id"])) == joint_id and f(r, "t") - t0 >= 3.0
            ]
            if not selected:
                continue
            rows_out.append(
                {
                    "case": label,
                    "repeat": repeat,
                    "joint_id": joint_id,
                    "joint_name": JOINT_LABEL[joint_id],
                    "log_path": repo_relpath(path),
                    "n": len(selected),
                    "q_rmse": rms(f(r, "debug_30") for r in selected),
                    "q_p95_abs": quantile_abs((f(r, "debug_30") for r in selected), 0.95),
                    "q_peak_abs": peak_abs(f(r, "debug_30") for r in selected),
                    "dq_rmse": rms(f(r, "debug_31") for r in selected),
                    "eta_u_rms": rms(f(r, "debug_32") for r in selected),
                    "tau_cmd_rms": rms(f(r, "tau_cmd") for r in selected),
                }
            )
    order = {f"Ku_u{i}": i for i in range(1, 5)}
    return sorted(rows_out, key=lambda r: (order.get(str(r["case"]), 99), int(r["joint_id"]), str(r["repeat"])))


def ku_relative_summary(scan_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    ku_rows = [r for r in scan_rows if str(r["case"]).startswith("Ku_u")]
    base: dict[int, dict[str, object]] = {
        int(r["joint_id"]): r for r in ku_rows if str(r["case"]) == "Ku_u1"
    }
    out = []
    for row in ku_rows:
        joint_id = int(row["joint_id"])
        ref = base[joint_id]
        q_ref = float(ref["q_rmse_mean"])
        eta_ref = float(ref["eta_u_rms_mean"])
        tau_ref = float(ref["tau_cmd_rms_mean"])
        out.append(
            {
                "case": row["case"],
                "joint_name": row["joint_name"],
                "q_rmse_mean": row["q_rmse_mean"],
                "q_rmse_delta_vs_ku1_pct": (float(row["q_rmse_mean"]) / q_ref - 1.0) * 100.0,
                "dq_rmse_mean": row["dq_rmse_mean"],
                "eta_u_rms_mean": row["eta_u_rms_mean"],
                "eta_u_delta_vs_ku1_pct": (float(row["eta_u_rms_mean"]) / eta_ref - 1.0) * 100.0,
                "tau_cmd_rms_mean": row["tau_cmd_rms_mean"],
                "tau_delta_vs_ku1_pct": (float(row["tau_cmd_rms_mean"]) / tau_ref - 1.0) * 100.0,
            }
        )
    return out


def contribution_breakdown(target_rows: list[dict[str, str]], config_rows: list[dict[str, str]]) -> list[dict[str, object]]:
    constants = {
        int(float(r["joint_id"])): {
            "joint_name": r["joint_name"],
            "pinv_q": float(r["pinv_q"]),
            "pinv_dq": float(r["pinv_dq"]),
            "observer_gain_q": float(r["observer_gain_q"]),
            "observer_gain_dq": float(r["observer_gain_dq"]),
        }
        for r in config_rows
    }
    rows = []
    for joint_id in (1, 2):
        selected = [r for r in target_rows if int(float(r["joint_id"])) == joint_id]
        c = constants[joint_id]
        q = np.asarray([f(r, "q") for r in selected], dtype=float)
        dq = np.asarray([f(r, "dq") for r in selected], dtype=float)
        eta_q = np.asarray([f(r, "debug_9") for r in selected], dtype=float)
        eta_dq = np.asarray([f(r, "debug_10") for r in selected], dtype=float)
        xhat_q = np.asarray([f(r, "debug_11") for r in selected], dtype=float)
        xhat_dq = np.asarray([f(r, "debug_12") for r in selected], dtype=float)
        old_eta_u = np.asarray([f(r, "debug_32") for r in selected], dtype=float)

        x_minus_xhat_q = q - xhat_q
        x_minus_xhat_dq = dq - xhat_dq
        x_minus_xbar_q = x_minus_xhat_q - eta_q
        x_minus_xbar_dq = x_minus_xhat_dq - eta_dq
        q_contrib = c["pinv_q"] * c["observer_gain_q"] * x_minus_xhat_q
        dq_contrib = c["pinv_dq"] * c["observer_gain_dq"] * x_minus_xhat_dq
        pinv_total = q_contrib + dq_contrib
        pinv_xbar = (
            c["pinv_q"] * c["observer_gain_q"] * x_minus_xbar_q
            + c["pinv_dq"] * c["observer_gain_dq"] * x_minus_xbar_dq
        )
        rows.append(
            {
                "joint_id": joint_id,
                "joint_name": c["joint_name"],
                "x_minus_xhat_q_rms": rms(x_minus_xhat_q),
                "x_minus_xhat_dq_rms": rms(x_minus_xhat_dq),
                "eta_q_rms": rms(eta_q),
                "eta_dq_rms": rms(eta_dq),
                "old_eta_u_rms": rms(old_eta_u),
                "pinv_q_contrib_rms": rms(q_contrib),
                "pinv_dq_contrib_rms": rms(dq_contrib),
                "pinv_total_rms": rms(pinv_total),
                "pinv_xbar_rms": rms(pinv_xbar),
            }
        )
    return rows


def load_plot_font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    candidates = [
        Path(r"C:\Windows\Fonts\msyhbd.ttc" if bold else r"C:\Windows\Fonts\msyh.ttc"),
        Path(r"C:\Windows\Fonts\simhei.ttf"),
        Path(r"C:\Windows\Fonts\arialbd.ttf" if bold else r"C:\Windows\Fonts\arial.ttf"),
        Path(r"C:\Windows\Fonts\segoeuib.ttf" if bold else r"C:\Windows\Fonts\segoeui.ttf"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size=size)
    return ImageFont.load_default()


def draw_text(
    draw: ImageDraw.ImageDraw,
    xy: tuple[float, float],
    text: str,
    size: int = 12,
    anchor: str = "mm",
    bold: bool = False,
    fill: str = "#202124",
) -> None:
    draw.text(xy, text, font=load_plot_font(size, bold), fill=fill, anchor=anchor)


def draw_rotated_text(
    image: Image.Image,
    center: tuple[float, float],
    text: str,
    size: int = 12,
    angle: int = 90,
    bold: bool = False,
    fill: str = "#202124",
) -> None:
    font = load_plot_font(size, bold)
    bbox = ImageDraw.Draw(Image.new("RGBA", (1, 1))).textbbox((0, 0), text, font=font)
    w = bbox[2] - bbox[0] + 10
    h = bbox[3] - bbox[1] + 10
    txt = Image.new("RGBA", (w, h), (255, 255, 255, 0))
    d = ImageDraw.Draw(txt)
    d.text((w / 2, h / 2), text, font=font, fill=fill, anchor="mm")
    rotated = txt.rotate(angle, expand=True)
    x = int(center[0] - rotated.size[0] / 2)
    y = int(center[1] - rotated.size[1] / 2)
    image.alpha_composite(rotated, (x, y))


def save_png(image: Image.Image, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rgb = Image.new("RGB", image.size, "#ffffff")
    rgb.paste(image, mask=image.getchannel("A"))
    rgb.save(path, format="PNG", dpi=(300, 300), optimize=True)


def grouped_bar_png(
    path: Path,
    title: str,
    groups: list[str],
    series: list[str],
    values: list[list[float]],
    ylabel: str,
    log_scale: bool = False,
) -> None:
    colors = ["#2F6BFF", "#D97706", "#0F9D58", "#A142F4", "#5F6368"]
    width, height = 980, 460
    left, right, top, bottom = 88, 28, 58, 92
    plot_w = width - left - right
    plot_h = height - top - bottom
    image = Image.new("RGBA", (width, height), "#ffffff")
    draw = ImageDraw.Draw(image)
    flat = [v for row in values for v in row if math.isfinite(v) and v > 0.0]
    if not flat:
        flat = [1.0]
    if log_scale:
        ymin = 10 ** math.floor(math.log10(min(flat) * 0.8))
        ymax = 10 ** math.ceil(math.log10(max(flat) * 1.2))

        def ymap(v: float) -> float:
            v = max(v, ymin)
            return top + (math.log10(ymax) - math.log10(v)) / (math.log10(ymax) - math.log10(ymin)) * plot_h

        tick_values = [10 ** p for p in range(math.floor(math.log10(ymin)), math.ceil(math.log10(ymax)) + 1)]
    else:
        ymin, ymax = 0.0, max(flat) * 1.18

        def ymap(v: float) -> float:
            return top + (ymax - v) / (ymax - ymin) * plot_h

        tick_values = [ymax * i / 5.0 for i in range(6)]

    draw_text(draw, (width / 2, 26), title, 17, "mm", True)
    draw.line([(left, top + plot_h), (width - right, top + plot_h)], fill="#202124", width=1)
    draw.line([(left, top), (left, top + plot_h)], fill="#202124", width=1)
    for tv in tick_values:
        if tv < ymin or tv > ymax:
            continue
        y = ymap(tv)
        draw.line([(left, y), (width - right, y)], fill="#DADCE0", width=1)
        draw_text(draw, (left - 8, y), fmt(tv, 2), 10, "rm")
    draw_rotated_text(image, (22, top + plot_h / 2), ylabel, 12, 90)

    group_w = plot_w / len(groups)
    bar_w = min(34.0, group_w * 0.68 / max(1, len(series)))
    baseline = ymap(ymin)
    for gi, group in enumerate(groups):
        cx = left + group_w * (gi + 0.5)
        for si, name in enumerate(series):
            v = values[gi][si]
            x = cx - len(series) * bar_w / 2 + si * bar_w
            y = ymap(v)
            h = max(1.0, baseline - y)
            draw.rectangle([x, y, x + bar_w * 0.82, y + h], fill=colors[si % len(colors)])
        draw_text(draw, (cx, height - 52), group, 10, "mm")

    legend_x = left
    for si, name in enumerate(series):
        x = legend_x + si * 170
        draw.rectangle([x, height - 28, x + 13, height - 15], fill=colors[si % len(colors)])
        draw_text(draw, (x + 18, height - 21), name, 11, "lm")
    save_png(image, path)


def load_mujoco_signal(log_path: Path, signal: str) -> dict[int, list[tuple[float, float]]]:
    rows = read_csv(log_path)
    if not rows:
        return {}
    t0 = min(float(r["t"]) for r in rows)
    by_joint: dict[int, list[tuple[float, float]]] = defaultdict(list)
    for row in rows:
        joint_id = int(float(row["joint_id"]))
        if joint_id not in JOINT_LABEL:
            continue
        value = float(row[signal])
        if math.isfinite(value):
            by_joint[joint_id].append((float(row["t"]) - t0, value))
    return by_joint


def downsample_points(points: list[tuple[float, float]], max_points: int = 1400) -> list[tuple[float, float]]:
    if len(points) <= max_points:
        return points
    step = max(1, math.ceil(len(points) / max_points))
    return points[::step]


def comparison_timeseries_png(
    path: Path,
    title: str,
    signal: str,
    ylabel: str,
    cases: list[tuple[str, Path, str]],
) -> None:
    series = [(label, load_mujoco_signal(log_path, signal), color) for label, log_path, color in cases]
    joint_ids = [1, 2]
    width = 1080
    left = 92
    right = 168
    top = 82
    panel_h = 215
    gap = 70
    bottom = 62
    height = top + len(joint_ids) * panel_h + (len(joint_ids) - 1) * gap + bottom
    plot_w = width - left - right
    x_max = 0.0
    for _, by_joint, _ in series:
        for points in by_joint.values():
            if points:
                x_max = max(x_max, points[-1][0])
    x_max = max(8.0, x_max)

    image = Image.new("RGBA", (width, height), "#ffffff")
    draw = ImageDraw.Draw(image)
    draw_text(draw, (width / 2, 28), title, 18, "mm", True)
    legend_x = left
    for idx, (label, _, color) in enumerate(series):
        x = legend_x + idx * 235
        draw.line([(x, 45), (x + 28, 45)], fill=color, width=3)
        draw_text(draw, (x + 36, 45), label, 12, "lm")

    def xmap(t: float) -> float:
        ratio = max(0.0, min(1.0, t / x_max))
        return left + ratio * plot_w

    for panel_idx, joint_id in enumerate(joint_ids):
        panel_top = top + panel_idx * (panel_h + gap)
        panel_bottom = panel_top + panel_h
        values = [
            value
            for _, by_joint, _ in series
            for _, value in by_joint.get(joint_id, [])
            if math.isfinite(value)
        ]
        y_abs = max([abs(v) for v in values], default=1.0)
        if y_abs < 1.0e-9:
            y_abs = 1.0
        y_abs *= 1.12
        ymin, ymax = -y_abs, y_abs

        def ymap(v: float) -> float:
            return panel_top + (ymax - v) / (ymax - ymin) * panel_h

        draw_text(draw, (left, panel_top - 12), JOINT_LABEL[joint_id], 13, "ls", True)
        draw.line([(left, panel_bottom), (width - right, panel_bottom)], fill="#202124", width=1)
        draw.line([(left, panel_top), (left, panel_bottom)], fill="#202124", width=1)
        for tick_idx in range(5):
            tv = ymin + (ymax - ymin) * tick_idx / 4.0
            y = ymap(tv)
            draw.line([(left, y), (width - right, y)], fill="#E8EAED", width=1)
            draw_text(draw, (left - 8, y), fmt(tv, 3), 10, "rm")
        for xt in range(0, int(math.ceil(x_max)) + 1, 2):
            x = xmap(float(xt))
            draw.line([(x, panel_bottom), (x, panel_bottom + 5)], fill="#202124", width=1)
            if panel_idx == len(joint_ids) - 1:
                draw_text(draw, (x, panel_bottom + 22), f"{xt}", 10, "mm")
        x3 = xmap(3.0)
        dash_y = panel_top
        while dash_y < panel_bottom:
            draw.line([(x3, dash_y), (x3, min(dash_y + 5, panel_bottom))], fill="#5F6368", width=1)
            dash_y += 9
        draw_text(draw, (x3 + 5, panel_top + 14), "3 s", 10, "lm")

        for label, by_joint, color in series:
            points = downsample_points(by_joint.get(joint_id, []))
            if len(points) < 2:
                continue
            poly = [(xmap(t), ymap(v)) for t, v in points]
            draw.line(poly, fill=color, width=2, joint="curve")

    draw_rotated_text(image, (30, top + panel_h + gap / 2), ylabel, 12, 90)
    draw_text(draw, (left + plot_w / 2, height - 18), "time [s]", 12, "mm")
    save_png(image, path)


def plot_mujoco_timeseries_matplotlib(path: Path, title: str, signal: str, ylabel: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator

    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["Times New Roman", "DejaVu Serif"],
            "mathtext.fontset": "dejavuserif",
            "font.size": 9.0,
            "axes.titlesize": 9.5,
            "axes.labelsize": 9.0,
            "xtick.labelsize": 8.0,
            "ytick.labelsize": 8.0,
            "legend.fontsize": 8.2,
            "figure.dpi": 300,
            "savefig.dpi": 300,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.linewidth": 0.7,
            "lines.solid_capstyle": "round",
            "lines.dash_capstyle": "round",
        }
    )

    cases = [
        (r"Current $K_u=(12,1)$", MUJOCO_ORIGINAL_LOG, "#0072B2", "-"),
        (r"Input-inverse $K_u^{\mathrm{new}}$", MUJOCO_EQUIV_KU_LOG, "#D55E00", "-"),
    ]
    series = [(label, load_mujoco_signal(log_path, signal), color, linestyle) for label, log_path, color, linestyle in cases]
    fig, axes = plt.subplots(2, 1, figsize=(7.1, 4.15), sharex=True)
    fig.suptitle(title, fontsize=10.5, fontweight="bold", y=0.995)

    for ax, joint_id in zip(axes, (1, 2)):
        all_values: list[float] = []
        for label, by_joint, color, linestyle in series:
            points = by_joint.get(joint_id, [])
            if not points:
                continue
            t = np.asarray([p[0] for p in points], dtype=float)
            y = np.asarray([p[1] for p in points], dtype=float)
            all_values.extend(y[np.isfinite(y)].tolist())
            ax.plot(
                t,
                y,
                color=color,
                linestyle=linestyle,
                linewidth=1.05 if "Current" in label else 0.95,
                alpha=0.95,
                label=label,
                antialiased=True,
            )

        y_abs = max([abs(v) for v in all_values], default=1.0)
        if y_abs < 1.0e-9:
            y_abs = 1.0
        ax.set_ylim(-1.12 * y_abs, 1.12 * y_abs)
        ax.set_xlim(0.0, 8.0)
        ax.axhline(0.0, color="#4A4A4A", linewidth=0.65, alpha=0.55, zorder=0)
        ax.axvline(3.0, color="#4A4A4A", linewidth=0.75, linestyle=(0, (3, 3)), alpha=0.85)
        ax.text(
            3.03,
            0.92,
            "3 s",
            transform=ax.get_xaxis_transform(),
            ha="left",
            va="top",
            fontsize=7.6,
            color="#333333",
        )
        ax.text(
            0.012,
            0.9,
            JOINT_LABEL_EN[joint_id],
            transform=ax.transAxes,
            ha="left",
            va="top",
            fontsize=9.2,
            fontweight="bold",
            color="#202124",
        )
        ax.grid(axis="y", color="#D0D4DA", linewidth=0.55, alpha=0.55)
        ax.grid(axis="x", visible=False)
        ax.yaxis.set_major_locator(MaxNLocator(nbins=5, prune=None))
        ax.tick_params(axis="both", width=0.65, length=3.0, color="#333333")
        ax.spines["left"].set_color("#333333")
        ax.spines["bottom"].set_color("#333333")
        ax.set_ylabel(ylabel)

    axes[-1].set_xlabel("Time [s]")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.955),
        ncol=2,
        frameon=False,
        handlelength=2.7,
        columnspacing=2.2,
    )
    fig.subplots_adjust(left=0.09, right=0.985, top=0.86, bottom=0.12, hspace=0.24)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, format="png", dpi=300)
    plt.close(fig)


def plot_mujoco_timeseries_with_available_matplotlib(path: Path, title: str, signal: str, ylabel: str) -> None:
    python = PYTHON314 if PYTHON314.exists() else Path(sys.executable)
    result = subprocess.run(
        [
            str(python),
            str(Path(__file__).resolve()),
            "--plot-mujoco-timeseries",
            str(path),
            title,
            signal,
            ylabel,
        ],
        cwd=str(ROOT),
        text=True,
        capture_output=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            "Failed to generate publication PNG figure with Matplotlib.\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )


def write_mujoco_timeseries_figures() -> dict[str, Path]:
    figs: dict[str, Path] = {}
    if not MUJOCO_ORIGINAL_LOG.exists() or not MUJOCO_EQUIV_KU_LOG.exists():
        return figs
    figs["q_error"] = MUJOCO_FIG_DIR / "paper_q_error_timeseries.png"
    plot_mujoco_timeseries_with_available_matplotlib(
        figs["q_error"],
        "Position Tracking Error",
        "q_error_shaped",
        "Position error [rad]",
    )
    figs["eta_u"] = MUJOCO_FIG_DIR / "paper_eta_u_timeseries.png"
    plot_mujoco_timeseries_with_available_matplotlib(
        figs["eta_u"],
        "Input Compensation Torque",
        "eta_u",
        r"$\eta_u$ [N m]",
    )
    return figs


def write_figures(
    config_rows: list[dict[str, str]],
    log_rows: list[dict[str, str]],
    scan_rows: list[dict[str, object]],
    contribution_rows: list[dict[str, object]],
) -> dict[str, Path]:
    cfg = {int(float(r["joint_id"])): r for r in config_rows}
    logs = {int(float(r["joint_id"])): r for r in log_rows}
    figs: dict[str, Path] = {}

    coeff_groups = ["髋 q", "髋 dq", "膝 q", "膝 dq"]
    coeff_values = [
        [float(cfg[1]["ku_q"]), float(cfg[1]["pinv_q"]), float(cfg[1]["weighted_pinv_q"])],
        [float(cfg[1]["ku_dq"]), float(cfg[1]["pinv_dq"]), float(cfg[1]["weighted_pinv_dq"])],
        [float(cfg[2]["ku_q"]), float(cfg[2]["pinv_q"]), float(cfg[2]["weighted_pinv_q"])],
        [float(cfg[2]["ku_dq"]), float(cfg[2]["pinv_dq"]), float(cfg[2]["weighted_pinv_dq"])],
    ]
    figs["coeff"] = FIG_DIR / "input_inverse_coefficients.png"
    grouped_bar_png(figs["coeff"], "输入映射系数对比", coeff_groups, ["K_u", "g+", "gW+"], coeff_values, "coefficient", True)

    eta_groups = ["髋", "膝"]
    eta_values = [
        [float(logs[1]["old_eta_u_rms"]), float(logs[1]["pinv_eta_u_rms"]), float(logs[1]["weighted_eta_u_rms"])],
        [float(logs[2]["old_eta_u_rms"]), float(logs[2]["pinv_eta_u_rms"]), float(logs[2]["weighted_eta_u_rms"])],
    ]
    figs["eta"] = FIG_DIR / "eta_u_rms_comparison.png"
    grouped_bar_png(figs["eta"], "输入补偿 RMS 对比", eta_groups, ["K_u eta", "g+ Ko residual", "gW+ Ko residual"], eta_values, "RMS [Nm]", True)

    label_order = [f"Ko_o{i}" for i in range(6)] + [f"Ku_u{i}" for i in range(1, 5)]
    scan_by_case: dict[str, dict[int, dict[str, object]]] = defaultdict(dict)
    for row in scan_rows:
        scan_by_case[str(row["case"])][int(row["joint_id"])] = row
    scan_groups = [label for label in label_order if label in scan_by_case]
    scan_values = [
        [float(scan_by_case[label][1]["q_rmse_mean"]), float(scan_by_case[label][2]["q_rmse_mean"])]
        for label in scan_groups
    ]
    figs["tracking"] = FIG_DIR / "anti_phase_tracking_q_rmse.png"
    grouped_bar_png(
        figs["tracking"],
        "右髋俯仰关节与右膝稳态位置跟踪误差",
        [case_short_label(label) for label in scan_groups],
        ["髋", "膝"],
        scan_values,
        "q RMSE [rad]",
        True,
    )

    contrib_groups = [JOINT_SHORT[int(r["joint_id"])] for r in contribution_rows]
    contrib_values = [
        [float(r["pinv_q_contrib_rms"]), float(r["pinv_dq_contrib_rms"]), float(r["old_eta_u_rms"])]
        for r in contribution_rows
    ]
    figs["contrib"] = FIG_DIR / "pinv_contribution_breakdown.png"
    grouped_bar_png(figs["contrib"], "g+ 输入逆贡献拆分", contrib_groups, ["q contrib", "dq contrib", "old eta_u"], contrib_values, "RMS [Nm]", True)
    return figs


def write_report(
    config_rows: list[dict[str, str]],
    log_rows: list[dict[str, str]],
    target_rows: list[dict[str, object]],
    scan_rows: list[dict[str, object]],
    ku_repeat_rows: list[dict[str, object]],
    ku_relative_rows: list[dict[str, object]],
    contribution_rows: list[dict[str, object]],
    figs: dict[str, Path],
) -> None:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    fig_link = {k: markdown_relpath(v, REPORT_DIR) for k, v in figs.items()}
    target_after3 = [r for r in target_rows if r["window"] == "after3s"]
    target_4to6 = [r for r in target_rows if r["window"] == "4to6s"]
    ko_ku_focus = [
        r
        for r in scan_rows
        if str(r["case"]) in {"Ko_o0", "Ko_o5", "Ku_u1", "Ku_u2", "Ku_u3", "Ku_u4"}
    ]
    ku_aggregate = [r for r in scan_rows if str(r["case"]).startswith("Ku_u")]

    lines = [
        "# EID 输入逆改进推导、数值验证与现象分析",
        "",
        "## 摘要与核心结论",
        "",
        "本文整理当前右髋-右膝 EID 控制器的原始算法、输入逆改进思路、离线数值计算和实测日志对比。核心结论是：P4 稳态跟踪误差处在正常范围内，裸输入逆 `g+` 计算出的输入扰动却比当前经验 `Ku` 补偿大两个到三个数量级，因此它目前只能作为离线诊断量，不能直接替换实时控制器中的 `ku_q/ku_dq`。",
        "",
        "造成这个现象的主因是离散模型的速度输入逆系数约为 `J/Ts`。在 `Ts=0.002 s` 时，该系数天然达到百级到五百级；它会把 `dq-x_hat_dq` 这类很小但真实存在的速度残差放大成数牛米甚至数十牛米的等效输入扰动。",
        "",
        "## 1. 原始算法核心",
        "",
        "当前控制器采用单关节局部标称模型。状态定义为",
        "",
        "$$",
        "x_k=\\begin{bmatrix}q_k\\\\\\dot q_k\\end{bmatrix}",
        "$$",
        "",
        "控制器内部维护标称观测状态 `x_hat` 和 EID 状态估计 `eta`，并构造中心反馈状态：",
        "",
        "$$",
        "\\bar x_k=\\hat x_k+\\eta_k",
        "$$",
        "",
        "后续反馈不直接围绕测量状态 `x`，而是围绕中心反馈状态 `bar x`。这一步是当前实现里最关键的结构，因为它把观测器估计和反馈中心耦合在一起。",
        "",
        "解析逆模型根据当前参考 `r_k` 和下一步参考 `r_{k+1}` 计算标称前馈力矩：",
        "",
        "$$",
        "u_k^*=\\hat{\\mathcal I}(r_k,r_{k+1}-r_k)",
        "$$",
        "",
        "当前输入域 EID 并没有显式求 `g^{-1}`，而是使用手调的经验映射：",
        "",
        "$$",
        "\\eta_{u,k}=k_{u,q}\\eta_{q,k}+k_{u,\\dot q}\\eta_{\\dot q,k}",
        "$$",
        "",
        "最终控制律可写成：",
        "",
        "$$",
        "u_k=u_k^*-\\eta_{u,k}+K(r_k-\\bar x_k)",
        "$$",
        "",
        "其中第一项是标称前馈，第二项是输入端补偿，第三项是围绕中心反馈状态的 PD 修正。控制力矩计算后，观测器按当前控制器实际实现更新：",
        "",
        "$$",
        "\\hat x_{k+1}=\\hat\\Phi(\\bar x_k,u_k)",
        "$$",
        "",
        "$$",
        "\\eta_{k+1}=\\alpha K_o(x_k-\\bar x_k)+(1-\\alpha)\\eta_k",
        "$$",
        "",
        "这里要注意，当前代码中的 EID 更新残差是 `x-bar x`，不是理论草稿里更直接的 `x-x_hat`。这一区别会影响输入逆估计的尺度，但不是导致裸 `g+` 过大的主因。",
        "",
        "## 2. 输入逆改进思路",
        "",
        "改进思路来自一个更严格的输入域解释：如果观测误差项 `Ko(x-x_hat)` 要被注入逆模型输入端，就需要通过输入矩阵的逆把状态域量转换成力矩域量。",
        "",
        "离散系统写作：",
        "",
        "$$",
        "x_{k+1}=f(x_k)+g(x_k)u_k",
        "$$",
        "",
        "则输入端等效扰动可形式化为：",
        "",
        "$$",
        "\\hat d_u=g^{-1}K_o(x-\\hat x)",
        "$$",
        "",
        "在当前单关节半隐式欧拉模型中：",
        "",
        "$$",
        "\\dot q_{k+1}=\\dot q_k+\\frac{T_s}{J_\\mathrm{eff}}(u_k-\\beta_k)",
        "$$",
        "",
        "$$",
        "q_{k+1}=q_k+T_s\\dot q_{k+1}",
        "$$",
        "",
        "其中 `beta` 是阻尼、重力和偏置项，属于漂移项 `f(x)`。因此输入矩阵为：",
        "",
        "$$",
        "g=\\begin{bmatrix}T_s^2/J_\\mathrm{eff}\\\\T_s/J_\\mathrm{eff}\\end{bmatrix}",
        "$$",
        "",
        "由于 `g` 是 `2x1` 列向量，实际应使用 Moore-Penrose 左伪逆：",
        "",
        "$$",
        "g^+=(g^Tg)^{-1}g^T",
        "$$",
        "",
        "本轮实际操作只做离线计算：",
        "",
        "$$",
        "\\eta_u^{pinv}=g^+K_o(x-\\hat x)",
        "$$",
        "",
        "同时计算加权伪逆：",
        "",
        "$$",
        "g_W^+=(g^TWg)^{-1}g^TW",
        "$$",
        "",
        "加权形式用于观察权重选择对力矩反算的影响，不作为实时控制替代方案。",
        "",
        "## 3. 输入逆数值结果",
        "",
        "P4 `Ku_u3` 配置中，两个关节的输入矩阵和伪逆如下。",
        "",
        markdown_table(
            [
                {
                    "joint": r["joint_name"],
                    "J": float(r["Jeff"]),
                    "g_q": float(r["g_q"]),
                    "g_dq": float(r["g_dq"]),
                    "pinv_q": float(r["pinv_q"]),
                    "pinv_dq": float(r["pinv_dq"]),
                    "weighted_pinv_q": float(r["weighted_pinv_q"]),
                    "weighted_pinv_dq": float(r["weighted_pinv_dq"]),
                    "ku_q": float(r["ku_q"]),
                    "ku_dq": float(r["ku_dq"]),
                }
                for r in config_rows
            ],
            ["joint", "J", "g_q", "g_dq", "pinv_q", "pinv_dq", "weighted_pinv_q", "weighted_pinv_dq", "ku_q", "ku_dq"],
            {
                "joint": "关节",
                "J": "J_eff",
                "g_q": "g_q",
                "g_dq": "g_dq",
                "pinv_q": "g+_q",
                "pinv_dq": "g+_dq",
                "weighted_pinv_q": "gW+_q",
                "weighted_pinv_dq": "gW+_dq",
                "ku_q": "ku_q",
                "ku_dq": "ku_dq",
            },
        ),
        "",
        f"![输入映射系数对比]({fig_link['coeff']})",
        "",
        "这个表已经显示出尺度差异。经验 `ku_dq` 为 `1`，而裸 `g+` 的速度通道为：髋 `502.5406`，膝 `125.0737`。这意味着速度残差只要达到 `0.01 rad/s` 量级，就可能被反算成 `1 N·m` 量级的输入扰动。",
        "",
        "## 4. 实测日志重放结果",
        "",
        "使用 `data/20260623_170746/h1_real_p4_ku_u3_hip_knee_eid_log.csv` 重放，比较当前日志中的旧补偿 `eta_u=Ku eta` 与离线输入逆估计。",
        "",
        markdown_table(
            [
                {
                    "joint": r["joint_name"],
                    "old_eta_u_rms": float(r["old_eta_u_rms"]),
                    "pinv_eta_u_rms": float(r["pinv_eta_u_rms"]),
                    "weighted_eta_u_rms": float(r["weighted_eta_u_rms"]),
                    "pinv_over_old_rms": float(r["pinv_over_old_rms"]),
                    "weighted_over_old_rms": float(r["weighted_over_old_rms"]),
                    "corr_old_pinv": float(r["corr_old_pinv"]),
                }
                for r in log_rows
            ],
            ["joint", "old_eta_u_rms", "pinv_eta_u_rms", "weighted_eta_u_rms", "pinv_over_old_rms", "weighted_over_old_rms", "corr_old_pinv"],
            {
                "joint": "关节",
                "old_eta_u_rms": "旧 eta_u RMS",
                "pinv_eta_u_rms": "g+ 估计 RMS",
                "weighted_eta_u_rms": "gW+ 估计 RMS",
                "pinv_over_old_rms": "g+/旧",
                "weighted_over_old_rms": "gW+/旧",
                "corr_old_pinv": "相关系数",
            },
        ),
        "",
        f"![输入补偿 RMS 对比]({fig_link['eta']})",
        "",
        "可以看到，裸 `g+` 估计不是略大，而是显著大：髋约 `447` 倍，膝约 `124` 倍。加权伪逆在默认逆模型权重下更大，髋达到 `4669` 倍，膝达到 `904` 倍。",
        "",
        "进一步拆分 `g+` 的位置项和速度项：",
        "",
        markdown_table(
            [
                {
                    "joint": r["joint_name"],
                    "x_minus_xhat_q_rms": r["x_minus_xhat_q_rms"],
                    "x_minus_xhat_dq_rms": r["x_minus_xhat_dq_rms"],
                    "old_eta_u_rms": r["old_eta_u_rms"],
                    "pinv_q_contrib_rms": r["pinv_q_contrib_rms"],
                    "pinv_dq_contrib_rms": r["pinv_dq_contrib_rms"],
                    "pinv_total_rms": r["pinv_total_rms"],
                    "pinv_xbar_rms": r["pinv_xbar_rms"],
                }
                for r in contribution_rows
            ],
            [
                "joint",
                "x_minus_xhat_q_rms",
                "x_minus_xhat_dq_rms",
                "old_eta_u_rms",
                "pinv_q_contrib_rms",
                "pinv_dq_contrib_rms",
                "pinv_total_rms",
                "pinv_xbar_rms",
            ],
            {
                "joint": "关节",
                "x_minus_xhat_q_rms": "q残差 RMS",
                "x_minus_xhat_dq_rms": "dq残差 RMS",
                "old_eta_u_rms": "旧 eta_u RMS",
                "pinv_q_contrib_rms": "g+位置项",
                "pinv_dq_contrib_rms": "g+速度项",
                "pinv_total_rms": "g+总量",
                "pinv_xbar_rms": "用 x-barx",
            },
        ),
        "",
        f"![g+ 输入逆贡献拆分]({fig_link['contrib']})",
        "",
        "贡献拆分说明，裸 `g+` 的大数值几乎完全来自速度通道。髋位置项只有 `0.00071 N·m RMS`，速度项为 `5.91 N·m RMS`；膝位置项只有 `0.00019 N·m RMS`，速度项为 `2.28 N·m RMS`。",
        "",
        "## 5. 实际跟踪效果对比",
        "",
        "首先看同一条 `Ku_u3 r01` 日志。去掉 3 秒启动段后，跟踪误差如下：",
        "",
        markdown_table(
            target_after3,
            ["joint_name", "q_rmse", "q_p95_abs", "q_peak_abs", "dq_rmse", "eta_u_rms", "tau_cmd_rms"],
            {
                "joint_name": "关节",
                "q_rmse": "q RMSE",
                "q_p95_abs": "q 95%",
                "q_peak_abs": "q 峰值",
                "dq_rmse": "dq RMSE",
                "eta_u_rms": "eta_u RMS",
                "tau_cmd_rms": "tau RMS",
            },
        ),
        "",
        "在 `4-6 s` 稳态窗口中，误差进一步为：",
        "",
        markdown_table(
            target_4to6,
            ["joint_name", "q_rmse", "q_p95_abs", "q_peak_abs", "dq_rmse", "eta_u_rms"],
            {
                "joint_name": "关节",
                "q_rmse": "q RMSE",
                "q_p95_abs": "q 95%",
                "q_peak_abs": "q 峰值",
                "dq_rmse": "dq RMSE",
                "eta_u_rms": "eta_u RMS",
            },
        ),
        "",
        "这说明实际跟踪误差本身并不异常。以去掉启动段后的结果看，髋位置 RMSE 为 `0.00773 rad`，膝位置 RMSE 为 `0.01673 rad`，约等于 `0.44 deg` 和 `0.96 deg`。",
        "",
        "再看 P4 扫描均值。这里先给出 Ko 的两端点和 Ku 的完整均值对比，用来同时回答两个问题：观测器补偿是否有效，以及继续增大 `Ku` 是否明显改变稳态跟踪误差。",
        "",
        markdown_table(
            ko_ku_focus,
            [
                "case",
                "joint_name",
                "repeats",
                "q_rmse_mean",
                "q_p95_abs_mean",
                "q_peak_abs_mean",
                "dq_rmse_mean",
                "eta_u_rms_mean",
                "tau_cmd_rms_mean",
            ],
            {
                "case": "组别",
                "joint_name": "关节",
                "repeats": "重复",
                "q_rmse_mean": "q RMSE",
                "q_p95_abs_mean": "q 95%",
                "q_peak_abs_mean": "q 峰值",
                "dq_rmse_mean": "dq RMSE",
                "eta_u_rms_mean": "eta_u RMS",
                "tau_cmd_rms_mean": "tau RMS",
            },
        ),
        "",
        f"![P4 稳态位置跟踪误差]({fig_link['tracking']})",
        "",
        "`Ko_o0` 没有观测器补偿时，髋和膝的 RMSE 分别约为 `0.116` 和 `0.330 rad`；`Ko_o5` 后降到 `0.00767` 和 `0.01619 rad`。`Ku_u1-u4` 中，髋基本稳定在 `0.0078-0.0080 rad`，膝基本稳定在 `0.0168-0.0169 rad`。因此，当前现象不是“控制器没跟上”，而是“裸输入逆把正常的小速度残差反算成了过大的力矩量”。",
        "",
        "### 5.1 Ku 扫描完整稳态对比",
        "",
        "上一版报告把 Ku 对比压缩在 P4 扫描表里，容易让人误以为没有完整比较。这里单独展开 `Ku_u1-u4`，窗口统一为启动后 `t_rel >= 3 s`，每组 3 次重复。",
        "",
        markdown_table(
            ku_aggregate,
            [
                "case",
                "joint_name",
                "repeats",
                "q_rmse_mean",
                "q_rmse_min",
                "q_rmse_max",
                "dq_rmse_mean",
                "eta_u_rms_mean",
                "tau_cmd_rms_mean",
            ],
            {
                "case": "Ku组",
                "joint_name": "关节",
                "repeats": "重复",
                "q_rmse_mean": "q均值",
                "q_rmse_min": "q最小",
                "q_rmse_max": "q最大",
                "dq_rmse_mean": "dq均值",
                "eta_u_rms_mean": "eta_u均值",
                "tau_cmd_rms_mean": "tau均值",
            },
        ),
        "",
        "相对 `Ku_u1` 的变化如下。`eta_u` 随 Ku 增大明显增加，但位置 RMSE 基本没有同步下降；这说明在当前窗口里继续放大输入补偿不是主要误差杠杆。",
        "",
        markdown_table(
            ku_relative_rows,
            [
                "case",
                "joint_name",
                "q_rmse_mean",
                "q_rmse_delta_vs_ku1_pct",
                "dq_rmse_mean",
                "eta_u_rms_mean",
                "eta_u_delta_vs_ku1_pct",
                "tau_cmd_rms_mean",
                "tau_delta_vs_ku1_pct",
            ],
            {
                "case": "Ku组",
                "joint_name": "关节",
                "q_rmse_mean": "q RMSE",
                "q_rmse_delta_vs_ku1_pct": "q相对Ku_u1(%)",
                "dq_rmse_mean": "dq RMSE",
                "eta_u_rms_mean": "eta_u RMS",
                "eta_u_delta_vs_ku1_pct": "eta_u相对Ku_u1(%)",
                "tau_cmd_rms_mean": "tau RMS",
                "tau_delta_vs_ku1_pct": "tau相对Ku_u1(%)",
            },
        ),
        "",
        "每次重复的明细也已经输出到 `p4_ku_tracking_repeat_summary.csv`。明细用于检查均值是否被单次异常主导；本批 `Ku_u1-u4` 每组 3 次重复的误差范围很窄，所以均值有代表性。",
        "",
        "## 6. 现象原因分析",
        "",
        "第一，速度通道天然带有采样周期倒数。未加权伪逆可以近似理解为：",
        "",
        "$$",
        "g^+\\approx\\begin{bmatrix}J_\\mathrm{eff} & J_\\mathrm{eff}/T_s\\end{bmatrix}",
        "$$",
        "",
        "当 `Ts=0.002 s` 时，`1/Ts=500`。因此速度残差项会被百级到五百级系数放大。右髋 `g+_dq=502.5406`，乘 `observer_gain_dq=0.2` 后仍约为 `100.5`；速度残差 `0.05 rad/s` 就会得到约 `5 N·m` 的输入扰动估计。",
        "",
        "第二，当前经验 `Ku` 不是输入矩阵伪逆。当前 `Ku_u3` 使用 `ku_q=12, ku_dq=1`，它更像一个保守的闭环补偿强度，而不是完整物理反演。它还作用在滤波后的 `eta` 上，而不是直接作用在 `x-x_hat` 的原始残差上。",
        "",
        "第三，当前代码使用 `x-bar x` 更新 EID，而离线输入逆初步使用 `x-x_hat`。用 `x-bar x` 重新估计会小一些，髋从 `5.91` 降到 `4.97 N·m RMS`，膝从 `2.28` 降到 `1.91 N·m RMS`，但仍远大于旧 `eta_u`。所以残差定义会影响数值，但不是主因。",
        "",
        "第四，默认加权伪逆不适合直接当降噪方案。默认逆模型权重为 `w_q=0.5/Ts^2=125000`、`w_dq=1`。它强烈强调位置通道，而位置输入增益 `Ts^2/J` 极小，反算时会得到非常大的位置系数。因此加权伪逆在本日志中比裸 `g+` 更大。",
        "",
        "## 7. 工程结论",
        "",
        "1. 当前 P4 跟踪误差正常，且明显好于无观测器补偿的 `Ko_o0`。",
        "2. 裸 `g+K_o(x-x_hat)` 是合理的离线物理诊断量，但不是可直接上线的补偿律。",
        "3. 若后续要接入控制器，应至少使用 `x-bar x` 残差、低通滤波、限幅和缩放系数 `gamma`：",
        "",
        "$$",
        "\\eta_u=\\gamma\\,\\mathrm{sat}\\left(g^+K_o(x-\\bar x)\\right)",
        "$$",
        "",
        "4. `gamma` 应从很小的量级开始，例如 `0.001-0.01`，并以日志重放、mock/MuJoCo 和实机小幅扫描逐级验证。",
        "",
        "## 8. 产物与复现",
        "",
        f"- 配置输入逆表：`{repo_relpath(OUT_DIR / 'input_inverse_coefficients.csv')}`",
        f"- 日志重放表：`{repo_relpath(OUT_DIR / 'input_inverse_log_replay.csv')}`",
        f"- 跟踪扫描表：`{repo_relpath(OUT_DIR / 'p4_tracking_scan_summary.csv')}`",
        f"- Ku 重复明细表：`{repo_relpath(OUT_DIR / 'p4_ku_tracking_repeat_summary.csv')}`",
        f"- Ku 相对变化表：`{repo_relpath(OUT_DIR / 'p4_ku_tracking_relative_summary.csv')}`",
        f"- 贡献拆分表：`{repo_relpath(OUT_DIR / 'pinv_contribution_breakdown.csv')}`",
        f"- 构建脚本：`{repo_relpath(ROOT / 'scripts' / 'build_eid_input_inverse_full_report.py')}`",
        "",
    ]
    REPORT_PATH.write_text("\n".join(lines), encoding="utf-8")


def write_report_polished(
    config_rows: list[dict[str, str]],
    log_rows: list[dict[str, str]],
    target_rows: list[dict[str, object]],
    scan_rows: list[dict[str, object]],
    ku_relative_rows: list[dict[str, object]],
    contribution_rows: list[dict[str, object]],
    figs: dict[str, Path],
) -> None:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    fig_link = {k: markdown_relpath(v, REPORT_DIR) for k, v in figs.items()}

    target_after3 = [r for r in target_rows if r["window"] == "after3s"]
    target_4to6 = [r for r in target_rows if r["window"] == "4to6s"]
    ko_endpoints = [r for r in scan_rows if str(r["case"]) in {"Ko_o0", "Ko_o5"}]
    ku_aggregate = [r for r in scan_rows if str(r["case"]).startswith("Ku_u")]
    ku_relative_display = with_case_label(ku_relative_rows)

    config_display = [
        {
            "joint": JOINT_LABEL[int(float(r["joint_id"]))],
            "J": f"{float(r['Jeff']):.8f}",
            "g": f"$[{float(r['g_q']):.4e},\\ {float(r['g_dq']):.4e}]^T$",
            "pinv": f"$[{float(r['pinv_q']):.4f},\\ {float(r['pinv_dq']):.4f}]$",
            "weighted_pinv": f"$[{float(r['weighted_pinv_q']):.4e},\\ {float(r['weighted_pinv_dq']):.4f}]$",
            "ku": f"$({fmt(float(r['ku_q']))},\\ {fmt(float(r['ku_dq']))})$",
        }
        for r in config_rows
    ]
    log_display = [
        {
            "joint": JOINT_LABEL[int(float(r["joint_id"]))],
            "old_eta_u_rms": float(r["old_eta_u_rms"]),
            "pinv_eta_u_rms": float(r["pinv_eta_u_rms"]),
            "weighted_eta_u_rms": float(r["weighted_eta_u_rms"]),
            "pinv_over_old_rms": float(r["pinv_over_old_rms"]),
            "weighted_over_old_rms": float(r["weighted_over_old_rms"]),
            "corr_old_pinv": float(r["corr_old_pinv"]),
        }
        for r in log_rows
    ]
    contrib_display = [
        {
            "joint": JOINT_LABEL[int(r["joint_id"])],
            "state_q": r["x_minus_xhat_q_rms"],
            "state_dq": r["x_minus_xhat_dq_rms"],
            "old_eta": r["old_eta_u_rms"],
            "q_part": r["pinv_q_contrib_rms"],
            "dq_part": r["pinv_dq_contrib_rms"],
            "total": r["pinv_total_rms"],
            "xbar_total": r["pinv_xbar_rms"],
        }
        for r in contribution_rows
    ]
    ko_display = with_case_label(ko_endpoints)
    ku_display = with_case_label(ku_aggregate)

    lines = [
        "# EID 输入逆改进推导、数值验证与现象分析",
        "",
        "## 摘要",
        "",
        "本文说明右髋俯仰关节与右膝关节反相正弦跟踪实验中的输入域等效扰动控制方法，并分析一个新的输入逆估计思路。实验对象是 Unitree H1 右髋俯仰关节与右膝两个关节，参考轨迹为 0.8 Hz 反相正弦运动，控制周期为 0.002 s。统计稳态误差时去掉前 3 s 启动融合段。",
        "",
        "主要结论有三点。第一，当前控制器的稳态跟踪误差是正常的：在观测器增益为 $K_o=(0.8,0.2)$、输入补偿增益为 $K_u=(12,1)$ 的第 1 次重复实验中，启动后右髋位置均方根误差为 0.00773 rad，右膝为 0.01673 rad。第二，直接使用输入矩阵伪逆 $g^+$ 反算输入扰动会得到远大于当前补偿的力矩量：右髋从 0.0132 N m 增至 5.91 N m，右膝从 0.0184 N m 增至 2.28 N m。第三，这个放大主要来自速度通道的 $J_\\mathrm{eff}/T_s$ 系数，而不是跟踪失败。",
        "",
        "## 1. 原始算法核心",
        "",
        "单关节状态写为",
        "",
        "$$",
        "x_k=\\begin{bmatrix}q_k\\\\\\dot q_k\\end{bmatrix}.",
        "$$",
        "",
        "控制器维护标称观测状态 $\\hat x_k$ 和 EID 状态估计 $\\eta_k$，并用二者构造中心反馈状态",
        "",
        "$$",
        "\\bar x_k=\\hat x_k+\\eta_k.",
        "$$",
        "",
        "后续反馈并不直接围绕测量状态 $x_k$，而是围绕 $\\bar x_k$。解析逆模型根据当前参考 $r_k$ 与下一步参考 $r_{k+1}$ 计算标称前馈力矩",
        "",
        "$$",
        "u_k^*=\\hat{\\mathcal I}(r_k,r_{k+1}-r_k).",
        "$$",
        "",
        "当前实现中的输入补偿不是显式输入矩阵逆，而是经验映射",
        "",
        "$$",
        "\\eta_{u,k}=k_{u,q}\\eta_{q,k}+k_{u,\\dot q}\\eta_{\\dot q,k}.",
        "$$",
        "",
        "最终控制律为",
        "",
        "$$",
        "u_k=u_k^*-\\eta_{u,k}+K(r_k-\\bar x_k).",
        "$$",
        "",
        "其中 $u_k^*$ 是标称前馈，$-\\eta_{u,k}$ 是输入端补偿，$K(r_k-\\bar x_k)$ 是围绕中心反馈状态的 PD 修正。控制力矩计算后，观测器和 EID 估计按",
        "",
        "$$",
        "\\hat x_{k+1}=\\hat\\Phi(\\bar x_k,u_k)",
        "$$",
        "",
        "$$",
        "\\eta_{k+1}=\\alpha K_o(x_k-\\bar x_k)+(1-\\alpha)\\eta_k",
        "$$",
        "",
        "更新。这里的残差是 $x_k-\\bar x_k$，不是 $x_k-\\hat x_k$。这一点后面解释输入逆数值时很重要。",
        "",
        "## 2. 输入逆改进思路",
        "",
        "更严格的输入域解释是：如果观测误差项 $K_o(x-\\hat x)$ 要作为输入端等效扰动注入逆模型，需要先通过输入矩阵的逆从状态域转换到力矩域。离散系统可以写成",
        "",
        "$$",
        "x_{k+1}=f(x_k)+g(x_k)u_k.",
        "$$",
        "",
        "于是输入端扰动估计可写为",
        "",
        "$$",
        "\\hat d_u=g^{-1}K_o(x-\\hat x).",
        "$$",
        "",
        "当前单关节模型采用半隐式欧拉离散化：",
        "",
        "$$",
        "\\dot q_{k+1}=\\dot q_k+\\frac{T_s}{J_\\mathrm{eff}}(u_k-\\beta_k),",
        "$$",
        "",
        "$$",
        "q_{k+1}=q_k+T_s\\dot q_{k+1}.",
        "$$",
        "",
        "其中 $\\beta_k=b\\dot q_k+A\\sin q_k+B\\cos q_k+\\tau_0$ 属于漂移项。因此输入矩阵为",
        "",
        "$$",
        "g=\\begin{bmatrix}T_s^2/J_\\mathrm{eff}\\\\T_s/J_\\mathrm{eff}\\end{bmatrix}.",
        "$$",
        "",
        "由于 $g$ 是 $2\\times1$ 列向量，实际使用 Moore-Penrose 左伪逆",
        "",
        "$$",
        "g^+=(g^Tg)^{-1}g^T.",
        "$$",
        "",
        "本轮只进行离线估计：",
        "",
        "$$",
        "\\eta_u^{\\mathrm{pinv}}=g^+K_o(x-\\hat x).",
        "$$",
        "",
        "加权伪逆不是这一步的必要条件。本文额外计算它，只是为了观察不同残差权重会怎样改变输入反推尺度：",
        "",
        "$$",
        "g_W^+=(g^TWg)^{-1}g^TW",
        "$$",
        "",
        "这里的主分析仍然是未加权 Moore-Penrose 伪逆。加权形式只有在明确知道位置残差和速度残差的可信度、噪声水平和单位归一化方式之后，才适合作为候选方案；本文中的默认权重只作为尺度对照。二者都不直接接入实时控制器。",
        "",
        "## 3. 输入逆数值",
        "",
        "对于观测器增益 $K_o=(0.8,0.2)$、输入补偿增益 $K_u=(12,1)$ 的右髋俯仰关节与右膝反相正弦跟踪实验，输入矩阵和伪逆如下。",
        "",
        markdown_table(
            config_display,
            ["joint", "J", "g", "pinv", "weighted_pinv", "ku"],
            {
                "joint": "关节",
                "J": "$J_\\mathrm{eff}$",
                "g": "$g$",
                "pinv": "$g^+$",
                "weighted_pinv": "$g_W^+$",
                "ku": "$K_u$",
            },
        ),
        "",
        f"![输入映射系数对比]({fig_link['coeff']})",
        "",
        "右髋速度通道的 $g^+$ 系数为 502.5406，右膝为 125.0737；而当前经验补偿的速度通道系数均为 1。这说明二者不是同一个尺度的控制量。",
        "",
        "## 4. 离线重放结果",
        "",
        "对观测器增益 $K_o=(0.8,0.2)$、输入补偿增益 $K_u=(12,1)$ 的第 1 次重复实验进行离线重放，比较当前补偿 $K_u\\eta$ 与输入逆估计。",
        "",
        markdown_table(
            log_display,
            ["joint", "old_eta_u_rms", "pinv_eta_u_rms", "weighted_eta_u_rms", "pinv_over_old_rms", "weighted_over_old_rms", "corr_old_pinv"],
            {
                "joint": "关节",
                "old_eta_u_rms": "$K_u\\eta$ RMS",
                "pinv_eta_u_rms": "$g^+K_o(x-\\hat x)$ RMS",
                "weighted_eta_u_rms": "$g_W^+K_o(x-\\hat x)$ RMS",
                "pinv_over_old_rms": "$g^+$ / 当前",
                "weighted_over_old_rms": "$g_W^+$ / 当前",
                "corr_old_pinv": "相关系数",
            },
        ),
        "",
        f"![输入补偿 RMS 对比]({fig_link['eta']})",
        "",
        "裸伪逆估计的 RMS 远大于当前补偿：右髋约为当前补偿的 447 倍，右膝约为 124 倍。默认加权伪逆在这里不是降噪方案，反而更大；原因是默认权重强烈强调位置通道，而位置输入增益 $T_s^2/J_\\mathrm{eff}$ 很小，反算时会得到非常大的位置系数。",
        "",
        "贡献拆分如下。",
        "",
        markdown_table(
            contrib_display,
            ["joint", "state_q", "state_dq", "old_eta", "q_part", "dq_part", "total", "xbar_total"],
            {
                "joint": "关节",
                "state_q": "$q-\\hat q$ RMS",
                "state_dq": "$\\dot q-\\hat{\\dot q}$ RMS",
                "old_eta": "当前补偿 RMS",
                "q_part": "$g^+$ 位置项",
                "dq_part": "$g^+$ 速度项",
                "total": "$g^+$ 总量",
                "xbar_total": "若用 $x-\\bar x$",
            },
        ),
        "",
        f"![输入逆贡献拆分]({fig_link['contrib']})",
        "",
        "几乎全部放大都来自速度通道。右髋位置项只有 0.00071 N m，速度项为 5.91 N m；右膝位置项只有 0.00019 N m，速度项为 2.28 N m。",
        "",
        "## 5. 稳态跟踪误差对比",
        "",
        "首先看观测器增益 $K_o=(0.8,0.2)$、输入补偿增益 $K_u=(12,1)$ 的第 1 次重复实验。去掉前 3 s 启动融合段后，跟踪误差如下。",
        "",
        markdown_table(
            target_after3,
            ["joint_name", "q_rmse", "q_p95_abs", "q_peak_abs", "dq_rmse", "eta_u_rms", "tau_cmd_rms"],
            {
                "joint_name": "关节",
                "q_rmse": "位置 RMSE",
                "q_p95_abs": "位置 95%",
                "q_peak_abs": "位置峰值",
                "dq_rmse": "速度 RMSE",
                "eta_u_rms": "输入补偿 RMS",
                "tau_cmd_rms": "力矩 RMS",
            },
        ),
        "",
        "在 4 s 到 6 s 的稳态窗口中，误差为：",
        "",
        markdown_table(
            target_4to6,
            ["joint_name", "q_rmse", "q_p95_abs", "q_peak_abs", "dq_rmse", "eta_u_rms"],
            {
                "joint_name": "关节",
                "q_rmse": "位置 RMSE",
                "q_p95_abs": "位置 95%",
                "q_peak_abs": "位置峰值",
                "dq_rmse": "速度 RMSE",
                "eta_u_rms": "输入补偿 RMS",
            },
        ),
        "",
        "这说明跟踪误差本身并不异常。启动后右髋位置 RMSE 为 0.00773 rad，右膝为 0.01673 rad，约等于 0.44 deg 和 0.96 deg。",
        "",
        "### 5.1 观测器增益对比",
        "",
        "先比较无观测器补偿与较高观测器补偿。输入补偿增益固定为 $K_u=(12,1)$。",
        "",
        markdown_table(
            with_case_label(ko_endpoints),
            ["experiment", "joint_name", "repeats", "q_rmse_mean", "q_p95_abs_mean", "q_peak_abs_mean", "dq_rmse_mean", "eta_u_rms_mean", "tau_cmd_rms_mean"],
            {
                "experiment": "实验条件",
                "joint_name": "关节",
                "repeats": "重复次数",
                "q_rmse_mean": "位置 RMSE",
                "q_p95_abs_mean": "位置 95%",
                "q_peak_abs_mean": "位置峰值",
                "dq_rmse_mean": "速度 RMSE",
                "eta_u_rms_mean": "输入补偿 RMS",
                "tau_cmd_rms_mean": "力矩 RMS",
            },
        ),
        "",
        "没有观测器补偿时，右髋和右膝位置 RMSE 分别约为 0.116 rad 和 0.330 rad；使用 $K_o=(1.0,0.25)$ 后分别降到 0.00767 rad 和 0.01619 rad。观测器补偿对跟踪误差有决定性影响。",
        "",
        "### 5.2 输入补偿增益完整对比",
        "",
        "再比较输入补偿增益。此时观测器增益固定为 $K_o=(0.8,0.2)$，输入补偿增益从 $K_u=(6,0.5)$ 增加到 $K_u=(15,1.25)$。统计窗口均为启动后 $t_\\mathrm{rel}\\ge 3$ s，每组 3 次重复。",
        "",
        markdown_table(
            with_case_label(ku_aggregate),
            ["experiment", "joint_name", "repeats", "q_rmse_mean", "q_rmse_min", "q_rmse_max", "dq_rmse_mean", "eta_u_rms_mean", "tau_cmd_rms_mean"],
            {
                "experiment": "实验条件",
                "joint_name": "关节",
                "repeats": "重复次数",
                "q_rmse_mean": "位置 RMSE 均值",
                "q_rmse_min": "最小",
                "q_rmse_max": "最大",
                "dq_rmse_mean": "速度 RMSE",
                "eta_u_rms_mean": "输入补偿 RMS",
                "tau_cmd_rms_mean": "力矩 RMS",
            },
        ),
        "",
        f"![稳态位置跟踪误差]({fig_link['tracking']})",
        "",
        "相对最低输入补偿增益 $K_u=(6,0.5)$ 的变化如下。",
        "",
        markdown_table(
            ku_relative_display,
            ["experiment", "joint_name", "q_rmse_mean", "q_rmse_delta_vs_ku1_pct", "dq_rmse_mean", "eta_u_rms_mean", "eta_u_delta_vs_ku1_pct", "tau_cmd_rms_mean", "tau_delta_vs_ku1_pct"],
            {
                "experiment": "实验条件",
                "joint_name": "关节",
                "q_rmse_mean": "位置 RMSE",
                "q_rmse_delta_vs_ku1_pct": "位置变化百分比",
                "dq_rmse_mean": "速度 RMSE",
                "eta_u_rms_mean": "输入补偿 RMS",
                "eta_u_delta_vs_ku1_pct": "补偿变化百分比",
                "tau_cmd_rms_mean": "力矩 RMS",
                "tau_delta_vs_ku1_pct": "力矩变化百分比",
            },
        ),
        "",
        "输入补偿增益增大后，输入补偿 RMS 明显上升。例如从 $K_u=(6,0.5)$ 到 $K_u=(12,1)$，右髋输入补偿 RMS 增加约 82%，右膝增加约 89%。但是位置 RMSE 只下降约 0.3%。到 $K_u=(15,1.25)$ 时，输入补偿继续增大，而位置误差没有继续下降，右髋还略微变差。这说明在当前稳态窗口里，继续放大输入补偿不是主要误差杠杆。",
        "",
        "## 6. 现象原因分析",
        "",
        "第一，速度通道天然带有采样周期倒数。未加权伪逆近似为",
        "",
        "$$",
        "g^+\\approx\\begin{bmatrix}J_\\mathrm{eff} & J_\\mathrm{eff}/T_s\\end{bmatrix}.",
        "$$",
        "",
        "当 $T_s=0.002$ s 时，$1/T_s=500$。右髋 $g^+_{\\dot q}=502.5406$，再乘速度观测器增益 0.2 后仍约为 100.5。因此速度残差 0.05 rad/s 就会得到约 5 N m 的输入扰动估计。",
        "",
        "第二，当前经验输入补偿 $K_u\\eta$ 不是输入矩阵伪逆。它作用在滤波后的 $\\eta$ 上，而不是直接作用在原始残差 $x-\\hat x$ 上；它更像保守的闭环补偿强度。",
        "",
        "第三，当前控制器更新 EID 时使用 $x-\\bar x$。如果离线估计也使用 $x-\\bar x$，结果会小一些：右髋从 5.91 降到 4.97 N m，右膝从 2.28 降到 1.91 N m。但这仍然远大于当前补偿，所以残差定义不是主因。",
        "",
        "第四，加权伪逆本身不是问题，但权重选择决定结果。若目标是抑制速度噪声，应降低速度残差的权重，或者先对速度残差滤波和限幅；当前默认权重为 $w_q=0.5/T_s^2=125000$、$w_{\\dot q}=1$，它强调位置通道，而位置输入增益 $T_s^2/J_\\mathrm{eff}$ 极小，所以反算时会产生非常大的位置系数。",
        "",
        "## 7. 结论",
        "",
        "1. 当前右髋俯仰关节与右膝反相正弦跟踪实验的稳态跟踪误差正常。",
        "2. 观测器增益是当前跟踪改善的主要来源；输入补偿增益继续增大时，误差收益很小。",
        "3. 裸输入逆 $g^+K_o(x-\\hat x)$ 是有用的离线诊断量，但不应直接替换实时控制器中的经验输入补偿。",
        "4. 如果后续要接入输入逆补偿，应至少采用中心反馈残差、低通滤波、限幅和很小的缩放系数：",
        "",
        "$$",
        "\\eta_u=\\gamma\\,\\mathrm{sat}\\left(g^+K_o(x-\\bar x)\\right),\\qquad \\gamma\\ll1.",
        "$$",
        "",
        "## 附录数据表",
        "",
        f"- [输入逆系数表]({markdown_relpath(OUT_DIR / 'input_inverse_coefficients.csv', REPORT_DIR)})",
        f"- [输入补偿重放表]({markdown_relpath(OUT_DIR / 'input_inverse_log_replay.csv', REPORT_DIR)})",
        f"- [稳态跟踪扫描表]({markdown_relpath(OUT_DIR / 'anti_phase_tracking_scan_summary.csv', REPORT_DIR)})",
        f"- [输入补偿增益重复明细表]({markdown_relpath(OUT_DIR / 'input_gain_tracking_repeat_summary.csv', REPORT_DIR)})",
        f"- [输入补偿增益相对变化表]({markdown_relpath(OUT_DIR / 'input_gain_tracking_relative_summary.csv', REPORT_DIR)})",
        f"- [输入逆贡献拆分表]({markdown_relpath(OUT_DIR / 'pinv_contribution_breakdown.csv', REPORT_DIR)})",
        "",
    ]
    REPORT_PATH.write_text("\n".join(lines), encoding="utf-8")


def write_report_textbook(
    config_rows: list[dict[str, str]],
    log_rows: list[dict[str, str]],
    target_rows: list[dict[str, object]],
    scan_rows: list[dict[str, object]],
    ku_relative_rows: list[dict[str, object]],
    contribution_rows: list[dict[str, object]],
    figs: dict[str, Path],
) -> None:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    fig_link = {k: markdown_relpath(v, REPORT_DIR) for k, v in figs.items()}

    target_after3 = [r for r in target_rows if r["window"] == "after3s"]
    target_4to6 = [r for r in target_rows if r["window"] == "4to6s"]
    ko_endpoints = [r for r in scan_rows if str(r["case"]) in {"Ko_o0", "Ko_o5"}]
    ku_aggregate = [r for r in scan_rows if str(r["case"]).startswith("Ku_u")]
    ku_relative_display = with_case_label(ku_relative_rows)

    config_display = [
        {
            "joint": JOINT_LABEL[int(float(r["joint_id"]))],
            "J": f"{float(r['Jeff']):.8f}",
            "g": f"$[{float(r['g_q']):.4e},\\ {float(r['g_dq']):.4e}]^T$",
            "pinv": f"$[{float(r['pinv_q']):.4f},\\ {float(r['pinv_dq']):.4f}]$",
            "weighted_pinv": f"$[{float(r['weighted_pinv_q']):.4e},\\ {float(r['weighted_pinv_dq']):.4f}]$",
            "ku": f"$({fmt(float(r['ku_q']))},\\ {fmt(float(r['ku_dq']))})$",
        }
        for r in config_rows
    ]
    log_display = [
        {
            "joint": JOINT_LABEL[int(float(r["joint_id"]))],
            "old_eta_u_rms": float(r["old_eta_u_rms"]),
            "pinv_eta_u_rms": float(r["pinv_eta_u_rms"]),
            "weighted_eta_u_rms": float(r["weighted_eta_u_rms"]),
            "pinv_over_old_rms": float(r["pinv_over_old_rms"]),
            "weighted_over_old_rms": float(r["weighted_over_old_rms"]),
            "corr_old_pinv": float(r["corr_old_pinv"]),
        }
        for r in log_rows
    ]
    contrib_display = [
        {
            "joint": JOINT_LABEL[int(r["joint_id"])],
            "state_q": r["x_minus_xhat_q_rms"],
            "state_dq": r["x_minus_xhat_dq_rms"],
            "old_eta": r["old_eta_u_rms"],
            "q_part": r["pinv_q_contrib_rms"],
            "dq_part": r["pinv_dq_contrib_rms"],
            "total": r["pinv_total_rms"],
            "xbar_total": r["pinv_xbar_rms"],
        }
        for r in contribution_rows
    ]

    lines = [
        "# EID 输入逆：从原控制律到离线数值验证",
        "",
        "## 读者导引",
        "",
        "这一节先给出全局图景。当前控制器已经能较好地完成 Unitree H1 右髋俯仰关节与右膝的反相正弦跟踪。现在的问题不是“控制器为什么失效”，而是一个更细的问题：EID 观测器得到的是状态域里的误差修正，若要把它解释成输入端的等效力矩扰动，应该怎样从状态域换算到力矩域？",
        "",
        "答案分两步。第一步，用当前单关节离散模型写出输入矩阵 $g$，再求普通 Moore-Penrose 伪逆 $g^+$。第二步，只用已有实验日志做离线重放，检查这个输入逆估计的量级是否合理。这个报告的核心结论是：普通伪逆的数学推导成立，但直接得到的输入扰动力矩太大；因此它目前适合做诊断，不适合直接替换实时控制器里的经验输入补偿。",
        "",
        "## 1. 问题背景：EID 到底在补偿什么",
        "",
        "先看一个单关节。它的状态包括位置和速度：",
        "",
        "$$",
        "x_k=\\begin{bmatrix}q_k\\\\\\dot q_k\\end{bmatrix}.",
        "$$",
        "",
        "真实机器人不会完全等于标称模型。摩擦、重力参数误差、未建模耦合、传感器延迟都会让模型预测和真实测量之间出现差别。EID 的思想是把这些差别看成一种“等效扰动”，再通过观测器把它估出来。当前控制器维护两个量：一个是标称模型预测的状态 $\\hat x_k$，另一个是 EID 估计的状态修正 $\\eta_k$。二者相加得到反馈使用的中心状态：",
        "",
        "$$",
        "\\bar x_k=\\hat x_k+\\eta_k.",
        "$$",
        "",
        "这个中心状态很重要。控制器不是简单地围绕真实测量 $x_k$ 做反馈，也不是只相信模型预测 $\\hat x_k$，而是在二者之间用 $\\eta_k$ 做一个动态修正。这样做的好处是，反馈项看到的是一个已经包含扰动估计的状态。",
        "",
        "## 2. 原控制律：前馈、反馈与经验输入补偿",
        "",
        "当前控制律可以拆成三部分。第一部分是解析逆模型给出的标称前馈力矩：",
        "",
        "$$",
        "u_k^*=\\hat{\\mathcal I}(r_k,r_{k+1}-r_k).",
        "$$",
        "",
        "这里 $r_k$ 是当前参考状态，$r_{k+1}$ 是下一步参考状态。它回答的问题是：如果机器人完全符合标称模型，为了沿参考轨迹走，应该给多大力矩？",
        "",
        "第二部分是围绕中心状态 $\\bar x_k$ 的反馈修正：",
        "",
        "$$",
        "K(r_k-\\bar x_k).",
        "$$",
        "",
        "第三部分是当前实现中的输入端补偿：",
        "",
        "$$",
        "\\eta_{u,k}=k_{u,q}\\eta_{q,k}+k_{u,\\dot q}\\eta_{\\dot q,k}.",
        "$$",
        "",
        "于是总控制律为",
        "",
        "$$",
        "u_k=u_k^*-\\eta_{u,k}+K(r_k-\\bar x_k).",
        "$$",
        "",
        "这一式子说明，当前的输入补偿 $\\eta_{u,k}$ 不是由输入矩阵严格反推出来的，而是把 $\\eta_k$ 的位置分量和速度分量按经验系数映射成力矩。对右髋俯仰关节和右膝，这组实验中采用的是 $K_u=(12,1)$。",
        "",
        "观测器更新也要注意。EID 状态按下面形式更新：",
        "",
        "$$",
        "\\eta_{k+1}=\\alpha K_o(x_k-\\bar x_k)+(1-\\alpha)\\eta_k.",
        "$$",
        "",
        "这里用的是 $x_k-\\bar x_k$，即测量状态和中心状态之间的残差。它不是 $x_k-\\hat x_k$。后文比较输入逆时，这个差别会影响数值，但不是造成百倍放大的主因。",
        "",
        "## 3. 新问题：状态扰动怎样变成输入扰动",
        "",
        "现在提出一个更严格的问题。观测器项 $K_o(x-\\hat x)$ 是一个状态向量修正，它和 $q$、$\\dot q$ 同维。如果想把它放到输入端解释为力矩扰动，就必须回答：什么样的力矩扰动，经过离散动力学的一步传播，会造成这样的状态变化？",
        "",
        "把离散单关节模型写成",
        "",
        "$$",
        "x_{k+1}=f(x_k)+g(x_k)u_k.",
        "$$",
        "",
        "这里 $f$ 表示不含当前输入的漂移项，$g$ 表示当前输入如何影响下一步状态。如果要从状态修正反推输入端扰动，自然得到",
        "",
        "$$",
        "\\hat d_u=g^+K_o(x-\\hat x).",
        "$$",
        "",
        "这就是本报告中的“输入逆”。它不是新的闭环控制实验，而是先用日志离线计算：如果当时采用这种输入域解释，估出来的等效力矩扰动会有多大？",
        "",
        "## 4. 输入矩阵与普通伪逆",
        "",
        "当前单关节正模型采用半隐式欧拉离散化：",
        "",
        "$$",
        "\\dot q_{k+1}=\\dot q_k+\\frac{T_s}{J_\\mathrm{eff}}(u_k-\\beta_k),",
        "$$",
        "",
        "$$",
        "q_{k+1}=q_k+T_s\\dot q_{k+1}.",
        "$$",
        "",
        "其中 $\\beta_k=b\\dot q_k+A\\sin q_k+B\\cos q_k+\\tau_0$ 不乘当前输入 $u_k$，所以它属于漂移项 $f(x_k)$，不进入输入矩阵。现在只保留和 $u_k$ 有关的部分。由速度更新式可得",
        "",
        "$$",
        "\\dot q_{k+1}=\\cdots+\\frac{T_s}{J_\\mathrm{eff}}u_k,",
        "$$",
        "",
        "所以输入对下一步速度的增益是",
        "",
        "$$",
        "g_{\\dot q}=\\frac{T_s}{J_\\mathrm{eff}}.",
        "$$",
        "",
        "位置更新使用的是已经更新后的速度 $\\dot q_{k+1}$，因此输入先影响速度，再通过 $q_{k+1}=q_k+T_s\\dot q_{k+1}$ 影响位置：",
        "",
        "$$",
        "q_{k+1}=\\cdots+T_s\\frac{T_s}{J_\\mathrm{eff}}u_k.",
        "$$",
        "",
        "所以输入对下一步位置的增益是",
        "",
        "$$",
        "g_q=\\frac{T_s^2}{J_\\mathrm{eff}}.",
        "$$",
        "",
        "把位置和速度两个通道合在一起，就得到输入矩阵",
        "",
        "$$",
        "g=\\begin{bmatrix}T_s^2/J_\\mathrm{eff}\\\\T_s/J_\\mathrm{eff}\\end{bmatrix}.",
        "$$",
        "",
        "这个矩阵有两行一列：两行对应位置和速度，一个列对应一个关节力矩。因此不能求普通方阵逆，只能求左伪逆。记",
        "",
        "$$",
        "g=\\begin{bmatrix}g_q\\\\g_{\\dot q}\\end{bmatrix},",
        "$$",
        "",
        "则 Moore-Penrose 左伪逆为",
        "",
        "$$",
        "g^+=(g^Tg)^{-1}g^T.",
        "$$",
        "",
        "由于这里 $g^Tg$ 是一个标量，上式可以直接展开成",
        "",
        "$$",
        "g^+=\\frac{1}{g_q^2+g_{\\dot q}^2}\\begin{bmatrix}g_q & g_{\\dot q}\\end{bmatrix}.",
        "$$",
        "",
        "因此，输入逆估计本质上是在问：给定一个位置残差和速度残差，哪个等效力矩在最小二乘意义下最能解释这两个状态残差？",
        "",
        "加权伪逆不是必要步骤。本文也列出 $g_W^+=(g^TWg)^{-1}g^TW$，只是为了说明不同权重会怎样改变结果。主结论来自未加权的普通伪逆。",
        "",
        "## 5. 具体数值：先看系数有多大",
        "",
        "实验的控制周期是 $T_s=0.002$ s。对于右髋俯仰关节和右膝，输入矩阵和伪逆如下。",
        "",
        markdown_table(
            config_display,
            ["joint", "J", "g", "pinv", "weighted_pinv", "ku"],
            {
                "joint": "关节",
                "J": "$J_\\mathrm{eff}$",
                "g": "$g$",
                "pinv": "$g^+$",
                "weighted_pinv": "$g_W^+$",
                "ku": "当前 $K_u$",
            },
        ),
        "",
        f"![输入映射系数对比]({fig_link['coeff']})",
        "",
        "这个表已经给出第一个警告信号。右髋俯仰关节的普通伪逆速度系数是 502.5406，右膝是 125.0737；而当前经验输入补偿的速度系数是 1。也就是说，普通伪逆不是把当前补偿略微放大，而是把速度残差按百级系数映射到力矩。",
        "",
        "## 6. 离线重放：如果当时用普通伪逆，会估出多大力矩",
        "",
        "接下来用观测器增益 $K_o=(0.8,0.2)$、输入补偿增益 $K_u=(12,1)$ 的第一组反相正弦跟踪日志做离线重放。比较对象有三个：当前日志中的经验补偿 $K_u\\eta$，普通伪逆估计 $g^+K_o(x-\\hat x)$，以及只作为尺度对照的加权伪逆估计。",
        "",
        markdown_table(
            log_display,
            ["joint", "old_eta_u_rms", "pinv_eta_u_rms", "weighted_eta_u_rms", "pinv_over_old_rms", "weighted_over_old_rms", "corr_old_pinv"],
            {
                "joint": "关节",
                "old_eta_u_rms": "当前补偿 RMS",
                "pinv_eta_u_rms": "普通伪逆 RMS",
                "weighted_eta_u_rms": "加权伪逆 RMS",
                "pinv_over_old_rms": "普通伪逆 / 当前",
                "weighted_over_old_rms": "加权伪逆 / 当前",
                "corr_old_pinv": "与当前补偿相关系数",
            },
        ),
        "",
        f"![输入补偿 RMS 对比]({fig_link['eta']})",
        "",
        "普通伪逆的符号和趋势并非完全不相关：它与当前补偿的相关系数约为 0.92。但幅值差异太大。右髋俯仰关节从 0.0132 N m 增加到 5.91 N m，约为 447 倍；右膝从 0.0184 N m 增加到 2.28 N m，约为 124 倍。因此，普通伪逆目前只能说明“这个方向值得诊断”，不能说明“可以直接上控制器”。",
        "",
        "## 7. 放大来自哪里：位置项还是速度项",
        "",
        "为了找原因，把普通伪逆估计拆成位置残差贡献和速度残差贡献。",
        "",
        markdown_table(
            contrib_display,
            ["joint", "state_q", "state_dq", "old_eta", "q_part", "dq_part", "total", "xbar_total"],
            {
                "joint": "关节",
                "state_q": "$q-\\hat q$ RMS",
                "state_dq": "$\\dot q-\\hat{\\dot q}$ RMS",
                "old_eta": "当前补偿 RMS",
                "q_part": "位置项贡献",
                "dq_part": "速度项贡献",
                "total": "普通伪逆总量",
                "xbar_total": "若改用 $x-\\bar x$",
            },
        ),
        "",
        f"![输入逆贡献拆分]({fig_link['contrib']})",
        "",
        "结论非常集中：几乎全部放大都来自速度通道。右髋俯仰关节的位置项只有 0.00071 N m，速度项是 5.91 N m；右膝的位置项只有 0.00019 N m，速度项是 2.28 N m。换用 $x-\\bar x$ 会把结果稍微降下来，但仍然远大于当前补偿。",
        "",
        "## 8. 这是否说明跟踪误差异常",
        "",
        "不说明。输入逆估计很大，与当前跟踪坏掉是两回事。当前控制器实际运行时仍使用经验输入补偿，而不是普通伪逆补偿。去掉前 3 s 启动融合段后，观测器增益 $K_o=(0.8,0.2)$、输入补偿增益 $K_u=(12,1)$ 的第一组实验误差如下。",
        "",
        markdown_table(
            target_after3,
            ["joint_name", "q_rmse", "q_p95_abs", "q_peak_abs", "dq_rmse", "eta_u_rms", "tau_cmd_rms"],
            {
                "joint_name": "关节",
                "q_rmse": "位置 RMSE",
                "q_p95_abs": "位置误差 95%",
                "q_peak_abs": "位置误差峰值",
                "dq_rmse": "速度 RMSE",
                "eta_u_rms": "输入补偿 RMS",
                "tau_cmd_rms": "力矩 RMS",
            },
        ),
        "",
        "在 4 s 到 6 s 的更窄稳态窗口中，误差为：",
        "",
        markdown_table(
            target_4to6,
            ["joint_name", "q_rmse", "q_p95_abs", "q_peak_abs", "dq_rmse", "eta_u_rms"],
            {
                "joint_name": "关节",
                "q_rmse": "位置 RMSE",
                "q_p95_abs": "位置误差 95%",
                "q_peak_abs": "位置误差峰值",
                "dq_rmse": "速度 RMSE",
                "eta_u_rms": "输入补偿 RMS",
            },
        ),
        "",
        "启动段之后，右髋俯仰关节位置 RMSE 为 0.00773 rad，右膝为 0.01673 rad，约等于 0.44 deg 和 0.96 deg。这个量级说明当前跟踪本身是正常的。",
        "",
        "## 9. 和增益扫描放在一起看",
        "",
        "先固定输入补偿增益 $K_u=(12,1)$，比较没有观测器补偿和较高观测器补偿的情况。",
        "",
        markdown_table(
            with_case_label(ko_endpoints),
            ["experiment", "joint_name", "repeats", "q_rmse_mean", "q_p95_abs_mean", "q_peak_abs_mean", "dq_rmse_mean", "eta_u_rms_mean", "tau_cmd_rms_mean"],
            {
                "experiment": "实验条件",
                "joint_name": "关节",
                "repeats": "重复次数",
                "q_rmse_mean": "位置 RMSE",
                "q_p95_abs_mean": "位置误差 95%",
                "q_peak_abs_mean": "位置误差峰值",
                "dq_rmse_mean": "速度 RMSE",
                "eta_u_rms_mean": "输入补偿 RMS",
                "tau_cmd_rms_mean": "力矩 RMS",
            },
        ),
        "",
        "没有观测器补偿时，右髋俯仰关节和右膝位置 RMSE 分别约为 0.116 rad 和 0.330 rad；使用 $K_o=(1.0,0.25)$ 后分别降到 0.00767 rad 和 0.01619 rad。因此，当前实验中真正显著改善跟踪的是观测器补偿。",
        "",
        "再固定观测器增益 $K_o=(0.8,0.2)$，把输入补偿增益从 $K_u=(6,0.5)$ 增加到 $K_u=(15,1.25)$。每组统计三次重复实验，统计窗口均为启动后。",
        "",
        markdown_table(
            with_case_label(ku_aggregate),
            ["experiment", "joint_name", "repeats", "q_rmse_mean", "q_rmse_min", "q_rmse_max", "dq_rmse_mean", "eta_u_rms_mean", "tau_cmd_rms_mean"],
            {
                "experiment": "实验条件",
                "joint_name": "关节",
                "repeats": "重复次数",
                "q_rmse_mean": "位置 RMSE 均值",
                "q_rmse_min": "最小",
                "q_rmse_max": "最大",
                "dq_rmse_mean": "速度 RMSE",
                "eta_u_rms_mean": "输入补偿 RMS",
                "tau_cmd_rms_mean": "力矩 RMS",
            },
        ),
        "",
        f"![稳态位置跟踪误差]({fig_link['tracking']})",
        "",
        "相对最低输入补偿增益 $K_u=(6,0.5)$ 的变化如下。",
        "",
        markdown_table(
            ku_relative_display,
            ["experiment", "joint_name", "q_rmse_mean", "q_rmse_delta_vs_ku1_pct", "dq_rmse_mean", "eta_u_rms_mean", "eta_u_delta_vs_ku1_pct", "tau_cmd_rms_mean", "tau_delta_vs_ku1_pct"],
            {
                "experiment": "实验条件",
                "joint_name": "关节",
                "q_rmse_mean": "位置 RMSE",
                "q_rmse_delta_vs_ku1_pct": "位置变化百分比",
                "dq_rmse_mean": "速度 RMSE",
                "eta_u_rms_mean": "输入补偿 RMS",
                "eta_u_delta_vs_ku1_pct": "补偿变化百分比",
                "tau_cmd_rms_mean": "力矩 RMS",
                "tau_delta_vs_ku1_pct": "力矩变化百分比",
            },
        ),
        "",
        "这张表解释了为什么不能简单认为“输入补偿越大越好”。从 $K_u=(6,0.5)$ 到 $K_u=(12,1)$，右髋俯仰关节的输入补偿 RMS 增加约 82%，右膝增加约 89%，但位置 RMSE 只改善约 0.3%。继续增大到 $K_u=(15,1.25)$ 后，补偿更大，误差没有继续下降。这个现象支持前面的判断：输入逆直接给出的巨大力矩量级不是当前误差的主要解决方向。",
        "",
        "## 10. 为什么普通伪逆会这么大",
        "",
        "原因可以从一个近似式看出来。因为 $T_s$ 很小，普通伪逆近似为",
        "",
        "$$",
        "g^+\\approx\\begin{bmatrix}J_\\mathrm{eff} & J_\\mathrm{eff}/T_s\\end{bmatrix}.",
        "$$",
        "",
        "位置项的系数约为 $J_\\mathrm{eff}$，速度项的系数约为 $J_\\mathrm{eff}/T_s$。当 $T_s=0.002$ s 时，$1/T_s=500$。右髋俯仰关节的速度系数因此达到 502.5406，再乘速度观测器增益 0.2 后仍约为 100.5。一个 0.05 rad/s 的速度残差，就会变成约 5 N m 的输入扰动估计。",
        "",
        "这不是数学错误，而是单位和采样周期共同造成的尺度问题。速度残差每秒计量，输入作用在一个 0.002 s 的离散步长内。要让一步内的速度变化解释同样的残差，反推力矩自然会带上 $1/T_s$ 的放大。",
        "",
        "加权伪逆也不能自动解决这个问题。权重必须表达清楚“我们更相信哪个残差、哪个残差噪声更大、两个量纲如何归一化”。当前默认权重强调位置残差，而位置输入增益 $T_s^2/J_\\mathrm{eff}$ 极小，所以加权结果反而更大。若未来想用加权形式抑制速度噪声，应重新设计权重，并配合低通、限幅和缩放。",
        "",
        "## 11. 本章结论",
        "",
        "1. 当前控制器的跟踪误差正常；输入逆估计大，不等于当前闭环跟踪失败。",
        "2. 按当前单关节离散模型，普通伪逆 $g^+$ 是与理论推导一致的第一步。",
        "3. 普通伪逆的离线结果与当前补偿趋势相关，但幅值大两个到三个数量级。",
        "4. 放大的主因是速度通道中的 $J_\\mathrm{eff}/T_s$，不是位置项，也不是加权伪逆才导致的问题。",
        "5. 下一步若要进入控制器，应先使用中心反馈残差 $x-\\bar x$，再加低通、限幅和小缩放系数，例如",
        "",
        "$$",
        "\\eta_u=\\gamma\\,\\mathrm{sat}\\left(g^+K_o(x-\\bar x)\\right),\\qquad \\gamma\\ll1.",
        "$$",
        "",
        "## 附录数据表",
        "",
        f"- [输入逆系数表]({markdown_relpath(OUT_DIR / 'input_inverse_coefficients.csv', REPORT_DIR)})",
        f"- [输入补偿重放表]({markdown_relpath(OUT_DIR / 'input_inverse_log_replay.csv', REPORT_DIR)})",
        f"- [基准跟踪误差表]({markdown_relpath(OUT_DIR / 'reference_tracking_metrics.csv', REPORT_DIR)})",
        f"- [稳态跟踪扫描表]({markdown_relpath(OUT_DIR / 'anti_phase_tracking_scan_summary.csv', REPORT_DIR)})",
        f"- [输入补偿增益重复明细表]({markdown_relpath(OUT_DIR / 'input_gain_tracking_repeat_summary.csv', REPORT_DIR)})",
        f"- [输入补偿增益相对变化表]({markdown_relpath(OUT_DIR / 'input_gain_tracking_relative_summary.csv', REPORT_DIR)})",
        f"- [输入逆贡献拆分表]({markdown_relpath(OUT_DIR / 'pinv_contribution_breakdown.csv', REPORT_DIR)})",
        "",
    ]
    REPORT_PATH.write_text("\n".join(lines), encoding="utf-8")


def write_report_focused(
    config_rows: list[dict[str, str]],
    log_rows: list[dict[str, str]],
    target_rows: list[dict[str, object]],
    contribution_rows: list[dict[str, object]],
    mujoco_rows: list[dict[str, str]],
    mujoco_figs: dict[str, Path],
) -> None:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)

    target_after3 = [r for r in target_rows if r["window"] == "after3s"]

    config_display = [
        {
            "joint": JOINT_LABEL[int(float(r["joint_id"]))],
            "J": f"{float(r['Jeff']):.8f}",
            "g": f"$[{float(r['g_q']):.4e},\\ {float(r['g_dq']):.4e}]^T$",
            "pinv": f"$[{float(r['pinv_q']):.4f},\\ {float(r['pinv_dq']):.4f}]$",
            "ku": f"$({fmt(float(r['ku_q']))},\\ {fmt(float(r['ku_dq']))})$",
            "ku_new": (
                f"$[{float(r['pinv_q']) * float(r['observer_gain_q']):.4f},\\ "
                f"{float(r['pinv_dq']) * float(r['observer_gain_dq']):.4f}]$"
            ),
        }
        for r in config_rows
    ]
    log_display = [
        {
            "joint": JOINT_LABEL[int(float(r["joint_id"]))],
            "old_eta_u_rms": float(r["old_eta_u_rms"]),
            "pinv_eta_u_rms": float(r["pinv_eta_u_rms"]),
            "pinv_over_old_rms": float(r["pinv_over_old_rms"]),
            "corr_old_pinv": float(r["corr_old_pinv"]),
        }
        for r in log_rows
    ]
    contrib_display = [
        {
            "joint": JOINT_LABEL[int(r["joint_id"])],
            "q_part": r["pinv_q_contrib_rms"],
            "dq_part": r["pinv_dq_contrib_rms"],
            "total": r["pinv_total_rms"],
        }
        for r in contribution_rows
    ]
    case_labels = {
        "original_ku": "原控制器 $K_u=(12,1)$",
        "input_inverse_equiv_ku": "输入逆等效 $K_u^{\\mathrm{new}}$",
    }
    mujoco_display = [
        {
            "case": case_labels.get(r["case_id"], r["case_id"]),
            "joint": JOINT_LABEL[int(r["joint_id"])],
            "q_rmse": float(r["q_rmse"]),
            "q_rmse_delta_pct": "" if not r["q_rmse_delta_pct"] else float(r["q_rmse_delta_pct"]),
            "dq_rmse": float(r["dq_rmse"]),
            "eta_u_rms": float(r["eta_u_rms"]),
            "eta_u_rms_delta_pct": "" if not r["eta_u_rms_delta_pct"] else float(r["eta_u_rms_delta_pct"]),
            "tau_rms": float(r["tau_rms"]),
            "tau_rms_delta_pct": "" if not r["tau_rms_delta_pct"] else float(r["tau_rms_delta_pct"]),
        }
        for r in mujoco_rows
    ]
    mujoco_figure_lines: list[str] = []
    if "q_error" in mujoco_figs:
        mujoco_figure_lines.extend(
            [
                f"![MuJoCo 位置误差时间序列]({markdown_relpath(mujoco_figs['q_error'], REPORT_DIR)})",
                "",
            ]
        )
    if "eta_u" in mujoco_figs:
        mujoco_figure_lines.extend(
            [
                f"![MuJoCo 输入补偿时间序列]({markdown_relpath(mujoco_figs['eta_u'], REPORT_DIR)})",
                "",
            ]
        )

    lines = [
        "# EID 输入逆改进：计算方法、数值大小与实际对比",
        "",
        "## 1. 本文只回答什么问题",
        "",
        "本文只讨论一件事：在当前右髋俯仰关节与右膝反相正弦跟踪实验中，如何把 EID 观测器得到的状态域修正换算成输入端的等效力矩扰动。",
        "",
        "本轮实验条件固定：",
        "",
        "- 机器人关节：Unitree H1 右髋俯仰关节、右膝。",
        "- 参考轨迹：0.8 Hz 反相正弦跟踪。",
        "- 控制周期：$T_s=0.002$ s。",
        "- 观测器增益：$K_o=(0.8,0.2)$。",
        "- 当前输入补偿增益：$K_u=(12,1)$。",
        "",
        "因此，本文不展开其他增益组合。我们只比较当前实际补偿 $K_u\\eta$ 与按输入矩阵伪逆得到的新估计量。",
        "",
        "## 2. 改进点是什么",
        "",
        "当前控制器中的输入补偿是经验形式：",
        "",
        "$$",
        "\\eta_{u,k}=k_{u,q}\\eta_{q,k}+k_{u,\\dot q}\\eta_{\\dot q,k}.",
        "$$",
        "",
        "它把 EID 状态估计 $\\eta_k$ 的位置分量和速度分量按固定系数映射成力矩。这种做法能工作，但它不是由离散动力学中的输入矩阵严格反推得到的。",
        "",
        "新的改进思路是：先写出单关节离散模型的输入矩阵 $g$，然后用普通 Moore-Penrose 伪逆 $g^+$ 把状态域扰动估计换算到输入力矩域：",
        "",
        "$$",
        "\\eta_u^{\\mathrm{pinv}}=g^+K_o(x-\\hat x).",
        "$$",
        "",
        "如果把这个式子写成和原控制器 $K_u$ 类似的线性增益形式，就是",
        "",
        "$$",
        "\\eta_u^{\\mathrm{pinv}}=(g^+K_o)(x-\\hat x).",
        "$$",
        "",
        "因此，输入逆方法对应的等效新增益为",
        "",
        "$$",
        "K_u^{\\mathrm{new}}=g^+K_o.",
        "$$",
        "",
        "当前实验中 $K_o=(0.8,0.2)$，所以若 $g^+=[g_q^+,\\ g_{\\dot q}^+]$，则",
        "",
        "$$",
        "K_u^{\\mathrm{new}}=[0.8g_q^+,\\ 0.2g_{\\dot q}^+].",
        "$$",
        "",
        "这里的 $\\eta_u^{\\mathrm{pinv}}$ 是本轮离线计算出来的输入扰动估计。它还没有接入实时控制器，因此它不是新的实机闭环结果。",
        "",
        "## 3. 输入矩阵如何计算",
        "",
        "当前单关节正模型采用半隐式欧拉离散化：",
        "",
        "$$",
        "\\dot q_{k+1}=\\dot q_k+\\frac{T_s}{J_\\mathrm{eff}}(u_k-\\beta_k),",
        "$$",
        "",
        "$$",
        "q_{k+1}=q_k+T_s\\dot q_{k+1}.",
        "$$",
        "",
        "其中 $\\beta_k=b\\dot q_k+A\\sin q_k+B\\cos q_k+\\tau_0$ 不乘当前输入 $u_k$，所以属于漂移项，不进入输入矩阵。只看 $u_k$ 的影响，速度更新式给出",
        "",
        "$$",
        "g_{\\dot q}=\\frac{T_s}{J_\\mathrm{eff}}.",
        "$$",
        "",
        "位置更新使用已经更新后的速度，因此输入对位置的影响还要再乘一个 $T_s$：",
        "",
        "$$",
        "g_q=T_s\\frac{T_s}{J_\\mathrm{eff}}=\\frac{T_s^2}{J_\\mathrm{eff}}.",
        "$$",
        "",
        "所以单关节输入矩阵为",
        "",
        "$$",
        "g=\\begin{bmatrix}g_q\\\\g_{\\dot q}\\end{bmatrix}",
        "=\\begin{bmatrix}T_s^2/J_\\mathrm{eff}\\\\T_s/J_\\mathrm{eff}\\end{bmatrix}.",
        "$$",
        "",
        "因为 $g$ 是两行一列，不能求方阵逆，只能求左伪逆：",
        "",
        "$$",
        "g^+=(g^Tg)^{-1}g^T.",
        "$$",
        "",
        "展开成标量形式就是",
        "",
        "$$",
        "g^+=\\frac{1}{g_q^2+g_{\\dot q}^2}\\begin{bmatrix}g_q & g_{\\dot q}\\end{bmatrix}.",
        "$$",
        "",
        "这就是本文使用的普通伪逆。",
        "",
        "## 4. 伪逆和等效新增益有多大",
        "",
        markdown_table(
            config_display,
            ["joint", "J", "g", "pinv", "ku", "ku_new"],
            {
                "joint": "关节",
                "J": "$J_\\mathrm{eff}$",
                "g": "$g$",
                "pinv": "$g^+$",
                "ku": "原控制器 $K_u$",
                "ku_new": "输入逆等效 $K_u^{\\mathrm{new}}$",
            },
        ),
        "",
        "右髋俯仰关节的输入逆等效新增益为 $[0.8041,\\ 100.5081]$，右膝为 $[0.2001,\\ 25.0147]$。与原控制器 $K_u=(12,1)$ 相比，位置通道系数变小，但速度通道系数大幅变大：右髋从 1 变为 100.5081，右膝从 1 变为 25.0147。",
        "",
        "## 5. 和当前实际补偿相比有多大",
        "",
        "用当前日志离线重放，比较当前实际补偿 $K_u\\eta$ 与普通伪逆估计 $g^+K_o(x-\\hat x)$：",
        "",
        markdown_table(
            log_display,
            ["joint", "old_eta_u_rms", "pinv_eta_u_rms", "pinv_over_old_rms", "corr_old_pinv"],
            {
                "joint": "关节",
                "old_eta_u_rms": "当前补偿 RMS",
                "pinv_eta_u_rms": "普通伪逆 RMS",
                "pinv_over_old_rms": "放大倍数",
                "corr_old_pinv": "相关系数",
            },
        ),
        "",
        "结果很清楚：普通伪逆和当前补偿的趋势有一定一致性，相关系数约为 0.92；但幅值大得多。右髋俯仰关节约放大 447 倍，右膝约放大 124 倍。",
        "",
        "进一步拆分普通伪逆的来源：",
        "",
        markdown_table(
            contrib_display,
            ["joint", "q_part", "dq_part", "total"],
            {
                "joint": "关节",
                "q_part": "位置项贡献 RMS",
                "dq_part": "速度项贡献 RMS",
                "total": "普通伪逆总量 RMS",
            },
        ),
        "",
        "几乎全部放大都来自速度通道。右髋俯仰关节的位置项只有 0.00071 N m，速度项为 5.91 N m；右膝的位置项只有 0.00019 N m，速度项为 2.28 N m。",
        "",
        "## 6. MuJoCo 实际闭环对比",
        "",
        "为了补上实际闭环对比，本文在 MuJoCo 中增加一组 A/B 实验。两组仿真使用相同轨迹、相同观测器增益、相同控制周期，只改变输入补偿增益：第一组使用原控制器 $K_u=(12,1)$，第二组使用输入逆等效新增益 $K_u^{\\mathrm{new}}$。",
        "",
        "需要说明的是，这一组仿真测试的是“等效新增益闭环”：也就是把原控制器输入补偿通道中的 $K_u$ 替换为 $K_u^{\\mathrm{new}}$。它还不是完整的 $g^+K_o(x-\\hat x)$ 残差直注入控制器，但已经能回答一个关键问题：如果按输入逆给出的等效增益进入闭环，效果和原控制器相比会怎样。",
        "",
        *mujoco_figure_lines,
        markdown_table(
            mujoco_display,
            ["case", "joint", "q_rmse", "q_rmse_delta_pct", "dq_rmse", "eta_u_rms", "eta_u_rms_delta_pct", "tau_rms", "tau_rms_delta_pct"],
            {
                "case": "MuJoCo 条件",
                "joint": "关节",
                "q_rmse": "位置 RMSE",
                "q_rmse_delta_pct": "位置变化百分比",
                "dq_rmse": "速度 RMSE",
                "eta_u_rms": "输入补偿 RMS",
                "eta_u_rms_delta_pct": "补偿变化百分比",
                "tau_rms": "力矩 RMS",
                "tau_rms_delta_pct": "力矩变化百分比",
            },
        ),
        "",
        "结果显示，输入逆等效新增益在 MuJoCo 中没有发散。右髋俯仰关节位置 RMSE 从 0.01780 rad 降到 0.01462 rad，改善约 17.9%；右膝位置 RMSE 从 0.00272 rad 增加到 0.00293 rad，变差约 8.0%。同时，输入补偿 RMS 显著增大：右髋从 0.00583 N m 增至 0.50395 N m，右膝从 0.00396 N m 增至 0.08740 N m。",
        "",
        "这说明输入逆等效增益确实能改变闭环行为，但收益并不均匀，而且补偿量明显放大。因此它不能直接作为实机结论，只能作为 MuJoCo 中的第一版闭环证据。",
        "",
        "## 7. 当前实机结果作为基准",
        "",
        "当前控制器去掉前 3 s 启动段后的实际跟踪误差为：",
        "",
        markdown_table(
            target_after3,
            ["joint_name", "q_rmse", "q_p95_abs", "q_peak_abs", "dq_rmse", "tau_cmd_rms"],
            {
                "joint_name": "关节",
                "q_rmse": "位置 RMSE",
                "q_p95_abs": "位置误差 95%",
                "q_peak_abs": "位置误差峰值",
                "dq_rmse": "速度 RMSE",
                "tau_cmd_rms": "力矩 RMS",
            },
        ),
        "",
        "这说明原控制器在实机上的跟踪误差是正常的。MuJoCo 中的等效新增益对比可以作为下一步设计依据，但还不能替代实机验证。",
        "",
        "## 8. 结论",
        "",
        "1. 本轮改进的核心是把状态域扰动估计通过普通伪逆 $g^+$ 映射到输入力矩域。",
        "2. 普通伪逆的计算来自当前半隐式欧拉单关节模型，不需要额外辨识。",
        "3. 输入逆等效新增益满足 $K_u^{\\mathrm{new}}=g^+K_o$。右髋俯仰关节 $K_u^{\\mathrm{new}}=[0.8041,\\ 100.5081]$，右膝 $K_u^{\\mathrm{new}}=[0.2001,\\ 25.0147]$。",
        "4. 原控制器使用 $K_u=(12,1)$。相比之下，输入逆等效新增益的位置通道更小，速度通道大得多。",
        "5. 离线重放中，普通伪逆输入补偿 RMS 明显大于当前实际补偿：右髋约 447 倍，右膝约 124 倍。",
        "6. MuJoCo 等效新增益闭环显示：右髋位置误差改善约 17.9%，右膝位置误差变差约 8.0%，输入补偿 RMS 明显增大。",
        "7. 下一步若要实机使用，应采用",
        "",
        "$$",
        "\\eta_u=\\gamma\\,\\mathrm{sat}\\left(g^+K_o(x-\\bar x)\\right),\\qquad \\gamma\\ll1,",
        "$$",
        "",
        "并加入低通滤波和力矩限幅。",
        "",
        "## 附录数据表",
        "",
        f"- [输入逆系数表]({markdown_relpath(OUT_DIR / 'input_inverse_coefficients.csv', REPORT_DIR)})",
        f"- [输入补偿重放表]({markdown_relpath(OUT_DIR / 'input_inverse_log_replay.csv', REPORT_DIR)})",
        f"- [基准跟踪误差表]({markdown_relpath(OUT_DIR / 'reference_tracking_metrics.csv', REPORT_DIR)})",
        f"- [输入逆贡献拆分表]({markdown_relpath(OUT_DIR / 'pinv_contribution_breakdown.csv', REPORT_DIR)})",
        f"- [MuJoCo 等效新增益闭环对比表]({markdown_relpath(MUJOCO_COMPARISON, REPORT_DIR)})",
        "",
    ]
    REPORT_PATH.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    cleanup_legacy_outputs()

    config_rows = read_csv(CONFIG_SUMMARY)
    log_rows = read_csv(LOG_SUMMARY)
    mujoco_rows = read_csv(MUJOCO_COMPARISON) if MUJOCO_COMPARISON.exists() else []
    target_metrics, target_log_rows = target_tracking_metrics(TARGET_LOG)
    contribution_rows = contribution_breakdown(target_log_rows, config_rows)
    config_output_rows = []
    for row in config_rows:
        out = dict(row)
        out["equivalent_ku_new_q"] = float(row["pinv_q"]) * float(row["observer_gain_q"])
        out["equivalent_ku_new_dq"] = float(row["pinv_dq"]) * float(row["observer_gain_dq"])
        config_output_rows.append(out)
    mujoco_figs = write_mujoco_timeseries_figures()

    write_csv(OUT_DIR / "input_inverse_coefficients.csv", config_output_rows)
    write_csv(OUT_DIR / "input_inverse_log_replay.csv", [dict(r) for r in log_rows])
    write_csv(OUT_DIR / "reference_tracking_metrics.csv", target_metrics)
    write_csv(OUT_DIR / "pinv_contribution_breakdown.csv", contribution_rows)

    write_report_focused(
        config_rows,
        log_rows,
        target_metrics,
        contribution_rows,
        mujoco_rows,
        mujoco_figs,
    )
    print(f"Wrote {repo_relpath(REPORT_PATH)}")


def dispatch() -> None:
    if len(sys.argv) > 1 and sys.argv[1] == "--plot-mujoco-timeseries":
        if len(sys.argv) != 6:
            raise SystemExit("usage: build_eid_input_inverse_full_report.py --plot-mujoco-timeseries OUT TITLE SIGNAL YLABEL")
        plot_mujoco_timeseries_matplotlib(Path(sys.argv[2]), sys.argv[3], sys.argv[4], sys.argv[5])
        return
    main()


if __name__ == "__main__":
    dispatch()
