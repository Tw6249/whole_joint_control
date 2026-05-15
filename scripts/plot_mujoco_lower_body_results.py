#!/usr/bin/env python3
"""Plot multi-joint MuJoCo EID closed-loop results."""

from __future__ import annotations

import argparse
import csv
import math
import re
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt

from fit_mujoco_eid_params import DISPLAY_NAMES


JOINT_HEADER_RE = re.compile(r"^  ([0-9]+):\s*(?:#.*)?$")
KEY_VALUE_RE = re.compile(r"^\s+([A-Za-z0-9_]+):\s*([^#]*?)(?:\s*#.*)?$")


def finite_float(value: str | None, default: float = math.nan) -> float:
    try:
        return float(value) if value is not None else default
    except ValueError:
        return default


def parse_tau_limits(config: Path) -> dict[int, float]:
    limits: dict[int, float] = {}
    section: str | None = None
    current_joint: int | None = None
    for raw_line in config.read_text(encoding="utf-8").splitlines():
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if not raw_line.startswith(" "):
            section = stripped.rstrip(":") if stripped.endswith(":") else None
            current_joint = None
            continue
        if section != "eid_controllers":
            continue
        header = JOINT_HEADER_RE.match(raw_line)
        if header:
            current_joint = int(header.group(1))
            continue
        match = KEY_VALUE_RE.match(raw_line)
        if current_joint is not None and match and match.group(1) == "eid_tau_limit":
            limits[current_joint] = float(match.group(2))
    return limits


def load_rows(log_path: Path) -> dict[int, list[dict[str, float]]]:
    by_joint: dict[int, list[dict[str, float]]] = defaultdict(list)
    with log_path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            joint_id = int(float(row["joint_id"]))
            item = {
                "cycle": finite_float(row.get("cycle")),
                "t": finite_float(row.get("t")),
                "q": finite_float(row.get("q")),
                "dq": finite_float(row.get("dq")),
                "q_ref": finite_float(row.get("q_ref")),
                "q_error": finite_float(row.get("q_error")),
                "tau_cmd": finite_float(row.get("tau_cmd")),
                "flags": finite_float(row.get("flags"), 0.0),
            }
            if all(math.isfinite(item[k]) for k in ["t", "q", "q_ref", "q_error", "tau_cmd"]):
                by_joint[joint_id].append(item)
    if not by_joint:
        raise RuntimeError(f"no rows parsed from {log_path}")
    t0 = min(rows[0]["t"] for rows in by_joint.values() if rows)
    for rows in by_joint.values():
        for row in rows:
            row["time_s"] = row["t"] - t0
    return dict(sorted(by_joint.items()))


def summarize(by_joint: dict[int, list[dict[str, float]]], tau_limits: dict[int, float]) -> list[dict[str, float | int | str]]:
    summary = []
    for joint_id, rows in by_joint.items():
        errors = [row["q_error"] for row in rows]
        taus = [row["tau_cmd"] for row in rows]
        flags = sorted({int(row["flags"]) for row in rows})
        tau_limit = tau_limits.get(joint_id, math.nan)
        max_abs_tau = max(abs(v) for v in taus)
        saturated = math.isfinite(tau_limit) and tau_limit > 0.0 and max_abs_tau >= 0.98 * tau_limit
        summary.append(
            {
                "joint_id": joint_id,
                "name": DISPLAY_NAMES.get(joint_id, f"Joint{joint_id}"),
                "samples": len(rows),
                "duration_s": rows[-1]["time_s"] if rows else 0.0,
                "q_rmse": math.sqrt(sum(e * e for e in errors) / len(errors)),
                "q_max_abs_error": max(abs(e) for e in errors),
                "tau_cmd_abs_max": max_abs_tau,
                "eid_tau_limit": tau_limit,
                "flags": "|".join(str(v) for v in flags),
                "warning": "flags" if any(flags) else ("tau_limit" if saturated else ""),
            }
        )
    return summary


def write_summary(rows: list[dict[str, float | int | str]], path: Path) -> None:
    columns = [
        "joint_id",
        "name",
        "samples",
        "duration_s",
        "q_rmse",
        "q_max_abs_error",
        "tau_cmd_abs_max",
        "eid_tau_limit",
        "flags",
        "warning",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def setup_grid(title: str, count: int):
    cols = 2 if count <= 10 else 3
    rows = max(1, math.ceil(count / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(6.5 * cols, 2.7 * rows), sharex=True, constrained_layout=True)
    fig.suptitle(title)
    if hasattr(axes, "flatten"):
        return fig, axes.flatten()
    return fig, [axes]


def warning_color(summary_by_joint: dict[int, dict[str, float | int | str]], joint_id: int) -> str:
    return "#b00020" if summary_by_joint[joint_id]["warning"] else "#111111"


def plot_tracking(by_joint, summary_rows, out_path: Path) -> None:
    summary_by_joint = {int(row["joint_id"]): row for row in summary_rows}
    fig, axes = setup_grid("H1 EID tracking: reference vs MuJoCo actual q", len(by_joint))
    for ax, (joint_id, rows) in zip(axes, by_joint.items()):
        t = [row["time_s"] for row in rows]
        ax.plot(t, [row["q_ref"] for row in rows], label="q_ref", linewidth=1.3)
        ax.plot(t, [row["q"] for row in rows], label="actual q", linewidth=1.0)
        ax.set_title(f"{joint_id} {DISPLAY_NAMES.get(joint_id, '')}", color=warning_color(summary_by_joint, joint_id))
        ax.set_ylabel("rad")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)
    for ax in axes[len(by_joint):]:
        ax.set_visible(False)
    for ax in axes[-min(len(axes), 3):]:
        ax.set_xlabel("time (s)")
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def plot_error(by_joint, summary_rows, out_path: Path) -> None:
    summary_by_joint = {int(row["joint_id"]): row for row in summary_rows}
    fig, axes = setup_grid("H1 EID tracking error: q_ref - actual q", len(by_joint))
    for ax, (joint_id, rows) in zip(axes, by_joint.items()):
        t = [row["time_s"] for row in rows]
        ax.plot(t, [row["q_error"] for row in rows], color="#d62728", linewidth=1.0)
        ax.axhline(0.0, color="#444444", linewidth=0.6)
        ax.set_title(f"{joint_id} {DISPLAY_NAMES.get(joint_id, '')}", color=warning_color(summary_by_joint, joint_id))
        ax.set_ylabel("rad")
        ax.grid(True, alpha=0.3)
    for ax in axes[len(by_joint):]:
        ax.set_visible(False)
    for ax in axes[-min(len(axes), 3):]:
        ax.set_xlabel("time (s)")
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def plot_torque(by_joint, summary_rows, tau_limits, out_path: Path) -> None:
    summary_by_joint = {int(row["joint_id"]): row for row in summary_rows}
    fig, axes = setup_grid("H1 EID torque command", len(by_joint))
    for ax, (joint_id, rows) in zip(axes, by_joint.items()):
        t = [row["time_s"] for row in rows]
        ax.plot(t, [row["tau_cmd"] for row in rows], color="#9467bd", linewidth=1.0)
        limit = tau_limits.get(joint_id, math.nan)
        if math.isfinite(limit) and limit > 0.0:
            ax.axhline(limit, color="#555555", linestyle="--", linewidth=0.7)
            ax.axhline(-limit, color="#555555", linestyle="--", linewidth=0.7)
        ax.set_title(f"{joint_id} {DISPLAY_NAMES.get(joint_id, '')}", color=warning_color(summary_by_joint, joint_id))
        ax.set_ylabel("N m")
        ax.grid(True, alpha=0.3)
    for ax in axes[len(by_joint):]:
        ax.set_visible(False)
    for ax in axes[-min(len(axes), 3):]:
        ax.set_xlabel("time (s)")
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, default=Path("data/mujoco_fit/latest/eid_mujoco_closed_loop_log.csv"))
    parser.add_argument("--config", type=Path, default=Path("config/h1_full_body_mujoco_fit.yaml"))
    parser.add_argument("--out-dir", type=Path, default=Path("data/mujoco_fit/latest"))
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    tau_limits = parse_tau_limits(args.config)
    by_joint = load_rows(args.log)
    summary_rows = summarize(by_joint, tau_limits)
    summary_path = args.out_dir / "h1_eid_summary.csv"
    write_summary(summary_rows, summary_path)

    tracking_path = args.out_dir / "h1_eid_tracking_grid.png"
    error_path = args.out_dir / "h1_eid_error_grid.png"
    torque_path = args.out_dir / "h1_eid_torque_grid.png"
    plot_tracking(by_joint, summary_rows, tracking_path)
    plot_error(by_joint, summary_rows, error_path)
    plot_torque(by_joint, summary_rows, tau_limits, torque_path)

    print(f"summary={summary_path}")
    print(f"tracking={tracking_path}")
    print(f"error={error_path}")
    print(f"torque={torque_path}")
    warnings = [row for row in summary_rows if row["warning"]]
    print(f"warnings={len(warnings)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
