#!/usr/bin/env python3
"""Run isolated EID right-leg tracking tests and save plots."""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
PROJECT_SCRIPTS = REPO_ROOT / "scripts"
if str(PROJECT_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(PROJECT_SCRIPTS))

from fit_mujoco_eid_params import DISPLAY_NAMES, gather_joint_info
from run_mujoco import (
    EID_DEBUG_NAMES,
    FATAL_SAFETY_FLAGS,
    N_DEBUG_SLOTS,
    fix_suspended_base,
    initial_qpos,
    load_controller_references,
    send_state,
    start_stepper,
)


ANALYSIS_ROOT = Path(__file__).resolve().parents[1]
RIGHT_HIP_PITCH = 1
RIGHT_KNEE = 2
CSV_FIELDS = (
    "t",
    "q_ref_shaped",
    "dq_ref_shaped",
    "q_actual",
    "dq_actual",
    "u_star",
    "u_t",
    "u_raw",
    "motor_tau",
    "q_error_shaped",
    "dq_error_shaped",
)


@dataclass(frozen=True)
class MovingJoint:
    joint_id: int
    center: float
    amplitude: float
    frequency_hz: float
    phase_rad: float


@dataclass(frozen=True)
class Scenario:
    name: str
    title: str
    moving_joints: tuple[MovingJoint, ...]


def default_stepper_path() -> Path:
    for rel in (
        "build-h1/Debug/h1_controller_stepper.exe",
        "build/Debug/h1_controller_stepper.exe",
        "build-h1/h1_controller_stepper",
        "build/h1_controller_stepper",
    ):
        candidate = REPO_ROOT / rel
        if candidate.exists():
            return candidate
    return REPO_ROOT / "build-h1/Debug/h1_controller_stepper.exe"


def indentation(line: str) -> int:
    return len(line) - len(line.lstrip(" "))


def parse_key_value(line: str) -> tuple[str, str] | None:
    raw = line.split("#", 1)[0]
    if ":" not in raw:
        return None
    key, value = raw.split(":", 1)
    return key.strip(), value.strip()


def scalar_from_block(block: list[str], key: str, fallback: str) -> str:
    for line in block:
        parsed = parse_key_value(line)
        if parsed is None:
            continue
        parsed_key, value = parsed
        if indentation(line) == 6 and parsed_key == key:
            return value or fallback
    return fallback


def format_float(value: float) -> str:
    return f"{value:.12g}"


def rewrite_joint_block(block: list[str], moving_by_id: dict[int, MovingJoint]) -> tuple[int, list[str]]:
    header = parse_key_value(block[0])
    if header is None:
        raise RuntimeError(f"Cannot parse joint block header: {block[0].rstrip()}")
    joint_id = int(header[0])
    moving = moving_by_id.get(joint_id)

    if moving is None:
        desired = {
            "enabled": "false",
            "policy_source": "hold",
            "policy_center": scalar_from_block(block, "policy_center", "0"),
            "policy_amplitude": "0.0",
            "policy_frequency_hz": scalar_from_block(block, "policy_frequency_hz", "0.1"),
            "policy_phase_rad": scalar_from_block(block, "policy_phase_rad", "0.0"),
        }
    else:
        desired = {
            "enabled": "true",
            "policy_source": "sine",
            "policy_center": format_float(moving.center),
            "policy_amplitude": format_float(moving.amplitude),
            "policy_frequency_hz": format_float(moving.frequency_hz),
            "policy_phase_rad": format_float(moving.phase_rad),
        }

    order = (
        "enabled",
        "policy_source",
        "policy_center",
        "policy_amplitude",
        "policy_frequency_hz",
        "policy_phase_rad",
    )
    rewritten: list[str] = []
    seen: set[str] = set()
    insert_after_enabled = 0
    insert_before_tau_or_plant = len(block)

    for line in block:
        parsed = parse_key_value(line)
        if parsed is not None:
            key, _ = parsed
            if indentation(line) == 6 and key in desired:
                rewritten.append(f"      {key}: {desired[key]}\n")
                seen.add(key)
                if key == "enabled":
                    insert_after_enabled = len(rewritten)
                continue
            if indentation(line) == 6 and key in {"tau_limit", "plant"} and insert_before_tau_or_plant == len(block):
                insert_before_tau_or_plant = len(rewritten)
        rewritten.append(line)
        if parsed is not None and indentation(line) == 6 and parsed[0] == "enabled":
            insert_after_enabled = len(rewritten)

    missing = [f"      {key}: {desired[key]}\n" for key in order if key not in seen]
    if missing:
        insertion = insert_after_enabled or insert_before_tau_or_plant
        rewritten[insertion:insertion] = missing
    return joint_id, rewritten


def write_isolated_config(
    base_config: Path,
    output_config: Path,
    moving_joints: tuple[MovingJoint, ...],
    control_dt: float,
) -> None:
    moving_by_id = {joint.joint_id: joint for joint in moving_joints}
    lines = base_config.read_text(encoding="utf-8").splitlines(keepends=True)
    rewritten: list[str] = []
    section = ""
    controller_scope = ""
    found_joints: set[int] = set()
    i = 0

    while i < len(lines):
        line = lines[i]
        parsed = parse_key_value(line)
        indent = indentation(line)

        if parsed is not None:
            key, value = parsed
            if indent == 0 and key == "control_dt":
                rewritten.append(f"control_dt: {format_float(control_dt)}\n")
                i += 1
                continue
            if indent == 0 and value == "":
                section = key
                controller_scope = ""
            elif section == "controller" and indent == 2 and value == "" and key in {"defaults", "groups", "joints"}:
                controller_scope = key

        if section == "controller" and controller_scope == "joints" and parsed is not None:
            key, value = parsed
            if indent == 4 and value == "" and key.isdigit():
                block = [line]
                i += 1
                while i < len(lines):
                    next_line = lines[i]
                    next_parsed = parse_key_value(next_line)
                    next_indent = indentation(next_line)
                    if next_indent == 0:
                        break
                    if (
                        next_parsed is not None
                        and next_indent == 4
                        and next_parsed[1] == ""
                        and next_parsed[0].isdigit()
                    ):
                        break
                    block.append(next_line)
                    i += 1
                joint_id, new_block = rewrite_joint_block(block, moving_by_id)
                found_joints.add(joint_id)
                rewritten.extend(new_block)
                continue

        rewritten.append(line)
        i += 1

    missing = sorted(set(moving_by_id) - found_joints)
    if missing:
        raise RuntimeError(f"Moving joints not found in {base_config}: {missing}")
    output_config.parent.mkdir(parents=True, exist_ok=True)
    output_config.write_text("".join(rewritten), encoding="utf-8")


def joint_label(joint_id: int) -> str:
    return DISPLAY_NAMES.get(joint_id, f"Joint{joint_id}")


def lock_joints(data: mujoco.MjData, info: dict, locked_qpos: dict[int, float]) -> None:
    for joint_id, q in locked_qpos.items():
        joint_info = info[joint_id]
        data.qpos[joint_info.qposadr] = q
        data.qvel[joint_info.dofadr] = 0.0


def write_summary_csv(
    joint_ids: tuple[int, ...],
    joint_q_sse: dict[int, float],
    joint_q_count: dict[int, int],
    joint_abs_err_max: dict[int, float],
    joint_max_abs_tau: dict[int, float],
    joint_tau_sum: dict[int, float],
    path: Path,
) -> None:
    columns = [
        "joint_id",
        "name",
        "samples",
        "q_rmse",
        "q_max_abs_error",
        "tau_cmd_abs_max",
        "tau_mean_abs",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for joint_id in joint_ids:
            n = joint_q_count.get(joint_id, 0)
            writer.writerow({
                "joint_id": joint_id,
                "name": joint_label(joint_id),
                "samples": n,
                "q_rmse": f"{math.sqrt(joint_q_sse[joint_id] / n):.6f}" if n else "0.000000",
                "q_max_abs_error": f"{joint_abs_err_max.get(joint_id, 0.0):.6f}",
                "tau_cmd_abs_max": f"{joint_max_abs_tau.get(joint_id, 0.0):.4f}",
                "tau_mean_abs": f"{joint_tau_sum[joint_id] / n:.4f}" if n else "0.0000",
            })


def run_locked_simulation(
    scene: Path,
    config: Path,
    stepper: Path,
    out_dir: Path,
    duration: float,
    dt: float,
    height_m: float,
    controlled_joints: tuple[int, ...],
) -> tuple[Path, float]:
    model = mujoco.MjModel.from_xml_path(str(scene))
    info = gather_joint_info(model)
    refs = load_controller_references(config)
    missing = [joint_id for joint_id in controlled_joints if joint_id not in info]
    if missing:
        raise RuntimeError(f"Controlled joints missing from MuJoCo model: {missing}")

    physics_dt = float(model.opt.timestep)
    substeps = max(1, int(round(dt / physics_dt)))
    effective_dt = substeps * physics_dt
    if abs(effective_dt - dt) > 1.0e-9:
        raise RuntimeError(
            f"control dt {dt} must be a multiple of physics dt {physics_dt}; got {effective_dt}"
        )

    data = mujoco.MjData(model)
    data.qpos[:] = initial_qpos(model, info, refs, height_m)
    data.qvel[:] = 0.0
    locked_qpos = {
        joint_id: float(data.qpos[joint_info.qposadr])
        for joint_id, joint_info in info.items()
        if joint_id not in set(controlled_joints)
    }
    lock_joints(data, info, locked_qpos)
    fix_suspended_base(data, height_m)
    mujoco.mj_forward(model, data)

    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "input_config.yaml").write_text(config.read_text(encoding="utf-8"), encoding="utf-8")
    csv_path = out_dir / "mujoco_closed_loop_log.csv"
    proc = start_stepper(stepper, config)

    q_sse = 0.0
    q_count = 0
    combined_flags = 0
    joint_q_sse: dict[int, float] = defaultdict(float)
    joint_q_count: dict[int, int] = defaultdict(int)
    joint_max_abs_tau: dict[int, float] = defaultdict(float)
    joint_abs_err_max: dict[int, float] = defaultdict(float)
    joint_tau_sum: dict[int, float] = defaultdict(float)
    steps = int(round(duration / dt))
    log_interval = max(1, int(round(0.02 / dt)))
    motor_cols = ["motor_q", "motor_dq", "motor_tau", "motor_kp", "motor_kd"]
    debug_cols = [EID_DEBUG_NAMES.get(i, f"debug_{i}") for i in range(N_DEBUG_SLOTS)]
    csv_columns = ["cycle", "t", "joint_id"] + motor_cols + debug_cols + ["flags", "joint_flags"]

    try:
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=csv_columns)
            writer.writeheader()
            for step in range(steps):
                t = step * dt
                fix_suspended_base(data, height_m)
                lock_joints(data, info, locked_qpos)
                command = send_state(proc, step, t, dt, data, info)
                combined_flags |= command["flags"]
                data.ctrl[:] = 0.0

                for joint_id in controlled_joints:
                    joint_info = info[joint_id]
                    q = float(data.qpos[joint_info.qposadr])
                    dq = float(data.qvel[joint_info.dofadr])
                    tau_applied = (
                        float(command["kp"][joint_id]) * (float(command["q_cmd"][joint_id]) - q)
                        + float(command["kd"][joint_id]) * (float(command["dq_cmd"][joint_id]) - dq)
                        + float(command["tau"][joint_id])
                    )
                    data.ctrl[joint_id] = tau_applied

                    q_ref = float(command["debug_slots"][0][joint_id])
                    q_error = q_ref - q
                    q_sse += q_error * q_error
                    q_count += 1
                    combined_flags |= command["joint_flags"][joint_id]
                    joint_q_sse[joint_id] += q_error * q_error
                    joint_q_count[joint_id] += 1
                    joint_abs_err_max[joint_id] = max(joint_abs_err_max[joint_id], abs(q_error))
                    joint_max_abs_tau[joint_id] = max(joint_max_abs_tau[joint_id], abs(tau_applied))
                    joint_tau_sum[joint_id] += abs(tau_applied)

                    if step % log_interval == 0:
                        row = {
                            "cycle": step,
                            "t": f"{t:.12g}",
                            "joint_id": joint_id,
                            "motor_q": f"{command['q_cmd'][joint_id]:.12g}",
                            "motor_dq": f"{command['dq_cmd'][joint_id]:.12g}",
                            "motor_tau": f"{command['tau'][joint_id]:.12g}",
                            "motor_kp": f"{command['kp'][joint_id]:.12g}",
                            "motor_kd": f"{command['kd'][joint_id]:.12g}",
                            "flags": str(command["flags"]),
                            "joint_flags": str(command["joint_flags"][joint_id]),
                        }
                        for slot_idx in range(N_DEBUG_SLOTS):
                            col_name = EID_DEBUG_NAMES.get(slot_idx, f"debug_{slot_idx}")
                            row[col_name] = f"{command['debug_slots'][slot_idx][joint_id]:.12g}"
                        writer.writerow(row)

                for _ in range(substeps):
                    mujoco.mj_step(model, data)
                    lock_joints(data, info, locked_qpos)
                    fix_suspended_base(data, height_m)
                    mujoco.mj_forward(model, data)
    finally:
        if proc.stdin is not None:
            try:
                proc.stdin.write("quit\n")
                proc.stdin.flush()
            except BrokenPipeError:
                pass
        proc.wait(timeout=5)

    q_rmse = math.sqrt(q_sse / q_count) if q_count else 0.0
    fatal_flags = combined_flags & FATAL_SAFETY_FLAGS
    write_summary_csv(
        controlled_joints,
        joint_q_sse,
        joint_q_count,
        joint_abs_err_max,
        joint_max_abs_tau,
        joint_tau_sum,
        out_dir / "summary.csv",
    )
    manifest_path = out_dir / "mujoco_closed_loop_manifest.txt"
    manifest_path.write_text(
        "\n".join([
            "C++ EID controller closed-loop MuJoCo simulation with non-target joints locked",
            f"scene={scene}",
            f"config={config}",
            f"stepper={stepper}",
            f"csv={csv_path}",
            f"duration={duration}",
            f"control_dt={dt}",
            f"physics_dt={physics_dt}",
            f"physics_substeps={substeps}",
            f"controlled_joints={','.join(str(joint_id) for joint_id in controlled_joints)}",
            f"locked_joints={','.join(str(joint_id) for joint_id in sorted(locked_qpos))}",
            f"q_rmse={q_rmse}",
            f"combined_flags={combined_flags}",
            f"fatal_flags={fatal_flags}",
        ]) + "\n",
        encoding="utf-8",
    )
    if fatal_flags != 0:
        raise RuntimeError(f"fatal controller/safety flags: {fatal_flags} (combined={combined_flags})")
    return manifest_path, q_rmse


def load_joint_series(csv_path: Path, joint_ids: tuple[int, ...]) -> dict[int, dict[str, np.ndarray]]:
    values = {joint_id: {field: [] for field in CSV_FIELDS} for joint_id in joint_ids}
    with csv_path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            joint_id = int(row["joint_id"])
            if joint_id not in values:
                continue
            for field in CSV_FIELDS:
                values[joint_id][field].append(float(row[field]))
    result = {
        joint_id: {field: np.asarray(series, dtype=float) for field, series in field_values.items()}
        for joint_id, field_values in values.items()
    }
    for joint_id, field_values in result.items():
        if field_values["t"].size == 0:
            raise RuntimeError(f"No samples for joint {joint_id} in {csv_path}")
    return result


def rmse(error: np.ndarray) -> float:
    return float(np.sqrt(np.mean(error * error))) if error.size else 0.0


def setup_plot_style() -> None:
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "DejaVu Sans"],
        "font.size": 11,
        "axes.labelsize": 12,
        "axes.titlesize": 13,
        "legend.fontsize": 9,
        "figure.dpi": 150,
    })


def plot_single_joint(series: dict[str, np.ndarray], title: str, output_path: Path) -> None:
    t = series["t"]
    q_error = series["q_ref_shaped"] - series["q_actual"]
    dq_error = series["dq_ref_shaped"] - series["dq_actual"]

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.suptitle(title, fontweight="bold", fontsize=14)

    axes[0].plot(t, series["q_ref_shaped"], "k--", linewidth=1.2, label="q reference")
    axes[0].plot(t, series["q_actual"], color="#2563eb", linewidth=1.4, label="q actual")
    axes[0].set_ylabel("Position [rad]")
    axes[0].legend(loc="upper right", framealpha=0.9)
    axes[0].grid(True, alpha=0.3)
    axes[0].text(
        0.02,
        0.95,
        f"q RMSE={rmse(q_error):.5f} rad",
        transform=axes[0].transAxes,
        va="top",
        bbox=dict(boxstyle="round,pad=0.25", facecolor="white", alpha=0.75),
    )

    axes[1].plot(t, series["dq_ref_shaped"], "k--", linewidth=1.2, label="dq reference")
    axes[1].plot(t, series["dq_actual"], color="#16a34a", linewidth=1.4, label="dq actual")
    axes[1].set_ylabel("Velocity [rad/s]")
    axes[1].legend(loc="upper right", framealpha=0.9)
    axes[1].grid(True, alpha=0.3)
    axes[1].text(
        0.02,
        0.95,
        f"dq RMSE={rmse(dq_error):.5f} rad/s",
        transform=axes[1].transAxes,
        va="top",
        bbox=dict(boxstyle="round,pad=0.25", facecolor="white", alpha=0.75),
    )

    axes[2].plot(t, series["u_star"], "k--", linewidth=1.1, label="u* inverse torque reference")
    axes[2].plot(t, series["u_t"], color="#ea580c", linewidth=1.3, label="EID torque command")
    axes[2].plot(t, series["motor_tau"], color="#7c3aed", linewidth=0.9, alpha=0.75, label="motor torque command")
    axes[2].set_ylabel("Torque [N·m]")
    axes[2].set_xlabel("Time [s]")
    axes[2].legend(loc="upper right", framealpha=0.9)
    axes[2].grid(True, alpha=0.3)
    axes[2].set_xlim(float(t[0]), float(t[-1]))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def plot_dual_joint(series_by_joint: dict[int, dict[str, np.ndarray]], joint_ids: tuple[int, int], title: str, output_path: Path) -> None:
    colors = ("#2563eb", "#dc2626")
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(title, fontweight="bold", fontsize=14)

    for joint_id, color in zip(joint_ids, colors):
        s = series_by_joint[joint_id]
        label = joint_label(joint_id)
        t = s["t"]
        axes[0].plot(t, s["q_ref_shaped"], linestyle="--", color=color, linewidth=1.0, alpha=0.75, label=f"{label} q ref")
        axes[0].plot(t, s["q_actual"], color=color, linewidth=1.4, label=f"{label} q")
        axes[1].plot(t, s["dq_ref_shaped"], linestyle="--", color=color, linewidth=1.0, alpha=0.75, label=f"{label} dq ref")
        axes[1].plot(t, s["dq_actual"], color=color, linewidth=1.4, label=f"{label} dq")
        axes[2].plot(t, s["u_star"], linestyle="--", color=color, linewidth=1.0, alpha=0.75, label=f"{label} u* ref")
        axes[2].plot(t, s["u_t"], color=color, linewidth=1.4, label=f"{label} tau")

    axes[0].set_ylabel("Position [rad]")
    axes[1].set_ylabel("Velocity [rad/s]")
    axes[2].set_ylabel("Torque [N·m]")
    axes[2].set_xlabel("Time [s]")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", ncol=2, framealpha=0.9)
    axes[2].set_xlim(float(series_by_joint[joint_ids[0]]["t"][0]), float(series_by_joint[joint_ids[0]]["t"][-1]))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def correlation_text(x: np.ndarray, y: np.ndarray) -> str:
    if x.size < 2 or float(np.std(x)) <= 1.0e-12 or float(np.std(y)) <= 1.0e-12:
        return "corr=n/a"
    corr = float(np.corrcoef(x, y)[0, 1])
    return f"corr={corr:.3f}"


def plot_dual_correlation(series_by_joint: dict[int, dict[str, np.ndarray]], joint_ids: tuple[int, int], output_path: Path) -> None:
    first, second = joint_ids
    first_series = series_by_joint[first]
    second_series = series_by_joint[second]
    n = min(first_series["t"].size, second_series["t"].size)
    pairs = (
        ("q_actual", "Position correlation", "Position [rad]"),
        ("dq_actual", "Velocity correlation", "Velocity [rad/s]"),
        ("u_t", "Torque correlation", "Torque [N·m]"),
    )

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
    fig.suptitle(f"EID joint correlation — {joint_label(first)} vs {joint_label(second)}", fontweight="bold", fontsize=14)
    for ax, (field, title, unit) in zip(axes, pairs):
        x = first_series[field][:n]
        y = second_series[field][:n]
        ax.scatter(x, y, s=9, alpha=0.55, color="#4f46e5", edgecolors="none")
        ax.set_title(f"{title}\n{correlation_text(x, y)}")
        ax.set_xlabel(f"{joint_label(first)} {unit}")
        ax.set_ylabel(f"{joint_label(second)} {unit}")
        ax.grid(True, alpha=0.3)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def run_scenario(args: argparse.Namespace, scenario: Scenario) -> None:
    dt_tag = f"{args.dt:.3f}".replace(".", "p")
    scenario_dir = args.out_dir / f"{scenario.name}_dt_{dt_tag}"
    generated_config = scenario_dir / "generated_eid_test_config.yaml"
    write_isolated_config(args.base_config, generated_config, scenario.moving_joints, args.dt)

    joint_ids = tuple(joint.joint_id for joint in scenario.moving_joints)
    manifest_path, q_rmse = run_locked_simulation(
        scene=args.scene,
        config=generated_config,
        stepper=args.stepper,
        out_dir=scenario_dir,
        duration=args.duration,
        dt=args.dt,
        height_m=args.height_m,
        controlled_joints=joint_ids,
    )
    csv_path = scenario_dir / "mujoco_closed_loop_log.csv"
    series_by_joint = load_joint_series(csv_path, joint_ids)

    if len(joint_ids) == 1:
        joint_id = joint_ids[0]
        plot_single_joint(
            series_by_joint[joint_id],
            f"{scenario.title} — {joint_label(joint_id)}",
            scenario_dir / "right_knee_position_velocity_torque.png",
        )
    elif len(joint_ids) == 2:
        plot_dual_joint(
            series_by_joint,
            joint_ids,
            scenario.title,
            scenario_dir / "right_hip_knee_position_velocity_torque.png",
        )
        plot_dual_correlation(
            series_by_joint,
            joint_ids,
            scenario_dir / "right_hip_knee_correlation.png",
        )
    else:
        raise RuntimeError(f"Unsupported scenario joint count: {len(joint_ids)}")

    print(f"[{scenario.name}] manifest={manifest_path}")
    print(f"[{scenario.name}] q_rmse={q_rmse:.6f}")
    print(f"[{scenario.name}] outputs={scenario_dir}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-config", type=Path, default=REPO_ROOT / "config/h1_full_body_mujoco_fit.yaml")
    parser.add_argument("--scene", type=Path, default=REPO_ROOT / "h1_official_mujoco/scene.xml")
    parser.add_argument("--stepper", type=Path, default=default_stepper_path())
    parser.add_argument("--out-dir", type=Path, default=ANALYSIS_ROOT / "data/eid_right_leg_tests")
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--height-m", type=float, default=1.35)
    parser.add_argument("--frequency-hz", type=float, default=0.1)
    parser.add_argument("--phase-rad", type=float, default=-math.pi / 2.0)
    parser.add_argument("--knee-center", type=float, default=0.75)
    parser.add_argument("--knee-amplitude", type=float, default=0.10)
    parser.add_argument("--hip-center", type=float, default=-0.30)
    parser.add_argument("--hip-amplitude", type=float, default=0.20)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    setup_plot_style()
    knee = MovingJoint(RIGHT_KNEE, args.knee_center, args.knee_amplitude, args.frequency_hz, args.phase_rad)
    hip = MovingJoint(RIGHT_HIP_PITCH, args.hip_center, args.hip_amplitude, args.frequency_hz, args.phase_rad)
    scenarios = (
        Scenario("right_knee_only", "EID Test 1: non-target joints locked, right knee sine tracking", (knee,)),
        Scenario("right_hip_pitch_and_knee", "EID Test 2: right hip pitch and right knee sine tracking", (hip, knee)),
    )
    for scenario in scenarios:
        run_scenario(args, scenario)
    print(f"All EID right-leg test outputs saved under: {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
