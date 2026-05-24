#!/usr/bin/env python3
"""Fit per-joint EID plant parameters from the official H1 MuJoCo model.

The fitted model intentionally matches the simplified controller plant:

    tau = Jeff*qacc + b*dq + gravityA*sin(q) + gravityB*cos(q) + tau0

It is a local single-joint approximation of the full MuJoCo dynamics, not a
replacement for full-body coupled inverse dynamics.
"""

from __future__ import annotations

import argparse
import csv
import math
import shutil
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np


LOWER_BODY_JOINTS = [0, 1, 2, 3, 4, 5, 7, 8, 10, 11]
FULL_BODY_JOINTS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
ACTIVE_JOINTS = FULL_BODY_JOINTS

EXPECTED_ACTUATOR_JOINTS = {
    0: "right_hip_roll_joint",
    1: "right_hip_pitch_joint",
    2: "right_knee_joint",
    3: "left_hip_roll_joint",
    4: "left_hip_pitch_joint",
    5: "left_knee_joint",
    6: "torso_joint",
    7: "left_hip_yaw_joint",
    8: "right_hip_yaw_joint",
    9: "not_use_joint",
    10: "left_ankle_joint",
    11: "right_ankle_joint",
    12: "right_shoulder_pitch_joint",
    13: "right_shoulder_roll_joint",
    14: "right_shoulder_yaw_joint",
    15: "right_elbow_joint",
    16: "left_shoulder_pitch_joint",
    17: "left_shoulder_roll_joint",
    18: "left_shoulder_yaw_joint",
    19: "left_elbow_joint",
}

DISPLAY_NAMES = {
    0: "RightHipRoll",
    1: "RightHipPitch",
    2: "RightKnee",
    3: "LeftHipRoll",
    4: "LeftHipPitch",
    5: "LeftKnee",
    6: "WaistYaw",
    7: "LeftHipYaw",
    8: "RightHipYaw",
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

REFERENCE_DEFAULTS = {
    0: (0.0, 0.03, 0.10, -1.5707963267948966),
    1: (-0.30, 0.05, 0.10, -1.5707963267948966),
    2: (0.75, 0.10, 0.10, -1.5707963267948966),
    3: (0.0, 0.03, 0.10, -1.5707963267948966),
    4: (-0.30, 0.05, 0.10, -1.5707963267948966),
    5: (0.75, 0.10, 0.10, -1.5707963267948966),
    6: (0.0, 0.05, 0.08, -1.5707963267948966),
    7: (0.0, 0.03, 0.10, -1.5707963267948966),
    8: (0.0, 0.03, 0.10, -1.5707963267948966),
    10: (0.0, 0.03, 0.10, -1.5707963267948966),
    11: (0.0, 0.03, 0.10, -1.5707963267948966),
    12: (0.0, 0.08, 0.08, -1.5707963267948966),
    13: (0.0, 0.04, 0.08, -1.5707963267948966),
    14: (0.0, 0.08, 0.08, -1.5707963267948966),
    15: (0.3, 0.08, 0.08, -1.5707963267948966),
    16: (0.0, 0.08, 0.08, -1.5707963267948966),
    17: (0.0, 0.04, 0.08, -1.5707963267948966),
    18: (0.0, 0.08, 0.08, -1.5707963267948966),
    19: (0.3, 0.08, 0.08, -1.5707963267948966),
}

# Use the full official MuJoCo ctrlrange peak torque; see h1.xml actuator section.
# Earlier generated configs used a conservative 80% fraction for simulation.
TAU_LIMIT_FRACTIONS = {
    0: 1.00,
    1: 1.00,
    2: 1.00,
    3: 1.00,
    4: 1.00,
    5: 1.00,
    6: 1.00,
    7: 1.00,
    8: 1.00,
    10: 1.00,
    11: 1.00,
    12: 1.00,
    13: 1.00,
    14: 1.00,
    15: 1.00,
    16: 1.00,
    17: 1.00,
    18: 1.00,
    19: 1.00,
}

DQ_LIMITS = {
    0: 23.0,
    1: 23.0,
    2: 14.0,
    3: 23.0,
    4: 23.0,
    5: 14.0,
    6: 23.0,
    7: 23.0,
    8: 23.0,
    9: 0.0,
    10: 9.0,
    11: 9.0,
    12: 9.0,
    13: 9.0,
    14: 20.0,
    15: 20.0,
    16: 9.0,
    17: 9.0,
    18: 20.0,
    19: 20.0,
}

KP_LIMITS = {
    **{i: 120.0 for i in [0, 1, 2, 3, 4, 5, 7, 8]},
    6: 100.0,
    9: 10.0,
    10: 80.0,
    11: 80.0,
    12: 80.0,
    13: 80.0,
    14: 60.0,
    15: 60.0,
    16: 80.0,
    17: 80.0,
    18: 60.0,
    19: 60.0,
}

KD_LIMITS = {
    **{i: 5.0 for i in [0, 1, 2, 3, 4, 5, 6, 7, 8]},
    9: 1.0,
    10: 3.0,
    11: 3.0,
    12: 3.0,
    13: 3.0,
    14: 2.0,
    15: 2.0,
    16: 3.0,
    17: 3.0,
    18: 2.0,
    19: 2.0,
}

EID_PARAM_GROUPS = [
    (
        "legs",
        [0, 1, 2, 3, 4, 5, 7, 8],
        {
            "kp": 60.0,
            "kd": 10.0,
            "observer_gain_q": 0.8,
            "observer_gain_dq": 0.5,
            "filter_alpha": 0.7,
        },
    ),
    (
        "waist",
        [6],
        {
            "kp": 45.0,
            "kd": 8.0,
            "observer_gain_q": 0.6,
            "observer_gain_dq": 0.4,
            "filter_alpha": 0.6,
        },
    ),
    (
        "ankle_shoulder_main",
        [10, 11, 12, 13, 16, 17],
        {
            "kp": 40.0,
            "kd": 6.0,
            "observer_gain_q": 0.6,
            "observer_gain_dq": 0.4,
            "filter_alpha": 0.6,
        },
    ),
    (
        "arm_small",
        [14, 15, 18, 19],
        {
            "kp": 30.0,
            "kd": 4.0,
            "observer_gain_q": 0.5,
            "observer_gain_dq": 0.3,
            "filter_alpha": 0.5,
        },
    ),
]

DEFAULT_JOINT_TAU_LIMITS = {
    0: 200.0,
    1: 200.0,
    2: 300.0,
    3: 200.0,
    4: 200.0,
    5: 300.0,
    6: 200.0,
    7: 200.0,
    8: 200.0,
    9: 0.0,
    10: 40.0,
    11: 40.0,
    12: 40.0,
    13: 40.0,
    14: 18.0,
    15: 18.0,
    16: 40.0,
    17: 40.0,
    18: 18.0,
    19: 18.0,
}


@dataclass
class JointInfo:
    index: int
    actuator_name: str
    joint_name: str
    joint_id: int
    qposadr: int
    dofadr: int
    q_min: float
    q_max: float
    ctrl_min: float
    ctrl_max: float


@dataclass
class FitResult:
    info: JointInfo
    Jeff: float
    b: float
    gravityA: float
    gravityB: float
    tau0: float
    tau_max: float
    rmse: float
    max_abs_error: float
    r2: float
    warning: str
    y_true: np.ndarray
    y_pred: np.ndarray
    q: np.ndarray


def actuator_joint_name(model: mujoco.MjModel, actuator_index: int) -> str:
    joint_id = int(model.actuator_trnid[actuator_index][0])
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)


def gather_joint_info(model: mujoco.MjModel) -> dict[int, JointInfo]:
    if (model.nq, model.nv, model.nu) != (27, 26, 20):
        raise RuntimeError(f"unexpected H1 model size: nq={model.nq} nv={model.nv} nu={model.nu}")

    info: dict[int, JointInfo] = {}
    for actuator_index in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_index)
        joint_id = int(model.actuator_trnid[actuator_index][0])
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        expected = EXPECTED_ACTUATOR_JOINTS.get(actuator_index)
        if expected is not None and joint_name != expected:
            raise RuntimeError(
                f"actuator index {actuator_index} maps to {joint_name}, expected {expected}"
            )

        qposadr = int(model.jnt_qposadr[joint_id])
        dofadr = int(model.jnt_dofadr[joint_id])
        q_range = model.jnt_range[joint_id]
        ctrl_range = model.actuator_ctrlrange[actuator_index]
        info[actuator_index] = JointInfo(
            index=actuator_index,
            actuator_name=actuator_name,
            joint_name=joint_name,
            joint_id=joint_id,
            qposadr=qposadr,
            dofadr=dofadr,
            q_min=float(q_range[0]),
            q_max=float(q_range[1]),
            ctrl_min=float(ctrl_range[0]),
            ctrl_max=float(ctrl_range[1]),
        )
    return info


def base_qpos(model: mujoco.MjModel) -> np.ndarray:
    if model.nkey > 0:
        qpos = np.array(model.key_qpos[0], dtype=float)
    else:
        qpos = np.array(model.qpos0, dtype=float)
    if qpos.size >= 7:
        qpos[2] = max(qpos[2], 0.98)
        qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    return qpos


def fit_joint(
    model: mujoco.MjModel,
    info: JointInfo,
    q_samples: int,
    dq_samples: tuple[float, ...],
    qacc_samples: tuple[float, ...],
    margin_fraction: float,
) -> FitResult:
    data = mujoco.MjData(model)
    qpos0 = base_qpos(model)
    policy_center, policy_amp, _, _ = REFERENCE_DEFAULTS[info.index]
    q_width = info.q_max - info.q_min
    hard_margin = max(0.01, margin_fraction * q_width)
    safe_min = info.q_min + hard_margin
    safe_max = info.q_max - hard_margin
    if safe_min >= safe_max:
        safe_min, safe_max = info.q_min, info.q_max
    local_half_width = max(0.12, min(0.60, 0.20 * q_width), 3.0 * policy_amp)
    q_lo = max(safe_min, policy_center - local_half_width)
    q_hi = min(safe_max, policy_center + local_half_width)
    if q_lo >= q_hi:
        q_lo, q_hi = safe_min, safe_max

    rows: list[list[float]] = []
    targets: list[float] = []
    q_values: list[float] = []
    for q in np.linspace(q_lo, q_hi, q_samples):
        for dq in dq_samples:
            for qacc in qacc_samples:
                data.qpos[:] = qpos0
                data.qvel[:] = 0.0
                data.qacc[:] = 0.0
                data.qpos[info.qposadr] = q
                data.qvel[info.dofadr] = dq
                data.qacc[info.dofadr] = qacc
                mujoco.mj_inverse(model, data)
                tau = float(data.qfrc_inverse[info.dofadr])
                rows.append([qacc, dq, math.sin(q), math.cos(q), 1.0])
                targets.append(tau)
                q_values.append(q)

    x = np.asarray(rows, dtype=float)
    y = np.asarray(targets, dtype=float)
    coeff, *_ = np.linalg.lstsq(x, y, rcond=None)
    y_pred = x @ coeff
    error = y_pred - y
    rmse = float(np.sqrt(np.mean(error * error)))
    max_abs_error = float(np.max(np.abs(error)))
    denom = float(np.sum((y - np.mean(y)) ** 2))
    r2 = 1.0 - float(np.sum(error * error)) / denom if denom > 1.0e-12 else 1.0

    warning_parts = []
    if coeff[0] <= 0.0:
        warning_parts.append("nonpositive_Jeff")
    if r2 < 0.95:
        warning_parts.append("low_r2")
    if rmse > max(0.5, 0.10 * max(1.0, float(np.max(np.abs(y))))):
        warning_parts.append("high_rmse")

    ctrl_abs = max(abs(info.ctrl_min), abs(info.ctrl_max))
    tau_max = round(ctrl_abs * TAU_LIMIT_FRACTIONS[info.index], 6)

    return FitResult(
        info=info,
        Jeff=float(coeff[0]),
        b=float(coeff[1]),
        gravityA=float(coeff[2]),
        gravityB=float(coeff[3]),
        tau0=float(coeff[4]),
        tau_max=tau_max,
        rmse=rmse,
        max_abs_error=max_abs_error,
        r2=float(r2),
        warning="|".join(warning_parts),
        y_true=y,
        y_pred=y_pred,
        q=np.asarray(q_values, dtype=float),
    )


def yaml_float(value: float) -> str:
    if abs(value) < 5.0e-13:
        value = 0.0
    return f"{value:.9g}"


def write_config(
    path: Path,
    results: list[FitResult],
    all_info: dict[int, JointInfo],
    enabled_overrides: dict[int, bool] | None = None,
) -> None:
    enabled_overrides = enabled_overrides or {}
    lines: list[str] = [
        "# Generated by scripts/fit_mujoco_eid_params.py from h1_official_mujoco/h1.xml.",
        "# Plant parameters are local single-joint fits to MuJoCo inverse dynamics.",
        "robot: H1",
        "domain_id: 0",
        "network_interface: eth0",
        "control_dt: 0.002",
        "mock_duration: 5.0",
        "log_path: data/h1_mock_log.csv",
        "",
        "safe_hold:",
        "  kp: 10.0",
        "  kd: 1.0",
        "  lowstate_timeout: 0.05",
        "",
        "controller:",
        "  kind: eid",
        "  defaults:",
        "    kp: 45.0",
        "    kd: 8.0",
        "    observer_gain_q: 0.25",
        "    observer_gain_dq: 0.25",
        "    filter_alpha: 0.5",
        "    policy_interpolation: open_loop",
        "    policy_source: sine",
        "    policy_dt: 0.05",
        "    policy_step_time_s: 1.0",
        "    startup_blend_duration_s: 0.0",
        "    tau_slew_rate: 0",
        "    torque_safe_kp: 0.0",
        "    torque_safe_kd: 0.8",
        "    inverse_q_weight: 0.0",
        "    inverse_dq_weight: 0.0",
        "",
        "  groups:",
    ]

    for group_name, joint_ids, params in EID_PARAM_GROUPS:
        lines.extend(
            [
                f"    {group_name}:",
                f"      joints: [{', '.join(str(joint_id) for joint_id in joint_ids)}]",
            ]
        )
        for key, value in params.items():
            lines.append(f"      {key}: {yaml_float(value)}")

    lines.extend(
        [
            "",
            "  joints:",
        ]
    )

    by_joint = {r.info.index: r for r in results}
    for joint_id in ACTIVE_JOINTS:
        r = by_joint[joint_id]
        center, amp, freq, phase = REFERENCE_DEFAULTS[joint_id]
        lines.extend(
            [
                f"    {joint_id}:",
                f"      name: {DISPLAY_NAMES[joint_id]}",
                f"      enabled: {'true' if enabled_overrides.get(joint_id, True) else 'false'}",
                f"      policy_center: {yaml_float(center)}",
                f"      policy_amplitude: {yaml_float(amp)}",
                f"      policy_frequency_hz: {yaml_float(freq)}",
                f"      policy_phase_rad: {yaml_float(phase)}",
                f"      tau_limit: {yaml_float(r.tau_max)}",
                "      plant:",
                f"        Jeff: {yaml_float(r.Jeff)}",
                f"        b: {yaml_float(r.b)}",
                f"        gravityA: {yaml_float(r.gravityA)}",
                f"        gravityB: {yaml_float(r.gravityB)}",
                f"        tau0: {yaml_float(r.tau0)}",
                f"        q_min: {yaml_float(r.info.q_min)}",
                f"        q_max: {yaml_float(r.info.q_max)}",
                f"        tau_max: {yaml_float(r.tau_max)}",
            ]
        )

    lines.extend(
        [
            "",
            "joint_limits:",
            "  # q_min/q_max and conservative tau_max are generated from the official H1 MJCF.",
        ]
    )
    for joint_id in range(20):
        if joint_id in by_joint:
            r = by_joint[joint_id]
            q_min, q_max = r.info.q_min, r.info.q_max
            tau_max = r.tau_max
        elif joint_id in all_info:
            joint_info = all_info[joint_id]
            q_min, q_max = joint_info.q_min, joint_info.q_max
            ctrl_abs = max(abs(joint_info.ctrl_min), abs(joint_info.ctrl_max))
            tau_max = min(DEFAULT_JOINT_TAU_LIMITS.get(joint_id, ctrl_abs), ctrl_abs)
        else:
            q_min, q_max = (-3.14, 3.14)
            tau_max = 0.0 if joint_id == 9 else 10.0
        lines.extend(
            [
                f"  {joint_id}:",
                f"    q_min: {yaml_float(q_min)}",
                f"    q_max: {yaml_float(q_max)}",
                f"    dq_max: {yaml_float(DQ_LIMITS.get(joint_id, 10.0))}",
                f"    tau_max: {yaml_float(tau_max)}",
                f"    kp_max: {yaml_float(KP_LIMITS.get(joint_id, 80.0))}",
                f"    kd_max: {yaml_float(KD_LIMITS.get(joint_id, 3.0))}",
            ]
        )

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_summary(path: Path, results: list[FitResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "joint_id",
                "joint_name",
                "Jeff",
                "b",
                "gravityA",
                "gravityB",
                "tau0",
                "q_min",
                "q_max",
                "ctrl_min",
                "ctrl_max",
                "tau_max",
                "rmse",
                "max_abs_error",
                "r2",
                "fit_warning",
            ],
        )
        writer.writeheader()
        for r in results:
            writer.writerow(
                {
                    "joint_id": r.info.index,
                    "joint_name": r.info.joint_name,
                    "Jeff": r.Jeff,
                    "b": r.b,
                    "gravityA": r.gravityA,
                    "gravityB": r.gravityB,
                    "tau0": r.tau0,
                    "q_min": r.info.q_min,
                    "q_max": r.info.q_max,
                    "ctrl_min": r.info.ctrl_min,
                    "ctrl_max": r.info.ctrl_max,
                    "tau_max": r.tau_max,
                    "rmse": r.rmse,
                    "max_abs_error": r.max_abs_error,
                    "r2": r.r2,
                    "fit_warning": r.warning,
                }
            )


def plot_joint_fit(result: FitResult, path: Path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), constrained_layout=True)
    order = np.argsort(result.y_true)
    axes[0].plot(result.y_true[order], label="MuJoCo inverse tau", linewidth=1.2)
    axes[0].plot(result.y_pred[order], label="EID plant fit", linewidth=1.2)
    axes[0].set_ylabel("tau (N m)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="best")

    axes[1].scatter(result.q, result.y_pred - result.y_true, s=10, alpha=0.7)
    axes[1].axhline(0.0, color="black", linewidth=0.8)
    axes[1].set_xlabel("q (rad)")
    axes[1].set_ylabel("fit error (N m)")
    axes[1].grid(True, alpha=0.3)

    fig.suptitle(
        f"{result.info.index} {result.info.joint_name}: "
        f"RMSE={result.rmse:.3f}, R2={result.r2:.4f}"
    )
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_error_summary(results: list[FitResult], path: Path) -> None:
    labels = [f"{r.info.index}\n{DISPLAY_NAMES[r.info.index]}" for r in results]
    rmse = [r.rmse for r in results]
    max_err = [r.max_abs_error for r in results]
    r2 = [r.r2 for r in results]
    x = np.arange(len(results))

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), constrained_layout=True)
    axes[0].bar(x, rmse, color="#4c78a8")
    axes[0].set_ylabel("RMSE (N m)")
    axes[0].grid(True, axis="y", alpha=0.3)
    axes[1].bar(x, max_err, color="#f58518")
    axes[1].set_ylabel("max |error|")
    axes[1].grid(True, axis="y", alpha=0.3)
    axes[2].bar(x, r2, color="#54a24b")
    axes[2].set_ylabel("R2")
    axes[2].set_ylim(min(0.0, min(r2) - 0.05), 1.02)
    axes[2].set_xticks(x, labels, rotation=0)
    axes[2].grid(True, axis="y", alpha=0.3)
    fig.suptitle("MuJoCo to EID Plant Fit Quality")
    fig.savefig(path, dpi=160)
    plt.close(fig)


def configure_model_for_visibility(model: mujoco.MjModel) -> None:
    black_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MATERIAL, "black")
    if black_id >= 0:
        model.mat_rgba[black_id] = [0.62, 0.64, 0.67, 1.0]
    white_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MATERIAL, "white")
    if white_id >= 0:
        model.mat_rgba[white_id] = [0.95, 0.95, 0.9, 1.0]


def make_camera() -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.0, 0.0, 0.85]
    camera.distance = 3.0
    camera.azimuth = 135.0
    camera.elevation = -12.0
    return camera


def render_pose(model: mujoco.MjModel, info: dict[int, JointInfo], path: Path) -> str:
    try:
        configure_model_for_visibility(model)
        data = mujoco.MjData(model)
        data.qpos[:] = base_qpos(model)
        data.qpos[2] = 1.35
        for joint_id in ACTIVE_JOINTS:
            center, amp, _, phase = REFERENCE_DEFAULTS[joint_id]
            q = center + 2.5 * amp * math.sin(phase + 0.35 * joint_id)
            joint_info = info[joint_id]
            data.qpos[joint_info.qposadr] = min(max(q, joint_info.q_min), joint_info.q_max)
        mujoco.mj_forward(model, data)
        renderer = mujoco.Renderer(model, height=480, width=640)
        renderer.update_scene(data, camera=make_camera())
        image = renderer.render()
        renderer.close()
        plt.imsave(path, image)
        return ""
    except Exception as exc:  # pragma: no cover - depends on local GL backend.
        return f"render_failed:{type(exc).__name__}:{exc}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xml", type=Path, default=Path("h1_official_mujoco/h1.xml"))
    parser.add_argument("--config-out", type=Path, default=Path("config/h1_full_body_mujoco_fit.yaml"))
    parser.add_argument("--real-template-out", type=Path, default=Path("config/h1_full_body_real_template.yaml"))
    parser.add_argument("--out-dir", type=Path, default=Path("data/mujoco_fit/latest"))
    parser.add_argument("--q-samples", type=int, default=11)
    parser.add_argument("--margin-fraction", type=float, default=0.08)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.xml))
    info = gather_joint_info(model)
    dq_samples = (-1.0, 0.0, 1.0)
    qacc_samples = (-6.0, 0.0, 6.0)

    results = [
        fit_joint(model, info[joint_id], args.q_samples, dq_samples, qacc_samples, args.margin_fraction)
        for joint_id in ACTIVE_JOINTS
    ]

    for result in results:
        values = [result.Jeff, result.b, result.gravityA, result.gravityB, result.tau0, result.tau_max]
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError(f"non-finite fit result for joint {result.info.index}")
        if result.Jeff <= 0.0 or result.info.q_min >= result.info.q_max or result.tau_max <= 0.0:
            raise RuntimeError(f"invalid fit result for joint {result.info.index}")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    write_config(args.config_out, results, info)
    real_enabled = {joint_id: (joint_id in LOWER_BODY_JOINTS) for joint_id in ACTIVE_JOINTS}
    write_config(args.real_template_out, results, info, real_enabled)
    write_summary(args.out_dir / "fit_summary.csv", results)
    for result in results:
        plot_joint_fit(result, args.out_dir / f"joint_{result.info.index:02d}_{DISPLAY_NAMES[result.info.index]}_fit.png")
    plot_error_summary(results, args.out_dir / "fit_error_summary.png")
    render_warning = render_pose(model, info, args.out_dir / "mujoco_pose.png")

    config_copy = args.out_dir / args.config_out.name
    if args.config_out.resolve() != config_copy.resolve():
        shutil.copyfile(args.config_out, config_copy)
    real_template_copy = args.out_dir / args.real_template_out.name
    if args.real_template_out.resolve() != real_template_copy.resolve():
        shutil.copyfile(args.real_template_out, real_template_copy)

    print(f"xml={args.xml}")
    print(f"model_size=nq:{model.nq},nv:{model.nv},nu:{model.nu}")
    print(f"config_out={args.config_out}")
    print(f"real_template_out={args.real_template_out}")
    print(f"summary={args.out_dir / 'fit_summary.csv'}")
    print(f"plots_dir={args.out_dir}")
    if render_warning:
        print(f"render_warning={render_warning}")
    else:
        print(f"render={args.out_dir / 'mujoco_pose.png'}")
    for result in results:
        warning = f" warning={result.warning}" if result.warning else ""
        print(
            f"joint={result.info.index} {result.info.joint_name} "
            f"Jeff={result.Jeff:.6g} b={result.b:.6g} "
            f"rmse={result.rmse:.6g} r2={result.r2:.6f}{warning}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
