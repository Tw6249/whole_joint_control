#!/usr/bin/env python3
"""Run a C++ controller against MuJoCo and log all debug signals.

All control algorithms (EID, PD, etc.) run inside the C++ stepper subprocess.
The stepper is selected by `controller.kind` in the YAML config.
Python handles MuJoCo physics, stepper I/O, and CSV logging.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import subprocess
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from fit_mujoco_eid_params import DISPLAY_NAMES, gather_joint_info


JOINT_HEADER_RE = re.compile(r"^\s+([0-9]+):\s*(?:#.*)?$")
KEY_VALUE_RE = re.compile(r"^\s+([A-Za-z0-9_]+):\s*([^#]*?)(?:\s*#.*)?$")
SAFETY_LOWSTATE_TIMEOUT = 1 << 0
SAFETY_NONFINITE_COMMAND = 1 << 1
SAFETY_COMMAND_SATURATED = 1 << 2
SAFETY_INVALID_STATE = 1 << 3
FATAL_SAFETY_FLAGS = SAFETY_LOWSTATE_TIMEOUT | SAFETY_NONFINITE_COMMAND | SAFETY_INVALID_STATE

N_MOTORS = 20
N_DEBUG_SLOTS = 33

# Protocol: cmd <flags> <q_cmd[20]> <dq_cmd[20]> <tau[20]> <kp[20]> <kd[20]>
#                <debug[0][0..19]> ... <debug[32][0..19]> <joint_flags[20]>
# = 1 (tag) + 1 + 5*20 + 33*20 + 20 = 782 values
CMD_N_VALUES = 1 + 1 + 5 * N_MOTORS + N_DEBUG_SLOTS * N_MOTORS + N_MOTORS

# Signal name mappings per controller kind (debug slot index -> signal name)
EID_DEBUG_NAMES: dict[int, str] = {
    0: "q_ref_shaped", 1: "dq_ref_shaped",
    2: "q_actual", 3: "dq_actual",
    4: "q_error_raw", 5: "dq_error_raw",
    6: "u_star", 7: "u_feedback", 8: "u_t",
    9: "eta_q", 10: "eta_dq",
    11: "x_hat_q", 12: "x_hat_dq",
    13: "rho_q", 14: "rho_dq",
    15: "x_bar_q", 16: "q_ref_shaped_next", 17: "dq_ref_shaped_next",
    18: "x_bar_dq",
    19: "r_d_q", 20: "r_d_dq",
    21: "e_q", 22: "e_dq",
    23: "observer_qacc", 24: "observer_tau_applied", 25: "u_raw",
    26: "q_ref_raw", 27: "dq_ref_raw",
    28: "q_error_raw2", 29: "dq_error_raw2",
    30: "q_error_shaped", 31: "dq_error_shaped",
    32: "eta_u",
}

PD_DEBUG_NAMES: dict[int, str] = {
    0: "q_ref_shaped", 1: "dq_ref_shaped",
    2: "q_actual", 3: "dq_actual",
    4: "q_error_shaped", 5: "dq_error_shaped",
    6: "kp_cmd", 7: "kd_cmd", 8: "tau_cmd",
}

# Signal categories for organizing dashboard display
SIGNAL_CATEGORIES = {
    "Position": ["q_actual", "q_ref_shaped", "q_ref_raw", "q_ref_shaped_next",
                  "q_error_shaped", "q_error_raw"],
    "Velocity": ["dq_actual", "dq_ref_shaped", "dq_ref_raw", "dq_ref_shaped_next",
                  "dq_error_shaped", "dq_error_raw"],
    "Torque": ["u_t", "u_raw", "u_star", "u_feedback", "eta_u", "tau_cmd",
               "observer_tau_applied"],
    "Observer": ["eta_q", "eta_dq", "x_hat_q", "x_hat_dq", "x_bar_q", "x_bar_dq",
                  "observer_qacc"],
    "EID Feedback Target": ["r_d_q", "r_d_dq", "e_q", "e_dq"],
    "Inverse Model": ["rho_q", "rho_dq"],
    "Command": ["kp_cmd", "kd_cmd"],
}


@dataclass(frozen=True)
class JointReference:
    source: str
    center: float
    amplitude: float
    phase_rad: float
    step_time_s: float


@dataclass(frozen=True)
class DisturbanceSpec:
    enabled: bool = False
    joints: tuple[int, ...] = ()
    torques: tuple[float, ...] = ()
    start_s: float = 0.0
    end_s: float = 0.0
    ramp_s: float = 0.0
    waveform: str = "smooth_rect"


def initial_reference_value(ref: JointReference) -> float:
    if ref.source == "step":
        return ref.center if ref.step_time_s > 0.0 else ref.center + ref.amplitude
    if ref.source == "sine":
        return ref.center + ref.amplitude * math.sin(ref.phase_rad)
    return ref.center


def default_stepper_path() -> Path:
    candidates = [
        Path("build-h1/Debug/h1_controller_stepper.exe"),
        Path("build/Debug/h1_controller_stepper.exe"),
        Path("build-h1/h1_controller_stepper"),
        Path("build/h1_controller_stepper"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(x, hi))


def parse_int_list(value: str) -> list[int]:
    value = value.strip()
    if value.startswith("[") and value.endswith("]"):
        value = value[1:-1]
    return [int(item.strip()) for item in value.split(",") if item.strip()]


def parse_float_list(value: str) -> list[float]:
    value = value.strip()
    if not value:
        return []
    if value.startswith("[") and value.endswith("]"):
        value = value[1:-1]
    return [float(item.strip()) for item in value.split(",") if item.strip()]


def parse_bool(value: str) -> bool:
    token = value.strip().lower().replace("-", "_")
    if token in {"true", "1", "yes", "on"}:
        return True
    if token in {"false", "0", "no", "off"}:
        return False
    raise RuntimeError(f"invalid boolean value: {value}")


def load_config_disturbance(config: Path) -> DisturbanceSpec:
    section: str | None = None
    values: dict[str, str] = {}
    for raw_line in config.read_text(encoding="utf-8").splitlines():
        raw_line = raw_line.split("#", 1)[0].rstrip()
        if not raw_line.strip():
            continue
        indent = len(raw_line) - len(raw_line.lstrip(" "))
        stripped = raw_line.strip()
        if indent == 0:
            section = stripped[:-1] if stripped.endswith(":") else None
            continue
        if section != "software_disturbance" or indent != 2:
            continue
        match = KEY_VALUE_RE.match(raw_line)
        if match:
            values[match.group(1)] = match.group(2).strip()

    if not values:
        return DisturbanceSpec()

    start = values.get("start_s", values.get("start", "0.0"))
    end = values.get("end_s", values.get("end", "0.0"))
    ramp = values.get("ramp_s", values.get("ramp", values.get("ramp_time_s", "0.0")))
    torques = values.get("torques", values.get("torque", values.get("amplitudes", "")))
    return DisturbanceSpec(
        enabled=parse_bool(values.get("enabled", "false")),
        joints=tuple(parse_int_list(values.get("joints", ""))),
        torques=tuple(parse_float_list(torques)),
        start_s=float(start),
        end_s=float(end),
        ramp_s=float(ramp),
        waveform=values.get("waveform", "smooth_rect").strip().lower().replace("-", "_"),
    )


def resolve_disturbance_spec(args: argparse.Namespace) -> DisturbanceSpec:
    spec = load_config_disturbance(args.config)
    cli_joints = parse_int_list(args.disturbance_joints)
    cli_torques = parse_float_list(args.disturbance_torques)
    if cli_joints or cli_torques:
        spec = DisturbanceSpec(
            enabled=True,
            joints=tuple(cli_joints),
            torques=tuple(cli_torques),
            start_s=spec.start_s,
            end_s=spec.end_s,
            ramp_s=spec.ramp_s,
            waveform=spec.waveform,
        )
    if args.disturbance_start is not None:
        spec = DisturbanceSpec(
            spec.enabled, spec.joints, spec.torques,
            float(args.disturbance_start), spec.end_s, spec.ramp_s, spec.waveform)
    if args.disturbance_end is not None:
        spec = DisturbanceSpec(
            spec.enabled, spec.joints, spec.torques,
            spec.start_s, float(args.disturbance_end), spec.ramp_s, spec.waveform)
    if args.disturbance_ramp is not None:
        spec = DisturbanceSpec(
            spec.enabled, spec.joints, spec.torques,
            spec.start_s, spec.end_s, float(args.disturbance_ramp), spec.waveform)
    if args.disturbance_waveform is not None:
        spec = DisturbanceSpec(
            spec.enabled, spec.joints, spec.torques,
            spec.start_s, spec.end_s, spec.ramp_s,
            args.disturbance_waveform.strip().lower().replace("-", "_"))

    if spec.enabled:
        if not spec.joints:
            raise RuntimeError("software disturbance enabled requires at least one joint")
        if len(spec.joints) != len(spec.torques):
            raise RuntimeError("software disturbance joints and torques must have the same length")
        if spec.end_s <= spec.start_s:
            raise RuntimeError("software disturbance requires start_s < end_s")
        if spec.ramp_s < 0.0:
            raise RuntimeError("software disturbance ramp_s must be >= 0")
        if spec.waveform not in {"rect", "rectangle", "rectangular", "step",
                                 "smooth_rect", "smooth_rectangle",
                                 "half_cosine", "cosine"}:
            raise RuntimeError("software disturbance waveform must be rectangular or smooth_rect")
    return spec


def disturbance_window(t: float, spec: DisturbanceSpec) -> float:
    if not spec.enabled or spec.end_s <= spec.start_s or t < spec.start_s or t > spec.end_s:
        return 0.0
    if spec.waveform in {"rect", "rectangle", "rectangular", "step"} or spec.ramp_s <= 0.0:
        return 1.0
    duration = spec.end_s - spec.start_s
    ramp = min(spec.ramp_s, 0.5 * duration)
    if ramp <= 0.0:
        return 1.0
    if t < spec.start_s + ramp:
        s = clamp((t - spec.start_s) / ramp, 0.0, 1.0)
        return 0.5 * (1.0 - math.cos(math.pi * s))
    if t > spec.end_s - ramp:
        s = clamp((spec.end_s - t) / ramp, 0.0, 1.0)
        return 0.5 * (1.0 - math.cos(math.pi * s))
    return 1.0


def load_controller_references(config: Path) -> dict[int, JointReference]:
    section: str | None = None
    controller_scope: str | None = None
    current_joint: int | None = None
    current_group: dict[str, object] | None = None
    defaults: dict[str, str] = {}
    groups: list[dict[str, object]] = []
    values: dict[int, dict[str, str]] = {}

    for raw_line in config.read_text(encoding="utf-8").splitlines():
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if not raw_line.startswith(" "):
            section = stripped.rstrip(":") if stripped.endswith(":") else None
            controller_scope = None
            current_joint = None
            current_group = None
            continue

        if section != "controller":
            continue
        indent = len(raw_line) - len(raw_line.lstrip(" "))
        match = KEY_VALUE_RE.match(raw_line)
        if indent == 2 and match:
            if match.group(1) in {"defaults", "groups", "joints"}:
                controller_scope = match.group(1)
                current_joint = None
                current_group = None
            continue
        if controller_scope == "defaults" and indent == 4 and match and match.group(1) in {
            "policy_source", "policy_center", "policy_amplitude",
            "policy_phase_rad", "policy_step_time_s",
        }:
            defaults[match.group(1)] = match.group(2).strip()
            continue
        if controller_scope == "groups" and indent == 4:
            header = re.match(r"^\s+([A-Za-z0-9_]+):\s*(?:#.*)?$", raw_line)
            if header:
                current_group = {"joints": [], "values": {}}
                groups.append(current_group)
            continue
        if controller_scope == "groups" and current_group is not None and indent == 6 and match:
            key = match.group(1)
            raw_value = match.group(2).strip()
            if key == "joints":
                current_group["joints"] = parse_int_list(raw_value)
            elif key in {
                "policy_source", "policy_center", "policy_amplitude",
                "policy_phase_rad", "policy_step_time_s", "enabled",
            }:
                current_group_values = current_group["values"]
                assert isinstance(current_group_values, dict)
                current_group_values[key] = raw_value
            continue
        if controller_scope == "joints" and indent == 4:
            header = JOINT_HEADER_RE.match(raw_line)
            if header:
                current_joint = int(header.group(1))
                values.setdefault(current_joint, {})
            continue
        if controller_scope != "joints" or current_joint is None or indent != 6:
            continue
        if match and match.group(1) in {
            "policy_source", "policy_center", "policy_amplitude",
            "policy_phase_rad", "policy_step_time_s", "enabled",
        }:
            values[current_joint][match.group(1)] = match.group(2).strip()

    refs: dict[int, JointReference] = {}
    for joint_id in sorted(values):
        item = dict(defaults)
        for group in groups:
            group_joints = group.get("joints", [])
            if joint_id in group_joints:
                group_values = group.get("values", {})
                if isinstance(group_values, dict):
                    item.update(group_values)
        item.update(values[joint_id])
        if not parse_bool(item.get("enabled", "true")):
            continue
        required = {"policy_center", "policy_amplitude"}
        missing = sorted(required - set(item))
        if missing:
            raise RuntimeError(f"{config}: controller.joints.{joint_id} missing {missing}")
        source = item.get("policy_source", "sine").strip().lower()
        if source not in {"hold", "sine", "step"}:
            raise RuntimeError(
                f"{config}: controller.joints.{joint_id}.policy_source must be hold, sine, or step"
            )
        refs[joint_id] = JointReference(
            source=source,
            center=float(item["policy_center"]),
            amplitude=float(item["policy_amplitude"]),
            phase_rad=float(item.get("policy_phase_rad", "0.0")),
            step_time_s=float(item.get("policy_step_time_s", "1.0")),
        )
    if not refs:
        raise RuntimeError(f"{config}: no controller.joints found")
    return refs


def initial_qpos(model: mujoco.MjModel, info: dict, refs: dict[int, JointReference],
                 height: float) -> np.ndarray:
    if model.nkey > 0:
        qpos = np.array(model.key_qpos[0], dtype=float)
    else:
        qpos = np.array(model.qpos0, dtype=float)
    qpos[0:3] = [0.0, 0.0, height]
    qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for joint_id, ref in refs.items():
        joint_info = info[joint_id]
        qpos[joint_info.qposadr] = clamp(
            initial_reference_value(ref), joint_info.q_min, joint_info.q_max)
    return qpos


def fix_suspended_base(data: mujoco.MjData, height: float) -> None:
    data.qpos[0:3] = [0.0, 0.0, height]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[0:6] = 0.0


def start_stepper(stepper: Path, config: Path) -> subprocess.Popen:
    if not stepper.exists():
        raise FileNotFoundError(
            f"{stepper} not found. Build it first, e.g. "
            "`cmake --build build --config Debug --target h1_controller_stepper`."
        )
    proc = subprocess.Popen(
        [str(stepper), str(config)],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, bufsize=1,
    )
    assert proc.stdout is not None
    ready = proc.stdout.readline().strip()
    if not ready.startswith("ready"):
        stderr = proc.stderr.read() if proc.stderr is not None else ""
        proc.kill()
        raise RuntimeError(f"stepper did not become ready: {ready}\n{stderr}")
    return proc


def send_state(proc: subprocess.Popen, cycle: int, t: float, dt: float,
               data: mujoco.MjData, info: dict) -> dict:
    assert proc.stdin is not None
    assert proc.stdout is not None
    fields = ["state", str(cycle), f"{t:.12g}", f"{dt:.12g}", "0.0"]
    for motor_id in range(N_MOTORS):
        if motor_id in info:
            ji = info[motor_id]
            q = float(data.qpos[ji.qposadr])
            dq = float(data.qvel[ji.dofadr])
            tau_est = float(data.actuator_force[motor_id]) if motor_id < len(data.actuator_force) else 0.0
        else:
            q = dq = tau_est = 0.0
        fields.extend([f"{q:.17g}", f"{dq:.17g}", f"{tau_est:.17g}"])
    proc.stdin.write(" ".join(fields) + "\n")
    proc.stdin.flush()

    line = proc.stdout.readline().strip()
    if not line.startswith("cmd "):
        raise RuntimeError(f"unexpected stepper response: {line}")
    parts = line.split()
    values = [float(x) for x in parts[1:]]
    if len(values) != CMD_N_VALUES - 1:
        raise RuntimeError(f"expected {CMD_N_VALUES - 1} values, got {len(values)}")

    offset = 0
    flags = int(values[offset]); offset += 1

    def _slice20():
        nonlocal offset
        s = values[offset:offset + N_MOTORS]
        offset += N_MOTORS
        return s

    q_cmd = _slice20()
    dq_cmd = _slice20()
    tau = _slice20()
    kp = _slice20()
    kd = _slice20()

    debug_slots = []
    for _ in range(N_DEBUG_SLOTS):
        debug_slots.append(_slice20())

    joint_flags = [int(v) for v in _slice20()]

    return {
        "flags": flags,
        "q_cmd": q_cmd, "dq_cmd": dq_cmd, "tau": tau, "kp": kp, "kd": kd,
        "debug_slots": debug_slots,
        "joint_flags": joint_flags,
    }


def detect_controller_kind(config: Path) -> str:
    """Read controller.kind from the YAML config."""
    for line in config.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith("kind:"):
            return stripped.split(":", 1)[1].strip().lower()
    return "eid"


def _write_summary_csv(active_joints, joint_q_sse, joint_q_count,
                       joint_abs_err_max, joint_max_abs_tau,
                       joint_tau_sum, path: Path) -> None:
    columns = [
        "joint_id", "name", "samples", "duration_s",
        "q_rmse", "q_max_abs_error", "tau_cmd_abs_max", "tau_mean_abs",
        "eid_tau_limit", "flags", "warning",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for joint_id in sorted(active_joints):
            n = joint_q_count.get(joint_id, 0)
            q_rmse = math.sqrt(joint_q_sse[joint_id] / n) if n else 0.0
            max_abs_tau = joint_max_abs_tau.get(joint_id, 0.0)
            tau_mean = joint_tau_sum[joint_id] / n if n else 0.0
            writer.writerow({
                "joint_id": joint_id,
                "name": DISPLAY_NAMES.get(joint_id, f"Joint{joint_id}"),
                "samples": n, "duration_s": "",
                "q_rmse": f"{q_rmse:.6f}",
                "q_max_abs_error": f"{joint_abs_err_max.get(joint_id, 0.0):.6f}",
                "tau_cmd_abs_max": f"{max_abs_tau:.4f}",
                "tau_mean_abs": f"{tau_mean:.4f}",
                "eid_tau_limit": "", "flags": "0", "warning": "",
            })
    print(f"Summary saved: {path}")


def run_simulation(args: argparse.Namespace) -> tuple[Path, float]:
    model = mujoco.MjModel.from_xml_path(str(args.scene))
    info = gather_joint_info(model)
    refs = load_controller_references(args.config)
    active_joints = sorted(refs)
    controller_kind = detect_controller_kind(args.config)

    data = mujoco.MjData(model)
    data.qpos[:] = initial_qpos(model, info, refs, args.height_m)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    csv_path = args.out_dir / "mujoco_closed_loop_log.csv"
    args.out_dir.mkdir(parents=True, exist_ok=True)

    # Save config copy alongside simulation outputs.
    config_copy = args.out_dir / "input_config.yaml"
    config_copy.write_text(args.config.read_text(encoding="utf-8"), encoding="utf-8")

    proc = start_stepper(args.stepper, args.config)

    q_sse = 0.0
    q_count = 0
    combined_flags = 0
    joint_q_sse: dict[int, float] = defaultdict(float)
    joint_q_count: dict[int, int] = defaultdict(int)
    joint_max_abs_tau: dict[int, float] = defaultdict(float)
    joint_abs_err_max: dict[int, float] = defaultdict(float)
    joint_tau_sum: dict[int, float] = defaultdict(float)
    steps = int(round(args.duration / args.dt))
    if args.log_every_step:
        log_interval = 1
    else:
        log_interval = max(1, int(round(1.0 / max(args.log_hz, 1.0e-9) / args.dt)))
    disturbance = resolve_disturbance_spec(args)
    disturbance_by_joint = dict(zip(disturbance.joints, disturbance.torques))

    # Build CSV columns: cycle, t, joint_id, plus motor commands + all debug slots
    motor_cols = ["motor_q", "motor_dq", "motor_tau", "motor_kp", "motor_kd"]
    torque_cols = [
        "tau_controller", "tau_disturbance", "tau_before_limit",
        "tau_sent", "tau_limit", "saturation_flag",
    ]
    debug_names = EID_DEBUG_NAMES if controller_kind == "eid" else PD_DEBUG_NAMES
    debug_cols = [debug_names.get(i, f"debug_{i}") for i in range(N_DEBUG_SLOTS)]
    csv_columns = ["cycle", "t", "joint_id"] + motor_cols + torque_cols + debug_cols + ["flags", "joint_flags"]

    try:
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=csv_columns)
            writer.writeheader()
            for step in range(steps):
                t = step * args.dt
                window = disturbance_window(t, disturbance)
                fix_suspended_base(data, args.height_m)
                command = send_state(proc, step, t, args.dt, data, info)
                combined_flags |= command["flags"]
                data.ctrl[:] = 0.0

                for joint_id in active_joints:
                    ji = info[joint_id]
                    q = float(data.qpos[ji.qposadr])
                    dq = float(data.qvel[ji.dofadr])
                    tau_controller = (
                        float(command["kp"][joint_id]) * (float(command["q_cmd"][joint_id]) - q)
                        + float(command["kd"][joint_id]) * (float(command["dq_cmd"][joint_id]) - dq)
                        + float(command["tau"][joint_id])
                    )
                    tau_disturbance = disturbance_by_joint.get(joint_id, 0.0) * window
                    tau_before_limit = tau_controller + tau_disturbance
                    tau_limit = min(abs(float(ji.ctrl_min)), abs(float(ji.ctrl_max)))
                    tau_sent = clamp(tau_before_limit, -tau_limit, tau_limit) if tau_limit > 0.0 else 0.0
                    saturation_flag = int(abs(tau_sent - tau_before_limit) > 1.0e-9)
                    data.ctrl[joint_id] = tau_sent

                    # Use debug slot 0 (shaped q_ref) as reference for summary tracking
                    q_ref = float(command["debug_slots"][0][joint_id])
                    q_error = q_ref - q
                    q_sse += q_error * q_error
                    q_count += 1
                    combined_flags |= command["joint_flags"][joint_id]

                    joint_q_sse[joint_id] += q_error * q_error
                    joint_q_count[joint_id] += 1
                    joint_abs_err_max[joint_id] = max(joint_abs_err_max[joint_id], abs(q_error))
                    joint_max_abs_tau[joint_id] = max(joint_max_abs_tau[joint_id], abs(tau_sent))
                    joint_tau_sum[joint_id] += abs(tau_sent)

                    if step % log_interval == 0:
                        row = {
                            "cycle": step, "t": f"{t:.12g}", "joint_id": joint_id,
                            "motor_q": f"{command['q_cmd'][joint_id]:.12g}",
                            "motor_dq": f"{command['dq_cmd'][joint_id]:.12g}",
                            "motor_tau": f"{command['tau'][joint_id]:.12g}",
                            "motor_kp": f"{command['kp'][joint_id]:.12g}",
                            "motor_kd": f"{command['kd'][joint_id]:.12g}",
                            "tau_controller": f"{tau_controller:.12g}",
                            "tau_disturbance": f"{tau_disturbance:.12g}",
                            "tau_before_limit": f"{tau_before_limit:.12g}",
                            "tau_sent": f"{tau_sent:.12g}",
                            "tau_limit": f"{tau_limit:.12g}",
                            "saturation_flag": str(saturation_flag),
                            "flags": str(command["flags"]),
                            "joint_flags": str(command["joint_flags"][joint_id]),
                        }
                        for slot_idx in range(N_DEBUG_SLOTS):
                            col_name = debug_names.get(slot_idx, f"debug_{slot_idx}")
                            row[col_name] = f"{command['debug_slots'][slot_idx][joint_id]:.12g}"
                        writer.writerow(row)

                mujoco.mj_step(model, data)
                fix_suspended_base(data, args.height_m)
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

    if args.export_summary:
        summary_path = args.out_dir / "summary.csv"
        _write_summary_csv(active_joints, joint_q_sse, joint_q_count,
                           joint_abs_err_max, joint_max_abs_tau,
                           joint_tau_sum, summary_path)

    manifest_path = args.out_dir / "mujoco_closed_loop_manifest.txt"
    manifest_path.write_text(
        "\n".join([
            "C++ controller closed-loop MuJoCo simulation",
            f"scene={args.scene}",
            f"config={args.config}",
            f"stepper={args.stepper}",
            f"controller_kind={controller_kind}",
            f"csv={csv_path}",
            f"duration={args.duration}",
            f"dt={args.dt}",
            f"log_hz={'every_step' if args.log_every_step else args.log_hz}",
            f"active_joints={','.join(str(j) for j in active_joints)}",
            f"disturbance_enabled={disturbance.enabled}",
            f"disturbance_joints={','.join(str(j) for j in disturbance.joints)}",
            f"disturbance_torques={','.join(str(v) for v in disturbance.torques)}",
            f"disturbance_window={disturbance.start_s},{disturbance.end_s}",
            f"disturbance_ramp={disturbance.ramp_s}",
            f"disturbance_waveform={disturbance.waveform}",
            f"q_rmse={q_rmse}",
            f"combined_flags={combined_flags}",
            f"fatal_flags={fatal_flags}",
        ]) + "\n",
        encoding="utf-8",
    )

    if fatal_flags != 0:
        raise RuntimeError(
            f"fatal controller/safety flags: {fatal_flags} (combined={combined_flags})"
        )

    return manifest_path, q_rmse


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=Path("h1_official_mujoco/scene.xml"))
    parser.add_argument("--config", type=Path, default=Path("config/h1_full_body_mujoco_fit.yaml"))
    parser.add_argument("--stepper", type=Path, default=default_stepper_path())
    parser.add_argument("--out-dir", type=Path, default=Path("data/mujoco_fit/latest"))
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--height-m", type=float, default=1.35)
    parser.add_argument("--log-hz", type=float, default=50.0,
                        help="CSV logging frequency. Use 500 for raw control-rate diagnostics.")
    parser.add_argument("--log-every-step", action="store_true",
                        help="Write one CSV row per control step.")
    parser.add_argument("--disturbance-joints", default="",
                        help="Comma-separated MuJoCo actuator ids to receive external torque.")
    parser.add_argument("--disturbance-torques", default="",
                        help="Comma-separated external torques [Nm], aligned with --disturbance-joints.")
    parser.add_argument("--disturbance-start", type=float, default=None,
                        help="Start time for external disturbance torque.")
    parser.add_argument("--disturbance-end", type=float, default=None,
                        help="End time for external disturbance torque.")
    parser.add_argument("--disturbance-ramp", type=float, default=None,
                        help="Half-cosine ramp time for smooth rectangular disturbance.")
    parser.add_argument("--disturbance-waveform", default=None,
                        help="Disturbance waveform: smooth_rect or rectangular.")
    parser.add_argument("--export-summary", action="store_true",
                        help="Export per-joint summary CSV.")
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    manifest_path, q_rmse = run_simulation(args)
    print(f"manifest={manifest_path}")
    print(f"q_rmse={q_rmse:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
