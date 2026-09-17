#!/usr/bin/env python3
"""Run constant-stand landing test with 3-point preview MPC and EID Ku.

Dry-run by default. Add --execute to command the real robot.
"""

from __future__ import annotations

# Support direct execution as well as python -m from the project root.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))


import argparse
import csv
import datetime as dt
import re
import statistics
import subprocess
import sys
import time
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = REPO_ROOT / "experiments" / "landing_stand" / "configs" / "h1_real_landing_stand_preview_mpc_10j_eid_ku.yaml"
DEFAULT_DIRECT = REPO_ROOT / "build-h1" / "h1_direct"
DEFAULT_STEPPER = REPO_ROOT / "build-h1" / "h1_controller_stepper"
DEFAULT_OUT_DIR = REPO_ROOT / "analysis_artifacts" / "real_landing_stand_preview_mpc_10j_eid_ku"
DEFAULT_CONDITION = "landing_stand_const_preview_mpc_10j_eid_ku"
CONFIRM_TOKEN = "RUN_LANDING_STAND"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run H1 constant-stand landing test with EID, pseudo-inverse Ku, "
            "and 3-point preview MPC reference generation."
        )
    )
    parser.add_argument("--execute", action="store_true", help="Actually run h1_direct. Default is dry-run.")
    parser.add_argument("--yes", action="store_true", help=f"Skip the {CONFIRM_TOKEN} confirmation prompt.")
    parser.add_argument("--no-sudo", action="store_true", help="Run h1_direct directly instead of through sudo.")
    parser.add_argument("--direct", type=Path, default=DEFAULT_DIRECT, help="Path to h1_direct executable.")
    parser.add_argument("--stepper", type=Path, default=DEFAULT_STEPPER, help="Path to h1_controller_stepper.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="Experiment YAML config.")
    parser.add_argument("--duration", type=float, default=60.0, help="Duration per run in seconds.")
    parser.add_argument("--repeats", type=int, default=1, help="Number of repeated runs.")
    parser.add_argument("--pause", type=float, default=12.0, help="Pause between repeated runs in seconds.")
    parser.add_argument("--start-index", type=int, default=1, help="Repeat index for the first run.")
    parser.add_argument("--condition", default=DEFAULT_CONDITION, help="Condition ID written to the CSV log.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="Manifest output directory.")
    return parser.parse_args()


def relative_to_repo(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT.resolve()))
    except ValueError:
        return str(path)


def shell_join(cmd: list[str]) -> str:
    return " ".join(f"'{part}'" if any(c.isspace() for c in part) else part for part in cmd)


def validate_args(args: argparse.Namespace) -> None:
    if args.duration < 25.0:
        raise SystemExit("--duration must be at least 25 seconds for ramp, hold, and lowering")
    if args.repeats <= 0:
        raise SystemExit("--repeats must be positive")
    if args.pause < 0.0:
        raise SystemExit("--pause must be non-negative")
    if args.start_index <= 0:
        raise SystemExit("--start-index must be positive")
    if not args.config.exists():
        raise SystemExit(f"config not found: {args.config}")
    if args.execute and not args.direct.exists():
        raise SystemExit(f"h1_direct not found: {args.direct}")


def command_for(args: argparse.Namespace, repeat: int) -> list[str]:
    cmd = [
        str(args.direct),
        relative_to_repo(args.config),
        "--duration",
        f"{args.duration:g}",
        "--experiment",
        "LANDING_STAND",
        "--condition",
        args.condition,
        "--repeat",
        f"r{repeat:02d}",
        "--disturbance-target",
        "ground_contact",
        "--disturbance-method",
        "manual_slow_lowering",
    ]
    if not args.no_sudo:
        cmd = ["sudo", "-n", *cmd]
    return cmd


def config_log_name(config: Path) -> str:
    for line in config.read_text(encoding="utf-8").splitlines():
        if line.strip().startswith("log_path:"):
            value = line.split(":", 1)[1].strip()
            return Path(value).name
    raise RuntimeError(f"{config} missing log_path")


def preview_pinv_ku(observer_gain_q: float, observer_gain_dq: float, jeff: float, control_dt: float) -> tuple[float, float]:
    g_q = control_dt * control_dt / jeff
    g_dq = control_dt / jeff
    den = g_q * g_q + g_dq * g_dq
    return observer_gain_q * g_q / den, observer_gain_dq * g_dq / den


def experiment_summary(config: Path) -> tuple[dict, list[dict]]:
    cfg = yaml.safe_load(config.read_text(encoding="utf-8"))
    control_dt = float(cfg["control_dt"])
    defaults = dict(cfg["controller"].get("defaults", {}))
    groups = cfg["controller"].get("groups", {})
    joints = cfg["controller"].get("joints", {})
    joint_limits = cfg.get("joint_limits", {})

    group_by_joint: dict[int, dict] = {}
    for group in groups.values():
        for joint_id in group.get("joints", []):
            merged = dict(group_by_joint.get(int(joint_id), {}))
            merged.update({k: v for k, v in group.items() if k != "joints"})
            group_by_joint[int(joint_id)] = merged

    rows = []
    for raw_joint_id, joint_cfg in joints.items():
        joint_id = int(raw_joint_id)
        merged = dict(defaults)
        merged.update(group_by_joint.get(joint_id, {}))
        merged.update({k: v for k, v in joint_cfg.items() if k not in {"name", "enabled", "plant"}})
        if not bool(joint_cfg.get("enabled", True)):
            continue
        plant = joint_cfg["plant"]
        safety_limit = joint_limits.get(joint_id, joint_limits.get(str(joint_id), {}))
        ku_q, ku_dq = preview_pinv_ku(
            float(merged["observer_gain_q"]),
            float(merged["observer_gain_dq"]),
            float(plant["Jeff"]),
            control_dt,
        )
        rows.append(
            {
                "joint_id": joint_id,
                "name": joint_cfg.get("name", ""),
                "target_q": float(merged["policy_center"]),
                "kp": float(merged["kp"]),
                "kd": float(merged["kd"]),
                "ko_q": float(merged["observer_gain_q"]),
                "ko_dq": float(merged["observer_gain_dq"]),
                "ku_q": ku_q,
                "ku_dq": ku_dq,
                "tau_limit": float(merged["tau_limit"]),
                "tau_slew_rate": float(merged["tau_slew_rate"]),
                "plant_tau_max": float(plant["tau_max"]),
                "safety_tau_max": float(safety_limit.get("tau_max", 0.0)),
            }
        )

    rows.sort(key=lambda item: item["joint_id"])
    return cfg, rows


def config_uses_input_inverse_equiv(config: Path) -> bool:
    text = config.read_text(encoding="utf-8")
    return bool(re.search(r"^\s*input_compensation_gain_mode:\s*input_inverse_equiv\s*$", text, re.MULTILINE))


def preflight_config(args: argparse.Namespace) -> None:
    if not args.stepper.exists():
        print(f"Warning: stepper not found; skipping config parse check: {args.stepper}")
        return
    proc = subprocess.run(
        [str(args.stepper), relative_to_repo(args.config), "--reference-only"],
        input="quit\n",
        text=True,
        cwd=REPO_ROOT,
        capture_output=True,
    )
    if proc.returncode != 0:
        if proc.stdout:
            print(proc.stdout)
        if proc.stderr:
            print(proc.stderr, file=sys.stderr)
        raise SystemExit(f"config preflight failed with return code {proc.returncode}")
    first = proc.stdout.splitlines()[0] if proc.stdout.splitlines() else ""
    print(f"Config preflight passed: {first}")


def preflight_processes() -> None:
    checks = [
        ["pgrep", "-af", "h1_direct"],
        ["pgrep", "-af", "h1_knee_pid"],
    ]
    active: list[str] = []
    for cmd in checks:
        proc = subprocess.run(cmd, text=True, capture_output=True)
        if proc.returncode == 0:
            for line in proc.stdout.splitlines():
                if "pgrep" not in line:
                    active.append(line)
    if active:
        print("Active controller-like processes found:")
        for line in active:
            print(f"  {line}")
        raise SystemExit("refusing to start while another low-level controller may be active")


def inspect_log_for_completion(
    path: Path,
    expected_repeat: str,
    expected_condition: str,
    expected_duration: float,
) -> bool:
    try:
        with path.open(newline="", encoding="utf-8-sig") as fh:
            reader = csv.DictReader(fh)
            first_cycle = None
            last_cycle = None
            dts: list[float] = []
            repeat_seen = ""
            condition_seen = ""
            target_seen = ""
            method_seen = ""
            rows = 0
            for row in reader:
                rows += 1
                if rows == 1:
                    repeat_seen = row.get("repeat_id", "")
                    condition_seen = row.get("condition_id", "")
                    target_seen = row.get("disturbance_target", "")
                    method_seen = row.get("disturbance_method", "")
                cycle = int(row["cycle"])
                first_cycle = cycle if first_cycle is None else min(first_cycle, cycle)
                last_cycle = cycle if last_cycle is None else max(last_cycle, cycle)
                if len(dts) < 2000:
                    dts.append(float(row["dt"]))
    except Exception as exc:
        print(f"Warning: could not inspect {path}: {exc}")
        return False

    if rows == 0 or first_cycle is None or last_cycle is None:
        return False
    if repeat_seen != expected_repeat or condition_seen != expected_condition:
        return False
    if target_seen != "ground_contact" or method_seen != "manual_slow_lowering":
        return False
    median_dt = statistics.median(dts) if dts else 0.0
    logged_duration = max(0, last_cycle - first_cycle) * median_dt
    return logged_duration >= 0.90 * expected_duration


def recent_completed_log(
    config: Path,
    repeat: int,
    condition: str,
    started_at: dt.datetime,
    expected_duration: float,
) -> Path | None:
    log_name = config_log_name(config)
    candidates = sorted(
        (REPO_ROOT / "data").glob(f"*/{log_name}"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    min_mtime = started_at.timestamp() - 2.0
    expected_repeat = f"r{repeat:02d}"

    for path in candidates:
        if path.stat().st_mtime < min_mtime:
            break
        if inspect_log_for_completion(path, expected_repeat, condition, expected_duration):
            return path
    return None


def print_plan(args: argparse.Namespace) -> None:
    cfg, joints = experiment_summary(args.config)
    print(
        f"Landing stand plan: {args.repeats} run(s), duration={args.duration:g}s, "
        f"config={relative_to_repo(args.config)}"
    )
    print(f"Controller: {cfg['controller'].get('kind', 'unknown')}")
    print("Reference generation: hold source, 3-point preview MPC, policy_dt=0.10 s, control_dt=0.002 s.")
    print("Input compensation: input_inverse_equiv pseudo-inverse Ku computed from Jeff and observer gains.")
    print("Manual stages:")
    print("  0-10 s: ramp from measured suspended posture to trained stand posture.")
    print("  10-15 s: hold stand posture while still suspended; verify stable joints.")
    print("  after 15 s: lower the suspension slowly; stop immediately on tilt, vibration, or abnormal torque.")
    if not config_uses_input_inverse_equiv(args.config):
        raise SystemExit("config must use input_compensation_gain_mode: input_inverse_equiv")
    print("Active leg joints, target angles, Kp/Kd, Ko, and computed pseudo-inverse Ku:")
    for target in joints:
        print(
            f"  {target['joint_id']:02d} {target['name']}: q={target['target_q']:+.3f} rad, "
            f"Kp/Kd=({target['kp']:g}, {target['kd']:g}), "
            f"Ko=({target['ko_q']:g}, {target['ko_dq']:g}), "
            f"Ku=({target['ku_q']:.6g}, {target['ku_dq']:.6g})"
        )
    print("\nCommand plan:")
    for repeat in range(args.start_index, args.start_index + args.repeats):
        print(shell_join(command_for(args, repeat)))


def confirm_or_exit(args: argparse.Namespace) -> None:
    print_plan(args)
    preflight_config(args)

    if not args.execute:
        print("\nDry-run only. Add --execute to command the real robot.")
        return
    preflight_processes()
    if args.yes:
        return

    print("\nThis will command all 10 leg joints on the real H1 with the EID controller.")
    print("Use only with the robot suspended, E-stop ready, watcher present, and manual lowering controlled.")
    print("Do not fully release the suspension until the partial-load phase is stable.")
    answer = input(f"Type {CONFIRM_TOKEN} to start: ").strip()
    if answer != CONFIRM_TOKEN:
        raise SystemExit("confirmation failed; run not started")


def ensure_sudo(args: argparse.Namespace) -> None:
    if args.no_sudo or not args.execute:
        return
    print("Checking sudo credentials with `sudo -v`...")
    subprocess.run(["sudo", "-v"], check=True)


def open_manifest(args: argparse.Namespace) -> tuple[Path, csv.DictWriter, object]:
    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = args.out_dir / f"landing_stand_preview_mpc_10j_eid_ku_manifest_{stamp}.csv"
    fh = path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(
        fh,
        fieldnames=[
            "batch_time",
            "repeat",
            "condition",
            "config",
            "duration_s",
            "pause_s",
            "command",
            "start_time",
            "end_time",
            "returncode",
            "status",
            "log_path",
        ],
    )
    writer.writeheader()
    return path, writer, fh


def run_batch(args: argparse.Namespace) -> Path | None:
    if not args.execute:
        return None

    ensure_sudo(args)
    manifest_path, writer, fh = open_manifest(args)
    batch_time = dt.datetime.now().isoformat(timespec="seconds")
    try:
        repeat_values = range(args.start_index, args.start_index + args.repeats)
        for run_no, repeat in enumerate(repeat_values, start=1):
            cmd = command_for(args, repeat)
            print(f"\n[{run_no}/{args.repeats}] landing stand r{repeat:02d}")
            print(shell_join(cmd))
            start = dt.datetime.now()
            proc = subprocess.run(cmd, input="\n", text=True, cwd=REPO_ROOT)
            end = dt.datetime.now()

            status = "ok" if proc.returncode == 0 else "failed"
            completed_log = recent_completed_log(args.config, repeat, args.condition, start, args.duration)
            if proc.returncode == -6 and completed_log is not None:
                status = "accepted_after_complete_abort"
                print(
                    "Warning: h1_direct returned -6 after a complete log was written; "
                    f"continuing. log={completed_log}"
                )

            writer.writerow(
                {
                    "batch_time": batch_time,
                    "repeat": f"r{repeat:02d}",
                    "condition": args.condition,
                    "config": relative_to_repo(args.config),
                    "duration_s": args.duration,
                    "pause_s": args.pause,
                    "command": shell_join(cmd),
                    "start_time": start.isoformat(timespec="seconds"),
                    "end_time": end.isoformat(timespec="seconds"),
                    "returncode": proc.returncode,
                    "status": status,
                    "log_path": relative_to_repo(completed_log) if completed_log else "",
                }
            )
            fh.flush()

            if proc.returncode != 0 and status != "accepted_after_complete_abort":
                raise SystemExit(f"run failed with return code {proc.returncode}; manifest={manifest_path}")
            if run_no < args.repeats and args.pause > 0.0:
                print(f"Pausing {args.pause:g}s before next run...")
                time.sleep(args.pause)
    finally:
        fh.close()
    return manifest_path


def main() -> None:
    args = parse_args()
    validate_args(args)
    confirm_or_exit(args)
    manifest_path = run_batch(args)
    if manifest_path:
        print(f"\nRun complete. Manifest: {manifest_path}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user.", file=sys.stderr)
        raise SystemExit(130)
