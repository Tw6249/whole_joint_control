#!/usr/bin/env python3
"""Run online LSTM policy + EID landing stand test.

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
import subprocess
import sys
from pathlib import Path

import yaml

import experiments.landing_stand.run_real_landing_stand_preview_mpc_10j_eid_ku as base

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = REPO_ROOT / "experiments" / "landing_stand" / "configs" / "h1_real_landing_stand_online_policy_lstm_eid_ku_ko08.yaml"
DEFAULT_OUT_DIR = REPO_ROOT / "analysis_artifacts" / "real_landing_stand_online_policy_lstm_eid_ku_ko08"
DEFAULT_CONDITION = "landing_stand_online_policy_lstm_preview_hold3_eid_ku_ko08"
CONFIRM_TOKEN = "RUN_ONLINE_POLICY_LANDING_STAND"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run H1 landing stand with online policy_lstm_1.pt inference, "
            "hold-3 preview MPC, EID, pseudo-inverse Ku, and Ko=(0.8,0.2)."
        )
    )
    parser.add_argument("--execute", action="store_true", help="Actually run h1_direct. Default is dry-run.")
    parser.add_argument("--yes", action="store_true", help=f"Skip the {CONFIRM_TOKEN} confirmation prompt.")
    parser.add_argument("--no-sudo", action="store_true", help="Run h1_direct directly instead of through sudo.")
    parser.add_argument("--direct", type=Path, default=base.DEFAULT_DIRECT, help="Path to h1_direct executable.")
    parser.add_argument("--stepper", type=Path, default=base.DEFAULT_STEPPER, help="Path to h1_controller_stepper.")
    parser.add_argument("--torque-logger", type=Path, default=REPO_ROOT / "build-h1" / "h1_torque_logger")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="Experiment YAML config.")
    parser.add_argument("--duration", type=float, default=60.0, help="Duration per run in seconds.")
    parser.add_argument("--repeats", type=int, default=1, help="Number of repeated runs.")
    parser.add_argument("--pause", type=float, default=12.0, help="Pause between repeated runs in seconds.")
    parser.add_argument("--start-index", type=int, default=1, help="Repeat index for the first run.")
    parser.add_argument("--condition", default=DEFAULT_CONDITION, help="Condition ID written to the CSV log.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="Manifest output directory.")
    parser.add_argument("--enable-posture-check", action="store_true", help="Run read-only current posture check before execution.")
    parser.add_argument("--skip-posture-check", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--posture-tolerance", type=float, default=0.35, help="Allowed current q deviation from standing target in rad.")
    return parser.parse_args()


def print_plan(args: argparse.Namespace) -> None:
    cfg, joints = base.experiment_summary(args.config)
    policy = cfg.get("online_policy", {})
    safe_hold = cfg.get("safe_hold", {})
    global_speed_trip = float(safe_hold.get("measured_speed_trip", 0.0))
    joint8_speed_trip = float(safe_hold.get("measured_speed_trip_joint_8", global_speed_trip))
    print(
        f"Online policy landing stand plan: {args.repeats} run(s), duration={args.duration:g}s, "
        f"config={base.relative_to_repo(args.config)}"
    )
    print(f"Controller: {cfg['controller'].get('kind', 'unknown')}")
    print(
        "Reference generation: policy_lstm_1.pt runs online every "
        f"{float(policy.get('policy_dt', 0.10)):.3f} s; preview MPC uses [q_policy, q_policy, q_policy]."
    )
    print(
        "Policy command: "
        f"{policy.get('command', [0.0, 0.0, 0.0])}; "
        f"model={policy.get('model_path', 'models/policies/policy_lstm_1.pt')}, deploy_yaml={policy.get('deploy_yaml', 'config/policy/h1.yaml')}"
    )
    print(
        "Startup: no suspended-to-stand trajectory is commanded; "
        "robot must already be manually placed near the initial standing posture."
    )
    print(
        "Safety gates: initial q_ref tolerance="
        f"{float(policy.get('initial_q_ref_tolerance', 0.35)):.3f} rad, "
        f"max_abs_action={float(policy.get('max_abs_action', 4.0)):.2f}, "
        f"q_ref_slew_rate={float(policy.get('q_ref_slew_rate', 3.0)):.2f} rad/s."
    )
    print(
        "Measured speed trip: "
        f"global={global_speed_trip:.2f} rad/s, joint 8 override={joint8_speed_trip:.2f} rad/s."
    )
    print(
        "Measured jump trip: "
        f"{float(safe_hold.get('measured_jump_trip', 0.0)):.3f} rad."
    )
    print("Initial posture check: disabled by default.")
    if not base.config_uses_input_inverse_equiv(args.config):
        raise SystemExit("config must use input_compensation_gain_mode: input_inverse_equiv")
    print("Active leg joints, Kp/Kd, Ko, computed pseudo-inverse Ku, and torque limits:")
    for target in joints:
        print(
            f"  {target['joint_id']:02d} {target['name']}: "
            f"Kp/Kd=({target['kp']:g}, {target['kd']:g}), "
            f"Ko=({target['ko_q']:g}, {target['ko_dq']:g}), "
            f"Ku=({target['ku_q']:.6g}, {target['ku_dq']:.6g}), "
            f"tau_limit={target['tau_limit']:g} Nm, "
            f"safe_tau_max={target['safety_tau_max']:g} Nm, "
            f"tau_slew_rate={target['tau_slew_rate']:g} Nm/s"
        )
    print("\nCommand plan:")
    for repeat in range(args.start_index, args.start_index + args.repeats):
        print(base.shell_join(base.command_for(args, repeat)))


def confirm_or_exit(args: argparse.Namespace) -> None:
    print_plan(args)
    base.preflight_config(args)

    if not args.execute:
        print("\nDry-run only. Add --execute to command the real robot.")
        return
    base.preflight_processes()
    if args.enable_posture_check and not args.skip_posture_check:
        check_current_posture(args)
    if args.yes:
        return

    print("\nThis will command all 10 leg joints on the real H1 with online LSTM policy + EID torque control.")
    print("Start only after the robot is manually adjusted near the policy initial standing posture.")
    print("Use suspension protection, E-stop, watcher, and slow manual lowering; stop on tilt, vibration, or abnormal torque.")
    answer = input(f"Type {CONFIRM_TOKEN} to start: ").strip()
    if answer != CONFIRM_TOKEN:
        raise SystemExit("confirmation failed; run not started")


def check_current_posture(args: argparse.Namespace) -> None:
    if not args.torque_logger.exists():
        raise SystemExit(f"h1_torque_logger not found: {args.torque_logger}")

    cfg = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    _, joints = base.experiment_summary(args.config)
    joint_ids = [str(item["joint_id"]) for item in joints]
    targets = {int(item["joint_id"]): float(item["target_q"]) for item in joints}
    names = {int(item["joint_id"]): item["name"] for item in joints}
    out = REPO_ROOT / "analysis_artifacts" / "online_policy_posture_check.csv"
    out.parent.mkdir(parents=True, exist_ok=True)

    cmd = [
        str(args.torque_logger),
        base.relative_to_repo(args.config),
        "--duration",
        "1.0",
        "--sample-period",
        "0.01",
        "--joints",
        ",".join(joint_ids),
        "--out",
        str(out),
        "--iface",
        str(cfg.get("network_interface", "eth0")),
        "--domain",
        str(cfg.get("domain_id", 0)),
    ]
    print("\nRead-only current posture check:")
    print(base.shell_join(cmd))
    proc = subprocess.run(cmd, cwd=REPO_ROOT)
    if proc.returncode not in (0, -6):
        raise SystemExit(f"posture check logger failed with return code {proc.returncode}")

    samples: dict[int, list[float]] = {int(j): [] for j in joint_ids}
    try:
        with out.open(newline="", encoding="utf-8-sig") as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                joint_id = int(row["joint_id"])
                if joint_id in samples:
                    samples[joint_id].append(float(row["q"]))
    except Exception as exc:
        raise SystemExit(f"could not read posture check log: {out}: {exc}") from exc

    if any(not values for values in samples.values()):
        raise SystemExit(f"posture check did not record all active joints: {out}")

    max_delta = 0.0
    worst_joint = -1
    print("Current posture mean q vs standing target:")
    for joint_id in sorted(samples):
        values = samples[joint_id]
        mean_q = sum(values) / len(values)
        delta = mean_q - targets[joint_id]
        if abs(delta) > max_delta:
            max_delta = abs(delta)
            worst_joint = joint_id
        print(
            f"  {joint_id:02d} {names[joint_id]}: "
            f"mean_q={mean_q:+.3f}, target={targets[joint_id]:+.3f}, delta={delta:+.3f}"
        )

    if max_delta > args.posture_tolerance:
        raise SystemExit(
            f"current posture is not close enough to standing target: "
            f"joint {worst_joint} delta={max_delta:.3f} rad > {args.posture_tolerance:.3f} rad"
        )
    print(f"Posture check passed: max_delta={max_delta:.3f} rad")


def main() -> None:
    args = parse_args()
    base.validate_args(args)
    confirm_or_exit(args)
    manifest_path = base.run_batch(args)
    if manifest_path:
        print(f"\nRun complete. Manifest: {manifest_path}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user.", file=sys.stderr)
        raise SystemExit(130)
