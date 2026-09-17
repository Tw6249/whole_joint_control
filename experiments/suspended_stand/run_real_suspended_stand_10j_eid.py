#!/usr/bin/env python3
"""Run suspended H1 10-leg-joint slow transition to standing posture.

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
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = REPO_ROOT / "experiments" / "suspended_stand" / "configs" / "h1_real_suspended_stand_10j_eid.yaml"
DEFAULT_DIRECT = REPO_ROOT / "build-h1" / "h1_direct"
DEFAULT_OUT_DIR = REPO_ROOT / "analysis_artifacts" / "real_suspended_stand_10j_eid"
DEFAULT_CONDITION = "suspended_10j_slow_stand_eid"
CONFIRM_TOKEN = "RUN_SUSPENDED_STAND"


@dataclass(frozen=True)
class TargetJoint:
    joint_id: int
    name: str
    target_q: float


TARGET_JOINTS = [
    TargetJoint(0, "RightHipRoll", 0.0),
    TargetJoint(1, "RightHipPitch", -0.1),
    TargetJoint(2, "RightKnee", 0.3),
    TargetJoint(3, "LeftHipRoll", 0.0),
    TargetJoint(4, "LeftHipPitch", -0.1),
    TargetJoint(5, "LeftKnee", 0.3),
    TargetJoint(7, "LeftHipYaw", 0.0),
    TargetJoint(8, "RightHipYaw", 0.0),
    TargetJoint(10, "LeftAnkle", -0.2),
    TargetJoint(11, "RightAnkle", -0.2),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run suspended H1 10-leg-joint slow stand transition with the EID controller."
    )
    parser.add_argument("--execute", action="store_true", help="Actually run h1_direct. Default is dry-run.")
    parser.add_argument("--yes", action="store_true", help=f"Skip the {CONFIRM_TOKEN} confirmation prompt.")
    parser.add_argument("--no-sudo", action="store_true", help="Run h1_direct directly instead of through sudo.")
    parser.add_argument("--direct", type=Path, default=DEFAULT_DIRECT, help="Path to h1_direct executable.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="Experiment YAML config.")
    parser.add_argument("--duration", type=float, default=15.0, help="Duration per run in seconds.")
    parser.add_argument("--repeats", type=int, default=1, help="Number of repeated runs.")
    parser.add_argument("--pause", type=float, default=8.0, help="Pause between repeated runs in seconds.")
    parser.add_argument("--start-index", type=int, default=1, help="Repeat index for the first run.")
    parser.add_argument("--condition", default=DEFAULT_CONDITION, help="Condition ID written to the CSV log.")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="Manifest output directory.")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.duration <= 0.0:
        raise SystemExit("--duration must be positive")
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


def shell_join(cmd: list[str]) -> str:
    return " ".join(f"'{part}'" if any(c.isspace() for c in part) else part for part in cmd)


def relative_to_repo(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(REPO_ROOT.resolve()))
    except ValueError:
        return str(path)


def command_for(args: argparse.Namespace, repeat: int) -> list[str]:
    cmd = [
        str(args.direct),
        relative_to_repo(args.config),
        "--duration",
        f"{args.duration:g}",
        "--experiment",
        "SUSPENDED_STAND",
        "--condition",
        args.condition,
        "--repeat",
        f"r{repeat:02d}",
        "--disturbance-target",
        "none",
        "--disturbance-method",
        "none",
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
    if target_seen != "none" or method_seen != "none":
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


def confirm_or_exit(args: argparse.Namespace) -> None:
    print(
        f"Suspended stand plan: {args.repeats} run(s), "
        f"duration={args.duration:g}s, config={relative_to_repo(args.config)}"
    )
    print("Active leg joints and target standing angles:")
    for target in TARGET_JOINTS:
        print(f"  {target.joint_id:02d} {target.name}: {target.target_q:+.3f} rad")

    print("\nCommand plan:")
    for repeat in range(args.start_index, args.start_index + args.repeats):
        print(shell_join(command_for(args, repeat)))

    if not args.execute:
        print("\nDry-run only. Add --execute to command the real robot.")
        return
    if args.yes:
        return

    print("\nThis will command all 10 leg joints on the real H1 with the EID controller.")
    print("Use only while suspended, with feet clear of the ground, hard limits verified, E-stop ready, and a watcher present.")
    print("The config ramps from measured joint angles to the standing target over 10 seconds, then holds.")
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
    path = args.out_dir / f"suspended_stand_10j_eid_manifest_{stamp}.csv"
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
            print(f"\n[{run_no}/{args.repeats}] suspended stand r{repeat:02d}")
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
