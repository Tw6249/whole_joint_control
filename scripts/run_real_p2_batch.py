#!/usr/bin/env python3
"""Run the real P2 hip-knee disturbance experiment batch.

The script is intentionally conservative:
  - dry-run by default;
  - explicit --execute required for real robot motion;
  - confirmation required unless --yes is passed;
  - sudo credentials are checked once before the batch.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DIRECT = REPO_ROOT / "build-h1" / "h1_direct"
DEFAULT_OUT_DIR = REPO_ROOT / "analysis_artifacts" / "real_p2_batch"


@dataclass(frozen=True)
class MethodSpec:
    method: str
    config: Path


METHODS = {
    "pd": MethodSpec(
        method="pd",
        config=REPO_ROOT / "config" / "h1_real_p2_anti_hip_knee_pd.yaml",
    ),
    "eid": MethodSpec(
        method="eid",
        config=REPO_ROOT / "config" / "h1_real_p2_anti_hip_knee_eid.yaml",
    ),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run P2 real H1 PD/EID disturbance experiments, N repeats per method."
    )
    parser.add_argument("--execute", action="store_true", help="Actually run h1_direct. Default is dry-run.")
    parser.add_argument("--yes", action="store_true", help="Skip the RUN_P2 confirmation prompt.")
    parser.add_argument("--no-sudo", action="store_true", help="Run h1_direct directly instead of through sudo.")
    parser.add_argument("--direct", type=Path, default=DEFAULT_DIRECT, help="Path to h1_direct executable.")
    parser.add_argument("--repeats", type=int, default=10, help="Repeats per method.")
    parser.add_argument("--duration", type=float, default=10.0, help="Duration per run in seconds.")
    parser.add_argument("--pause", type=float, default=5.0, help="Pause between runs in seconds.")
    parser.add_argument("--start-index", type=int, default=1, help="Repeat index for the first run.")
    parser.add_argument(
        "--target",
        choices=["hip", "knee"],
        default="hip",
        help="Disturbed joint. hip maps to P2-H; knee maps to P2-K.",
    )
    parser.add_argument(
        "--disturbance-method",
        default="manual_push",
        help="Metadata recorded in disturbance_method. Default: manual_push.",
    )
    parser.add_argument(
        "--order",
        choices=["alternating", "pd-then-eid", "eid-then-pd"],
        default="alternating",
        help="Batch order. Alternating runs PD then EID for each repeat.",
    )
    parser.add_argument(
        "--methods",
        choices=["both", "pd", "eid"],
        default="both",
        help="Subset of methods to run.",
    )
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="Manifest output directory.")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.repeats <= 0:
        raise SystemExit("--repeats must be positive")
    if args.duration <= 0:
        raise SystemExit("--duration must be positive")
    if args.pause < 0:
        raise SystemExit("--pause must be non-negative")
    if args.start_index <= 0:
        raise SystemExit("--start-index must be positive")
    if not args.direct.exists():
        raise SystemExit(f"h1_direct not found: {args.direct}")
    for spec in METHODS.values():
        if not spec.config.exists():
            raise SystemExit(f"config not found: {spec.config}")


def selected_methods(args: argparse.Namespace) -> list[str]:
    if args.methods == "pd":
        return ["pd"]
    if args.methods == "eid":
        return ["eid"]
    return ["pd", "eid"]


def build_plan(args: argparse.Namespace) -> list[tuple[int, MethodSpec]]:
    indices = range(args.start_index, args.start_index + args.repeats)
    methods = selected_methods(args)
    plan: list[tuple[int, MethodSpec]] = []

    if args.order == "alternating":
        ordered_methods = [m for m in ["pd", "eid"] if m in methods]
        for repeat in indices:
            for method in ordered_methods:
                plan.append((repeat, METHODS[method]))
    else:
        if args.order == "pd-then-eid":
            ordered_methods = ["pd", "eid"]
        else:
            ordered_methods = ["eid", "pd"]
        for method in ordered_methods:
            if method not in methods:
                continue
            for repeat in indices:
                plan.append((repeat, METHODS[method]))
    return plan


def condition_for(args: argparse.Namespace, spec: MethodSpec) -> str:
    target_code = "H" if args.target == "hip" else "K"
    return f"P2-{target_code}_{spec.method.upper()}_anti_phase_{args.target}_disturbance"


def command_for(args: argparse.Namespace, repeat: int, spec: MethodSpec) -> list[str]:
    cmd = [
        str(args.direct),
        str(spec.config.relative_to(REPO_ROOT)),
        "--duration",
        f"{args.duration:g}",
        "--condition",
        condition_for(args, spec),
        "--repeat",
        f"r{repeat:02d}",
        "--disturbance-target",
        args.target,
        "--disturbance-method",
        args.disturbance_method,
    ]
    if not args.no_sudo:
        cmd = ["sudo", "-n", *cmd]
    return cmd


def shell_join(cmd: list[str]) -> str:
    return " ".join(f"'{part}'" if any(c.isspace() for c in part) else part for part in cmd)


def config_log_name(spec: MethodSpec) -> str:
    for line in spec.config.read_text(encoding="utf-8").splitlines():
        if line.strip().startswith("log_path:"):
            value = line.split(":", 1)[1].strip()
            return Path(value).name
    raise RuntimeError(f"{spec.config} missing log_path")


def recent_completed_log(
    args: argparse.Namespace,
    spec: MethodSpec,
    repeat: int,
    started_at: dt.datetime,
    expected_duration: float,
) -> Path | None:
    log_name = config_log_name(spec)
    candidates = sorted(
        (REPO_ROOT / "data").glob(f"*/{log_name}"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    min_mtime = started_at.timestamp() - 2.0
    expected_repeat = f"r{repeat:02d}"
    expected_condition = condition_for(args, spec)

    for path in candidates:
        if path.stat().st_mtime < min_mtime:
            break
        try:
            with path.open(newline="", encoding="utf-8") as fh:
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
            continue

        if rows == 0 or first_cycle is None or last_cycle is None:
            continue
        if repeat_seen != expected_repeat or condition_seen != expected_condition:
            continue
        if target_seen != args.target or method_seen != args.disturbance_method:
            continue
        median_dt = statistics.median(dts) if dts else 0.0
        duration = max(0, last_cycle - first_cycle) * median_dt
        if duration >= 0.90 * expected_duration:
            return path
    return None


def confirm_or_exit(args: argparse.Namespace, plan: list[tuple[int, MethodSpec]]) -> None:
    target_code = "P2-H" if args.target == "hip" else "P2-K"
    print(
        f"{target_code} batch plan: {len(plan)} runs, duration={args.duration:g}s, "
        f"pause={args.pause:g}s, disturbance_method={args.disturbance_method}"
    )
    for i, (repeat, spec) in enumerate(plan, start=1):
        print(
            f"{i:02d}. {spec.method.upper()} r{repeat:02d}: "
            f"{spec.config.relative_to(REPO_ROOT)}"
        )

    if not args.execute:
        print("\nDry-run only. Add --execute to run the real robot batch.")
        return

    if args.yes:
        return

    print("\nThis will command the real robot and expects manual disturbance during each run.")
    print("Ensure suspension/limits, E-stop, a watcher, and repeatable disturbance timing are ready.")
    answer = input("Type RUN_P2 to start the batch: ").strip()
    if answer != "RUN_P2":
        raise SystemExit("confirmation failed; batch not started")


def ensure_sudo(args: argparse.Namespace) -> None:
    if args.no_sudo or not args.execute:
        return
    print("Checking sudo credentials with `sudo -v`...")
    subprocess.run(["sudo", "-v"], check=True)


def open_manifest(args: argparse.Namespace) -> tuple[Path, csv.DictWriter, object]:
    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    target_code = "p2h" if args.target == "hip" else "p2k"
    path = args.out_dir / f"{target_code}_batch_manifest_{stamp}.csv"
    fh = path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(
        fh,
        fieldnames=[
            "batch_time",
            "target",
            "disturbance_method",
            "method",
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


def run_batch(args: argparse.Namespace, plan: list[tuple[int, MethodSpec]]) -> Path | None:
    if not args.execute:
        for repeat, spec in plan:
            print(shell_join(command_for(args, repeat, spec)))
        return None

    ensure_sudo(args)
    manifest_path, writer, fh = open_manifest(args)
    batch_time = dt.datetime.now().isoformat(timespec="seconds")
    try:
        for run_no, (repeat, spec) in enumerate(plan, start=1):
            cmd = command_for(args, repeat, spec)
            condition = condition_for(args, spec)
            print(f"\n[{run_no}/{len(plan)}] {spec.method.upper()} r{repeat:02d} {condition}")
            print(shell_join(cmd))
            start = dt.datetime.now()
            # h1_direct waits for Enter after printing the safety warning.
            proc = subprocess.run(
                cmd,
                input="\n",
                text=True,
                cwd=REPO_ROOT,
            )
            end = dt.datetime.now()
            status = "ok" if proc.returncode == 0 else "failed"
            completed_log = recent_completed_log(args, spec, repeat, start, args.duration)
            if proc.returncode == -6 and completed_log is not None:
                status = "accepted_after_complete_abort"
                print(
                    "Warning: h1_direct returned -6 after a complete log was written; "
                    f"continuing. log={completed_log}"
                )
            writer.writerow(
                {
                    "batch_time": batch_time,
                    "target": args.target,
                    "disturbance_method": args.disturbance_method,
                    "method": spec.method,
                    "repeat": f"r{repeat:02d}",
                    "condition": condition,
                    "config": str(spec.config.relative_to(REPO_ROOT)),
                    "duration_s": args.duration,
                    "pause_s": args.pause,
                    "command": shell_join(cmd),
                    "start_time": start.isoformat(timespec="seconds"),
                    "end_time": end.isoformat(timespec="seconds"),
                    "returncode": proc.returncode,
                    "status": status,
                    "log_path": str(completed_log.relative_to(REPO_ROOT)) if completed_log else "",
                }
            )
            fh.flush()
            if proc.returncode != 0 and status != "accepted_after_complete_abort":
                raise SystemExit(f"run failed with return code {proc.returncode}; manifest={manifest_path}")
            if run_no < len(plan) and args.pause > 0:
                print(f"Pausing {args.pause:g}s before next run...")
                time.sleep(args.pause)
    finally:
        fh.close()
    return manifest_path


def main() -> None:
    args = parse_args()
    validate_args(args)
    plan = build_plan(args)
    confirm_or_exit(args, plan)
    manifest_path = run_batch(args, plan)
    if manifest_path:
        print(f"\nBatch complete. Manifest: {manifest_path}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user.", file=sys.stderr)
        raise SystemExit(130)
