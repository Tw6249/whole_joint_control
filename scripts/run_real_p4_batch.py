#!/usr/bin/env python3
"""Run the real P4 hip-knee EID parameter scan batch.

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
DEFAULT_OUT_DIR = REPO_ROOT / "analysis_artifacts" / "real_p4_batch"


@dataclass(frozen=True)
class ParamSpec:
    group: str
    scan: str
    label: str
    config: Path
    ko_scale: float
    ku_scale: float


KO_SPECS = [
    ParamSpec("o0", "ko", "Ko_o0", REPO_ROOT / "config" / "h1_real_p4_ko_o0_hip_knee_eid.yaml", 0.0, 1.0),
    ParamSpec("o1", "ko", "Ko_o1", REPO_ROOT / "config" / "h1_real_p4_ko_o1_hip_knee_eid.yaml", 0.25, 1.0),
    ParamSpec("o2", "ko", "Ko_o2", REPO_ROOT / "config" / "h1_real_p4_ko_o2_hip_knee_eid.yaml", 0.5, 1.0),
    ParamSpec("o3", "ko", "Ko_o3", REPO_ROOT / "config" / "h1_real_p4_ko_o3_hip_knee_eid.yaml", 0.75, 1.0),
    ParamSpec("o4", "ko", "Ko_o4", REPO_ROOT / "config" / "h1_real_p4_ko_o4_hip_knee_eid.yaml", 1.0, 1.0),
    ParamSpec("o5", "ko", "Ko_o5", REPO_ROOT / "config" / "h1_real_p4_ko_o5_hip_knee_eid.yaml", 1.25, 1.0),
]

KU_SPECS = [
    ParamSpec("u1", "ku", "Ku_u1", REPO_ROOT / "config" / "h1_real_p4_ku_u1_hip_knee_eid.yaml", 1.0, 0.5),
    ParamSpec("u2", "ku", "Ku_u2", REPO_ROOT / "config" / "h1_real_p4_ku_u2_hip_knee_eid.yaml", 1.0, 0.75),
    ParamSpec("u3", "ku", "Ku_u3", REPO_ROOT / "config" / "h1_real_p4_ku_u3_hip_knee_eid.yaml", 1.0, 1.0),
    ParamSpec("u4", "ku", "Ku_u4", REPO_ROOT / "config" / "h1_real_p4_ku_u4_hip_knee_eid.yaml", 1.0, 1.25),
]

ALL_SPECS = KO_SPECS + KU_SPECS
SPEC_BY_TOKEN = {
    token: spec
    for spec in ALL_SPECS
    for token in (
        spec.group,
        spec.label.lower(),
        f"{spec.scan}_{spec.group}",
    )
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run P4 real H1 EID Ko/Ku parameter scans."
    )
    parser.add_argument("--execute", action="store_true", help="Actually run h1_direct. Default is dry-run.")
    parser.add_argument("--yes", action="store_true", help="Skip the RUN_P4 confirmation prompt.")
    parser.add_argument("--no-sudo", action="store_true", help="Run h1_direct directly instead of through sudo.")
    parser.add_argument("--direct", type=Path, default=DEFAULT_DIRECT, help="Path to h1_direct executable.")
    parser.add_argument("--repeats", type=int, default=3, help="Repeats per parameter group.")
    parser.add_argument("--duration", type=float, default=8.0, help="Duration per run in seconds.")
    parser.add_argument("--pause", type=float, default=5.0, help="Pause between runs in seconds.")
    parser.add_argument("--start-index", type=int, default=1, help="Repeat index for the first run.")
    parser.add_argument(
        "--scan",
        choices=["ko", "ku", "both"],
        default="ko",
        help="Parameter scan to run. Default only scans Ko; run Ku after a stable Ko is selected.",
    )
    parser.add_argument(
        "--groups",
        default="",
        help="Comma-separated parameter groups overriding --scan, e.g. o0,o1,o2 or o5,u4.",
    )
    parser.add_argument(
        "--order",
        choices=["grouped", "by-repeat"],
        default="grouped",
        help="Run order. grouped completes all repeats for one group before increasing gain.",
    )
    parser.add_argument(
        "--target",
        choices=["none", "hip", "knee"],
        default="none",
        help="Disturbed joint metadata. Use none for no-disturbance stability scans.",
    )
    parser.add_argument(
        "--disturbance-method",
        default="manual_push",
        help="Metadata recorded when --target is hip/knee. Ignored when --target none.",
    )
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="Manifest output directory.")
    return parser.parse_args()


def selected_specs(args: argparse.Namespace) -> list[ParamSpec]:
    if args.groups.strip():
        specs: list[ParamSpec] = []
        seen: set[str] = set()
        for raw in args.groups.split(","):
            token = raw.strip().lower()
            if not token:
                continue
            if token not in SPEC_BY_TOKEN:
                choices = ", ".join(spec.group for spec in ALL_SPECS)
                raise SystemExit(f"unknown group '{raw}'. Choices: {choices}")
            spec = SPEC_BY_TOKEN[token]
            if spec.group in seen:
                continue
            specs.append(spec)
            seen.add(spec.group)
        if not specs:
            raise SystemExit("--groups did not contain any valid groups")
        return specs

    if args.scan == "ko":
        return KO_SPECS
    if args.scan == "ku":
        return KU_SPECS
    return ALL_SPECS


def validate_args(args: argparse.Namespace, specs: list[ParamSpec]) -> None:
    if args.repeats <= 0:
        raise SystemExit("--repeats must be positive")
    if args.duration <= 0:
        raise SystemExit("--duration must be positive")
    if args.pause < 0:
        raise SystemExit("--pause must be non-negative")
    if args.start_index <= 0:
        raise SystemExit("--start-index must be positive")
    if args.target != "none" and args.disturbance_method.strip() in {"", "none"}:
        raise SystemExit("--disturbance-method must be non-empty when --target is hip/knee")
    if not args.direct.exists():
        raise SystemExit(f"h1_direct not found: {args.direct}")
    for spec in specs:
        if not spec.config.exists():
            raise SystemExit(f"config not found: {spec.config}")


def build_plan(args: argparse.Namespace, specs: list[ParamSpec]) -> list[tuple[int, ParamSpec]]:
    indices = range(args.start_index, args.start_index + args.repeats)
    plan: list[tuple[int, ParamSpec]] = []

    if args.order == "by-repeat":
        for repeat in indices:
            for spec in specs:
                plan.append((repeat, spec))
    else:
        for spec in specs:
            for repeat in indices:
                plan.append((repeat, spec))
    return plan


def disturbance_method_for(args: argparse.Namespace) -> str:
    return "none" if args.target == "none" else args.disturbance_method


def condition_for(args: argparse.Namespace, spec: ParamSpec) -> str:
    if args.target == "none":
        return f"P4_EID_{spec.label}_anti_phase_no_disturbance"
    target_code = "H" if args.target == "hip" else "K"
    return f"P4-{target_code}_EID_{spec.label}_anti_phase_{args.target}_disturbance"


def command_for(args: argparse.Namespace, repeat: int, spec: ParamSpec) -> list[str]:
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
        disturbance_method_for(args),
    ]
    if not args.no_sudo:
        cmd = ["sudo", "-n", *cmd]
    return cmd


def shell_join(cmd: list[str]) -> str:
    return " ".join(f"'{part}'" if any(c.isspace() for c in part) else part for part in cmd)


def config_log_name(spec: ParamSpec) -> str:
    for line in spec.config.read_text(encoding="utf-8").splitlines():
        if line.strip().startswith("log_path:"):
            value = line.split(":", 1)[1].strip()
            return Path(value).name
    raise RuntimeError(f"{spec.config} missing log_path")


def recent_completed_log(
    args: argparse.Namespace,
    spec: ParamSpec,
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
    expected_method = disturbance_method_for(args)

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
        if target_seen != args.target or method_seen != expected_method:
            continue
        median_dt = statistics.median(dts) if dts else 0.0
        duration = max(0, last_cycle - first_cycle) * median_dt
        if duration >= 0.90 * expected_duration:
            return path
    return None


def confirm_or_exit(args: argparse.Namespace, plan: list[tuple[int, ParamSpec]]) -> None:
    method = disturbance_method_for(args)
    print(
        f"P4 batch plan: {len(plan)} runs, duration={args.duration:g}s, pause={args.pause:g}s, "
        f"target={args.target}, disturbance_method={method}, order={args.order}"
    )
    for i, (repeat, spec) in enumerate(plan, start=1):
        print(
            f"{i:02d}. {spec.label} r{repeat:02d}: "
            f"Ko={spec.ko_scale:g}, Ku={spec.ku_scale:g}, {spec.config.relative_to(REPO_ROOT)}"
        )

    if not args.execute:
        print("\nDry-run only. Add --execute to run the real robot batch.")
        return

    if args.yes:
        return

    print("\nThis will command the real robot.")
    if args.target == "none":
        print("No external disturbance is expected; this is a stability/smoke scan.")
    else:
        print("Manual disturbance is expected during each run; keep timing and direction repeatable.")
    print("Ensure suspension/limits, E-stop, and a watcher are ready.")
    answer = input("Type RUN_P4 to start the batch: ").strip()
    if answer != "RUN_P4":
        raise SystemExit("confirmation failed; batch not started")


def ensure_sudo(args: argparse.Namespace) -> None:
    if args.no_sudo or not args.execute:
        return
    print("Checking sudo credentials with `sudo -v`...")
    subprocess.run(["sudo", "-v"], check=True)


def open_manifest(args: argparse.Namespace) -> tuple[Path, csv.DictWriter, object]:
    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    scan_label = args.scan if not args.groups.strip() else "custom"
    target_label = args.target if args.target != "none" else "nodist"
    path = args.out_dir / f"p4_{scan_label}_{target_label}_batch_manifest_{stamp}.csv"
    fh = path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(
        fh,
        fieldnames=[
            "batch_time",
            "scan",
            "group",
            "ko_scale",
            "ku_scale",
            "target",
            "disturbance_method",
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


def run_batch(args: argparse.Namespace, plan: list[tuple[int, ParamSpec]]) -> Path | None:
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
            print(f"\n[{run_no}/{len(plan)}] {spec.label} r{repeat:02d} {condition}")
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
                    "scan": spec.scan,
                    "group": spec.group,
                    "ko_scale": spec.ko_scale,
                    "ku_scale": spec.ku_scale,
                    "target": args.target,
                    "disturbance_method": disturbance_method_for(args),
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
    specs = selected_specs(args)
    validate_args(args, specs)
    plan = build_plan(args, specs)
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
